#!/usr/bin/env python3
"""panel_server.py — 원격 제어판 서버 (젯슨에서 돈다).

설계: docs/원격_제어판_설계.md

  · 127.0.0.1:8765 에만 bind 한다. 바깥에서 직접 못 붙는다 — 보안은 SSH 에 맡긴다.
      노트북:  ssh -N -L 8765:127.0.0.1:8765 jetson@<IP>
  · 줄 단위 JSON over TCP (NDJSON). 젯슨에 websockets·fastapi 가 없고 대회장에서
    pip 를 못 쓸 수 있어 표준 라이브러리만 쓴다.
  · **서버 주도 UI.** 레이아웃·버튼 이름·명령·검증·경고가 전부 여기 있다.
    클라이언트는 그리기만 한다 → race.sh 에 모드를 추가해도 클라이언트를 안 고친다.
  · 서버가 프로세스를 소유한다. 노트북이 끊겨도 주행 노드는 안 죽는다.

★ 안전 — 무선 E-STOP 은 그 자체로 안전장치가 아니다
  와이파이가 끊기면 정지 명령도 안 간다. 그래서 반대로 한다: 클라이언트가 10Hz
  하트비트를 보내고, 끊기면 **서버가** 멈춘다 (아래 DEADMAN_SEC).
  최종 안전장치는 여전히 조이스틱 LB 다 — 2.4GHz 직결이라 와이파이와 무관하다.
"""
import json
import os
import queue
import signal
import socket
import socketserver
import subprocess
import sys
import threading
import time

PROTOCOL = 2                 # 위젯 어휘·메시지 형식이 바뀌면 올린다
HOST, PORT = '127.0.0.1', 8765
REPO = os.path.dirname(os.path.abspath(__file__))
LOG_ROOT = os.path.join(REPO, 'logs', 'panel')
DEADMAN_SEC = 0.3            # 수동 모드에서 하트비트가 이만큼 끊기면 정지
LOG_TAIL = 400               # 새 클라이언트에게 되돌려 줄 줄 수

# 제어판에서 띄울 수 있는 모드. (키, 표시이름, 설명)
#   ★ 여기만 고치면 클라이언트는 그대로다.
MODES = [
    ('box',               'box',      '스택만 기동 — 바퀴 안 돎'),
    ('q1',                'q1',       '라인만 따라 주행'),
    ('race',              'race',     '라인 + 장애물 회피'),
    ('mapping-carto-joy', '맵핑',     'Cartographer 맵핑'),
]


# ══════════════════════════════════════════════════════════════════════════════
# ROS — 없으면 E-STOP 만 빠지고 나머지는 그대로 돈다 (ROS 없이 개발·테스트 가능)
# ══════════════════════════════════════════════════════════════════════════════
class Ros:
    """/teleop 으로 정지 명령을 낸다. mux priority 100 이라 자율 /drive(10)를 이긴다."""

    def __init__(self, log):
        self.ok = False
        self.stop_flag = threading.Event()
        try:
            import rclpy
            from ackermann_msgs.msg import AckermannDriveStamped
            rclpy.init(args=None)
            self._rclpy = rclpy
            self._msg_cls = AckermannDriveStamped
            self.node = rclpy.create_node('panel_server')
            self.pub = self.node.create_publisher(AckermannDriveStamped, '/teleop', 10)
            self.ok = True
            threading.Thread(target=self._spin, daemon=True).start()
            threading.Thread(target=self._pump, daemon=True).start()
            log('ROS 연결됨 — E-STOP 사용 가능 (/teleop)')
        except Exception as exc:
            log(f'ROS 없음 — E-STOP 비활성 ({exc.__class__.__name__}: {exc})')

    def _spin(self):
        try:
            self._rclpy.spin(self.node)
        except Exception:
            pass

    def _pump(self):
        """정지 상태인 동안 20Hz 로 0 을 계속 낸다.
        mux timeout 이 0.2s 라 한 번 보내고 말면 자율 명령이 다시 이긴다."""
        while True:
            if self.stop_flag.is_set():
                m = self._msg_cls()
                m.header.stamp = self.node.get_clock().now().to_msg()
                m.drive.speed = 0.0
                m.drive.steering_angle = 0.0
                try:
                    self.pub.publish(m)
                except Exception:
                    pass
            time.sleep(0.05)

    def halt(self, on):
        if not self.ok:
            return False
        self.stop_flag.set() if on else self.stop_flag.clear()
        return True


# ══════════════════════════════════════════════════════════════════════════════
# 설정 — 전부 tune.sh 를 거친다
#   ★ yaml 을 직접 쓰지 않는 이유: tune.sh 에 백업·검증·--undo 가 이미 있고,
#     에디터로 연 파일과 충돌하지 않는다. 두 경로로 쓰면 반드시 어긋난다.
# ══════════════════════════════════════════════════════════════════════════════
MAPS_DIR = os.path.join(REPO, 'src', 'race_stack', 'maps')
WPS_DIR = os.path.join(REPO, 'src', 'race_stack', 'waypoints')
LOC_MODES = [('amcl', 'AMCL', '파티클 필터. 길을 잃어도 복구를 시도한다'),
             ('slam_toolbox', 'slam_toolbox', '스캔 매칭. .posegraph 필요'),
             ('cartographer', 'carto', '스캔 매칭 + pose graph. .pbstream 필요')]


def tune_get(key, default=''):
    try:
        out = subprocess.run(['./tune.sh', key], cwd=REPO, timeout=20,
                             capture_output=True, text=True).stdout.strip()
        return out.splitlines()[-1].strip() if out else default
    except Exception:
        return default


def tune_set(key, value):
    """성공이면 None, 실패면 사람이 읽을 메시지."""
    try:
        r = subprocess.run(['./tune.sh', f'{key}={value}'], cwd=REPO, timeout=25,
                           capture_output=True, text=True)
        if r.returncode != 0:
            return (r.stderr or r.stdout).strip()[:300] or f'{key} 설정 실패'
        return None
    except Exception as exc:
        return f'{key} 설정 실패: {exc}'


def scan_maps():
    """.pgm 이 있는 것만 맵이다 — maps/ 에는 미리보기 PNG 도 같이 산다."""
    try:
        return sorted(f[:-4] for f in os.listdir(MAPS_DIR) if f.endswith('.pgm'))
    except OSError:
        return []


def scan_wps():
    try:
        return sorted(f[:-4] for f in os.listdir(WPS_DIR) if f.endswith('.csv'))
    except OSError:
        return []


def _base_names(map_name):
    """<맵>, 그리고 파생 접미사를 벗긴 원본 이름.
    _clean 은 원본에서 파생된 **이미지**라 자기 posegraph/pbstream 이 없다."""
    out = [map_name]
    for suf in ('_clean_loc', '_clean'):
        if map_name.endswith(suf):
            out.append(map_name[:-len(suf)])
            break
    return out


def check_ready(map_name, wp_name, loc_mode):
    """주행을 시작해도 되는지. 안 되면 사람이 읽을 이유를 돌려준다.

    ★ 전부 실제로 겪은 실패다:
      · 2026-11-09 waypoints 를 1109-2_clean_wp 로 적었는데 실제 파일은
        ..._map_wp.csv 라 launch 가 죽었다 — 라인까지 다 만든 뒤였다.
      · cartographer 인데 pbstream 이 없어 주행 직전에 막혔다.
    """
    if not map_name:
        return '맵이 선택되지 않았습니다.'
    pgm = os.path.join(MAPS_DIR, map_name + '.pgm')
    if not os.path.exists(pgm):
        return f'맵 이미지가 없습니다: {map_name}.pgm'
    if not os.path.exists(os.path.join(MAPS_DIR, map_name + '.yaml')):
        return f'맵 yaml 이 없습니다: {map_name}.yaml'
    if not wp_name:
        return '웨이포인트가 선택되지 않았습니다.'
    csv_path = os.path.join(WPS_DIR, wp_name + '.csv')
    if not os.path.exists(csv_path):
        return f'웨이포인트 파일이 없습니다: {wp_name}.csv'
    try:
        if sum(1 for _ in open(csv_path)) < 3:
            return f'{wp_name}.csv 가 비어 있습니다 (점이 없습니다).'
    except OSError as exc:
        return f'{wp_name}.csv 를 읽을 수 없습니다: {exc}'
    need = {'cartographer': '.pbstream', 'slam_toolbox': '.posegraph'}.get(loc_mode)
    if need:
        cands = [b + need for b in _base_names(map_name)]
        if not any(os.path.exists(os.path.join(MAPS_DIR, c)) for c in cands):
            return (f'{loc_mode} 모드인데 {" / ".join(cands)} 가 없습니다.\n'
                    f'그 방식은 .pgm 이 아니라 이 파일을 읽습니다 — 역산할 방법이 '
                    f'없으니 다시 측량해야 합니다.')
    return None


# ══════════════════════════════════════════════════════════════════════════════
# 상태 — 모든 클라이언트가 공유한다
# ══════════════════════════════════════════════════════════════════════════════
class State:
    def __init__(self):
        self.lock = threading.RLock()
        self.clients = []                 # [ClientHandler]
        self.proc = None                  # 지금 돌고 있는 race.sh
        self.mode = None
        self.sel_mode = MODES[0][0]       # 라디오 선택값 (아직 실행 전)
        self.estop = False
        self.manual = False               # 수동 조작 중인가 (데드맨 대상)
        self.last_beat = {}               # client -> time
        self.logs = {}                    # tab -> [line]
        self.log_dir = os.path.join(LOG_ROOT, time.strftime('%Y%m%d_%H%M%S'))
        self.files = {}                   # tab -> file object
        os.makedirs(self.log_dir, exist_ok=True)
        self.ros = Ros(lambda s: self.log('panel', s))

    # ── 로그 ──────────────────────────────────────────────────────────────
    def log(self, tab, line):
        line = line.rstrip('\n')
        with self.lock:
            buf = self.logs.setdefault(tab, [])
            buf.append(line)
            del buf[:-LOG_TAIL]
            f = self.files.get(tab)
            if f is None:
                f = open(os.path.join(self.log_dir, f'{tab}.log'), 'a', buffering=1)
                self.files[tab] = f
            try:
                f.write(line + '\n')
            except Exception:
                pass
            self.broadcast({'op': 'log', 'tab': tab, 'line': line})

    def broadcast(self, msg):
        for c in list(self.clients):
            c.send(msg)

    # ── 프로세스 ──────────────────────────────────────────────────────────
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, mode):
        with self.lock:
            if self.running():
                return f'{self.mode} 이(가) 이미 실행 중입니다. 먼저 정지하세요.'
            if mode not in [m[0] for m in MODES]:
                return f'모르는 모드: {mode}'
            self.estop = False
            self.ros.halt(False)
            # ★ 새 세션으로 띄운다. race.sh 가 launch 를 통째로 띄우므로
            #   정리할 때 프로세스 그룹째 다뤄야 한다.
            self.proc = subprocess.Popen(
                ['./race.sh', mode], cwd=REPO,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, start_new_session=True)
            self.mode = mode
            threading.Thread(target=self._drain, args=(self.proc, mode), daemon=True).start()
            self.log('panel', f'시작: ./race.sh {mode}')
            self.push_status()
            return None

    def _drain(self, proc, mode):
        for line in proc.stdout:
            self.log(mode, line)
        rc = proc.wait()
        self.log('panel', f'종료: {mode} (코드 {rc})')
        self.push_status()

    def stop(self):
        """./race.sh stop 으로 정리한다. SIGKILL 로 죽이면 고아 노드가 남아
        다음 실행에서 'Failed to find a free participant index' 가 난다."""
        with self.lock:
            self.log('panel', '정지: ./race.sh stop')
            try:
                subprocess.run(['./race.sh', 'stop'], cwd=REPO, timeout=40,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception as exc:
                self.log('panel', f'  stop 실패: {exc}')
            p = self.proc
            if p is not None and p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)
                    p.wait(timeout=10)
                except Exception:
                    try:
                        os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    except Exception:
                        pass
            self.proc = None
            self.mode = None
            self.push_status()
            return None

    # ── 레이아웃 / 상태 ───────────────────────────────────────────────────
    # 키 → (속성 이름, 기본값). tune.sh 키와 서버 속성의 유일한 대응표다.
    CFG_KEYS = (('map_name', 'map_name', ''), ('waypoints', 'wp_name', ''),
                ('localization.mode', 'loc_mode', 'amcl'),
                ('cap', 'cap', '0'), ('q1', 'eta', '0.5'))

    def cfg_refresh(self, only=None):
        """race_script.yaml 의 현재 값을 읽어 둔다 (tune.sh 경유).

        ★ only 를 주면 그 키만 읽는다. tune.sh 는 한 번 돌 때마다 python 을
          띄워 yaml 을 파싱하므로 5개를 매번 다시 읽으면 **6초를 넘긴다** —
          값 하나 바꿀 때마다 화면이 멎는 것처럼 보였다. 바뀐 것만 읽는다.
        """
        with self.lock:
            for key, attr, dflt in self.CFG_KEYS:
                if only is not None and key != only:
                    continue
                setattr(self, attr, tune_get(key, dflt))

    def layout(self):
        self.cfg_refresh()
        return {'op': 'layout', 'widgets': [
            {'t': 'group', 'text': '실행', 'children': [
                {'t': 'radio', 'id': 'mode', 'value': self.sel_mode,
                 'options': [[k, lbl, tip] for k, lbl, tip in MODES]},
                {'t': 'row', 'children': [
                    {'t': 'button', 'id': 'start', 'text': '시작', 'color': '#2e7d32'},
                    {'t': 'button', 'id': 'stop', 'text': '정지'},
                ]},
                {'t': 'label', 'id': 'runstate', 'text': ''},
            ]},
            {'t': 'group', 'text': '맵 / 경로', 'children': [
                {'t': 'combo', 'id': 'map_name', 'label': '맵',
                 'value': self.map_name, 'options': scan_maps()},
                {'t': 'combo', 'id': 'waypoints', 'label': '웨이포인트',
                 'value': self.wp_name, 'options': scan_wps()},
                {'t': 'radio', 'id': 'loc_mode', 'value': self.loc_mode,
                 'options': [list(x) for x in LOC_MODES]},
                {'t': 'row', 'children': [
                    {'t': 'button', 'id': 'cfg_reload', 'text': '새로고침'},
                ]},
                {'t': 'label', 'id': 'cfgstate', 'text': ''},
            ]},
            {'t': 'group', 'text': '속도', 'children': [
                {'t': 'spin', 'id': 'cap', 'label': '전 모드 상한 cap',
                 'value': self.cap, 'min': 0.0, 'max': 8.0, 'step': 0.5,
                 'suffix': ' m/s  (0=끔)'},
                {'t': 'spin', 'id': 'q1', 'label': 'q1 마찰예산 η',
                 'value': self.eta, 'min': 0.1, 'max': 1.0, 'step': 0.1,
                 'suffix': ''},
            ]},
            {'t': 'group', 'text': '안전', 'children': [
                {'t': 'button', 'id': 'estop', 'text': '긴급 정지', 'color': '#c62828'},
                {'t': 'label', 'id': 'safety', 'text': ''},
            ]},
            {'t': 'logtabs', 'id': 'logs'},
            {'t': 'statusline', 'id': 'status'},
        ]}

    def push_status(self):
        run = f'실행 중: {self.mode}' if self.running() else '정지 상태'
        problem = check_ready(self.map_name, self.wp_name, self.loc_mode)
        cfg = f'{self.map_name} / {self.wp_name} / {self.loc_mode}'
        if problem:
            cfg += f'\n⚠ {problem}'
        # ★ cap 이 걸려 있으면 **항상** 보여준다. 저속 시험용으로 걸어둔 걸 잊어
        #   "왜 느리지" 로 헤맨 적이 있다 (2026-11-09).
        try:
            capv = float(self.cap)
        except ValueError:
            capv = 0.0
        if capv > 0:
            cfg += f'\n■ cap {capv:g} m/s 가 걸려 있습니다 (전 모드 상한)'
        self.broadcast({'op': 'update', 'id': 'cfgstate', 'text': cfg})
        es = '■ 긴급 정지 걸림' if self.estop else ('대기' if self.ros.ok else 'ROS 없음 — E-STOP 비활성')
        self.broadcast({'op': 'update', 'id': 'runstate', 'text': run})
        self.broadcast({'op': 'update', 'id': 'safety', 'text': es})
        self.broadcast({'op': 'update', 'id': 'estop',
                        'text': '긴급 정지 해제' if self.estop else '긴급 정지'})
        self.broadcast({'op': 'update', 'id': 'status',
                        'text': f'{run}   |   {es}   |   로그 {self.log_dir}'})

    # ── 액션 ──────────────────────────────────────────────────────────────
    def action(self, aid, value):
        if aid == 'mode':
            self.sel_mode = value
            return None
        # ★ 실행 중에는 맵/경로/위치추정을 못 바꾼다. launch 시점에만 읽히므로
        #   바꿔도 안 먹는데 화면만 바뀌면 거짓말이 된다.
        if aid in ('map_name', 'waypoints', 'loc_mode', 'cfg_reload'):
            if aid != 'cfg_reload' and self.running():
                self.push_status()
                return (f'{self.mode} 실행 중에는 바꿀 수 없습니다.\n'
                        f'설정은 기동할 때만 읽힙니다 — 먼저 정지하세요.')
            if aid == 'map_name':
                err = tune_set('map_name', value)
            elif aid == 'waypoints':
                err = tune_set('waypoints', value)
            elif aid == 'loc_mode':
                err = tune_set('localization.mode', value)
            else:
                err = None
            self.cfg_refresh(only={'map_name': 'map_name', 'waypoints': 'waypoints',
                                   'loc_mode': 'localization.mode'}.get(aid))
            self.push_status()
            return err
        if aid in ('cap', 'q1'):
            if aid == 'cap':
                try:
                    v = float(value)
                except (TypeError, ValueError):
                    return f'숫자가 아닙니다: {value}'
                # ★ 0.5 m/s ≈ 2100 erpm 은 센서리스 BLDC 가 가장 불안정한 구간이다.
                #   2026-11-09 박스 실측: 지령 500~1000 erpm 에서 전류 30~50A 를
                #   먹고 거의 안 돌았고, 800 erpm 지령에 -314 erpm(역방향)까지 튀었다.
                #   4000 erpm 에서는 전류 6.2A 로 깨끗하게 추종한다.
                warn = (0 < v <= 0.5)
            else:
                warn = False
            err = tune_set('cap' if aid == 'cap' else 'q1', value)
            self.cfg_refresh(only=aid)
            self.push_status()
            if err:
                return err
            if warn:
                return ('cap 0.5 m/s 이하는 권하지 않습니다.\n'
                        '그 속도(≈2100 erpm)는 센서리스 모터가 가장 불안정한 '
                        '구간이라 울컥거리고 오도메트리가 0 으로 읽힙니다.\n'
                        '저속 시험은 1.0 m/s 이상으로 하세요.')
            return None
        if aid == 'start':
            problem = check_ready(self.map_name, self.wp_name, self.loc_mode)
            if problem and self.sel_mode not in ('mapping-carto-joy',):
                return f'시작할 수 없습니다 — {problem}'
            return self.start(self.sel_mode)
        if aid == 'stop':
            return self.stop()
        if aid == 'estop':
            self.estop = not self.estop
            if not self.ros.halt(self.estop):
                self.estop = False
                self.push_status()
                return 'ROS 가 없어 E-STOP 을 쓸 수 없습니다. panel.sh 로 띄웠는지 확인하세요.'
            self.log('panel', '■ 긴급 정지' if self.estop else '긴급 정지 해제')
            self.push_status()
            return None
        return f'모르는 action: {aid}'

    # ── 데드맨 ────────────────────────────────────────────────────────────
    def deadman_loop(self):
        while True:
            time.sleep(0.05)
            with self.lock:
                if not self.manual or not self.clients:
                    continue
                newest = max(self.last_beat.values()) if self.last_beat else 0
                if time.time() - newest > DEADMAN_SEC and not self.estop:
                    self.estop = True
                    self.ros.halt(True)
                    self.log('panel', f'데드맨: 하트비트 {DEADMAN_SEC}s 끊김 → 정지')
                    self.push_status()


STATE = None


# ══════════════════════════════════════════════════════════════════════════════
# TCP
# ══════════════════════════════════════════════════════════════════════════════
class ClientHandler(socketserver.StreamRequestHandler):
    def send(self, msg):
        try:
            self.wfile.write((json.dumps(msg, ensure_ascii=False) + '\n').encode())
        except Exception:
            pass

    def handle(self):
        st = STATE
        # ★ hello 가 **반드시 첫 줄**이어야 한다. 클라이언트가 프로토콜 번호를 먼저
        #   확인한 뒤에야 나머지를 해석한다. 그래서 clients 에 등록하기 전에 보낸다 —
        #   등록 뒤에 보내면 다른 곳의 broadcast(log 등)가 먼저 끼어들 수 있다.
        self.send({'op': 'hello', 'protocol': PROTOCOL, 'title': 'car1 제어판'})
        self.send(st.layout())
        with st.lock:
            for tab, lines in st.logs.items():
                for ln in lines[-LOG_TAIL:]:
                    self.send({'op': 'log', 'tab': tab, 'line': ln})
            st.clients.append(self)
            st.last_beat[self] = time.time()
        st.push_status()
        st.log('panel', f'클라이언트 접속 {self.client_address}')
        try:
            # 클라이언트가 그냥 끊으면 ConnectionResetError 가 난다 — 정상 종료로 본다.
            for raw in self.rfile:
                try:
                    m = json.loads(raw.decode())
                except Exception:
                    continue
                op = m.get('op')
                if op == 'heartbeat':
                    with st.lock:
                        st.last_beat[self] = time.time()
                elif op == 'action':
                    err = st.action(m.get('id'), m.get('value'))
                    if err:
                        self.send({'op': 'alert', 'level': 'warn', 'text': err})
                elif op == 'manual':
                    with st.lock:
                        st.manual = bool(m.get('on'))
        except (ConnectionResetError, BrokenPipeError, OSError):
            pass
        finally:
            with st.lock:
                if self in st.clients:
                    st.clients.remove(self)
                st.last_beat.pop(self, None)
            st.log('panel', f'클라이언트 끊김 {self.client_address}')


class Server(socketserver.ThreadingTCPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    global STATE
    STATE = State()
    threading.Thread(target=STATE.deadman_loop, daemon=True).start()
    srv = Server((HOST, PORT), ClientHandler)
    print(f'panel_server: {HOST}:{PORT}  protocol {PROTOCOL}')
    print(f'  로그: {STATE.log_dir}')
    print(f'  노트북에서:  ssh -N -L {PORT}:{HOST}:{PORT} jetson@<젯슨IP>')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\n종료 — 실행 중인 주행은 그대로 둡니다 (./race.sh stop 으로 정리하세요)')


if __name__ == '__main__':
    main()
