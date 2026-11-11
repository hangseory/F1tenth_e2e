#!/usr/bin/env python3
"""
steer_limit_logs.py — 주행 로그에서 차가 **실제로** 도는 조향 한계를 잰다.

  ./race.sh steer-meas                 가장 최근 조향 설정의 로그들로
  ./race.sh steer-meas 20261111        이름에 20261111 이 들어간 로그만
  ./race.sh steer-meas --car car1      wheelbase 를 그 차 값으로

방법:
  driven_path.csv 에서 명령 조향이 한계(|δ| ≥ 로그 최대의 99%)에 연속으로 붙은
  구간(0.6 s 이상, 23° 이상 회전)을 고르고, 서보가 다 돌아가기 전인 첫 0.25 s 를
  버린 뒤 궤적(x,y)에 원을 맞춰 반경을 잰다. 유효 조향 = atan(L / R).

  명목 max_steer 는 서보 리밋에서 **계산한** 명령 한계라, 바퀴가 실제로 그만큼
  꺾이는지는 모른다. 1호차 2026-11-11: 명목 27.5° → 실측 21.2° (34구간).

  로그마다 조향 최대값이 다르면(서보 리밋·중립을 바꾼 전후) 섞지 않는다 —
  기본은 가장 최근 설정의 로그들만 쓴다.

결과의 '권장값' 을 config/vehicle_<차>.yaml 의 geometry.steer_eff_max 에 넣으면
라인 생성기(mintime)와 ./race.sh wp 가 그 값으로 판단한다.
"""
import argparse
import glob
import math
import os

import numpy as np
import yaml

REPO = os.environ.get('RACE_REPO') or os.path.abspath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..'))


def fit_circle(x, y):
    A = np.column_stack([2 * x, 2 * y, np.ones_like(x)])
    c, *_ = np.linalg.lstsq(A, x ** 2 + y ** 2, rcond=None)
    R = math.sqrt(max(c[2] + c[0] ** 2 + c[1] ** 2, 0.0))
    rms = float(np.sqrt(np.mean((np.hypot(x - c[0], y - c[1]) - R) ** 2)))
    return R, rms


def load(d):
    try:
        a = np.genfromtxt(os.path.join(d, 'driven_path.csv'), delimiter=',', names=True)
    except Exception:
        return None
    if a.size < 50 or not {'t', 'x', 'y', 'speed', 'steer', 'yaw'} <= set(a.dtype.names):
        return None
    return a


def segments(a, st_lim, settle=0.25, tail=0.2, min_dur=0.6, min_turn=0.4):
    t, x, y, v, st = a['t'], a['x'], a['y'], a['speed'], a['steer']
    yaw = np.unwrap(a['yaw'])
    sat = np.abs(st) >= st_lim
    out, i = [], 0
    while i < len(t):
        if not sat[i]:
            i += 1
            continue
        j = i
        while (j + 1 < len(t) and sat[j + 1] and np.sign(st[j + 1]) == np.sign(st[i])
               and t[j + 1] - t[j] < 0.3):
            j += 1
        if t[j] - t[i] >= min_dur:
            m = (t >= t[i] + settle) & (t <= t[j] + tail)
            if m.sum() >= 6 and abs(yaw[m][-1] - yaw[m][0]) > min_turn:
                R, rms = fit_circle(x[m], y[m])
                out.append(dict(side=int(np.sign(st[i])), R=R, rms=rms,
                                v=float(v[m].mean()), dur=float(t[m][-1] - t[m][0]),
                                turn=float(math.degrees(abs(yaw[m][-1] - yaw[m][0])))))
        i = j + 1
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('filter', nargs='?', default='', help='로그 폴더 이름 조각')
    ap.add_argument('--car', default=None)
    ap.add_argument('--logs', default=os.path.join(REPO, 'logs'))
    args = ap.parse_args()

    car = args.car
    if not car:
        try:
            car = open(os.path.join(REPO, '.car')).read().strip()
        except OSError:
            car = None
    L, nominal = 0.33, None
    if car:
        v = yaml.safe_load(open(os.path.join(REPO, 'config', f'vehicle_{car}.yaml')))
        L = float(v['geometry']['wheelbase'])
        g, off = float(v['vesc']['steering_angle_to_servo_gain']), float(v['vesc']['steering_angle_to_servo_offset'])
        lo, hi = float(v['vesc']['servo_min']), float(v['vesc']['servo_max'])
        nominal = min(abs((lo - off) / g), abs((hi - off) / g))
        cur = v['geometry'].get('steer_eff_max')

    dirs = sorted(d for d in glob.glob(os.path.join(args.logs, '*_*'))
                  if os.path.isdir(d) and args.filter in os.path.basename(d))
    logs = []
    for d in dirs:
        a = load(d)
        if a is not None:
            logs.append((os.path.basename(d), a, round(float(np.abs(a['steer']).max()), 2)))
    if not logs:
        raise SystemExit('  로그가 없습니다 (driven_path.csv)')

    # 조향 설정이 같은 로그끼리만. 로그에는 어느 차인지 안 남으므로, 그 차의
    # 명목 max_steer 와 조향 최대값이 맞는 로그만 그 차 것으로 본다 — 안 그러면
    # 다른 차 로그로 잰 값을 이 차에 넣게 된다 (logs/ 는 두 차가 같이 쓴다).
    if nominal:
        mine = [l for l in logs if abs(l[2] - nominal) < 0.015]
        if not mine:
            seen = sorted({l[2] for l in logs})
            raise SystemExit(
                f'  ✗ {car} 의 명목 max_steer {nominal:.3f} 와 조향 최대값이 맞는 로그가 없습니다.\n'
                f'    로그들의 조향 최대값: {seen}\n'
                f'    다른 차 로그일 수 있습니다 — --car 를 확인하세요 (예: --car car1)')
        logs = mine
    smax = logs[-1][2]
    use = [(n, a) for n, a, m in logs if m == smax]
    skipped = len(logs) - len(use)
    print(f'\n  차 {car or "?"}  wheelbase {L:.3f} m'
          + (f'  명목 max_steer {nominal:.4f} rad ({math.degrees(nominal):.1f}°)' if nominal else ''))
    print(f'  로그 {len(use)}개 사용 (조향 최대 {smax:.2f} rad 인 것)'
          + (f', 다른 조향 설정 {skipped}개 제외' if skipped else ''))

    segs = []
    for n, a in use:
        for s in segments(a, 0.99 * smax):
            s['log'] = n
            segs.append(s)
    if len(segs) < 3:
        raise SystemExit(f'  조향 한계에 붙은 구간이 {len(segs)}개뿐 — 판단 불가. '
                         f'헤어핀이 있는 트랙을 몇 바퀴 더 달린 로그가 필요합니다.')

    R = np.array([s['R'] for s in segs])
    sides = np.array([s['side'] for s in segs])
    vv = np.array([s['v'] for s in segs])
    print(f'\n  한계 구간 {len(segs)}개  (속도 {vv.min():.2f}~{vv.max():.2f} ㎧, 평균 {vv.mean():.2f})')
    print(f'  {"":6s} {"구간":>4s} {"R 중앙":>7s} {"p25":>6s} {"최소":>6s} {"유효조향":>8s}')
    for sg, nm in ((1, '좌'), (-1, '우'), (0, '전체')):
        r = R if sg == 0 else R[sides == sg]
        if not len(r):
            print(f'  {nm:6s} {0:4d}')
            continue
        med = float(np.median(r))
        print(f'  {nm:6s} {len(r):4d} {med:7.3f} {np.percentile(r, 25):6.3f} {r.min():6.3f}'
              f' {math.degrees(math.atan(L / med)):7.1f}°')
    med = float(np.median(R))
    eff = math.atan(L / med)
    a_lat = float(np.median(vv ** 2 / R))
    print(f'\n  ★ 권장 geometry.steer_eff_max: {eff:.3f}   (R {med:.3f} m, {math.degrees(eff):.1f}°'
          + (f', 명목의 {100*eff/nominal:.0f}%' if nominal else '') + ')')
    if car and cur is not None:
        print(f'    지금 vehicle_{car}.yaml 값: {float(cur):.3f}  (차이 {math.degrees(eff - float(cur)):+.1f}°)')
    print(f'    횡가속 중앙 {a_lat:.2f} ㎧² ({a_lat/9.81:.2f} g) — '
          + ('낮다 → 미끄러진 게 아니라 바퀴가 덜 꺾인 것' if a_lat < 4.0 else
             '높다 → 타이어 슬립 몫이 섞였을 수 있다'))
    if vv.max() - vv.min() < 0.8:
        print(f'    ⚠ 속도 범위가 좁다({vv.min():.1f}~{vv.max():.1f} ㎧). 더 빠를 때 한계는 이 값으로 모른다.')


if __name__ == '__main__':
    main()
