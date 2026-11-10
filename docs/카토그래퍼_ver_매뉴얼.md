# Cartographer ver. 매뉴얼

맵 따기 → 정제 → 웨이포인트 → 주행. **명령은 전부 한 줄입니다. 그대로 복사해 붙이세요.**

| | 어디서 | 무엇 |
|---|---|---|
| **터미널 A** | 젯슨 (SSH) | 맵핑 / 주행 — 켜 둔 채로 |
| **터미널 B** | 젯슨 (SSH) | 저장 / 정제 / 라인 / 설정 |
| **노트북** | 노트북 로컬 | RViz 화면 |

```bash
ssh jetson@<젯슨IP>          # 비밀번호 jetsonjetson
```

> ### ★ 젯슨 IP 는 장소마다 바뀝니다
> 팀 공유기 `192.168.50.x` / 연구실 `192.168.1.x` / 공용 WiFi — 그때그때 다릅니다.
> **젯슨에서 확인:**
> ```bash
> ip -4 -br addr | grep -v docker
> ```
> `wlP1p1s0` 줄의 주소가 젯슨 IP 입니다. `enP8p1s0`(192.168.0.x)은 **라이다 전용망**이니 쓰지 마세요.
>
> `network.subnet` 은 **`auto`** 라 자동으로 맞춰집니다 (기본 라우트가 나가는 인터페이스에서 감지).
> **`network.peers` 만** 젯슨 IP 로 맞추면 됩니다 — 차와 노트북 **양쪽** 파일 모두:
> ```bash
> cd ~/yax && sed -i 's|^    - 192\.168\.[0-9]*\.[0-9]*.*|    - <젯슨IP>|' config/vehicle_car1.yaml && grep -nE 'subnet:|- 192\.168' config/vehicle_car1.yaml | grep -v '#'
> ```
> `<젯슨IP>` 를 실제 숫자로 바꿔서 치세요. 출력에 숫자만 보여야 합니다.
> 공유기에 **고정 IP 할당**을 걸어두면 이 단계가 아예 없어집니다.

---

## 제어판으로 하면 터미널이 하나로 줄어듭니다

모드 실행·정지, 로그, 긴급정지를 **노트북 창 하나**에서 합니다. 설계·구현은 [원격_제어판_설계.md](원격_제어판_설계.md).

### 처음 한 번 — 노트북에 코드 받기

```bash
cd ~/yax && git pull
```

노트북에 **PyQt5 만** 있으면 됩니다. **ROS 는 필요 없습니다.**

```bash
python3 -c "from PyQt5 import QtNetwork; print('OK')" || sudo apt install -y python3-pyqt5
```

### 띄우기 — 순서대로

**① 젯슨 (SSH)** — 서버. 창은 안 뜨고 세 줄만 찍힌 뒤 멈춘 것처럼 보이면 정상입니다.

```bash
cd ~/yax && ./panel.sh --server
```
```
panel_server: 127.0.0.1:8765  protocol 1
  로그: /home/jetson/yax/logs/panel/<시각>
  노트북에서:  ssh -N -L 8765:127.0.0.1:8765 jetson@<젯슨IP>
```

**② 노트북** — SSH 터널. `-N` 이라 **아무것도 안 찍히는 게 정상**입니다.

```bash
ssh -N -L 8765:127.0.0.1:8765 jetson@<젯슨IP> &
```

**③ 노트북** — 제어판.

```bash
cd ~/yax && ./panel.sh
```

### 붙었는지 보는 법

창 제목으로 압니다:

| 제목 | 뜻 |
|---|---|
| **`car1 제어판`** | 붙음 ✓ |
| `제어판 — 연결 끊김, 재접속 중…` | 터널이 없거나 서버가 안 떠 있음 |

젯슨에서도 확인됩니다 — `ESTAB` 줄이 보이면 붙은 겁니다:

```bash
ss -tnp | grep 8765
```

### 왜 SSH 터널인가 — RViz 와 다릅니다

```
제어판 (모드·로그·긴급정지)   SSH 터널    ROS·DDS 불필요
RViz   (맵·스캔·라인)        DDS         ROS 필요
```

제어판은 `ROS_DOMAIN_ID`·`subnet`·`peers`·멀티캐스트를 **쓰지 않습니다.** 2026-11-08~09 에 그 설정들이 안 맞아 RViz 가 하루 종일 안 떴는데, **그동안 SSH 는 멀쩡했습니다.**

그래서 둘을 나눠두면 원인이 좁혀집니다 — **제어판은 되는데 RViz 가 안 되면 네트워크 설정 문제**입니다.

### 안전

> ### ★ 제어판의 긴급정지는 **보조**입니다
> 와이파이가 끊기면 정지 명령도 안 갑니다. 그래서 반대로 설계했습니다 —
> 클라이언트가 10 Hz 하트비트를 보내고, **끊기면 서버가 알아서 멈춥니다.**
>
> **최종 안전장치는 조이스틱 `LB`** 입니다. 2.4 GHz 직결이라 와이파이와 무관합니다.
> **차 옆에는 항상 조이스틱을 든 사람이 있어야 합니다.**

- 노트북을 닫아도 **젯슨 노드는 안 죽습니다.** 다시 접속하면 현재 상태를 받습니다
- 긴급정지는 클라이언트가 끊겨도 **풀리지 않습니다**
- 정지는 `./race.sh stop` 으로 합니다 — 강제 종료하면 고아 노드가 포트를 쥐고 있어 다음 실행이 깨집니다

### 지금 되는 것 / 아직 안 되는 것

| | |
|---|---|
| 되는 것 | 모드 실행·정지(`box`/`q1`/`race`/맵핑), 로그 탭 + 파일 저장, 긴급정지, 데드맨 |
| 아직 | 맵·웨이포인트 선택, `tune.sh` 연동, 상태 표시(배터리·정합), 맵/경로 편집기 |

아직 안 되는 것은 **아래 절차대로 터미널에서** 하세요. 제어판과 같이 써도 됩니다.

---

## 0. 먼저 읽을 것 — 두 가지 함정

### ① `savemap` 이 아니라 `savemap-carto`

| | 저장되는 것 | Cartographer 주행 |
|---|---|---|
| `savemap` | `.pgm` + `.yaml` | **안 됨** |
| `savemap-carto` | `.pgm` + `.yaml` + **`.pbstream`** | 됨 |

Cartographer 위치추정은 점유격자(`.pgm`)가 아니라 **스캔 그래프(`.pbstream`)** 를 읽습니다. `.pgm` 에서 역산할 방법이 **없습니다.**

`savemap` 도 에러 없이 성공합니다 — Cartographer 가 `/map` 을 발행하니까요. 그래서 **조용히 틀립니다.** 라인까지 다 만들고 주행 직전에 막힙니다.

### ② `savemap-carto` 는 같은 이름을 덮어쓰지 않습니다

```
RuntimeError: Already exists: .../maps/1109.pgm. Choose a new map name.
```

`.pbstream`/`.pgm`/`.yaml` 중 **하나라도** 있으면 거부합니다. trajectory 종료 **전에** 검사하므로 맵핑은 안 끊깁니다 — 이름만 바꿔 다시 실행하면 됩니다.

이름을 비우려면:
```bash
cd ~/yax && rm -f src/race_stack/maps/1109.pgm src/race_stack/maps/1109.yaml ~/f1tenth_ws/install/race_stack/share/race_stack/maps/1109.pgm ~/f1tenth_ws/install/race_stack/share/race_stack/maps/1109.yaml
```
`install` 쪽 복사본도 같이 지워야 합니다. 안 지우면 또 거부당합니다.

---

## 1. 맵핑 — 터미널 A

차를 **출발시킬 자리에 놓고** 시작합니다.

```bash
cd ~/yax && ./race.sh mapping-carto-joy
```

> ### ★ 차를 세운 자리가 곧 맵 원점 `(0, 0, 0°)` 입니다
> 나중에 주행할 때도 **그 자리에 그 방향으로** 세워야 위치추정이 맞습니다.

- 처음 몇 초는 **정지**합니다 (IMU 바이어스 초기화)
- `mapping-joy` 와 같은 조이스틱 설정, `speed.mapping_max` 제한이 걸립니다
- 기본 bringup 의 `joy`/`joy_teleop` 은 이 모드에서 처음부터 비활성화됩니다
- 천천히 트랙을 돌아 **시작 지점으로 되돌아옵니다**

### 화면 — 노트북 로컬 (SSH 아님)

```bash
cd ~/yax && ./rviz.sh car1
```

RViz 에서 Fixed Frame `map`, 지도 토픽 `/map`. 벽이 빠짐없이 그려졌는지 확인하세요.

기동 줄에 이게 떠야 통신됩니다:
```
[dds] ROS_DOMAIN_ID=57  (car1)
[dds] CycloneDDS  |  인터페이스 192.168.1.xx 고정  |  피어 1개
```
`⚠ CycloneDDS 폴백` 이 뜨면 그 기기 IP 가 설정된 대역이 아닙니다.

---

## 2. 저장 — 터미널 B (A 를 끄기 전에!)

차를 **완전히 세우고**:

```bash
cd ~/yax && ./race.sh savemap-carto 1109
```

이름은 영문/숫자로 시작, `영문·숫자·_·-` 만.

**이 줄들이 떠야 성공입니다:**
```
Finishing trajectory 0; mapping stops here.
Saved .../maps/1109.pbstream
Saved: 1109.pbstream / .pgm / .yaml
You may now stop the mapping terminal with Ctrl+C.
```

trajectory 를 **종료**하므로 이후 주행은 지도에 추가되지 않습니다. 파일은 `~/yax/src/race_stack/maps/` 에 저장됩니다.

확인:
```bash
cd ~/yax && ls -la src/race_stack/maps/ | grep 1109
```

`.pbstream` 이 보여야 합니다. 확인 후 **터미널 A 에서 `Ctrl+C`**.

### 변환만 실패했을 때

`.pbstream` 은 남습니다. 차량 노드 없이 재변환할 수 있습니다:

```bash
source /opt/ros/humble/setup.bash && ros2 run cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=$HOME/yax/src/race_stack/maps/1109.pbstream -map_filestem=$HOME/yax/src/race_stack/maps/1109 -resolution=0.05
```

⚠ 이 명령은 같은 이름의 PGM/YAML 을 **덮어씁니다.**

---

## 3. 맵 정제 — 터미널 B

```bash
cd ~/yax && ./race.sh clean 1109
```

트랙 경계 틈으로 새어 나간 **가는 빛살**(1~3 px)을 지우고 `1109_clean.pgm` 을 만듭니다. 화면이 필요 없어 SSH 에서 됩니다. 반경을 바꿔가며 되풀이한 뒤 저장합니다.

> ### ★ `maps/1109_clean_preview.png` 를 반드시 눈으로 확인하세요
> **빨강 = 지워진 영역**입니다. 맞는 반경은 트랙마다 다릅니다.
> 트랙 일부가 빨갛게 지워졌으면 **반경을 낮추고** 다시 하세요.

노트북에서 받아 보려면:
```bash
scp jetson@<젯슨IP>:~/yax/src/race_stack/maps/1109_clean_preview.png .
```

정제가 하는 일은 **"트랙만 자유, 나머지 전부 벽"** 입니다. 미지 영역을 벽으로 메워 **닫힌 경계**를 만드는 것이고, 그게 다음 단계 라인 생성(QP)의 전제입니다.

⚠ `_clean` 은 **라인 생성·RViz 용**입니다. 위치추정은 `.pbstream` 을 쓰므로, `map_name` 에 `_clean` 을 적어도 원본 pbstream 을 자동으로 찾습니다.

### 자동 정제가 안 될 때 — 손으로 폴리곤 찍기

```bash
cd ~/yax && DISPLAY=:1 python3 src/race_stack/maps/clean_map.py --map 1109
```

젯슨 모니터에 창이 뜹니다. 설치할 건 없습니다 (matplotlib).

**`clean` 이 자동으로 못 고르는 경우가 있습니다.** `clean` 은 벽을 분석하지 않고 **자유공간의 구멍**만 봅니다:

```
seed    = 가장 큰 연결 성분          트랙 복도
outer   = fill_holes(seed)           구멍을 메움 = 트랙 ∪ 인필드
infield = outer & ~seed              메워진 부분 = 안쪽 섬
```

즉 **"안쪽 벽" 이라는 개념이 "고리 한가운데 구멍이 있다" 는 사실에서만 나옵니다.** 그래서 **바깥 벽이 뚫려 있으면** 빔이 새어 나가 위상이 뒤집히고, 엉뚱한 영역을 트랙으로 고릅니다. 자동으로는 구제할 방법이 없습니다.

폴리곤 방식은 **벽이 끊겨 있어도 상관없습니다.** "여기 안쪽이 트랙" 이라고 사람이 직접 정하니까요.

| 키 | 동작 |
|---|---|
| **`1`** | **바깥 폴리곤** (빨강) — 트랙 바깥 경계. **이 밖은 전부 벽** |
| **`2`** | **안쪽 폴리곤** (파랑) — 인필드 경계. **이 안도 전부 벽** |
| 좌클릭 | 꼭짓점 추가 |
| 우클릭 / `z` | 마지막 꼭짓점 취소 |
| **Enter** | 두 폴리곤 확정 → 마스크 적용 |
| `c` | 틈 봉합(closing) 횟수 토글 `0 → 2 → 4` |
| **`s`** | **저장** → `<맵>_clean.pgm` |
| `r` | 전체 리셋 |
| `q` / `Esc` | 종료 |

두 폴리곤 **사이만** 트랙이 되고, 그 안에서는 원본 픽셀을 그대로 쓰면서 틈을 봉합합니다. 꼭짓점은 촘촘할 필요 없습니다 — 코너마다 하나씩 **10~20개**면 충분합니다.

순서: `1` → 바깥 찍기 → `2` → 안쪽 찍기 → `Enter` → 확인 → 틈이 남으면 `c` → `s` 저장.

> ⚠ slam_toolbox 맵에서는 폴리곤 안의 **미관측 영역이 길로 바뀝니다** — 미지(205)가 `occupancy 0.196` 인데 그 맵의 `free_thresh` 가 0.25 라서입니다. Cartographer 맵(미지 128 → 0.498)은 해당 없습니다. 눈으로 확인하며 쓰는 도구라 당장 막히진 않지만 알고 계세요.

### 그림판처럼 칠해서 벽을 메우고 싶으면

`.pgm` 은 그냥 흑백 이미지라 아무 편집기로나 열립니다. **검정(0)** 으로 뚫린 곳에 2~3 px 선을 그으면 벽이 닫힙니다. 안티앨리어싱은 끄세요.

```bash
sudo apt install -y --no-install-recommends mtpaint
cd ~/yax && cp src/race_stack/maps/1109.pgm src/race_stack/maps/1109.pgm.bak && DISPLAY=:1 mtpaint src/race_stack/maps/1109.pgm
```

부채꼴이 **새어 나간 뿌리**만 막으면 부채꼴 전체가 트랙과 끊겨 `clean` 이 알아서 버립니다. 넓은 면적을 칠할 필요가 없습니다.

---

## 4. 웨이포인트 — 터미널 B

두 가지가 있습니다. **`line` 이 자동, `waypoints-kappa` 가 수동**입니다.

```bash
cd ~/yax && ./race.sh line 1109_clean
```
```bash
cd ~/yax && ./race.sh waypoints-kappa 1109_clean
```

`0814-1_clean` 실측 비교 (같은 맵·같은 물리):

| | 길이 | R_min | 코너여유 | 랩 |
|---|---|---|---|---|
| `waypoints-kappa` (수동, GUI 필요) | 26.60 m | 1.021 | 0.749 | 8.75 s |
| `line` (자동, 화면 불필요) | 24.77 m | 1.079 | 0.550 | **7.52 s** |

`line` 이 1.2 초 빠르지만 **벽에 더 붙습니다.**

### ★ 출력에서 반드시 볼 줄

```
★★ 차량 최소회전반경(0.872m)을 넘는 점이 N개 — 이 라인은 못 돈다
```

**이 줄이 뜨면 그 트랙은 이 차로 못 돕니다.** `--rmin` 을 올려 재생성해도 소용없습니다 — 트랙이 모자란 겁니다. 속도를 줄여도 안 됩니다. **조향각 한계는 속도와 무관**해서, 그 코너에서 차는 핸들을 끝까지 꺾고도 벽으로 갑니다.

### 좁은 트랙이면 기본값이 안 들어갑니다

```bash
cd ~/yax && ./race.sh line 1109_clean --safety 0.10 --wallclear 0.06 --apex-extra 0 --step 0.08
```

| 인자 | 기본값 | 뜻 |
|---|---|---|
| `--safety` | 0.35 | QP 가 쓸 트랙 폭을 이만큼 좁힘. **올리면** 라인이 중앙에 붙음 |
| `--wallclear` | 0.18 | 벽까지 최소 여유 (차폭 **제외**) |
| `--apex-extra` | **0.40** | 코너 안쪽에만 **추가로** 더 주는 여유 |

> ### ★ 좁은 트랙에서는 `--apex-extra 0` 을 꼭 주세요
> 기본 0.40 이 그대로 걸립니다. 폭 0.6 m 트랙은 가용 횡여유가
> `0.6 − 0.31(차폭) − 0.12 = 0.17 m` 뿐인데 코너에서 0.40 을 더 빼라는 셈이 됩니다.
> R_min 에 직접 영향이 갑니다.

여유를 **줄이는 게 답이 아닙니다.** QP 는 반복마다 코너를 더 파고들어 반경은 좋아지지만 **루프가 짧아지고 벽에 붙습니다.** `--wallclear 0` 까지 내려도 선택되는 라인은 바뀌지 않습니다 — 반경이 좋은 반복들은 전부 **차폭여유가 음수**(차체가 벽을 파고듦)입니다.

### 마진을 한계까지 밀어보기 — 자동 스윕

손으로 하나씩 낮춰볼 필요 없습니다. 조합 10개를 보수적 → 공격적으로 쓸어보고 **통과한 것 중 가장 빠른 것**을 고릅니다.

```bash
cd ~/yax/src/race_stack && RACE_WS=$HOME/f1tenth_ws python3 -m race_stack.gen_raceline_aggro --map 1109_clean
```

```
 safety   wclr   apex │   R_min      여유      길이       랩 │ 판정
   0.35   0.18   0.40 │   0.441   0.216   15.59    4.75 │ ✗ 못 돈다 (29점)
   0.20   0.10   0.10 │   0.982   0.128   12.91    3.27 │ ✗ 여유 부족
   0.15   0.08   0.05 │   0.974   0.090   12.46    3.13 │ ← 최단
   0.12   0.07   0.00 │   0.810   0.084   13.16    3.37 │ ✗ 못 돈다 (3점)
```

**공격적으로 갈수록 빨라지는 게 아닙니다.** 여유를 더 주면 QP 가 코너를 파고들어 반경은 좋아지나 벽에 붙고, 덜 주면 라인이 꺾여 반경이 나빠집니다. **최적점이 중간에 있습니다.**

| 인자 | 기본 | 뜻 |
|---|---|---|
| `--cte` | 0.25 | 흡수해야 할 **추종오차** [m]. 벽 여유 하한이 됩니다 |
| `--floor` | 0.06 | 절대 하한. `--cte` 가 더 낮아도 여기는 안 깎입니다 |
| `--keep` | — | 실제로 저장 (없으면 표만). 출력은 `<맵>_aggro_wp` |

**차량 최소회전반경은 절대 완화하지 않습니다.** `R = wheelbase / tan(max_steer)` 는 기하학이지 취향이 아닙니다.

> ### ★ 마진을 줄이는 것이 먼저가 아닙니다
> 벽 여유의 존재 이유는 **라인과 실제 주행의 차이를 흡수**하는 것입니다.
> 이 차의 실측 추종오차(CTE)는 **직선 0.16 m / 코너 0.39~0.55 m** 로 코너에 몰립니다.
>
> 상위팀이 벽을 스치듯 가는 건 마진을 줄여서가 아니라 **추종오차가 작아서**입니다.
> 순서가 반대입니다:
> ```
> ① 위치추정 안정화 → ② 컨트롤러 튜닝으로 CTE 축소 → ③ 그만큼 마진 축소
> ```
> ①을 건너뛰고 ③만 하면 그냥 벽입니다. `--cte` 에는 **실측값**을 넣으세요
> (저속으로 한 바퀴 돌고 `logs/<런>/driven_path.csv` 로 잽니다).

### 결과 확인

```bash
cd ~/yax && ls -la src/race_stack/waypoints/ | grep 1109
```

`waypoints/1109_clean_map_wp.png` 에 속도색 오버레이가 저장됩니다. **주행 전에 라인이 맵과 정렬됐는지 눈으로 확인하세요.**

---

## 5. 설정 — 터미널 B

`race_script.yaml` 을 열지 않고 `tune.sh` 로 바꿉니다. **에디터 저장 충돌이 안 생깁니다.**

```bash
cd ~/yax && ./tune.sh localization.mode=cartographer && ./tune.sh map_name=1109_clean && ./tune.sh waypoints=1109_clean_map_wp
```

현재 값만 보려면 `=` 없이:
```bash
cd ~/yax && ./tune.sh localization.mode && ./tune.sh map_name && ./tune.sh waypoints
```

`.pbstream` 은 **따로 안 적어도 됩니다.** `map_name` 에서 `_clean`/`_clean_loc` 을 벗겨 `1109.pbstream` 을 자동으로 찾습니다.

맵 이름과 다른 pbstream 을 쓸 때만 `race_script.yaml` 의 `localization:` 절에 직접 추가합니다. `tune.sh` 는 **이미 있는 키만** 고치므로 신규 키는 거부합니다:

```yaml
localization:
  cartographer_map: 1109.pbstream    # 또는 절대경로
```

### 초기 위치 (화면 필요)

```bash
cd ~/yax && ./race.sh start_pos --map 1109_clean --wp-file 1109_clean_map_wp --gui
```

위치 → 방향을 클릭하고 `w` 로 저장합니다. **차량도 그 위치와 방향으로 놓습니다.**

---

## 6. 저속 확인 → 주행

### 먼저 정지 상태로 확인 — 터미널 A

```bash
cd ~/yax && ./race.sh stop && ./race.sh box
```

`box` 는 바퀴가 안 돕니다. 노트북에서 `cd ~/yax && ./rviz.sh car1` — **스캔(빨간 점)이 맵 벽과 겹치는지** 확인합니다. 어긋나 있으면 차는 자기가 어디 있는지 모르는 상태입니다.

어긋났으면 RViz **`2D Pose Estimate`** 로 실제 위치를 클릭한 채 진행 방향으로 드래그. 그다음 터미널 B 에서:

```bash
cd ~/yax && ./race.sh savepose
```

⚠ `/initialpose` 는 **최신 odom 이 있고 차가 정지했을 때만** 받습니다. 주행 중 재설정 요청은 거부됩니다. 현재 trajectory 를 종료하고 지정한 위치에서 새 trajectory 를 시작하며, 원본 맵은 이때도 고정 상태입니다.

> ### ★ 초기 위치는 RViz 로 찍어도 **저장되지 않습니다**
> `2D Pose Estimate` 는 `/initialpose` 토픽이라 **그 순간만** 적용됩니다.
> 스택을 끄면 사라지고, 다시 띄우면 `race_script.yaml` 의 옛 값으로 돌아옵니다.
> 파일에 써넣는 방법은 셋뿐입니다:
>
> ```bash
> # ① 지금 추정 중인 위치를 저장 (스택을 켠 채로, 다른 터미널에서)
> cd ~/yax && ./race.sh savepose
>
> # ② 맵 그림에서 클릭 — 1회째=위치, 2회째=바라볼 방향, w=저장
> cd ~/yax && DISPLAY=:1 ./race.sh start_pos --map 1109_clean --wp-file 1109_clean_map_wp --gui
>
> # ③ 웨이포인트 0번에 붙이기 (가장 안전 — 클릭이 없어 틀릴 수 없다)
> cd ~/yax && python3 set_init_pose.py --map 1109_clean --wp-file 1109_clean_map_wp --wp 0 --write
> ```
>
> `start_pos` 는 **`w` 를 눌러야** 저장됩니다. `set_init_pose.py` 는 **`--write` 가 없으면** 미리보기만 합니다.
> 어느 쪽이든 그다음 **스택 재시작**이 필요합니다 — launch 시점에만 읽습니다.

### 저속 시험 — 터미널 A

```bash
cd ~/yax && ./tune.sh cap=0.5 && ./race.sh q1
```

`cap=0.5` 는 **이후 실행에도 남는** 0.5 m/s 전 모드 천장입니다.

기동 로그 확인:
```
[race.launch] 위치추정: cartographer
```

### 출발이 느린 건 정상입니다 — `출발제한`

로그에 이게 뜹니다:
```
[출발제한] 상한 1.59㎧²  1.01→1.00㎧
[출발제한] 상한 1.78㎧²  1.41→1.40㎧
```

```
config/vehicle_car1.yaml
  launch_acc:  0.8      출발 직후 가속 상한 [㎧²]
  launch_time: 3.0      이 시간에 걸쳐 평소 값으로 선형 복귀 [s]
```

**속도 문제가 아니라 위치추정 문제 때문에 있습니다.**

출발 순간엔 휠 스핀이 납니다 (정지마찰 깨기 + 구동계 백래시 + 하중 뒤쏠림). 바퀴는 돌았는데 차는 덜 나간 만큼 **오도메트리가 거리를 뻥튀기**합니다 — 실측으로 **출발 5초에 허위거리 87 cm** (odom 17.35 m vs AMCL 16.35 m).

문제는 위치추정이 그걸 못 되돌린다는 겁니다. 복도에서는 **종방향 관측이 거의 안 됩니다** (앞뒤 잔차 0.2~1.0 cm vs 좌우 6~7 cm). 벽이 평행하면 앞뒤로 얼마나 갔는지 스캔으로 알 수가 없습니다. 그래서 차가 "실제보다 더 갔다" 고 믿고 **첫 코너를 일찍 돕니다.**

비용은 **1랩 +1.16초** 이고 **누적되지 않습니다** (3랩 +1.20초). 출발 때만 내는 비용입니다. 감속에는 절대 안 걸립니다.

```bash
cd ~/yax && ./tune.sh limits.launch_acc && ./tune.sh limits.launch_time   # 현재 값
cd ~/yax && ./tune.sh limits.launch_acc=0        # 끔
cd ~/yax && ./tune.sh limits.launch_acc=1.2      # 덜 조이기
cd ~/yax && ./tune.sh limits.launch_time=2.0     # 더 빨리 복귀
```

> 끄면 첫 코너를 일찍 도는 증상이 돌아옵니다. **위치추정이 안정된 걸 확인한 뒤에** 손대세요.

### 랩 내내 느리면 — `cap` 을 확인하세요

```bash
cd ~/yax && ./tune.sh cap
```

`cap` 은 **전 모드 공통 천장**이고 **이후 실행에도 남습니다.** 저속 시험하려고 `cap=0.5` 를 걸어둔 걸 잊으면 계속 느립니다. `0` 이 꺼짐입니다.

괜찮으면 천장을 풀고 속도를 올립니다:
```bash
cd ~/yax && ./tune.sh cap=0 && ./tune.sh q1=0.8
```

`q1` 은 마찰 예산 비율입니다 (`v ∝ √η` — 0.4→0.8 이면 약 1.4 배).

콘을 놓고 회피까지 보려면:
```bash
cd ~/yax && ./race.sh stop && ./race.sh race
```

### ⚠ Cartographer 는 길을 잃으면 스스로 복구하지 않습니다

AMCL 의 `recovery_alpha` 에 해당하는 장치가 없습니다. 어긋나면 **사람이 다시 심어줘야** 합니다. 맵에 없는 물체에 AMCL 보다 **더 민감합니다** — 가설 다양성이 없어 틀린 정합으로 그대로 끌려갑니다.

이상하면 되돌리면 즉시 원래대로 갑니다 (AMCL 을 지우지 않았습니다):
```bash
cd ~/yax && ./tune.sh localization.mode=amcl
```

---

## 7. 마무리

```bash
cd ~/yax && ./race.sh stop
```

**세션이 끝나면 꼭 실행**하세요.

---

## 명령 한 장 요약

```bash
# 터미널 A — 맵핑 (켜 둔 채로)
cd ~/yax && ./race.sh mapping-carto-joy

# 터미널 B — 저장 → 정제 → 라인 → 설정
cd ~/yax && ./race.sh savemap-carto 1109
cd ~/yax && ./race.sh clean 1109                  # 자동 정제
# 자동이 엉뚱하게 고르면:  DISPLAY=:1 python3 src/race_stack/maps/clean_map.py --map 1109
cd ~/yax && ./race.sh line 1109_clean             # 라인 생성
# 마진을 한계까지 밀어보려면:
#   cd ~/yax/src/race_stack && RACE_WS=$HOME/f1tenth_ws python3 -m race_stack.gen_raceline_aggro --map 1109_clean
cd ~/yax && ./tune.sh localization.mode=cartographer && ./tune.sh map_name=1109_clean && ./tune.sh waypoints=1109_clean_map_wp
cd ~/yax && ./race.sh start_pos --map 1109_clean --wp-file 1109_clean_map_wp --gui

# 터미널 A — 확인 후 주행
cd ~/yax && ./race.sh stop && ./race.sh box
cd ~/yax && ./tune.sh cap=0.5 && ./race.sh q1

# 노트북 (로컬)
cd ~/yax && ./rviz.sh car1

# 노트북 — 제어판 (선택. 터미널을 하나로 줄인다)
ssh -N -L 8765:127.0.0.1:8765 jetson@<젯슨IP> &   # 터널
cd ~/yax && ./panel.sh                                  # 창
#   젯슨 쪽에서 먼저:  cd ~/yax && ./panel.sh --server
```

---

## 잘 안 될 때

| 증상 | 원인 / 확인 |
|---|---|
| `Already exists: ... Choose a new map name` | 그 이름이 이미 있다 → 새 이름을 쓰거나 **install 복사본까지** 삭제 (0절) |
| `Cartographer 주행용 .pbstream 없음` | `savemap` 을 썼다 → `savemap-carto` 로 **재측량** |
| `savemap-carto` 가 멈춰 있음 | 맵핑(터미널 A)이 켜져 있는가. 끄면 맵도 사라진다 |
| `경로 생성 중단: 주행 가능한 영역이 없습니다` | 정제가 트랙까지 지웠다 → `_clean_preview.png` 확인, 반경 낮춰 다시 |
| `이 라인은 못 돈다` | 트랙이 차 회전반경보다 좁다. `--apex-extra 0` 부터, 그래도 안 되면 트랙을 넓혀야 한다 |
| 라인이 맵과 어긋남 | `waypoints/<이름>.png` 확인. `map_name` 과 `waypoints` 가 같은 맵인가 |
| RViz 에 아무것도 안 뜸 | 노트북·젯슨이 같은 와이파이인가. `[dds]` 줄에 `폴백` 이 떴는가 |
| 차가 엉뚱한 데 있다고 나옴 | 스캔과 맵이 겹치는가 → `2D Pose Estimate` → `savepose` |
| 갑자기 조향이 안 먹음 | 스택 켠 뒤 **와이파이를 바꿨는가** → `./race.sh stop` 후 재시작 |
| 토픽이 도는지 모르겠음 | `cd ~/yax && ./race.sh check` |
| VS Code 가 저장을 거부함 | 파일이 디스크에서 바뀌었다 → `File > Revert File`. `tune.sh` 를 쓰면 안 생긴다 |
| 제어판 제목이 `연결 끊김` | 터널이 없거나 서버가 안 떠 있다. 젯슨에서 `ss -tnp \| grep 8765` 로 ESTAB 확인 |
| 제어판 띄웠는데 젯슨에 창이 없음 | **정상.** 서버는 헤드리스다. 창은 노트북에만 뜬다 |

> ### ★ 스택을 켠 뒤에는 와이파이를 바꾸지 마세요
> CycloneDDS 가 **기동 시점의 IP 로 인터페이스를 고정**하고 그 뒤 재바인딩을
> 못 합니다. 노드는 살아 있는데 서로 못 듣는 상태가 되고, 조향이 갑자기 먹통이
> 되는 식이라 하드웨어 고장처럼 보입니다.
> 바꿨다면 반드시 `./race.sh stop` 후 다시 띄우세요.

---
---

# 참고 — 동작 원리

여기부터는 **무슨 일이 일어나는지** 입니다. 절차만 필요하면 위까지만 보면 됩니다.

## 주행 중 Cartographer 가 하는 일

`localization.mode: cartographer` 는 처음 저장한 `.pbstream` 을 **고정된 원본 맵**으로 불러옵니다. 달리는 동안 새 submap 을 **메모리에 계속 추가**하고, 그 관측으로 차량 위치도 계속 추정합니다. 원본 trajectory 는 frozen 상태라 최적화로 변형되지 않습니다.

종료할 때 추가분을 저장하지 않으므로 **다음 실행은 다시 처음 저장한 맵에서 시작**합니다. 원본 파일을 다시 읽는 시점은 주행 스택을 종료한 뒤 재실행할 때입니다. 랩 완료로 차가 멈췄더라도 프로세스가 살아 있으면 그 세션의 추가 맵은 남아 있습니다.

긴 주행에서는 메모리와 최적화 부하가 늘 수 있습니다. 추가 지도가 생겨도 저장한 웨이포인트가 자동으로 재생성되지는 않습니다.

⚠ 주행 중 추가분을 원본에 반영하려고 `savemap-carto` 를 실행하지 마세요. 그 명령은 **처음 원본 지도를 만드는 매핑 전용**이며 trajectory 를 종료합니다.

## 위치가 제어기까지 전달되는 경로

`cartographer_localizer` 가 저장 맵의 trajectory 를 조회한 뒤 `init_x/y/yaw` 를 적용해 새 trajectory 를 시작합니다. `/tracked_pose` 를 `PoseWithCovarianceStamped` 로 바꾸어 기존 **`/amcl_pose` 구독자에게 전달**합니다.

`/amcl_pose` 는 세 방식이 공유하는 **호환 토픽 이름**입니다. `cartographer`·`slam_toolbox` 모드에서 AMCL 노드는 실행되지 않습니다. 이 구조 덕분에 `/amcl_pose` 를 구독하는 9개 노드(`stanley_controller`·`pure_pursuit`·`frenet_planner`·`map_controller`·`mission_manager`·`localization_monitor`·`auto_localize`·`waypoint_logger`·`sim_bridge`)를 한 줄도 고치지 않았습니다.

⚠ 공분산은 **호환용 고정값**이며 Cartographer 가 계산한 신뢰도가 아닙니다. 최근 scan-matched 데이터가 없으면 위치 전달을 멈춥니다.

## 입력 및 TF

| | |
|---|---|
| `/scan` | LaserScan, `laser` 프레임 |
| `/odom_imu` | 차량별 부호 보정 + IMU yaw 반영된 Odometry |
| `imu_odom` | `odom → base_link` 발행 |
| 차량 bringup | `base_link → laser` 고정 TF |
| Cartographer | `map → odom` 발행. 원시 VESC IMU 는 직접 입력하지 않음 |
| Lua | `src/race_stack/config/cartographer_2d.lua`, 지도 해상도 0.05 m |

## 위치추정 세 방식 비교

`localization.mode` 로 고릅니다. 세 방식 모두 남아 있어 같은 트랙에서 번갈아 돌려 비교할 수 있습니다.

| | `amcl` | `slam_toolbox` | `cartographer` |
|---|---|---|---|
| 원리 | 파티클 필터 | 스캔 매칭 | 스캔 매칭 + pose graph |
| 맵 파일 | `.pgm` | `.pgm` + `.posegraph` | `.pgm` + `.pbstream` |
| 저장 명령 | `savemap` | `savemap` | **`savemap-carto`** |
| 정지 시 pose | **안 나옴** (2.5 cm 이동 필요) | 나옴 | 나옴 |
| 길을 잃으면 | 복구 시도함 | 복구 못 함 | 복구 못 함 |
| CPU | 가벼움 | 중간 | 무거움 |

맵 종류가 다르니 **방식을 바꾸면 그 방식용 맵이 있어야 합니다.** 없으면 launch 가 그 사실을 알려주며 멈춥니다.

기존 SLAM 매핑은 `./race.sh mapping-joy` / `./race.sh savemap <이름>` 입니다. **매핑 엔진은 한 번에 하나만** 실행합니다.

비교는 `localization_monitor` 의 잔차로 하세요 — 그게 필터 교체 효과를 재는 유일한 기준선입니다.

## 재빌드 및 하드웨어 없는 검증

`f1tenth_stack/launch/bringup_launch.py` 에 `enable_joystick` 인자가 추가돼 있으므로, 다른 장비로 옮길 때는 **이 파일 변경도 함께 전달**해야 합니다. 기본값은 `true` 입니다.

```bash
cd ~/f1tenth_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select f1tenth_stack race_stack --symlink-install && source install/setup.bash
```

센서 없이 노드만 띄워 구성만 확인하려면:

```bash
cd ~/f1tenth_ws && source install/setup.bash && ros2 launch race_stack mapping.launch.py backend:=cartographer start_vehicle:=false enable_joystick:=false
```

Cartographer 와 지도 발행 노드만 뜹니다. 외부에서 센서·TF 를 제공해야 맵이 생깁니다. rosbag 재생 시에는 `use_sim_time:=true` 와 bag 의 `/clock` 이 필요합니다.

### 이미 확인된 것 / 아직 아닌 것

구성 검증에서 확인: 별도 ROS 도메인의 가상 LaserScan/Odometry/TF 로 `/map`·`map → odom` 발행, 세 가지 저장 파일 생성 및 덮어쓰기 거부. 주행 모드에서는 원본 frozen 유지, 새 submap 생성, 연속 위치 출력, 센서 단절 시 오래된 위치 전달 중단, 정지 상태 초기 위치 재설정, 재실행 시 추가 trajectory 소멸, 원본 pbstream 해시 불변.

**아직 확인 안 된 것: 실차 맵 품질, 실제 센서 주기, CPU/메모리 부하.** 현장 주행으로 확인해야 합니다.
