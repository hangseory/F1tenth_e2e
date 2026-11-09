# Cartographer ver. 매뉴얼

맵 따기 → 정제 → 웨이포인트 → 주행. **명령은 전부 한 줄입니다. 그대로 복사해 붙이세요.**

| | 어디서 | 무엇 |
|---|---|---|
| **터미널 A** | 젯슨 (SSH) | 맵핑 / 주행 — 켜 둔 채로 |
| **터미널 B** | 젯슨 (SSH) | 저장 / 정제 / 라인 / 설정 |
| **노트북** | 노트북 로컬 | RViz 화면 |

```bash
ssh jetson@192.168.1.9
```
비밀번호 `jetsonjetson`. IP 가 바뀌었으면 `config/vehicle_car1.yaml` 의 `network.subnet`·`network.peers` 를 고쳐야 합니다 (젯슨에서 `ip -4 addr` 로 확인).

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
scp jetson@192.168.1.9:~/yax/src/race_stack/maps/1109_clean_preview.png .
```

정제가 하는 일은 **"트랙만 자유, 나머지 전부 벽"** 입니다. 미지 영역을 벽으로 메워 **닫힌 경계**를 만드는 것이고, 그게 다음 단계 라인 생성(QP)의 전제입니다.

⚠ `_clean` 은 **라인 생성·RViz 용**입니다. 위치추정은 `.pbstream` 을 쓰므로, `map_name` 에 `_clean` 을 적어도 원본 pbstream 을 자동으로 찾습니다.

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

### 저속 시험 — 터미널 A

```bash
cd ~/yax && ./tune.sh cap=0.5 && ./race.sh q1
```

`cap=0.5` 는 **이후 실행에도 남는** 0.5 m/s 전 모드 천장입니다.

기동 로그 확인:
```
[race.launch] 위치추정: cartographer
```

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
cd ~/yax && ./race.sh clean 1109
cd ~/yax && ./race.sh line 1109_clean
cd ~/yax && ./tune.sh localization.mode=cartographer && ./tune.sh map_name=1109_clean && ./tune.sh waypoints=1109_clean_map_wp
cd ~/yax && ./race.sh start_pos --map 1109_clean --wp-file 1109_clean_map_wp --gui

# 터미널 A — 확인 후 주행
cd ~/yax && ./race.sh stop && ./race.sh box
cd ~/yax && ./tune.sh cap=0.5 && ./race.sh q1

# 노트북 (로컬)
cd ~/yax && ./rviz.sh car1
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
