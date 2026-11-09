# Cartographer로 2D 맵 따기

Jetson의 `~/yax` 또는 `~/f1tenth_ws`에서 실행한다.

## 매핑

차량·라이다·조이스틱을 연결하고 차량을 시작 지점에 둔다.

```bash
./race.sh mapping-carto-joy
```

IMU 바이어스 초기화 동안 처음 몇 초는 정지한다. 기존 mapping-joy와 같은
조이스틱 설정과 `speed.mapping_max` 제한으로 천천히 트랙을 돌아 시작 지점으로
되돌아온다. 기본 bringup의 joy/joy_teleop은 이 모드에서 처음부터 비활성화된다.
기존 RViz에서 Fixed Frame을 `map`, 지도 토픽을 `/map`으로 보면 된다.

## 저장

차량을 완전히 세우고, 매핑 터미널을 켜 둔 채 다른 터미널에서 실행한다.
맵 이름은 영문/숫자로 시작하고 영문, 숫자, `_`, `-`를 사용한다.

```bash
cd ~/yax
./race.sh savemap-carto carto_test_01
```

이 명령은 현재 trajectory를 종료하므로 이후 주행은 지도에 추가되지 않는다.
`.pbstream`을 저장한 뒤 그 상태에서 `.pgm/.yaml`을 생성한다. 파일은
`~/f1tenth_ws/src/race_stack/maps/`에 저장되며 같은 이름이 있으면 덮어쓰지 않는다.
완료 메시지가 나온 뒤 매핑 터미널에서 Ctrl+C 한다.

변환에 실패해도 저장된 `.pbstream`은 남는다. 차량 노드 없이 재변환할 수 있다:

```bash
source /opt/ros/humble/setup.bash
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename=$HOME/f1tenth_ws/src/race_stack/maps/carto_test_01.pbstream \
  -map_filestem=$HOME/f1tenth_ws/src/race_stack/maps/carto_test_01 \
  -resolution=0.05
```

직접 재변환 명령은 같은 이름의 PGM/YAML을 덮어쓸 수 있다.

## 주행 중에도 Cartographer 사용

`localization.mode: cartographer`는 처음 저장한 `.pbstream`을 **고정된 원본 맵**으로
불러온다. 달리는 동안 새 submap을 메모리에 계속 추가하고, 그 관측으로 차량 위치도
계속 추정한다. 원본 trajectory는 frozen 상태라 최적화로 변형되지 않는다.
종료할 때 추가분을 저장하지 않으므로 다음 실행은 다시 처음 저장한 맵에서 시작한다.
원본 파일을 다시 읽는 시점은 주행 스택을 종료한 뒤 재실행할 때다.
랩 완료로 차가 멈췄더라도 프로세스가 살아 있으면 그 세션의 추가 맵은 남아 있다.

새 맵을 처음 준비할 때(경로/초기 위치 GUI는 화면 필요):

```bash
cd ~/yax
./race.sh waypoints-kappa carto_test_01
./race.sh start_pos --map carto_test_01 --wp-file carto_test_01_wp_kappa --gui
./tune.sh localization.mode=cartographer
```

`start_pos`에서 위치→방향을 클릭하고 `w`로 저장한다. 차량도 그 위치와 방향으로 놓는다.
이후 정지 확인과 저속 시험:

```bash
./race.sh box
# 다른 창에서 ./rviz.sh — 맵 벽과 /scan이 맞는지 확인
# 필요하면 정지한 채 RViz 2D Pose Estimate → ./race.sh savepose
# box 종료 후:
./tune.sh cap=0.5
./race.sh q1 stanley
```

`cap=0.5`는 이후 실행에도 남는 0.5 m/s 속도 상한이다.
AMCL로 전환하는 단계는 필요 없다. `/amcl_pose`는 기존 제어기 호환용 토픽 이름이며,
이 모드에서 실제 위치를 계산하는 노드는 Cartographer다.

`map_name`과 같은 이름의 `.pbstream`을 찾고, `_clean`/`_clean_loc` 지도는 원본 이름도
찾는다. 별도 경로를 쓰려면 `localization.cartographer_map`에 `.pbstream` 경로를 지정한다.
PGM/YAML, 초기 위치, 웨이포인트는 반드시 그 pbstream에서 생성한 지도와 같은 좌표계여야 한다.
기존 slam_toolbox의 PGM/YAML만으로는 Cartographer 주행을 시작할 수 없다.

이 모드는 새로운 submap을 세션 동안 유지하므로 긴 주행에서는 메모리와 최적화 부하가
늘 수 있다. 추가 지도가 생겨도 저장한 웨이포인트가 자동으로 재생성되는 것은 아니다.
주행 중 추가분을 원본에 반영하려고 `savemap-carto`를 실행하지 않는다. 해당 명령은
처음 원본 지도를 만드는 매핑 전용이며 trajectory를 종료한다.

## 주행 위치 연결

`cartographer_localizer`가 저장 맵의 trajectory를 조회한 뒤 `init_x/y/yaw`를 적용해
새 trajectory를 시작한다. `/tracked_pose`를 `PoseWithCovarianceStamped`로 바꾸어
기존 `/amcl_pose` 구독자에게 전달한다. 공분산은 호환용 고정값이며 Cartographer가
계산한 신뢰도가 아니다. 최근 scan-matched 데이터가 없으면 위치 전달을 멈춘다.

`/initialpose`는 최신 odom이 있고 차량이 정지했을 때만 받는다. 현재 trajectory를
종료하고 지정한 위치에서 새 trajectory를 시작한다. 주행 중 재설정 요청은 거부한다.
원본 맵은 이때도 고정 상태다.

## 입력 및 TF

- `/scan`: LaserScan, `laser` 프레임.
- `/odom_imu`: 차량별 부호 보정 및 IMU yaw가 반영된 Odometry.
- `imu_odom`: `odom → base_link` 발행.
- 차량 bringup: `base_link → laser` 고정 TF 발행.
- Cartographer: `map → odom` 발행. 원시 VESC IMU는 직접 입력하지 않는다.
- Lua: `src/race_stack/config/cartographer_2d.lua`, 지도 해상도 0.05 m.

기존 SLAM 매핑은 `./race.sh mapping-joy` / `./race.sh savemap <이름>`이다.
매핑 엔진은 한 번에 하나만 실행한다.

## 재빌드 및 하드웨어 없는 검증

`f1tenth_stack/launch/bringup_launch.py`에 `enable_joystick` 인자가 추가돼 있으므로
다른 장비로 옮길 때는 이 파일 변경도 함께 전달해야 한다. 기본값은 true다.

```bash
cd ~/f1tenth_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select f1tenth_stack race_stack --symlink-install
source install/setup.bash
ros2 launch race_stack mapping.launch.py backend:=cartographer \
  start_vehicle:=false enable_joystick:=false
```

마지막 명령은 Cartographer와 지도 발행 노드만 띄운다. 외부에서 센서·TF를
제공해야 맵이 생긴다. rosbag 재생 시에는 `use_sim_time:=true`와 bag의 `/clock`도 필요하다.

구성 검증에서는 별도 ROS 도메인의 가상 LaserScan/Odometry/TF로 `/map`,
`map → odom`, 세 가지 저장 파일 생성 및 덮어쓰기 거부를 확인했다.
주행 모드는 원본 frozen 상태 유지, 새 submap 생성, 연속 위치 출력, 센서 단절 시
오래된 위치 전달 중단, 정지 상태 초기 위치 재설정, 재실행 시 추가 trajectory 소멸,
원본 pbstream 파일 해시 불변까지 확인했다.
실차 맵 품질, 실제 센서 주기, CPU/메모리 부하는 현장 주행으로 확인해야 한다.
