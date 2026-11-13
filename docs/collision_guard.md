# 예측 충돌 가드

기존 주행 설정을 보존하고, 필요한 실행에서만 사용하는 Ackermann 정지·감속 필터다. 경로 생성과 장애물 회피는 기존 컨트롤러가 계속 담당한다. `race-safe`나 `--guard`로 켜면 지도 기반의 짧은 후진 복구도 기본으로 사용한다. 후진 복구만 끄려면 `--no-recover`를 함께 지정한다.

## 켜기와 끄기

```bash
# 기존 동작 그대로: 충돌 가드를 추가하지 않는다.
./race.sh race map
./race.sh race map --no-guard

# 같은 race 모드에 충돌 가드와 짧은 후진 복구를 추가한다.
./race.sh race-safe map --bag
./race.sh race map --guard --bag

# 아래 별칭도 같은 동작이다.
./race.sh race-recover map --bag
./race.sh race map --recover --bag

# 복구만 끄고 충돌 가드는 유지한다.
./race.sh race-safe map --no-recover
./race.sh race map --guard --no-recover

# q1/q2/q3/quali/quali-mini/buffer/race2/10laprace 에도 적용 가능하다.
./race.sh q1 map --guard

# 기존 bench 속도 0 조건을 가드의 최종 명령에도 적용한다.
./race.sh box race --guard
```

`race-safe`는 `race --guard`, `race-recover`는 `race --recover`의 별칭이며 두 별칭의 동작은 같다. `--guard`와 `--recover`는 가드와 복구를 함께 켜고, `--no-recover`는 복구만 끈다. `--no-guard`는 둘 다 끈다. 명시한 플래그는 별칭보다 우선하며 지정 순서대로 적용된다. 예를 들어 `--guard --no-guard`는 둘 다 끄고, `--no-guard --guard`는 둘 다 켠다. `--guard --no-recover`는 가드만 남기며, 반대로 `--no-recover --guard`는 마지막 `--guard`가 복구도 다시 켠다.

선택은 해당 실행에만 적용되며 `race_script.yaml`을 고치지 않는다. 일반 `race`를 옵션 없이 실행하면 가드와 자동 후진 모두 추가하지 않는다. mapping/joy 같은 별도 실행 명령에 가드·복구 플래그를 쓰면 지원하지 않는다고 종료한다. 주행 중 가드를 해제하는 서비스는 제공하지 않는다.

가드 모드에서는 기존 `safety_node`를 함께 실행하지 않는다. 원래 bringup의 `joy_teleop`도 실행하지 않으며, `/joy` 드라이버와 기존 `race_stack/joystick_teleop`을 사용한다. LB 수동 전환 등 기존 조작은 유지되지만, **수동 명령에도 가드가 적용되고 수동 후진은 계속 차단된다.** 후진을 허용하는 경로는 별도로 켠 복구 상태기에서 검사한 짧은 자동 후진뿐이다.

## 명령 경로

```text
controller /drive ──┐
                    ackermann_mux → /safety/raw_drive
joystick /teleop ───┘                       ↓
                                     collision_guard
                                           ↓
                                    /ackermann_cmd
                                           ↓
                                ackermann_to_vesc_node
                                           ↓
                        /safety/motor_request, /safety/servo_request
                                           ↓
                              collision_motor_watchdog
                                           ↓
                   /commands/motor/speed, /commands/servo/position
                                           ↓
                                      vesc_driver
```

실제 설치된 mux의 출력 이름은 `ackermann_cmd`이다. 가드 bringup은 이 출력을 `/safety/raw_drive`로 바꾸며, 원래 bringup의 `ackermann_cmd_out` remap에 의존하지 않는다. mux의 수동 우선순위는 그대로 두되, 선택된 명령은 모두 최종 가드를 거친다. 컨트롤러의 최소속도 제한보다 뒤에서 감속하므로 1.4m/s 최소속도가 가드의 정지·감속을 다시 올리지 않는다.

가드는 스캔을 TF로 `base_link`에 옮기고 차체가 지나갈 영역을 검사한다. 라이다의 기존 정적 TF는 `base_link` 앞 0.27m, 위 0.11m다. 기본 전방 AEB는 절대 위치 AMCL을 입력으로 요구하지 않는다. 측정된 움직임과 요청 경로, 반응 시간과 제동거리를 반영해 속도를 제한하며, 장애물 해제 여유가 일정 시간 유지된 뒤 가속 제한을 걸어 다시 출발한다. 지도 기반 후진 복구를 켠 경우에는 아래의 지도·위치 조건이 추가된다.

## 지도 기반 후진 복구

`race-safe`, `--guard`, `--recover` 실행에서는 다음 순서로 동작한다. `--no-recover`로 끈 경우 이 절차를 실행하지 않는다.

1. 자율 주행이 허용되고 전진 요청이 있는 상태에서, 장애물 때문에 정지한 상태가 1초 지속되는지 확인한다. 정지 속도 문턱은 odom 기준 0.05m/s다. 출발 대기나 센서·통신 장애를 복구 대상으로 삼지 않는다.
2. `/map`과 지도상의 차량 자세로 후진 조향 후보를 검사한다. 차체가 지나갈 영역이 맵의 장애물·미확인 영역·맵 밖과 겹치면 그 후보를 거부한다. 후진 뒤 전진할 공간이 개선되는 후보를 선택한다.
3. 최대 0.25m/s로 최대 0.35m 후진한다. 3초 제한도 함께 적용하며, 1초 안에 0.03m 이상 진행하지 못하면 더 밀지 않고 정지한다.
4. 후진을 마치면 0.3초 정지한 뒤 전진 요청을 최대 0.25m/s로 제한한다. 기존 전방 가드의 정지 유지·해제 검사와 가속 제한을 그대로 통과해야 출발한다. 실제로 0.25m 전진한 것이 확인될 때까지 이 상한을 유지하고, 이후 컨트롤러의 정상 요청으로 복귀한다. 센서·위치 조건이 나빠지거나 수동 전환·미션 종료·비상정지가 발생하면 복구를 취소한다. bench에서는 속도 0을 유지한다.

자동 복구 시도는 최대 2회이며, 이후 정상 전진을 1m 진행해야 시도 예산을 다시 얻는다. 적합한 후보가 없으면 정지를 유지한다. 복구가 `/race/enabled`를 발행하지 않으므로 출발·완주·수동 전환의 권한은 기존 mission manager에 남는다.

현재 차량은 센서리스 구동의 낮은 속도 문제 때문에 일반 주행 최소속도가 1.4m/s로 설정돼 있다. 복구용 0.25m/s에서 실제로 움직이는지는 아직 실차 검증하지 않았다. 진행이 없으면 1초 뒤 정지하며, 움직이게 하려고 속도를 자동으로 높이지 않는다. `race-safe`의 기본 동작에 복구를 포함한 것은 실행 설정 변경이며 실차 복구 검증 완료를 뜻하지 않는다.

지도는 `/map`의 `OccupancyGrid`, 위치는 `/amcl_pose` 이름으로 제공되는 위치추정 결과를 사용한다. 현재 Cartographer 구성도 이 이름으로 pose를 공급한다. 정적 맵에는 수신 나이 제한을 적용하지 않는다. 위치는 최대 10초 이내의 추정값을 최신 odom으로 전파하며, 위치·각도 점프와 지도-스캔 끝점 정합도 함께 확인한다. Cartographer의 호환용 공분산만으로 위치가 맞다고 판단하지 않는다.

**지도상 후방이 비어 있어도, 맵을 만든 뒤 뒤쪽으로 들어온 사람·상대 차량·물체를 현재 라이다로 확인할 수는 없다.** 후방 중앙 사각지대는 이 복구 기능으로 없어지지 않는다. 지도와 위치가 맞는다는 조건에서 제한된 후진을 선택하는 기능이며, 후방 센서를 갖춘 충돌 방지와 같은 보장은 제공하지 않는다.

## 설정

[collision_guard.yaml](../src/race_stack/config/collision_guard.yaml)에 독립 설정이 있다. `wheelbase`와 `max_steer`는 일반 `race.sh` 실행 시 차량 overlay가 적용된 제어기 값으로 덮어쓴다.

| 설정 | 초기값 | 의미 |
| --- | ---: | --- |
| `front`, `rear`, `half_width` | 0.50, 0.15, 0.17m | `base_link` 기준 차체 전방·후방·반폭 |
| `margin` | 0.06m | 차체 주변 추가 여유 |
| `reaction_time` | 0.45s | 센서·제어·구동 지연을 포함할 반응 여유 |
| `brake_decel` | 2.0m/s² | 제동거리 계산용 감속도 |
| `prediction_dt`, `max_horizon` | 0.02s, 4.0s | 궤적 검사 간격과 최대 예측 시간 |
| `max_speed` | 6.0m/s | 가드가 허용하는 최대 요청 속도 |
| `release_margin`, `release_hold` | 0.15m, 0.5s | 정지 해제에 필요한 추가 여유와 유지시간 |
| `restart_accel` | 0.8m/s² | 출발 시 출력 가속 제한 |
| `command_timeout`, `scan_timeout`, `odom_timeout` | 각각 0.2s | 입력 최신성 제한 |
| `motor_ready_timeout` | 0.25s | 출력 watchdog 준비 heartbeat 최신성 제한 |
| `max_unknown_gap_deg` | 5° | 전방 미확인 구간 허용 한계 |
| `required_forward_fov_deg` | 180° | 전방에 필요한 스캔 범위 |

복구 설정은 같은 파일의 `recovery_` 접두사 항목이다. `recovery_enabled`의 파일 값은 단독 노드의 기본값 false이며, 실행 옵션이 최종 값을 지정한다. `race.launch.py guard:=true`는 `recover`를 생략하면 true로 정하고, `guarded_bringup.launch.py`도 `recover:=true`가 기본이다. 가드만 시험할 때는 `recover:=false`를 명시한다.

| 복구 설정 | 초기값 | 의미 |
| --- | ---: | --- |
| `recovery_blocked_wait` | 1.0s | 장애물 정지가 지속돼야 하는 시간 |
| `recovery_reverse_speed`, `recovery_reverse_distance` | 0.25m/s, 0.35m | 후진 속도·거리 상한 |
| `recovery_max_reverse_time`, `recovery_stop_hold` | 3.0s, 0.3s | 후진 시간 제한·후진 뒤 정지 시간 |
| `recovery_reentry_speed`, `recovery_reentry_distance` | 0.25m/s, 0.25m | 복귀 전진 상한·이 상한을 유지할 실제 전진 거리 |
| `recovery_stopped_speed` | 0.05m/s | 정지 판정용 odom 속도 문턱 |
| `recovery_min_progress`, `recovery_progress_timeout` | 0.03m, 1.0s | 후진 진행 여부 확인 |
| `recovery_max_attempts`, `recovery_reset_forward_distance` | 2회, 1.0m | 재시도 제한·시도 예산 초기화 조건 |
| `recovery_sample_step`, `recovery_forward_probe` | 0.02m, 1.0m | 후보 검사 거리 간격·후진 뒤 전진 공간 탐색 거리 |
| `recovery_min_escape_gain` | 0.05m | 후보에 필요한 전진 여유 개선량 |
| `recovery_occupancy_threshold` | 65 | 맵의 장애물 점유도 문턱; 미확인 셀도 차단 |
| `recovery_pose_timeout` | 10.0s | odom으로 전파할 원래 위치추정값의 나이 제한 |
| `recovery_enabled_timeout`, `recovery_teleop_timeout` | 1.2s, 0.3s | 자율 허용 신호 최신성·최근 수동 명령 확인 구간 |

위치 표준편차와 위치·각도 점프 문턱은 각각 0.25m/0.25rad다. 지도-스캔 검사는 기본 0.15m 허용 오차, 정합 비율 0.25, 최소 20개 점을 사용한다. 이 값들도 현재 차량·지도에 맞는지 검증할 설정값이며, 공분산 또는 지도 일치 하나만으로 후방의 안전을 보장하지 않는다.

**차체 치수와 제동 수치는 실측 완료값이 아닌 초기 설정이다.** 범퍼·바퀴를 포함한 차체 치수, 지연, 실제 노면의 제동거리를 확인해야 한다. `reaction_time + max_speed / brake_decel`보다 짧은 예측 시간으로 제동 궤적이 잘리지 않게 설정한다. 초기값은 6m/s에서 3.45초의 반응·제동 시간을 4초 안에 포함한다. 속도 상한 6m/s가 그 속도의 실차 검증을 의미하지는 않는다.

센서가 끊기거나 입력이 오래됐거나 좌표 변환을 얻지 못하면 정지한다. 넓은 미검출 구간은 멀리 비어 있는 공간으로 대체하지 않는다. 이 기능은 전방 라이다 기반의 예측 AEB이며, 회전 중 차체 뒤·옆이 지나가는 사각 영역과 짧은 미검출 구간까지 완전하게 확인하지 못한다. 유리·가림·스캔 사이의 장애물 움직임도 모두 관측할 수 없으므로 충돌 방지를 보장하는 장치는 아니다.

## 정지와 종료

가드가 멈추거나 죽어 VESC 변환기의 요청이 끊기면, 별도 프로세스 `collision_motor_watchdog`가 0.2초 입력 제한에 따라 모터 0을 반복 발행한다. watchdog은 시작·입력 장애 시 출력을 잠그고, 신선한 모터 0 요청이 `rearm_hold=0.2s` 동안 유지돼야 다시 준비 상태로 바뀐다. 매 주기 발행하는 `/safety/motor_ready`가 false이거나 0.25초 동안 갱신되지 않으면 가드도 정지를 유지한다. 따라서 변환기가 잠시 멈췄다가 돌아와도 남아 있던 높은 속도 요청으로 바로 재출발하지 않는다. watchdog 설정의 `command_timeout=0.2`, `rearm_hold=0.2`, `control_rate=50Hz`는 `guarded_bringup.launch.py`에서 지정한다.

guard/mux/converter/watchdog 중 하나가 종료되면 launch가 `collision_stop`을 먼저 실행하고, 정지 전송이 끝난 뒤 나머지 스택을 종료한다. `/safety/emergency_stop`의 `true`는 재시작 전까지 유지하는 비상정지다.

`race.sh --guard`는 주행 프로세스 그룹을 따로 만들고 Ctrl+C/TERM 때 `collision_stop`을 먼저 실행한 다음 그룹을 종료한다. 따라서 실제 주행은 위 `race.sh` 명령을 사용한다. 가드 모드에서는 이 종료 순서를 위해 statusline 파이프를 생략하고 원본 로그를 출력한다. 기존 bag 저장과 로그 정리 동작은 유지한다. 시스템 전체 정지·전원 상실·모든 안전 프로세스의 동시 강제종료까지 소프트웨어 프로세스만으로 해결하지는 못한다.

## 로그와 하드웨어 없는 검증

`--bag`에는 기존 센서/컨트롤러 로그와 함께 `/safety/raw_drive`, `/ackermann_cmd`, `/safety/collision_guard/status`, `/safety/motor_ready`, `/safety/motor_watchdog/status`, `/safety/emergency_stop`, `/commands/motor/speed`, `/tf`, `/tf_static`을 기록한다. 지도 기반 복구 재현을 위해 `/map`, `/race/manual`도 포함하며, `/amcl_pose`와 `/race/enabled`는 기존 기록 항목이다. 원래 요청과 필터 출력, watchdog 준비 상태와 정지 이유를 같은 시각에 비교할 수 있다. 가드 상태 JSON의 `recovery` 객체에 복구 상태·이유·시도 정보를 함께 남긴다.

하드웨어 없는 그래프 검증은 **실차와 다른 ROS domain**에서 실행한다. 예를 들어 다음 domain은 기존 노드가 사용하지 않는지 확인하고 선택한다.

```bash
ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 \
  ros2 launch race_stack guarded_bringup.launch.py hardware:=false recover:=false
```

이 경로는 mux·guard·converter·watchdog만 띄운다. 모터·서보 최종 출력은 `/safety/test/motor_speed`, `/safety/test/servo_position`으로 바꾸며 라이다·조이스틱·VESC 드라이버·실차 odom·정적 TF를 띄우지 않는다. 테스트가 `/scan`, `/odom_imu`, `/drive`, `base_link→laser` TF를 별도로 공급해야 한다. 같은 domain에 기존 실차 스택을 켜놓으면 `/ackermann_cmd` 같은 중간 토픽을 공유할 수 있으므로 domain 분리는 생략하지 않는다.

위 예시는 `recover:=false`로 전방 AEB만 검사한다. `recover:=true`로 바꾸면 복구 경로도 켜진다. 이 경우 테스트가 `/map`, `/amcl_pose`, `/race/enabled`, `/race/manual`도 공급해야 한다. 실차 지도나 위치를 대신 추정해 넣는 경로는 없다.

새 노드와 launch/config 파일은 `race_stack` 빌드 후 사용한다. `hardware:=false` 검증은 센서·제어 배관과 고장 시 정지 동작을 검사하며, 실차 제동 성능을 검증하지 않는다.

저장소 루트에서 같은 검사를 재현할 수 있다.

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/f1tenth_ws/install/setup.bash
python3 -m pytest -q \
  src/race_stack/test/test_collision_guard_core.py \
  src/race_stack/test/test_recovery_core.py \
  src/race_stack/test/test_recovery_context.py
python3 src/race_stack/test/test_collision_guard_ros.py
python3 src/race_stack/test/test_map_recovery_ros.py
```

pytest는 기하·정지·재출발 정책과 지도 기반 복구·입력 상태 검사를 실행한다. 전방 AEB ROS 검증은 domain 92, `hardware:=false`, `recover:=false`로 합성 센서 입력과 프로세스 중단·복구를 검사하며, 결과는 `/tmp/yax_collision_guard_test/result.json`에 기록한다.

지도 기반 후진 ROS 검증은 domain 93, `hardware:=false`, `recover:=true`로 격리하며 결과를 `/tmp/yax_map_recovery_test/result.json`에 기록한다. 두 스크립트 모두 실제 차량 센서나 구동 명령을 사용하지 않는다. 이 검사들의 통과는 실차에서 0.25m/s 후진이 가능한지, 제동거리와 후방 공간이 충분한지에 대한 검증을 대신하지 않는다.
