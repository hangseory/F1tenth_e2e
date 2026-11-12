#!/usr/bin/env bash
# ============================================================
#  servo.sh — 서보를 한 값에 잡아둔다 (벤치 측정용)
#
#    ./servo.sh 0.485     그 값으로 보내고 계속 유지
#    ./servo.sh           지금 값 유지 중지 (서보는 마지막 위치에 머문다)
#    ./servo.sh --list    지금 떠 있는 유지 프로세스 확인
#
#  ★★ 앞바퀴를 공중에 띄우고 쓸 것 (박스 위). 모터에는 아무 명령도 안 간다.
#
#  왜 "계속" 쏘나 — 단발 publish 로는 안 움직인다. DDS discovery 가 늦고,
#  VESC 는 명령이 끊기면 마지막 값을 유지할 뿐 새 값으로 가지 않는다. 20Hz 로 쏜다.
#
#  ★★★ 2026-11-11 사고 — 유지 프로세스가 **두 개** 떠서 각각 다른 값을 20Hz 로
#      쏘고 있었다. 바퀴가 두 값 사이에서 떨렸고 측정값이 전부 쓰레기가 됐다
#      (그걸 모르고 "휴대폰 나침반이 모터 자석에 왜곡된다" 고 잘못 짚었다).
#      그래서 이 스크립트는 **시작 전에 기존 유지를 전부 죽이고, 하나만
#      떠 있는지 확인한 뒤에야** 값을 보고한다. 둘 이상이면 중단한다.
#      같은 실수를 2026-11-10 에도 했다 (0.485 와 0.553 이 싸움).
#
#  사전 준비 (한 번):
#      ./race.sh stop                      # 스택이 떠 있으면 안 된다 —
#                                          # ackermann_to_vesc·throttle_interpolator
#                                          # 도 같은 토픽을 쓴다
#      source /opt/ros/humble/setup.bash
#      source ~/f1tenth_ws/install/setup.bash
#      ros2 run vesc_driver vesc_driver_node --ros-args \
#          --params-file ~/yax/config/vesc.yaml &
# ============================================================
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOLDER="$DIR/.servo_hold.py"

kill_all () {
    local pids; pids=$(pgrep -f "[.]servo_hold\.py" 2>/dev/null || true)
    [ -z "$pids" ] && return 0
    kill $pids 2>/dev/null || true
    sleep 1
    pids=$(pgrep -f "[.]servo_hold\.py" 2>/dev/null || true)
    [ -n "$pids" ] && { kill -9 $pids 2>/dev/null || true; sleep 1; }
    return 0
}

case "${1:-}" in
    --list)
        echo "유지 프로세스:"
        pgrep -af "[.]servo_hold\.py" || echo "  (없음)"
        exit 0 ;;
    '')
        kill_all
        echo "[servo] 유지 중지. 서보는 마지막 위치에 머뭅니다."
        exit 0 ;;
esac

VAL="$1"
# 범위 가드 — servo_min/max 밖은 기구 스토퍼에 물려 서보가 발열한다
python3 - "$VAL" "$DIR" <<'PY'
import sys, yaml, math, os
v = float(sys.argv[1]); d = sys.argv[2]
p = yaml.safe_load(open(os.path.join(d, 'config', 'vesc.yaml')))['/**']['ros__parameters']
lo, hi = float(p.get('servo_min', 0.12)), float(p.get('servo_max', 0.85))
g, o = float(p['steering_angle_to_servo_gain']), float(p['steering_angle_to_servo_offset'])
if not (0.02 <= v <= 0.98):
    sys.exit(f'[servo] ✗ {v} 는 안전 범위(0.02~0.98) 밖입니다')
deg = math.degrees((v - o) / g)
mark = '' if lo <= v <= hi else f'   ⚠ 소프트 리밋({lo}~{hi}) 밖 — 스토퍼에 물릴 수 있음, 오래 두지 말 것'
print(f'[servo] {v}  →  모델상 바퀴 {deg:+.2f}°  (30cm 자에서 '
      f'{30*math.sin(math.radians(abs(deg))):.1f}cm){mark}')
PY

kill_all

cat > "$HOLDER" <<'PY'
import sys, time, rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
val = float(sys.argv[1])
rclpy.init()
n = Node('servo_hold')
pub = n.create_publisher(Float64, '/commands/servo/position', 10)
n.create_timer(0.05, lambda: pub.publish(Float64(data=val)))
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.1)
PY

[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f "$HOME/f1tenth_ws/install/setup.bash" ] && source "$HOME/f1tenth_ws/install/setup.bash"
export RACE_SCRIPT="$DIR/race_script.yaml"
[ -f "$DIR/config/dds_env.sh" ] && source "$DIR/config/dds_env.sh" >/dev/null 2>&1

nohup python3 "$HOLDER" "$VAL" >/dev/null 2>&1 &
sleep 2

# ★ 하나만 떠 있는지 반드시 확인한다 (위 사고 주석 참고)
N=$(pgrep -cf "[.]servo_hold\.py" 2>/dev/null || echo 0)
if [ "$N" != "1" ]; then
    echo "[servo] ✗ 유지 프로세스가 ${N}개입니다 — 측정하지 마세요."
    pgrep -af "[.]servo_hold\.py" || true
    echo "        ./servo.sh      로 전부 끄고 다시 시도하세요."
    exit 1
fi
echo "[servo] 유지 중 (프로세스 1개 확인). 다음 값으로 넘어가려면 ./servo.sh <값>"
