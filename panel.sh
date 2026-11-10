#!/usr/bin/env bash
# ============================================================
#  panel.sh — 원격 제어판
#
#    젯슨에서:   ./panel.sh --server      서버 (ROS 환경 필요)
#    노트북에서: ./panel.sh               제어판 (ROS 불필요)
#
#  노트북은 먼저 터널을 열어둔다:
#    ssh -N -L 8765:127.0.0.1:8765 jetson@<젯슨IP> &
#
#  설계: docs/원격_제어판_설계.md
# ============================================================
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "${1:-}" = "--server" ]; then
    # 서버는 ROS 가 있어야 E-STOP(/teleop)을 쓴다. 없으면 경고만 하고 계속 돈다.
    [ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
    [ -f "$HOME/f1tenth_ws/install/setup.bash" ] && source "$HOME/f1tenth_ws/install/setup.bash"
    export RACE_SCRIPT="$DIR/race_script.yaml"
    [ -f "$DIR/config/dds_env.sh" ] && source "$DIR/config/dds_env.sh"
    exec python3 "$DIR/panel_server.py" "${@:2}"
fi

# 클라이언트 — ROS 를 일부러 안 부른다 (DDS 설정에 영향받지 않게)
exec python3 "$DIR/panel_client.py" "$@"
