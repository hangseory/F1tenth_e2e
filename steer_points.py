#!/usr/bin/env python3
"""steer_points.py — 서보 몇 점을 잡아두고 **실제 바퀴 각도**를 받아 적는다.

왜 필요한가 (2026-11-11)
────────────────────────
모델이 말하는 조향과 차가 실제로 내는 조향이 다르다.

    모델: δ = (servo − offset) / (−gain)      풀락 27.5°
    주행 로그에서 원 맞춤:                      실제 21.2°

6.3° 차이의 원인이 셋인데 처방이 전부 다르다:

  ① 게인 과대      공중에서도 모델보다 덜 꺾인다. **전 구간이 비례해서** 작다
                   → vehicle_car*.yaml 의 steering_angle_to_servo_gain 수정
  ② 기구 스토퍼    중간은 모델대로인데 **끝에서만** 안 꺾인다
                   → servo_min / servo_max 를 실제 스토퍼 값으로
  ③ 하중 시 밀림   공중에선 모델대로 나오는데 주행 중만 작다 (서보세이버·유격)
                   → 기계 수리. geometry.steer_eff_max 를 실효값으로 유지

중간점과 끝점을 **같이** 재야 셋이 갈린다. 끝만 재면 ①과 ②를 구분할 수 없다.

쓰는 법
───────
    ./race.sh stop                       # ★ 스택을 먼저 끈다 (아래 참고)
    source /opt/ros/humble/setup.bash && source ~/f1tenth_ws/install/setup.bash
    source ~/yax/config/dds_env.sh
    ros2 run vesc_driver vesc_driver_node --ros-args \
        --params-file ~/yax/config/vesc.yaml &     # 드라이버만
    python3 ~/yax/steer_points.py

각 점에서 서보를 20Hz 로 **계속** 쏘며 잡아둔다. 측정한 각도를 입력하면 다음 점.
그냥 Enter = 건너뜀, q = 중단. 끝나면 판정과 권장값을 띄운다.

★★ 반드시 앞바퀴를 공중에 띄우고 할 것 (박스 위). ★★
   모터에는 아무 명령도 안 보내므로 차는 나가지 않는다.
   ★ 스택이 떠 있으면 안 된다 — /commands/servo/position 은 ackermann_to_vesc 와
     throttle_interpolator 도 쓴다. 두 발행자가 싸우면 측정이 통째로 무의미하다.
   ★ 끝점(스토퍼)에 오래 물려두면 서보가 발열한다. 각 점은 짧게 끝낼 것.

각도 읽는 법
────────────
30cm 자를 타이어 옆면에 평평하게 대고, 차 길이방향 기준 어긋남을 잰다.
    각도 = atan(어긋남 / 자 길이)
       0° → 0.0cm   14.6° → 7.8cm   21.2° → 11.6cm   27.5° → 15.6cm
휴대폰 각도계 앱을 타이어 옆면에 대도 된다 (중립에서 영점을 잡고 상대각).
"""
import math
import os
import sys

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Float64

TOPIC = '/commands/servo/position'
REPO = os.path.dirname(os.path.realpath(__file__))


def load_cal():
    """config/vesc.yaml (apply_vehicle.py 생성물) 에서 캘리브레이션을 읽는다."""
    p = os.path.join(REPO, 'config', 'vesc.yaml')
    d = (yaml.safe_load(open(p)) or {})['/**']['ros__parameters']
    return (float(d['steering_angle_to_servo_gain']),
            float(d['steering_angle_to_servo_offset']),
            float(d.get('servo_min', 0.12)), float(d.get('servo_max', 0.85)))


def model_deg(servo, gain, offset):
    """모델이 말하는 바퀴 각도 [deg]. + = 좌 (gain 이 음수인 규약)."""
    return math.degrees((servo - offset) / (-gain))


class Hold(Node):
    def __init__(self):
        super().__init__('steer_points')
        self.pub = self.create_publisher(Float64, TOPIC, 10)
        self.val = None
        # ★ 20Hz 로 계속 쏜다. 단발로는 안 움직인다 — DDS discovery 가 늦고,
        #   VESC 는 명령이 끊기면 마지막 값을 유지할 뿐 새 값으로 가지 않는다.
        self.create_timer(0.05, self._tick)

    def _tick(self):
        if self.val is not None:
            self.pub.publish(Float64(data=float(self.val)))

    def settle(self, val, secs=1.5):
        self.val = val
        end = self.get_clock().now().nanoseconds * 1e-9 + secs
        while self.get_clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    gain, offset, smin, smax = load_cal()
    print(f'\nconfig/vesc.yaml:  gain {gain}  offset {offset}  '
          f'servo {smin}~{smax}')
    print(f'모델 풀락: 좌 {model_deg(smin, gain, offset):+.1f}°  '
          f'우 {model_deg(smax, gain, offset):+.1f}°')
    print('\n★ 앞바퀴가 공중에 떠 있는지 확인하세요. 모터는 안 돕니다.')
    print('  각 점에서 측정한 각도를 입력 (부호 무시, 크기만). '
          'Enter=건너뜀, q=중단\n')

    mid_l = offset + (smin - offset) * 0.5
    mid_r = offset + (smax - offset) * 0.5
    pts = [('중립',     offset), ('좌 중간', mid_l), ('좌 풀락', smin),
           ('우 중간',  mid_r),  ('우 풀락', smax)]

    rclpy.init()
    node = Hold()
    out = []
    try:
        for name, sv in pts:
            node.settle(sv)
            want = abs(model_deg(sv, gain, offset))
            ruler = 30.0 * math.sin(math.radians(want))
            print(f'── {name}:  서보 {sv:.3f}   모델 {want:4.1f}° '
                  f'(30cm 자에서 {ruler:4.1f}cm)')
            try:
                s = input('   실측 각도[deg] > ').strip()
            except (EOFError, KeyboardInterrupt):
                break
            if s.lower() == 'q':
                break
            if not s:
                continue
            try:
                out.append((name, sv, want, abs(float(s))))
            except ValueError:
                print('   숫자가 아닙니다 — 건너뜁니다')
    finally:
        print('\n중립 복귀…')
        node.settle(offset, 1.0)
        node.val = None
        node.destroy_node()
        rclpy.shutdown()

    if not out:
        print('측정값이 없습니다.')
        return

    print('\n' + '=' * 58)
    print(f"{'점':9s}{'서보':>7s}{'모델':>8s}{'실측':>8s}{'실측/모델':>11s}")
    for name, sv, want, got in out:
        r = got / want if want > 1e-6 else float('nan')
        print(f'{name:9s}{sv:7.3f}{want:7.1f}°{got:7.1f}°'
              + ('      —' if want <= 1e-6 else f'{r:10.2f}'))

    mids = [(w, g) for n, _, w, g in out if '중간' in n and w > 1e-6]
    ends = [(w, g) for n, _, w, g in out if '풀락' in n and w > 1e-6]
    if not (mids and ends):
        print('\n중간점과 끝점을 둘 다 재야 원인이 갈립니다.')
        return
    rm = sum(g / w for w, g in mids) / len(mids)
    re_ = sum(g / w for w, g in ends) / len(ends)
    print('=' * 58)
    print(f'중간 실측/모델 {rm:.2f}   끝 실측/모델 {re_:.2f}\n')

    if rm < 0.95 and abs(rm - re_) < 0.07:
        print('→ ① 게인 과대. 전 구간이 같은 비율로 덜 꺾입니다.')
        print(f'   steering_angle_to_servo_gain {gain}  →  {gain * rm:.4f}')
        print('   config/vehicle_car*.yaml 의 vesc 절을 고치세요 '
              '(max_steer 는 자동 재계산됩니다).')
    elif rm >= 0.95 and re_ < 0.93:
        print('→ ② 기구 스토퍼가 먼저 닿습니다. 중간은 정상입니다.')
        print('   steer_limit_check.py 로 **바퀴가 더 안 움직이는 서보 값**을')
        print('   찾아 servo_min / servo_max 에 넣으세요.')
    elif rm >= 0.95 and re_ >= 0.93:
        print('→ ③ 공중에서는 모델대로 나옵니다. 주행 중에만 작다면')
        print('   하중 문제입니다 (서보세이버·링키지 유격). 기계를 보세요.')
        print('   geometry.steer_eff_max 는 실효값으로 그대로 두면 됩니다.')
    else:
        print('→ 중간이 끝보다 더 나쁩니다 — 예상 밖입니다.')
        print('   영점(중립)이 틀어졌는지, 자가 바퀴 옆면에 평평했는지 '
              '먼저 확인하고 다시 재세요.')


if __name__ == '__main__':
    main()
