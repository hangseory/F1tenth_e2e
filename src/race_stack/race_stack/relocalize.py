#!/usr/bin/env python3
"""relocalize.py — 스캔을 맵에 맞춰 **지금 위치를 다시 찾는다**.

왜 있나
───────
충돌하거나 차를 손으로 옮기면 위치추정이 깨진다. 그런데 우리 스택에는 **복구
수단이 없다**:

    amcl          recovery_alpha 가 0 이라 스스로 못 빠져나온다
    slam_toolbox  복구 기능 없음
    cartographer  복구 기능 없음 — /initialpose 를 사람이 넣어줘야 한다

auto_localize 는 **현재 추정 주변 ±0.5m** 만 훑는다. 이미 대충 맞을 때 다듬는
용도라, 완전히 잃어버린 상태에서는 소용이 없다. 이 도구가 그 자리를 메운다.

★ 왜 "전역" 이 아니라 반경 제한인가
  충돌해도 차는 몇 미터 안에 있다. 순간이동하지 않는다. 그런데 맵 전체를
  훑으면:
    · 느리다 (c1109 기준 후보 150만 개)
    · **비슷한 구간끼리 경쟁한다** — 복도가 반복되면 엉뚱한 곳이 1등이 된다
  반경 5m 로 줄이면 후보가 1/40 이 되고, 모호성이 거의 사라진다.

★ 자동으로 적용하지 않는다
  Cartographer 는 /initialpose 를 받으면 **trajectory 를 죽이고 새로 시작**
  한다. 2026-11-09 에 프리셋을 16번 눌러 /initialpose 가 연달아 들어가자
  잔차가 7cm → 71cm 로 무너지고 80초에 20번 재시작했다. 틀린 자세를 밀어넣는
  것이 가만히 두는 것보다 나쁘다.
  그래서 기본 동작은 **후보를 찍어 보여주는 것까지**다. 적용은 --apply 로
  사람이 확인한 뒤에만 한다.

★ 1등·2등 점수차를 같이 낸다
  비슷한 구간이 많으면 1등 점수만으로는 믿을 수 없다. 2등과 가까우면
  "애매하다" 고 말해야 한다.

사용
────
    # 후보만 찾아서 보여준다 (아무것도 바꾸지 않는다)
    ros2 run race_stack relocalize --ros-args -p map:=c1109_clean

    # 사람이 확인한 자세를 실제로 적용 (/initialpose 발행)
    ros2 run race_stack relocalize --ros-args -p apply:=true \\
        -p x:=1.23 -p y:=-0.45 -p yaw:=1.57

  주행 스택이 떠 있어야 한다 (/scan 이 필요하다). **차는 멈춰 있어야 한다** —
  스캔 한 장으로 판단하므로 움직이면 흐려진다.
"""
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from PIL import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def _maps_dir():
    """RACE_WS 가 있으면 그쪽 src 를 본다 (maps/ 는 빌드 스냅샷이면 낡는다)."""
    ws = os.environ.get('RACE_WS')
    if ws:
        p = os.path.join(os.path.expanduser(ws), 'src', 'race_stack', 'maps')
        if os.path.isdir(p):
            return p
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(here, 'maps')


class MapField:
    """맵에서 '자유공간의 각 점이 가장 가까운 벽까지 얼마나 먼가' 를 만든다.

    auto_localize 와 **같은 규약**이다 (map_server: 밝을수록 자유공간).
    점수는 거리에 대한 가우시안 — 스캔 점이 벽에 가까울수록 높다.
    """

    def __init__(self, yaml_path, sigma=0.12):
        cfg = yaml.safe_load(open(yaml_path))
        self.res = float(cfg['resolution'])
        self.ox, self.oy = float(cfg['origin'][0]), float(cfg['origin'][1])
        occ_th = float(cfg.get('occupied_thresh', 0.65))
        img = np.asarray(Image.open(
            os.path.join(os.path.dirname(yaml_path), cfg['image'])).convert('L'))
        self.H, self.W = img.shape
        occupied = ((255 - img.astype(np.float32)) / 255.0) > occ_th
        free = (~occupied).astype(np.uint8)
        # 벽까지 거리 [m]. 벽 자체는 0.
        dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.res
        self.score = np.exp(-0.5 * (dist / sigma) ** 2).astype(np.float32)
        self.free = free.astype(bool)

    def to_px(self, x, y):
        """map 좌표 → 픽셀. y 는 이미지가 위아래 뒤집혀 있어 H 에서 뺀다."""
        px = (np.asarray(x) - self.ox) / self.res
        py = self.H - (np.asarray(y) - self.oy) / self.res
        return px, py

    def lookup(self, x, y):
        px, py = self.to_px(x, y)
        ix = np.clip(px.astype(np.int32), 0, self.W - 1)
        iy = np.clip(py.astype(np.int32), 0, self.H - 1)
        return self.score[iy, ix]

    def is_free(self, x, y):
        px, py = self.to_px(x, y)
        ix = np.clip(px.astype(np.int32), 0, self.W - 1)
        iy = np.clip(py.astype(np.int32), 0, self.H - 1)
        return self.free[iy, ix]

    def free_poses(self, cx, cy, radius, step):
        """중심 반경 안의 **자유공간** 격자점만 후보로 낸다.
        벽 속은 차가 있을 수 없으므로 애초에 빼는 게 빠르고 정확하다."""
        n = int(radius / step)
        g = np.arange(-n, n + 1) * step
        gx, gy = np.meshgrid(cx + g, cy + g)
        gx, gy = gx.ravel(), gy.ravel()
        keep = (gx - cx) ** 2 + (gy - cy) ** 2 <= radius ** 2
        gx, gy = gx[keep], gy[keep]
        px, py = self.to_px(gx, gy)
        ok = ((px >= 0) & (px < self.W) & (py >= 0) & (py < self.H))
        gx, gy, px, py = gx[ok], gy[ok], px[ok], py[ok]
        ok2 = self.free[py.astype(np.int32), px.astype(np.int32)]
        return gx[ok2], gy[ok2]


def search(field, pts, cx, cy, radius, xy_step, yaw_step_deg, yaw_center=None,
           yaw_span_deg=180.0):
    """반경 안에서 (x, y, yaw) 를 훑어 점수가 높은 순으로 돌려준다.

    pts: 라이다 점을 **base_link 기준**으로 옮긴 (N,2) 배열.
    반환: [(score, x, y, yaw), ...] 점수 내림차순.
    """
    gx, gy = field.free_poses(cx, cy, radius, xy_step)
    if len(gx) == 0:
        return []
    if yaw_center is None:
        yaws = np.arange(-180.0, 180.0, yaw_step_deg)
    else:
        yaws = np.arange(-yaw_span_deg, yaw_span_deg + 1e-9, yaw_step_deg) \
            + math.degrees(yaw_center)
    best = []
    for yd in yaws:
        c, s = math.cos(math.radians(yd)), math.sin(math.radians(yd))
        # 점들을 한 번만 회전시키고, 이동은 브로드캐스트로 끝낸다
        rx = pts[:, 0] * c - pts[:, 1] * s
        ry = pts[:, 0] * s + pts[:, 1] * c
        ex = gx[:, None] + rx[None, :]      # 빔 끝점
        ey = gy[:, None] + ry[None, :]
        # ★ 끝점 점수만 쓰면 안 된다 — 벽 **안쪽**은 거리 0 이라 점수가 최대다.
        #   그러면 빔을 벽 속으로 밀어 넣는 자세가 1등이 된다 (실제로 그랬다:
        #   합성 스캔 5회 전부 1.5~4.7m 어긋난 곳을 골랐다).
        #   광선이 **자유공간을 지나왔는가** 를 같이 본다. 중간점이 벽이면
        #   그 빔은 물리적으로 불가능하므로 점수를 깎는다.
        mx = gx[:, None] + rx[None, :] * 0.5
        my = gy[:, None] + ry[None, :] * 0.5
        passes = field.is_free(mx, my)
        sc = (field.lookup(ex, ey) * passes).mean(axis=1)
        i = int(np.argmax(sc))
        best.append((float(sc[i]), float(gx[i]), float(gy[i]),
                     math.radians(((yd + 180) % 360) - 180)))
    best.sort(key=lambda t: -t[0])
    return best


def thin_scan(msg, max_beams=160, rmin=0.1, rmax=10.0):
    """스캔을 솎아 base_link 기준 점으로. 빔을 다 쓰면 느리고 이득도 적다."""
    r = np.asarray(msg.ranges, dtype=np.float32)
    a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
    ok = np.isfinite(r) & (r > rmin) & (r < rmax)
    r, a = r[ok], a[ok]
    if len(r) > max_beams:
        idx = np.linspace(0, len(r) - 1, max_beams).astype(int)
        r, a = r[idx], a[idx]
    return np.stack([r * np.cos(a), r * np.sin(a)], axis=1)


class Relocalize(Node):
    def __init__(self):
        super().__init__('relocalize')
        d = self.declare_parameter
        d('map', '')
        d('scan_topic', '/scan')
        d('radius', 5.0)          # 충돌해도 몇 미터 안에 있다
        d('xy_step', 0.10)
        d('yaw_step_deg', 3.0)
        d('laser_offset_x', 0.27)  # base_link → laser (auto_localize 와 같은 값)
        d('laser_offset_y', 0.0)
        d('apply', False)
        d('x', 0.0)
        d('y', 0.0)
        d('yaw', 0.0)
        self.pub = self.create_publisher(PoseWithCovarianceStamped,
                                         '/initialpose', 10)
        self.scan = None
        self.pose = None
        self.create_subscription(LaserScan, self.p('scan_topic'),
                                 self._scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._pose, 10)

    def p(self, k):
        return self.get_parameter(k).value

    def _scan(self, m):
        self.scan = m

    def _pose(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y ** 2 + q.z ** 2))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def publish(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = float(x)
        m.pose.pose.position.y = float(y)
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05 ** 2
        cov[35] = math.radians(5.0) ** 2
        m.pose.covariance = cov
        # 한 번만 보내면 탐색 직후라 구독자가 아직 못 붙었을 수 있다
        for _ in range(5):
            self.pub.publish(m)
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    n = Relocalize()

    if n.p('apply'):
        x, y, yaw = float(n.p('x')), float(n.p('y')), float(n.p('yaw'))
        n.publish(x, y, yaw)
        print(f'적용: ({x:+.3f}, {y:+.3f}) {math.degrees(yaw):+.1f}° → /initialpose')
        print(json.dumps({'applied': True, 'x': x, 'y': y, 'yaw': yaw}))
        rclpy.shutdown()
        return

    map_name = str(n.p('map')).strip()
    if not map_name:
        print('✗ map 파라미터가 필요합니다'); rclpy.shutdown(); sys.exit(2)
    ypath = os.path.join(_maps_dir(), map_name + '.yaml')
    if not os.path.exists(ypath):
        print(f'✗ 맵이 없습니다: {ypath}'); rclpy.shutdown(); sys.exit(2)

    print(f'맵 {map_name} 로딩...', flush=True)
    field = MapField(ypath)

    # 스캔과 마지막 추정 위치를 기다린다
    t0 = time.time()
    while time.time() - t0 < 6.0 and (n.scan is None or n.pose is None):
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.scan is None:
        print('✗ /scan 이 안 옵니다 — 주행 스택이 떠 있어야 합니다')
        rclpy.shutdown(); sys.exit(3)
    if n.pose is None:
        print('✗ /amcl_pose 가 안 옵니다 — 탐색 중심을 정할 수 없습니다')
        rclpy.shutdown(); sys.exit(3)

    pts = thin_scan(n.scan)
    pts = pts + np.array([float(n.p('laser_offset_x')),
                          float(n.p('laser_offset_y'))], dtype=np.float32)
    cx, cy, cyaw = n.pose
    r = float(n.p('radius'))
    print(f'탐색: 중심 ({cx:+.2f}, {cy:+.2f}) 반경 {r:.1f} m, 빔 {len(pts)}개', flush=True)

    t1 = time.time()
    res = search(field, pts, cx, cy, r, float(n.p('xy_step')),
                 float(n.p('yaw_step_deg')))
    dt = time.time() - t1
    if not res:
        print('✗ 반경 안에 자유공간이 없습니다 — 맵이나 중심 위치를 확인하세요')
        rclpy.shutdown(); sys.exit(4)

    # 1등과 **충분히 떨어진** 2등을 찾는다. 같은 봉우리의 옆자리는 2등이 아니다.
    b = res[0]
    second = next((t for t in res[1:]
                   if math.hypot(t[1] - b[1], t[2] - b[2]) > 0.5
                   or abs(((t[3] - b[3] + math.pi) % (2 * math.pi)) - math.pi) > 0.35),
                  None)
    margin = (b[0] - second[0]) / b[0] if second and b[0] > 0 else 1.0
    cur = field.lookup(np.array([cx]) + pts[:, 0] * math.cos(cyaw) - pts[:, 1] * math.sin(cyaw),
                       np.array([cy]) + pts[:, 0] * math.sin(cyaw) + pts[:, 1] * math.cos(cyaw)).mean()

    print(f'\n{dt:.2f}초, 후보 {len(res)}개')
    print(f'  현재 추정  ({cx:+.2f}, {cy:+.2f}) {math.degrees(cyaw):+6.1f}°  점수 {cur:.3f}')
    print(f'  1등        ({b[1]:+.2f}, {b[2]:+.2f}) {math.degrees(b[3]):+6.1f}°  점수 {b[0]:.3f}')
    if second:
        print(f'  2등        ({second[1]:+.2f}, {second[2]:+.2f}) '
              f'{math.degrees(second[3]):+6.1f}°  점수 {second[0]:.3f}')
    print(f'  1-2등 차이 {margin*100:.0f}%', end='')
    if margin < 0.10:
        print('  ⚠ 애매합니다 — 비슷한 구간이 여럿입니다. 눈으로 확인하세요')
    else:
        print('  ✓')
    if b[0] <= cur * 1.02:
        print('  ※ 현재 추정과 점수가 비슷합니다 — 굳이 바꿀 필요가 없을 수 있습니다')

    print(json.dumps({'applied': False, 'took': round(dt, 2),
                      'cur': {'x': cx, 'y': cy, 'yaw': cyaw, 'score': float(cur)},
                      'best': {'x': b[1], 'y': b[2], 'yaw': b[3], 'score': b[0]},
                      'second': ({'x': second[1], 'y': second[2], 'yaw': second[3],
                                  'score': second[0]} if second else None),
                      'margin': margin}))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
