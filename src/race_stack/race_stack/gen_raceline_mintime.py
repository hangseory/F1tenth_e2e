#!/usr/bin/env python3
"""
gen_raceline_mintime.py — 최소 시간(min-time) 레이싱라인 생성기 + 라인 비교 평가

왜 또 하나인가
──────────────
기존 생성기(gen_raceline / _tumftm / gen_waypoints_kappa)는 전부
**"경로 형태를 먼저 정하고 → 속도를 나중에 붙이는"** 2단계다.
최소 곡률은 랩타임의 대리 목표일 뿐이라, 긴 직선 앞 코너를 일찍 넣는 것처럼
**곡률이 조금 늘어도 시간이 주는** 형태를 못 찾는다
(Christ et al. 2021, Heilmeier 2019 비교 — 최소 곡률이 최소 시간보다 수 % 느림).
게다가 생성기마다 자기 마찰값(A_LAT_MAX=3.7 등)을 따로 들고 있어서, 정작
실제로 속도를 정하는 map_controller(speed_profile.solve_profile)와 어긋났다
(docs/마지막_한일.md 6절: 1.5배 차이).

이 도구는 경로와 속도를 **한 번에** 푸는 최적제어(OCP)다:

    min  Σ dt  +  λ_κ' Σ (dκ/ds)²
    상태  n(횡오프셋) ξ(진행각 오차) u=v²       곡선좌표 (중심선 기준)
    입력  κ(차의 경로 곡률)  a⁺/a⁻(가속/제동)
    제약  (a⁺/a_acc)² + (a⁻/a_brk)² + (u·κ/a_g)² ≤ 1     ← 컨트롤러와 같은 마찰원
          |κ| ≤ tan(steer_util·max_steer)/L                ← 추종조향 여유
          |dκ/ds| ≤ dkappa_max                               ← L1 추종 가능성
          −w_right ≤ n ≤ w_left                              ← 벽 (safety 반영)

a_g = grip_util·μ·pac_d·g, a_acc/a_brk/v_min 은 **map_controller 가 실제로 쓰는
값**(race_script.yaml + config/.vehicle_overlay.yaml)을 그대로 읽는다.
그래서 여기서 나온 예상 랩타임과 컨트롤러 기동 로그의 "예상 랩" 이 같은 모델이다.

참고:
  Christ, Wischnewski, Heilmeier, Lohmann, "Time-optimal trajectory planning for
    a race car considering variable tyre-road friction coefficients", VSD 2021
    (TUMFTM/global_racetrajectory_optimization 의 opt_mintime_traj)
  Heilmeier et al., VSD 2019 (최소 곡률 QP — gen_raceline_tumftm.py)
  ForzaETH race_stack (같은 TUM 코드 + 실측 마찰로 F1TENTH 적용)
  arXiv 2602.15642 — 컨트롤러 추종오차로 구간별 한계를 갱신하는 적응 루프
    (→ --kappa-scale-csv 로 구간별 감속을 수동/로그 기반으로 넣을 수 있다)

사용법:
  # 평가만: 이 맵의 기존 라인들을 같은 물리로 비교 (예상 랩·추종조향·벽 여유)
  python3 -m race_stack.gen_raceline_mintime --map c11_clean --eval

  # 생성: 기존 라인 하나를 방향·출발점 기준으로 삼는다
  python3 -m race_stack.gen_raceline_mintime --map c11_clean \\
      --like c11_clean_wp_kappa_pre --out c11_clean_wp_mintime

의존: casadi (pip install --user --no-deps casadi — numpy 1.26.4 를 건드리지 않게)
"""

import argparse
import csv
import glob
import math
import os
import re
import sys

import numpy as np
import yaml
from scipy.interpolate import splprep, splev, CubicSpline
from scipy.spatial import cKDTree

from race_stack.gen_raceline import MAPS_DIR, WP_DIR, extract_reftrack, save_overlay
from race_stack.speed_profile import solve_profile, GRAVITY

_REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                     '..', '..', '..'))
if os.environ.get('RACE_REPO'):
    _REPO = os.path.expanduser(os.environ['RACE_REPO'])


# ══════════════════════════════════════════════════════════════════════════════
# 차량·컨트롤러 파라미터 — map_controller 와 같은 곳에서 같은 우선순위로 읽는다
# ══════════════════════════════════════════════════════════════════════════════
def _deep_merge(a, b):
    out = dict(a)
    for k, v in (b or {}).items():
        out[k] = _deep_merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


def overlay_for(car):
    """apply_vehicle.py 와 같은 규칙으로 overlay 를 메모리에서 만든다 (파일 안 씀).
    노트북에서 '다른 차' 기준으로 라인을 만들 때 .car / vesc.yaml 을 안 건드리려고."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        '_apply_vehicle', os.path.join(_REPO, 'config', 'apply_vehicle.py'))
    av = importlib.util.module_from_spec(spec); spec.loader.exec_module(av)
    v = yaml.safe_load(open(os.path.join(_REPO, 'config', f'vehicle_{car}.yaml')))
    ovl = {}
    for src, dst in av.OVERLAY_MAP:
        try:
            av.put(ovl, dst, av.dig(v, src))
        except KeyError:
            pass
    _, _, lim, _ = av.steer_limits(v)
    av.put(ovl, 'pure_pursuit.max_steer', round(lim, 4))
    return ovl


def load_params(mode='race', car=None):
    rs = yaml.safe_load(open(os.path.join(_REPO, 'race_script.yaml'))) or {}
    if car:
        ov = overlay_for(car)
    else:
        ov_path = os.path.join(_REPO, 'config', '.vehicle_overlay.yaml')
        ov = yaml.safe_load(open(ov_path)) if os.path.exists(ov_path) else {}
        car = '?'
        try:
            car = open(os.path.join(_REPO, '.car')).read().strip() + ' (.vehicle_overlay)'
        except OSError:
            pass
    c = _deep_merge(rs, ov or {})
    m, lim, pp, sp = c.get('map', {}), c.get('limits', {}), c.get('pure_pursuit', {}), c.get('speed', {})
    gu = lim.get('grip_util', 0.6)
    if isinstance(gu, dict):
        gu = gu.get(mode, gu.get('race', 0.6))
    p = dict(
        car=car,
        mu_eff=float(m.get('mu', 0.6)) * float(m.get('pac_d', 1.15)),
        grip_util=float(gu),
        lf=float(m.get('lf', 0.17)), lr=float(m.get('lr', 0.15)),
        a_lon_acc=float(lim.get('a_lon_acc', 0.0)),
        a_lon_brk=float(lim.get('a_lon_brk', 0.0)),
        v_min=float(sp.get('min_speed', 0.4)),
        hard_cap=float(m.get('hard_cap', 0.0) or 0.0),
        wheelbase=float(pp.get('wheelbase', 0.33)),
        max_steer=float(pp.get('max_steer', 0.36)),
        l1_min=float(m.get('l1_min', 1.2)), l1_k=float(m.get('l1_k', 0.75)),
    )
    a_g = p['grip_util'] * p['mu_eff'] * GRAVITY
    L = p['lf'] + p['lr']
    p['a_g'] = a_g
    p['a_acc'] = min(p['a_lon_acc'] if p['a_lon_acc'] > 0 else a_g * p['lf'] / L, a_g)
    p['a_brk'] = min(p['a_lon_brk'] if p['a_lon_brk'] > 0 else a_g, a_g)
    return p


# ══════════════════════════════════════════════════════════════════════════════
# 라인 평가 — 컨트롤러와 같은 곡률 계산(주기 쿠빅 스플라인) + solve_profile
# ══════════════════════════════════════════════════════════════════════════════
def read_csv_xy(path):
    pts = []
    for row in csv.reader(open(path)):
        if not row or row[0].strip().startswith('#'):
            continue
        try:
            pts.append((float(row[0]), float(row[1]), float(row[2]) if len(row) > 2 else -1.0))
        except ValueError:
            continue
    return np.array(pts)


def controller_curvature(pts):
    """map_controller._compute_curvature 와 같은 식 (부호 포함 반환)."""
    ds = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    keep = np.r_[True, ds > 1e-6]
    pts = pts[keep]
    ds = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(ds)])
    s_ext = np.append(s, s[-1] + float(np.linalg.norm(pts[0] - pts[-1])))
    cx = CubicSpline(s_ext, np.append(pts[:, 0], pts[0, 0]), bc_type='periodic')
    cy = CubicSpline(s_ext, np.append(pts[:, 1], pts[0, 1]), bc_type='periodic')
    dx, dy, ddx, ddy = cx(s, 1), cy(s, 1), cx(s, 2), cy(s, 2)
    return pts, (dx * ddy - dy * ddx) / np.maximum((dx ** 2 + dy ** 2) ** 1.5, 1e-6)


def wall_tree(map_name, safety=0.0):
    """벽 점 KD-tree — 라인의 실제 벽 여유를 잰다 (맵의 점유 픽셀)."""
    from PIL import Image
    cfg = yaml.safe_load(open(os.path.join(MAPS_DIR, map_name + '.yaml')))
    img_path = next(p for p in (os.path.join(MAPS_DIR, map_name + e) for e in ('.pgm', '.png'))
                    if os.path.exists(p))
    img = np.array(Image.open(img_path).convert('L'))
    occ_th = float(cfg.get('occupied_thresh', 0.65))
    neg = int(cfg.get('negate', 0))
    occ = ((255 - img) / 255.0 if not neg else img / 255.0) > occ_th
    py, px = np.nonzero(occ)
    res = float(cfg['resolution']); org = np.array(cfg['origin'][:2], float)
    h = img.shape[0]
    xy = [np.column_stack([px * res + org[0], (h - py) * res + org[1]])]
    # gen_waypoints 가 저장한 트랙 경계 폴리곤(픽셀 꼭짓점)도 벽으로 본다 —
    # _clean 맵은 바깥벽이 군데군데 비어 있어 점유 픽셀만으로는 새어 나간다.
    for nm in ('outer_bound.npy', 'inner_bound.npy'):
        bp = os.path.join(MAPS_DIR, map_name, nm)
        if not os.path.exists(bp):
            continue
        poly = np.load(bp).astype(float)
        poly = np.vstack([poly, poly[:1]])
        seg = []
        for a, b in zip(poly[:-1], poly[1:]):
            m = max(2, int(np.hypot(*(b - a))))
            seg.append(a + (b - a) * np.linspace(0, 1, m, endpoint=False)[:, None])
        seg = np.vstack(seg)
        xy.append(np.column_stack([seg[:, 0] * res + org[0], (h - seg[:, 1]) * res + org[1]]))
    return cKDTree(np.vstack(xy))


def smooth_curvature(pts, span=0.15):
    """±span[m] 떨어진 세 점의 외접원 곡률 — 점 노이즈에 둔감한 '기하' 곡률.
    조향 사용률은 이걸로 본다 (PP 는 L1≥1.2m 로 보므로 cm 단위 노이즈는 안 따라간다)."""
    ds = np.hypot(*np.diff(np.vstack([pts, pts[:1]]), axis=0).T)
    k = max(1, int(round(span / max(np.median(ds), 1e-3))))
    a, b, c = np.roll(pts, k, axis=0), pts, np.roll(pts, -k, axis=0)
    ab = np.linalg.norm(b - a, axis=1); bc = np.linalg.norm(c - b, axis=1)
    ca_ = np.linalg.norm(a - c, axis=1)
    cr = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
    return 2 * cr / np.maximum(ab * bc * ca_, 1e-12)


def evaluate_line(pts_xy, prm, walls=None, csv_v=None, kscale=None):
    pts, ks = controller_curvature(pts_xy)
    k = np.abs(ks)                      # 컨트롤러가 실제로 속도를 정하는 곡률
    kg = smooth_curvature(pts)          # 기하 곡률 (조향 판단용)
    _, info_s = solve_profile(pts[:, 0], pts[:, 1], None, mu_eff=prm['mu_eff'],
                              grip_util=prm['grip_util'], lf=prm['lf'], lr=prm['lr'],
                              v_min=prm['v_min'], a_lon_acc=prm['a_lon_acc'],
                              a_lon_brk=prm['a_lon_brk'], hard_cap=prm['hard_cap'],
                              curvature=np.abs(kg))
    v, info = solve_profile(pts[:, 0], pts[:, 1], None, mu_eff=prm['mu_eff'],
                            grip_util=prm['grip_util'], lf=prm['lf'], lr=prm['lr'],
                            v_min=prm['v_min'], a_lon_acc=prm['a_lon_acc'],
                            a_lon_brk=prm['a_lon_brk'], hard_cap=prm['hard_cap'],
                            curvature=k)
    L, MS = prm['wheelbase'], prm['max_steer']
    steer = np.arctan(L * np.abs(kg)) / MS
    ds = np.hypot(np.diff(np.r_[pts[:, 0], pts[0, 0]]), np.diff(np.r_[pts[:, 1], pts[0, 1]]))
    # 곡률 변화율 — L1 을 따라가는 PP 가 감당하는 정도 (기하 곡률 기준)
    dk = np.abs(np.diff(np.r_[kg, kg[0]])) / np.maximum(ds, 1e-3)
    out = dict(n=len(pts), length=float(ds.sum()), lap=info['lap_time'],
               lap_smooth=info_s['lap_time'], k_noise=float(np.median(np.abs(k - np.abs(kg)))),
               v_min=info['v_min'], v_max=info['v_max'],
               steer_max=float(steer.max()), steer_p95=float(np.percentile(steer, 95)),
               sat_frac=float(np.mean(steer > 0.8)), dk_p99=float(np.percentile(dk, 99)),
               v=v, pts=pts)
    if walls is not None:
        d, _ = walls.query(pts)
        out['clear_min'] = float(d.min())
        out['clear_p5'] = float(np.percentile(d, 5))
    return out


# ══════════════════════════════════════════════════════════════════════════════
# 기준선(중심선) 준비
# ══════════════════════════════════════════════════════════════════════════════
def periodic_resample(xy, step, smooth=0.0):
    xy = np.asarray(xy, float)
    d = np.linalg.norm(np.diff(np.vstack([xy, xy[:1]]), axis=0), axis=1)
    keep = d > 1e-6
    xy = xy[keep]
    tck, _ = splprep([np.r_[xy[:, 0], xy[0, 0]], np.r_[xy[:, 1], xy[0, 1]]],
                     s=smooth * len(xy), per=1)
    uu = np.linspace(0, 1, 4000, endpoint=False)
    x, y = splev(uu, tck)
    seg = np.hypot(np.diff(np.r_[x, x[0]]), np.diff(np.r_[y, y[0]]))
    s = np.r_[0, np.cumsum(seg)]
    total = s[-1]
    n = max(20, int(round(total / step)))
    s_new = np.linspace(0, total, n, endpoint=False)
    u_new = np.interp(s_new, s, np.r_[uu, 1.0])
    x, y = splev(u_new, tck)
    dx, dy = splev(u_new, tck, der=1)
    ddx, ddy = splev(u_new, tck, der=2)
    kap = (dx * ddy - dy * ddx) / np.maximum((dx ** 2 + dy ** 2) ** 1.5, 1e-9)
    tn = np.hypot(dx, dy)
    tx, ty = dx / tn, dy / tn
    return np.column_stack([x, y]), np.column_stack([-ty, tx]), kap, total / n


def align_to(xy, ref_xy):
    """xy 를 ref 와 같은 진행 방향·같은 출발점으로 돌린다."""
    tree = cKDTree(xy)
    _, i0 = tree.query(ref_xy[0])
    _, i1 = tree.query(ref_xy[min(10, len(ref_xy) - 1)])
    n = len(xy)
    fwd = (i1 - i0) % n
    if fwd > n // 2:                    # 반대 방향
        xy = xy[::-1].copy()
        tree = cKDTree(xy)
        _, i0 = tree.query(ref_xy[0])
    return np.roll(xy, -i0, axis=0)


def side_widths(cl, nrm, walls, w_max=3.0, safety=0.0, step=0.01):
    """법선 방향으로 실제 벽까지 광선 진행 — 최근접 거리 근사보다 정확하다."""
    left = np.zeros(len(cl)); right = np.zeros(len(cl))
    ts = np.arange(0.0, w_max, step)
    for sgn, out in ((1, left), (-1, right)):
        for i in range(len(cl)):
            ray = cl[i] + sgn * ts[:, None] * nrm[i]
            d, _ = walls.query(ray)
            hit = np.nonzero(d < 0.03)[0]
            out[i] = ts[hit[0]] if len(hit) else w_max
    return np.maximum(left - safety, 0.0), np.maximum(right - safety, 0.0)



# ══════════════════════════════════════════════════════════════════════════════
# 클릭 스케치 — 기준선을 지도 위에서 바로 찍는다 (다른 생성기 안 거침)
# ══════════════════════════════════════════════════════════════════════════════
def _map_image(map_name):
    from PIL import Image
    cfg = yaml.safe_load(open(os.path.join(MAPS_DIR, map_name + '.yaml')))
    img_path = next(p for p in (os.path.join(MAPS_DIR, map_name + e) for e in ('.pgm', '.png'))
                    if os.path.exists(p))
    img = np.array(Image.open(img_path).convert('L'))
    res = float(cfg['resolution']); org = np.array(cfg['origin'][:2], float)
    h, w = img.shape
    return img, [org[0], org[0] + w * res, org[1], org[1] + h * res]


def sketch_reference(map_name, walls, under=None):
    """지도 위 클릭으로 기준선 점을 받는다. 반환 (N,2) 월드 좌표, 취소면 None.

    찍은 **순서가 주행 방향**, **첫 점이 출발점**이다. 코너마다 2~3점, 직선은
    양 끝만 찍으면 충분하다 — 최적화가 좌우 ±w_max 안에서 모양을 새로 정하므로
    여기서 정하는 건 '어느 틈으로 지나갈지' 뿐이다.
    """
    import matplotlib
    if matplotlib.get_backend().lower() == 'agg':
        try:
            matplotlib.use('TkAgg')
        except Exception:
            pass
    import matplotlib.pyplot as plt
    if matplotlib.get_backend().lower() == 'agg':
        sys.exit('✗ 화면(디스플레이)이 없어 스케치 창을 못 띄웁니다 — 노트북에서 하거나 '
                 '--sketch-file / --like 를 쓰세요')

    img, ext = _map_image(map_name)
    fig, ax = plt.subplots(figsize=(10, 9))
    ax.imshow(img, cmap='gray', extent=ext, origin='upper')
    W = walls.data
    ax.scatter(W[:, 0], W[:, 1], s=0.3, c='k')
    if under is not None:
        ax.plot(under[:, 0], under[:, 1], '-', c='tab:blue', lw=0.8, alpha=0.5, label='참고 라인')
        ax.legend(loc='upper right')
    ax.set_aspect('equal')
    pts = []
    art = {}
    state = {'done': False, 'cancel': False}

    def redraw():
        for a in art.values():
            try:
                a.remove()
            except Exception:
                pass
        art.clear()
        if pts:
            P = np.array(pts)
            art['pts'] = ax.plot(P[:, 0], P[:, 1], 'o', c='tab:red', ms=5)[0]
            art['start'] = ax.plot(P[0, 0], P[0, 1], '*', c='gold', ms=16, mec='k')[0]
            if len(P) >= 2:
                art['arrow'] = ax.annotate('', xy=P[1], xytext=P[0],
                                           arrowprops=dict(arrowstyle='->', color='gold', lw=2))
            if len(P) >= 4:
                try:
                    cl, _, _, _ = periodic_resample(P, 0.05, smooth=0.0)
                    cl = np.vstack([cl, cl[:1]])
                    art['spl'] = ax.plot(cl[:, 0], cl[:, 1], '-', c='tab:red', lw=1.2)[0]
                    d, _ = walls.query(cl)
                    near = d < 0.2
                    if near.any():
                        art['near'] = ax.scatter(cl[near, 0], cl[near, 1], s=10, c='m',
                                                 label='벽 0.2m 안')
                except Exception:
                    pass
        ax.set_title(f'{map_name} — 좌클릭 점 추가 / 우클릭·z 취소 / Enter 완료 / Esc 취소\n'
                     f'첫 점(★)=출발, 찍는 순서=주행 방향, 점 {len(pts)}개 (4개 이상)',
                     fontsize=10)
        fig.canvas.draw_idle()

    def on_click(ev):
        if ev.inaxes != ax or (fig.canvas.toolbar and fig.canvas.toolbar.mode):
            return                       # 확대/이동 모드 중 클릭은 무시
        if ev.button == 1:
            pts.append((ev.xdata, ev.ydata))
        elif ev.button == 3 and pts:
            pts.pop()
        redraw()

    def on_key(ev):
        if ev.key == 'z' and pts:
            pts.pop(); redraw()
        elif ev.key in ('enter', 'return'):
            if len(pts) < 4:
                ax.set_title('점이 4개 이상 필요합니다', color='r'); fig.canvas.draw_idle(); return
            state['done'] = True; plt.close(fig)
        elif ev.key == 'escape':
            state['cancel'] = True; plt.close(fig)

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)
    redraw()
    plt.show()
    if state['cancel'] or not state['done']:
        return None
    return np.array(pts, float)


def show_result(map_name, walls, ref, line, v, title):
    try:
        import matplotlib.pyplot as plt
        img, ext = _map_image(map_name)
        fig, ax = plt.subplots(figsize=(10, 9))
        ax.imshow(img, cmap='gray', extent=ext, origin='upper')
        ax.plot(ref[:, 0], ref[:, 1], 'o--', c='tab:blue', ms=3, lw=0.8, label='스케치')
        sc = ax.scatter(line[:, 0], line[:, 1], c=v, s=5, cmap='RdYlGn')
        ax.plot(line[0, 0], line[0, 1], '*', c='gold', ms=16, mec='k')
        fig.colorbar(sc, ax=ax, label='v [m/s]')
        ax.set_aspect('equal'); ax.legend(loc='upper right'); ax.set_title(title, fontsize=10)
        plt.show()
    except Exception as e:
        print(f'  (결과 창 생략: {e})')

# ══════════════════════════════════════════════════════════════════════════════
# 최소시간 OCP (CasADi + IPOPT)
# ══════════════════════════════════════════════════════════════════════════════
def solve_mintime(kr, h, wl, wr, prm, *, kappa_cap, dkappa_max, w_dk, v_max,
                  kscale=None, init=None, verbose=False):
    import casadi as ca
    N = len(kr)
    a_g, a_acc, a_brk = prm['a_g'], prm['a_acc'], prm['a_brk']
    ag_loc = a_g * (np.ones(N) if kscale is None else kscale)

    opti = ca.Opti()
    n = opti.variable(N); xi = opti.variable(N); u = opti.variable(N)
    kap = opti.variable(N); ap = opti.variable(N); am = opti.variable(N)

    def nxt(z):
        return ca.vertcat(z[1:], z[0])

    sf = (1 - n * kr) / ca.cos(xi)                     # ds_path / ds_ref
    v = ca.sqrt(u)
    dt = h * 0.5 * (sf / v + nxt(sf) / nxt(v))
    T = ca.sum1(dt)

    a = ap - am
    opti.subject_to(nxt(n) - n == h * 0.5 * ((1 - n * kr) * ca.tan(xi)
                                             + (1 - nxt(n) * np.roll(kr, -1)) * ca.tan(nxt(xi))))
    opti.subject_to(nxt(xi) - xi == h * 0.5 * ((kap * sf - kr) + (nxt(kap) * nxt(sf) - np.roll(kr, -1))))
    opti.subject_to(nxt(u) - u == h * (a * sf + nxt(a) * nxt(sf)))      # du/ds = 2a, 사다리꼴
    opti.subject_to((ap / a_acc) ** 2 + (am / a_brk) ** 2 + (u * kap / ag_loc) ** 2 <= 1)

    dk = (nxt(kap) - kap) / (h * 0.5 * (sf + nxt(sf)))
    if dkappa_max > 0:
        opti.subject_to(opti.bounded(-dkappa_max, dk, dkappa_max))
    opti.subject_to(opti.bounded(-kappa_cap, kap, kappa_cap))
    opti.subject_to(opti.bounded(-wr, n, wl))
    # 오프셋 곡선이 기준선 곡률 중심을 넘으면 접혀서 꼬인다 (1 − n·κ_ref → 0)
    opti.subject_to(1 - n * kr >= 0.3)
    opti.subject_to(opti.bounded(-0.9, xi, 0.9))
    opti.subject_to(opti.bounded(prm['v_min'] ** 2, u, v_max ** 2))
    opti.subject_to(ap >= 0); opti.subject_to(am >= 0)

    J = T + w_dk * h * ca.sumsqr(dk) + 1e-3 * h * ca.sumsqr(ap * am)
    opti.minimize(J)

    if init is None:
        v0 = np.full(N, max(prm['v_min'] + 0.3, 1.5))
        init = dict(n=np.zeros(N), xi=np.zeros(N), u=v0 ** 2,
                    kap=np.clip(kr, -kappa_cap, kappa_cap), ap=np.zeros(N), am=np.zeros(N))
    for var, key in ((n, 'n'), (xi, 'xi'), (u, 'u'), (kap, 'kap'), (ap, 'ap'), (am, 'am')):
        opti.set_initial(var, init[key])

    opti.solver('ipopt', {'print_time': False},
                {'max_iter': 3000, 'print_level': 5 if verbose else 0,
                 'tol': 1e-6, 'acceptable_tol': 1e-4, 'mu_strategy': 'adaptive'})
    try:
        sol = opti.solve()
        ok = True
        get = sol.value
    except RuntimeError as e:
        print(f'  ⚠ IPOPT: {e}'.splitlines()[0])
        ok = False
        get = opti.debug.value
    res = {k: np.array(get(vv)).ravel() for k, vv in
           (('n', n), ('xi', xi), ('u', u), ('kap', kap), ('ap', ap), ('am', am))}
    res['T'] = float(get(T)); res['ok'] = ok
    return res


# ══════════════════════════════════════════════════════════════════════════════
def eval_table(map_name, prm, filt=None):
    walls = wall_tree(map_name)
    files = sorted(glob.glob(os.path.join(WP_DIR, map_name + '*.csv')))
    if filt:
        files = [f for f in files if filt in os.path.basename(f)]
    print(f'\n  {"이름":40s} {"예상랩":>6s} {"(매끈)":>6s} {"길이":>6s} {"v범위":>11s} '
          f'{"추종조향max":>9s} {"p95":>5s} {">80%":>5s} {"dκ/ds p99":>9s} {"벽min":>6s}')
    rows = []
    for f in files:
        try:
            raw = read_csv_xy(f)
            if len(raw) < 10:
                continue
            r = evaluate_line(raw[:, :2], prm, walls)
        except Exception as e:                     # 깨진 CSV 하나로 표 전체가 죽지 않게
            print(f'  {os.path.basename(f)[:-4]:40s} (평가 실패: {e})')
            continue
        rows.append((os.path.basename(f)[:-4], r))
    for name, r in sorted(rows, key=lambda t: t[1]['lap']):
        flag = '✗' if r['steer_max'] > 1.0 else ('⚠' if r['steer_max'] > 0.8 else ' ')
        print(f'{flag} {name:40s} {r["lap"]:6.2f} {r["lap_smooth"]:6.2f} {r["length"]:6.2f} '
              f'{r["v_min"]:4.2f}~{r["v_max"]:4.2f}  {100*r["steer_max"]:8.0f}% '
              f'{100*r["steer_p95"]:4.0f}% {100*r["sat_frac"]:4.1f}% {r["dk_p99"]:9.2f} '
              f'{r.get("clear_min", float("nan")):6.3f}')
    print('\n  예상랩 = map_controller 와 같은 곡률·속도 모델 (지연·추종오차는 없음 → 하한).'
          '\n  (매끈) = 같은 라인을 점 노이즈 없이 봤을 때. 예상랩과 차이가 크면 CSV 점이 거칠어'
          '\n           컨트롤러의 스플라인 곡률이 튀고, 그 스파이크에서 쓸데없이 감속한다.'
          '\n  추종조향 = atan(L·κ)/max_steer. 80% 넘으면 보정 여유가 없다.'
          '\n  벽min = 라인에서 가장 가까운 벽(점유 픽셀)까지 [m]. 차 반폭(~0.15) 보다 커야 한다.')
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--map', required=True)
    ap.add_argument('--eval', nargs='?', const='', default=None,
                    help='기존 라인 비교만 (선택: 이름 필터)')
    ap.add_argument('--like', default=None,
                    help='기준선으로 쓸 기존 라인 이름 (--sketch 와 같이 쓰면 창에 참고용으로만 표시)')
    ap.add_argument('--sketch', action='store_true',
                    help='지도 위 클릭으로 기준선을 찍는다 (화면 필요)')
    ap.add_argument('--sketch-file', default=None,
                    help='예전에 저장된 스케치 CSV 로 다시 돌린다 (maps/<맵>/sketch_<이름>.csv)')
    ap.add_argument('--out', default=None, help='출력 이름 (기본 <map>_wp_mintime)')
    ap.add_argument('--force', action='store_true', help='같은 이름이 있으면 덮어쓴다')
    ap.add_argument('--car', default=None,
                    help='car1/car2 — 그 차의 vehicle yaml 로 계산 (기본: 지금 .vehicle_overlay)')
    ap.add_argument('--mode', default='race', help='grip_util 을 읽을 모드 (기본 race)')
    ap.add_argument('--grip', type=float, default=None, help='grip_util 직접 지정')
    ap.add_argument('--safety', type=float, default=0.25,
                    help='벽과 라인(차 중심) 최소 거리 [m]. 차 반폭 0.15 + 여유')
    ap.add_argument('--step', type=float, default=0.10, help='OCP 격자 [m]')
    ap.add_argument('--w-max', type=float, default=1.0,
                    help='기준선에서 좌우로 움직일 수 있는 최대 폭 [m] (경로 위상 유지)')
    ap.add_argument('--ref-smooth', type=float, default=0.0005,
                    help='기준선 스플라인 평활 (CSV 점 노이즈 제거)')
    ap.add_argument('--steer-util', type=float, default=0.70,
                    help='라인 곡률이 쓸 수 있는 조향 비율 (추종 여유 = 1 − 이 값)')
    ap.add_argument('--dkappa-max', type=float, default=1.5,
                    help='|dκ/ds| 상한 [1/m²]. 작을수록 PP(L1) 가 따라가기 쉽다. 0=끔')
    ap.add_argument('--w-dk', type=float, default=0.02, help='dκ/ds 매끈함 가중')
    ap.add_argument('--v-max', type=float, default=7.0)
    ap.add_argument('--passes', type=int, default=1,
                    help='결과를 새 기준선으로 다시 푸는 횟수. ⚠ 패스마다 창이 w_max 씩 옮겨가서 '
                         '스케치한 경로에서 최대 passes×w_max 까지 벗어난다 — 넓은 방 맵이면 '
                         '섬에 붙어 도는 작은 원으로 줄어든다 (c11: 19 m → 13.8 m). '
                         '트랙 경계가 지도에 다 그려져 있을 때만 올릴 것')
    ap.add_argument('--verbose', action='store_true')
    args = ap.parse_args()

    prm = load_params(args.mode, args.car)
    if args.grip is not None:
        prm['grip_util'] = args.grip
        prm['a_g'] = args.grip * prm['mu_eff'] * GRAVITY
        L = prm['lf'] + prm['lr']
        prm['a_acc'] = min(prm['a_lon_acc'] if prm['a_lon_acc'] > 0 else prm['a_g'] * prm['lf'] / L, prm['a_g'])
        prm['a_brk'] = min(prm['a_lon_brk'] if prm['a_lon_brk'] > 0 else prm['a_g'], prm['a_g'])
    print(f"[mintime] 차량 {prm['car']}  μ_eff {prm['mu_eff']:.3f}  η {prm['grip_util']:.2f}  "
          f"a_g {prm['a_g']:.2f}  가속 {prm['a_acc']:.2f}  제동 {prm['a_brk']:.2f} ㎧²  "
          f"v_min {prm['v_min']:.2f}  L {prm['wheelbase']:.3f}  max_steer {prm['max_steer']:.4f}")

    if args.eval is not None:
        eval_table(args.map, prm, args.eval or None)
        return

    out = args.out or f'{args.map}_wp_mintime'
    # 패스 수를 이름에 남긴다 — 1패스와 N패스는 경로 자체가 다르다(N패스는 스케치에서
    # 최대 N×w_max 까지 벗어남). 이름만 보고 어느 쪽인지 알 수 있어야 한다.
    if args.passes > 1 and not re.search(r'_p\d+$', out):
        out += f'_p{args.passes}'
    out_csv = os.path.join(WP_DIR, out + '.csv')
    if os.path.exists(out_csv) and not args.force:
        sys.exit(f'이미 있음: {out_csv}  (--out 으로 다른 이름, 또는 --force)')

    # 1) 기준선 — 사람이 고른 경로(--like)를 쓴다. 넓은 방 + 섬 구조 맵에서는
    #    '중심선' 이 경로 위상(어느 틈으로 지나갈지)을 멋대로 정해 버린다.
    walls = wall_tree(args.map)
    sketch_dir = os.path.join(MAPS_DIR, args.map)
    sketch_path = os.path.join(sketch_dir, f'sketch_{out}.csv')
    if args.sketch or args.sketch_file:
        if args.sketch_file:
            ref_raw = read_csv_xy(args.sketch_file)[:, :2]
            print(f'  스케치 파일: {args.sketch_file} ({len(ref_raw)}점)')
        else:
            under = read_csv_xy(os.path.join(WP_DIR, args.like + '.csv'))[:, :2] if args.like else None
            ref_raw = sketch_reference(args.map, walls, under)
            if ref_raw is None:
                sys.exit('  취소됨')
            # 스케치는 waypoints/ 가 아니라 maps/<맵>/ 에 둔다 — 점이 듬성듬성해서
            # 주행 라인으로 고르면 안 되는데, waypoints/ 에 있으면 ./race.sh wp 목록에 뜬다.
            os.makedirs(sketch_dir, exist_ok=True)
            with open(sketch_path, 'w', newline='') as f:
                wcsv = csv.writer(f); wcsv.writerow(['x', 'y'])
                for x_, y_ in ref_raw:
                    wcsv.writerow([f'{x_:.4f}', f'{y_:.4f}'])
            print(f'  스케치 {len(ref_raw)}점 저장: {sketch_path}\n'
                  f'    (같은 경로로 다시 돌리려면 --sketch-file {sketch_path})')
    elif args.like:
        ref_raw = read_csv_xy(os.path.join(WP_DIR, args.like + '.csv'))[:, :2]
    else:
        print('  ⚠ --like 없음 — 맵 중심선을 기준으로 쓴다. 진행 방향을 확인할 것')
        ref_raw = extract_reftrack(args.map, safety_m=0.0)[:, :2]
    # 패스 반복: 최적화는 기준선 ±w_max 안에서만 움직이므로, 결과를 새 기준선으로
    # 삼아 창을 최적 라인 쪽으로 옮기고 다시 푼다. 듬성한 스케치일수록 효과가 크다.
    ref_sketch = ref_raw
    best = None
    for ps in range(max(1, args.passes)):
        if args.passes > 1:
            print(f'  ── 패스 {ps + 1}/{args.passes}')
        cl, nrm, kr, h = periodic_resample(ref_raw, args.step, smooth=args.ref_smooth)
        wl, wr = side_widths(cl, nrm, walls, w_max=args.w_max + args.safety, safety=args.safety)
        wl, wr = np.minimum(wl, args.w_max), np.minimum(wr, args.w_max)
        print(f'  기준선 {len(cl)}점 × {h:.3f} m = {len(cl)*h:.1f} m   '
              f'통로(safety 후, ≤{args.w_max}) 좌 {wl.min():.2f}~{wl.max():.2f} 우 {wr.min():.2f}~{wr.max():.2f}')
        # 기준선 자체가 safety 안쪽이면 n=0 도 불가 → 그 점은 벽 반대쪽으로만 움직이게 둔다
        bad = (wl <= 0) & (wr <= 0)
        if bad.any():
            print(f'  ⚠ 기준선이 양쪽 벽 모두 safety 안: {int(bad.sum())}점 — --safety 를 낮출 것')

        kcap = math.tan(args.steer_util * prm['max_steer']) / prm['wheelbase']
        print(f'  곡률 상한 κ ≤ {kcap:.3f} (R ≥ {1/kcap:.3f} m, 조향 {100*args.steer_util:.0f}%)'
              f'   |dκ/ds| ≤ {args.dkappa_max}')

        # 2) OCP — 매끈한 문제부터 풀고(웜스타트) 실제 제약으로
        r = solve_mintime(kr, h, wl, wr, prm, kappa_cap=kcap, dkappa_max=0.0,
                          w_dk=max(args.w_dk, 0.2), v_max=args.v_max, verbose=args.verbose)
        print(f'  1단계(매끈) T={r["T"]:.3f}s ok={r["ok"]}')
        # 3) 벽 여유 검증 루프 — 법선 광선은 비스듬한 벽을 못 보므로, 풀린 라인의
        #    실제 최근접 벽거리를 재서 safety 를 어긴 곳만 폭을 좁혀 다시 푼다.
        for it in range(6):
            r = solve_mintime(kr, h, wl, wr, prm, kappa_cap=kcap, dkappa_max=args.dkappa_max,
                              w_dk=args.w_dk, v_max=args.v_max, init=r, verbose=args.verbose)
            P = cl + nrm * r['n'][:, None]
            d, _ = walls.query(P)
            viol = np.nonzero(d < args.safety - 0.01)[0]
            print(f'  2단계[{it}] T={r["T"]:.3f}s ok={r["ok"]}  벽min {d.min():.3f} m  위반 {len(viol)}점')
            if not len(viol):
                break
            for i in viol:
                short = args.safety - d[i] + 0.02
                for j in range(i - 2, i + 3):
                    j %= len(cl)
                    if r['n'][i] >= 0:
                        wl[j] = max(min(wl[j], r['n'][i] - short), -wr[j] + 0.01)
                    else:
                        wr[j] = max(min(wr[j], -r['n'][i] - short), -wl[j] + 0.01)

        if not r['ok']:
            break
        line_p, _, _, _ = periodic_resample(cl + nrm * r['n'][:, None], 0.05, smooth=0.0)
        ev_p = evaluate_line(line_p, prm, walls)
        print(f'    → 컨트롤러 모델 예상 랩 {ev_p["lap"]:.3f}s  추종조향 max {100*ev_p["steer_max"]:.0f}%')
        if best is None or ev_p['lap'] < best[0] - 1e-3:
            best = (ev_p['lap'], r, cl, nrm)
        else:
            break                                   # 더 안 좋아지면 멈춘다
        ref_raw = cl + nrm * r['n'][:, None]
    if best is not None:
        _, r, cl, nrm = best
    ref_raw = ref_sketch
    # 3) 라인 복원 → 0.05 m 재샘플 → 컨트롤러 모델로 재평가
    line = cl + nrm * r['n'][:, None]
    line, _, _, _ = periodic_resample(line, 0.05, smooth=0.0)
    ev = evaluate_line(line, prm, walls)
    print(f'  결과: 길이 {ev["length"]:.2f} m  예상 랩 {ev["lap"]:.3f} s (컨트롤러 모델)'
          f'  추종조향 max {100*ev["steer_max"]:.0f}%  벽min {ev["clear_min"]:.3f} m'
          f'  dκ/ds p99 {ev["dk_p99"]:.2f}')

    os.makedirs(WP_DIR, exist_ok=True)
    with open(out_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'speed'])
        for (x, y), v in zip(ev['pts'], ev['v']):
            w.writerow([f'{x:.6f}', f'{y:.6f}', f'{v:.3f}'])  # 소수 4자리면 반올림 노이즈가 컨트롤러 스플라인 곡률을 튀게 한다
    print(f'  저장: {out_csv}')
    try:
        save_overlay(args.map, ev['pts'], ev['v'], out_csv)
    except Exception as e:
        print(f'  (PNG 생략: {e})')
    if args.sketch:
        show_result(args.map, walls, ref_raw, ev['pts'], ev['v'],
                    f'{out} — 예상 랩 {ev["lap"]:.2f}s, 추종조향 max {100*ev["steer_max"]:.0f}%, '
                    f'벽min {ev["clear_min"]:.2f}m  (창을 닫으면 끝)')
    print('\n  비교:  ./race.sh mintime', args.map, '--eval')


if __name__ == '__main__':
    main()
