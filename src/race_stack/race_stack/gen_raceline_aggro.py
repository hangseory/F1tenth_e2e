#!/usr/bin/env python3
"""
gen_raceline_aggro.py — 마진을 물리 한계까지 밀어붙인 레이싱라인.

왜 있나
───────
gen_raceline 의 기본값은 **안전쪽으로 넉넉하다**:

    --safety 0.35   QP 가 쓸 트랙 폭을 양쪽에서 깎는다
    --wallclear 0.18  차 측면~벽 최소 여유 (합격선)
    --apex-extra 0.40  코너 안쪽에만 추가로 더 주는 여유

이 값들을 손으로 하나씩 낮춰가며 "어디까지 되나" 를 찾는 일을 사람이 반복
하고 있었다 (2026-11-08 에 1108_clean 에서 네 번 돌렸다). 그걸 자동화한다.

이 파일은 **최적화 알고리즘을 새로 만들지 않는다.** gen_raceline 의 재선형화
반복 최소곡률 QP 를 그대로 호출하고, 마진 조합만 쓸어본다. "꾸불꾸불한 길을
직선으로 스치듯 간다" 는 것이 바로 최소곡률 최적화의 결과이고, 그게 안 나오는
이유는 알고리즘이 아니라 **마진이 쓸 폭을 먹어서**이기 때문이다.

★ 절대 완화하지 않는 것 — 차량 최소회전반경
──────────────────────────────────────────
  R_CAR_MIN = wheelbase / tan(max_steer) 는 **기하학**이지 취향이 아니다.
  이걸 넘는 라인은 핸들을 끝까지 꺾어도 못 돈다. 속도를 줄여도 안 된다
  (조향각 한계는 속도와 무관). 그래서 이 스윕은 R_min 미달 조합을 **후보에서
  제외**하고, 하나도 안 남으면 트랙이 모자란 것이라고 말한다.

★ 여유를 줄이면 무엇을 잃나 — 반드시 읽을 것
────────────────────────────────────────────
  벽 여유의 존재 이유는 **라인과 실제 주행의 차이를 흡수**하는 것이다.
  이 차의 실측 추종오차 (CTE):

      직선  0.16 m
      코너  0.39~0.55 m      ← 오차는 코너에 몰린다

  여유를 0.1 m 로 깎아도 생성은 성공한다. 그런데 코너에서 0.4 m 밀리면
  그대로 벽이다. 상위팀이 벽을 스칠 수 있는 것은 마진을 줄였기 때문이 아니라
  **추종오차가 작기 때문**이다. 순서가 반대다:

      ① 위치추정을 안정시킨다      (원거리 잔차가 크면 여기서 멈출 것)
      ② 컨트롤러를 튜닝해 CTE 를 줄인다
      ③ 줄어든 CTE 만큼 마진을 깎는다   ← 이 도구는 ③ 단계용이다

  그래서 기본 하한(--floor)은 0.06 m 가 아니라 **실측 CTE 기반**으로 받는다.
  모르면 --cte 로 직접 주거나, 기본값(0.25 m, 코너 실측 중앙값 언저리)을 쓴다.

사용법
──────
    python3 -m race_stack.gen_raceline_aggro --map 1109-2_clean

    --cte 0.25      흡수해야 할 추종오차 [m]. 벽 여유 하한이 된다
                    (실측값을 넣을 것. logs/<런>/driven_path.csv 로 잴 수 있다)
    --floor 0.06    절대 하한 [m]. --cte 와 무관하게 이보다는 안 내려간다
                    (0.060 = 실주행 완주가 확인된 halfhot_forza_opt_wp 의 여유)
    --keep          가장 빠른 조합을 실제로 저장한다 (없으면 표만 보여준다)
    --out NAME      출력 이름 (기본 <맵>_aggro_wp)

    그 밖의 인자(--mu --vmax --step --rmin ...)는 gen_raceline 과 같다.
"""
import argparse
import contextlib
import csv
import io as _io
import os
import sys

import numpy as np

from race_stack.gen_raceline import (
    MAPS_DIR, WP_DIR, R_CAR_MIN,
    extract_reftrack, optimize_raceline, calc_vel_profile, save_overlay,
)


# 보수적 → 공격적. (safety, wallclear, apex_extra)
#   safety     : QP 작업폭. 낮출수록 벽 가까이 쓸 수 있다
#   wallclear  : 합격선. 이 값 미만인 반복은 채택되지 않는다
#   apex_extra : 코너 안쪽 추가 여유. 좁은 트랙에서 가장 먼저 버릴 것
LADDER = [
    (0.35, 0.18, 0.40),   # gen_raceline 기본값
    (0.30, 0.15, 0.30),
    (0.25, 0.12, 0.20),
    (0.20, 0.10, 0.10),
    (0.15, 0.08, 0.05),
    (0.12, 0.07, 0.00),
    (0.10, 0.06, 0.00),
    (0.08, 0.05, 0.00),
    (0.05, 0.04, 0.00),
    (0.03, 0.03, 0.00),
]


def lap_time(el, v):
    """구간길이/속도 적분. el 은 점 사이 거리, v 는 각 점의 목표속도."""
    vv = np.maximum(v, 1e-3)
    return float(np.sum(el / vv))


def try_one(map_name, safety, wallclear, apex, args):
    """한 조합을 돌려 결과를 돌려준다. 실패하면 None."""
    # gen_raceline 의 진행 출력(iter 로그·폭·속도)을 삼킨다 — 여기서는 조합마다
    # 한 줄씩만 보여야 표가 읽힌다. 실패 사유는 예외 메시지로 따로 받는다.
    sink = _io.StringIO()
    try:
        with contextlib.redirect_stdout(sink):
            reftrack, outer_w, inner_w = extract_reftrack(
                map_name, safety_m=safety, return_bounds=True,
                gap_fill=args.gapfill)
            r_act, nodes, raceline, el, kappa, total, wclear = optimize_raceline(
                reftrack, outer_w, inner_w,
                w_veh=args.wveh, r_min=args.rmin,
                stepsize=args.gridstep, out_step=args.step,
                safety_m=safety, iters=args.iters, step_limit=args.steplimit,
                apex_extra=apex, apex_kappa=None,
                wall_clear=wallclear)
            v = calc_vel_profile(kappa, el, mu=args.mu, v_max=args.vmax,
                                 v_min=args.vmin, a_lon_acc=args.acc,
                                 a_lon_brk=args.brk)
    except (RuntimeError, ValueError, IndexError) as exc:
        return {'ok': False, 'why': str(exc).split('\n')[0]}
    n_bad = int(np.sum(kappa > 1.0 / R_CAR_MIN))
    return {'ok': True, 'safety': safety, 'wallclear': wallclear, 'apex': apex,
            'r_act': r_act, 'clear': wclear, 'total': total, 'n_bad': n_bad,
            'lap': lap_time(el, v), 'raceline': raceline, 'v': v,
            'el': el, 'kappa': kappa}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--map', required=True)
    p.add_argument('--out', default=None)
    p.add_argument('--cte', type=float, default=0.25,
                   help='흡수해야 할 추종오차 [m] — 벽 여유 하한이 된다 (기본 0.25)')
    p.add_argument('--floor', type=float, default=0.06,
                   help='절대 하한 [m] (기본 0.06 = 실주행 완주 확인된 여유)')
    p.add_argument('--keep', action='store_true', help='가장 빠른 것을 저장한다')
    # gen_raceline 과 같은 의미의 인자들
    p.add_argument('--wveh', type=float, default=0.31)
    p.add_argument('--rmin', type=float, default=1.0)
    p.add_argument('--mu', type=float, default=0.90)
    p.add_argument('--vmax', type=float, default=7.0)
    p.add_argument('--vmin', type=float, default=0.5)
    p.add_argument('--acc', type=float, default=5.0)
    p.add_argument('--brk', type=float, default=8.0)
    p.add_argument('--step', type=float, default=0.1)
    p.add_argument('--gridstep', type=float, default=0.5)
    p.add_argument('--steplimit', type=float, default=0.15)
    p.add_argument('--iters', type=int, default=12)
    p.add_argument('--gapfill', type=float, default=0.05)
    p.add_argument('--ot-width', type=float, default=1.10, dest='ot_width')
    p.add_argument('--ot-exit-pad', type=float, default=2.50, dest='ot_exit_pad')
    p.add_argument('--ot-min-len', type=float, default=2.00, dest='ot_min_len')
    args = p.parse_args()

    if not os.path.exists(os.path.join(MAPS_DIR, args.map + '.pgm')):
        sys.exit(f'✗ 맵 이미지 없음: {os.path.join(MAPS_DIR, args.map + ".pgm")}')

    need = max(args.cte, args.floor)
    print(f'맵 {args.map}   차폭 {args.wveh} m   차량 최소회전반경 {R_CAR_MIN:.3f} m')
    print(f'벽 여유 하한 {need:.3f} m  (추종오차 {args.cte:.2f} / 절대하한 {args.floor:.2f} 중 큰 쪽)')
    print(f'\n{len(LADDER)}개 조합을 공격적인 쪽으로 쓸어본다...\n')

    hdr = (f'{"safety":>7} {"wclr":>6} {"apex":>6} │ {"R_min":>7} {"여유":>7} '
           f'{"길이":>7} {"랩":>7} │ 판정')
    print(hdr); print('─' * len(hdr))

    results = []
    for safety, wallclear, apex in LADDER:
        r = try_one(args.map, safety, wallclear, apex, args)
        if not r['ok']:
            print(f'{safety:7.2f} {wallclear:6.2f} {apex:6.2f} │ '
                  f'{"—":>7} {"—":>7} {"—":>7} {"—":>7} │ ✗ 해 없음')
            continue

        # ① 물리 — 절대 완화하지 않는다
        if r['n_bad']:
            verdict = f'✗ 못 돈다 ({r["n_bad"]}점)'
        # ② 추종오차를 흡수할 여유가 있나
        elif r['clear'] < need:
            verdict = f'✗ 여유 부족 (<{need:.2f})'
        else:
            verdict = '✓'
            results.append(r)
        print(f'{safety:7.2f} {wallclear:6.2f} {apex:6.2f} │ '
              f'{r["r_act"]:7.3f} {r["clear"]:7.3f} {r["total"]:7.2f} '
              f'{r["lap"]:7.2f} │ {verdict}')

    if not results:
        print(f'\n✗ 통과한 조합이 없다.')
        print(f'  · "못 돈다" 뿐이면 → 트랙이 차 회전반경보다 좁다. 마진 문제가 아니다')
        print(f'  · "여유 부족" 뿐이면 → --cte 를 실측값으로 낮추거나 트랙을 넓힐 것')
        print(f'    (지금 하한 {need:.3f} m. 추종오차를 줄이지 않고 이 값만 낮추면 벽이다)')
        sys.exit(1)

    best = min(results, key=lambda r: r['lap'])
    base = results[0]          # 사다리의 첫 통과 = 가장 보수적인 통과
    print(f'\n가장 빠른 조합: safety {best["safety"]:.2f} / wallclear '
          f'{best["wallclear"]:.2f} / apex {best["apex"]:.2f}')
    print(f'  랩 {best["lap"]:.2f}s   R_min {best["r_act"]:.3f} m   '
          f'벽 여유 {best["clear"]:.3f} m')
    if best is not base:
        print(f'  가장 보수적인 통과 대비: 랩 {base["lap"]:.2f}s → {best["lap"]:.2f}s '
              f'({best["lap"] - base["lap"]:+.2f}s), '
              f'여유 {base["clear"]:.3f} → {best["clear"]:.3f} m '
              f'({best["clear"] - base["clear"]:+.3f} m)')
    print(f'\n  ⚠ 벽 여유 {best["clear"]:.3f} m 는 추종오차 {args.cte:.2f} m 를 '
          f'{"흡수한다" if best["clear"] >= args.cte else "못 흡수한다"}. '
          f'실측 CTE 가 이보다 크면 벽에 닿는다.')

    if not args.keep:
        print('\n(--keep 이 없어 저장하지 않았다. 표만 보고 끝낸다)')
        return

    out_name = args.out or f'{args.map}_aggro_wp'
    wp_path = os.path.join(WP_DIR, out_name + '.csv')
    with open(wp_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'speed'])
        for (x, y), vi in zip(best['raceline'], best['v']):
            w.writerow([f'{x:.4f}', f'{y:.4f}', f'{vi:.4f}'])
    print(f'\n저장: {len(best["raceline"])}pts → {wp_path}')
    save_overlay(args.map, best['raceline'], best['v'], wp_path, None)
    print(f'  race_script.yaml → waypoints: {out_name}')
    print(f'  ★ 저속(./tune.sh cap=1.0)으로 먼저 돌려 실제 CTE 를 재고, '
          f'그 값으로 --cte 를 다시 줘서 생성할 것')


if __name__ == '__main__':
    main()
