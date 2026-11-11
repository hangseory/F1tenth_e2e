#!/usr/bin/env python3
"""
wp_show.py — 웨이포인트 라인의 오버레이 사진(PNG)을 바로 띄운다.

  ./race.sh wp-show                 지금 맵의 라인 목록 → 번호 골라 열기
  ./race.sh wp-show <이름>          그 라인 사진 열기 (정확한 이름)
  ./race.sh wp-show <이름조각>      이름에 포함된 것들 → 하나면 바로, 여럿이면 번호 선택
  ./race.sh wp-show all             전부에서 고르기

번호는 여러 개를 띄어 쓰거나(1 3 5) 범위(2-4)로 줄 수 있다.
PNG 가 없는 라인(오래된 생성기·녹화 라인)은 맵 위에 속도색으로 즉석에서 그린다.
화면이 없으면(젯슨 SSH) 열지 않고 경로만 알려준다.
"""
import glob
import os
import shutil
import subprocess
import sys

import numpy as np
import yaml

REPO = os.environ.get('RACE_REPO') or os.path.abspath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..'))
WP_DIR = os.path.join(REPO, 'src', 'race_stack', 'waypoints')
MAPS_DIR = os.path.join(REPO, 'src', 'race_stack', 'maps')
CACHE = os.path.join(os.path.expanduser('~'), '.cache', 'race_wp_show')


def current_map_and_wp():
    try:
        cfg = yaml.safe_load(open(os.path.join(REPO, 'race_script.yaml'))) or {}
    except OSError:
        cfg = {}
    return cfg.get('map_name') or '', os.environ.get('RACE_WAYPOINTS') or cfg.get('waypoints') or ''


def guess_map(wp_name):
    """라인 이름의 접두사 중 가장 긴 맵 이름 (c11_clean_wp_x → c11_clean)."""
    maps = {os.path.basename(p)[:-5] for p in glob.glob(os.path.join(MAPS_DIR, '*.yaml'))}
    best = ''
    for m in maps:
        if wp_name.startswith(m) and len(m) > len(best):
            best = m
    return best or None


def render_png(name):
    """PNG 가 없는 라인을 맵 위에 속도색으로 그린다 (repo 는 안 더럽히고 캐시에)."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image
    pts = []
    for line in open(os.path.join(WP_DIR, name + '.csv')):
        p = line.strip().split(',')
        try:
            pts.append((float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0))
        except (ValueError, IndexError):
            continue
    if len(pts) < 2:
        return None
    a = np.array(pts)
    fig, ax = plt.subplots(figsize=(9, 9))
    m = guess_map(name)
    if m:
        cfg = yaml.safe_load(open(os.path.join(MAPS_DIR, m + '.yaml')))
        img_path = next((os.path.join(MAPS_DIR, m + e) for e in ('.pgm', '.png')
                         if os.path.exists(os.path.join(MAPS_DIR, m + e))), None)
        if img_path:
            img = np.array(Image.open(img_path).convert('L'))
            r = float(cfg['resolution']); o = cfg['origin']
            ax.imshow(img, cmap='gray', origin='upper',
                      extent=[o[0], o[0] + img.shape[1] * r, o[1], o[1] + img.shape[0] * r])
    sc = ax.scatter(a[:, 0], a[:, 1], c=a[:, 2], s=4, cmap='RdYlGn')
    ax.plot(a[0, 0], a[0, 1], '*', c='gold', ms=14, mec='k')
    fig.colorbar(sc, ax=ax, label='speed [m/s]', shrink=0.8)
    pad = 1.0
    ax.set_xlim(a[:, 0].min() - pad, a[:, 0].max() + pad)
    ax.set_ylim(a[:, 1].min() - pad, a[:, 1].max() + pad)
    ax.set_aspect('equal')
    ax.set_title(f'{name}  (map: {m or "?"})  * = start', fontsize=10)
    os.makedirs(CACHE, exist_ok=True)
    out = os.path.join(CACHE, name + '.png')
    fig.savefig(out, dpi=90)
    plt.close(fig)
    return out


def png_for(name):
    p = os.path.join(WP_DIR, name + '.png')
    if os.path.exists(p):
        return p, False
    return render_png(name), True


def open_images(paths):
    paths = [p for p in paths if p]
    if not paths:
        return
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        print('\n  화면이 없어 열 수 없습니다 (SSH?). 노트북에서 열거나 복사해 가세요:')
        for p in paths:
            print(f'    {p}')
        return
    viewer = shutil.which('eog') or shutil.which('xdg-open')
    if not viewer:
        print('  이미지 뷰어(eog / xdg-open)가 없습니다. 경로:')
        for p in paths:
            print(f'    {p}')
        return
    if os.path.basename(viewer) == 'eog':
        subprocess.Popen([viewer] + paths, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        for p in paths:
            subprocess.Popen([viewer, p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f'  열었습니다: {len(paths)}장' + ('  (eog 에서 ←/→ 로 넘김)' if len(paths) > 1 else ''))


def parse_sel(text, n):
    out = []
    for tok in text.replace(',', ' ').split():
        if '-' in tok:
            a, b = tok.split('-', 1)
            if a.isdigit() and b.isdigit():
                out += list(range(int(a), int(b) + 1))
        elif tok.isdigit():
            out.append(int(tok))
    return [i for i in dict.fromkeys(out) if 1 <= i <= n]


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else ''
    names = sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(WP_DIR, '*.csv')))
    mapn, cur = current_map_and_wp()

    if arg and arg in names:
        p, made = png_for(arg)
        if made and p:
            print(f'  (PNG 가 없어 즉석에서 그렸습니다: {p})')
        open_images([p])
        return

    if arg == 'all':
        shown, why = names, '전부'
    elif arg:
        shown, why = [n for n in names if arg in n], f"'{arg}' 포함"
    else:
        shown = [n for n in names if mapn and n.startswith(mapn)]
        why = f"지금 맵 '{mapn}' 것만"
        if not shown:
            shown, why = names, '전부'
    if not shown:
        sys.exit(f"  '{arg}' 가 들어간 라인이 없습니다.  ./race.sh wp-show all 로 전체를 보세요.")
    if len(shown) == 1:
        p, _ = png_for(shown[0])
        print(f'  {shown[0]}')
        open_images([p])
        return

    print(f'\n  라인 {len(shown)}개 ({why})   ★ = 지금 선택된 것   · = PNG 없음(즉석에서 그림)\n')
    for i, n in enumerate(shown, 1):
        mark = '★' if n == cur else ' '
        nopng = ' ' if os.path.exists(os.path.join(WP_DIR, n + '.png')) else '·'
        print(f'  {i:3d}. {mark}{nopng} {n}')
    try:
        sel = input('\n  열 번호 (예: 3  /  1 4 7  /  2-5,  Enter=취소): ').strip()
    except (EOFError, KeyboardInterrupt):
        print(); return
    idx = parse_sel(sel, len(shown))
    if not idx:
        return
    open_images([png_for(shown[i - 1])[0] for i in idx])


if __name__ == '__main__':
    main()
