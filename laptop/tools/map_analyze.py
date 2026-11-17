#!/usr/bin/env python3
"""Map quality analyzer for slam_toolbox PGM maps (map_server trinary format).

Outputs
-------
* size / resolution / unknown-free-occupied percentages
* free-space connectivity (largest connected free component)
* refined Hough walls: angle, offset, inliers, length, flatness (residual std),
  continuity (fraction of 10 cm bins along the wall that contain points)
* structured fraction (cells belonging to long clean walls)
* double-wall (ghosting) detection: parallel walls 4-20 cm apart, overlapping
"""
import sys

import numpy as np
from scipy import ndimage


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    idx = 0
    tokens = []
    while len(tokens) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':
            while data[idx:idx + 1] not in (b'\n', b''):
                idx += 1
            continue
        start = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        tokens.append(data[start:idx])
    w, h = int(tokens[1]), int(tokens[2])
    idx += 1
    arr = np.frombuffer(data[idx:idx + w * h], dtype=np.uint8).reshape(h, w)
    return arr


def refine_line(pts, th, rho, tol=0.06):
    """Refit line through points within tol of the Hough line."""
    n = np.array([np.cos(th), np.sin(th)])
    d = np.abs(pts @ n - rho)
    m = d <= tol
    p = pts[m]
    if len(p) < 15:
        return None
    c = p.mean(axis=0)
    q = p - c
    # PCA: principal direction
    cov = q.T @ q / len(q)
    evals, evecs = np.linalg.eigh(cov)
    dirv = evecs[:, np.argmax(evals)]
    normal = np.array([-dirv[1], dirv[0]])
    resid = q @ normal
    proj = q @ dirv
    length = float(proj.max() - proj.min())
    # continuity: fraction of 10 cm bins along extent that contain a point
    nb = max(1, int(length / 0.10))
    hist, _ = np.histogram(proj, bins=nb, range=(proj.min(), proj.max()))
    cont = float((hist > 0).sum()) / nb
    return dict(theta=np.degrees(np.arctan2(dirv[1], dirv[0])) % 180.0,
                length=length, flat=float(resid.std()), cont=cont,
                n=int(len(p)), center=c, dir=dirv, normal=normal,
                proj=(float(proj.min()), float(proj.max())))


def detect_walls(pts, res, theta_step=0.5, rho_bin=0.05, min_peak=18):
    thetas = np.deg2rad(np.arange(0, 180, theta_step))
    raw_pts = pts
    peaks = []
    for th in thetas:
        rho = raw_pts @ np.array([np.cos(th), np.sin(th)])
        lo, hi = rho.min(), rho.max()
        if hi - lo < 1e-6:
            continue
        hist, edges = np.histogram(rho, bins=np.arange(lo, hi + rho_bin, rho_bin))
        for i in range(1, len(hist) - 1):
            if hist[i] >= min_peak and hist[i] >= hist[i - 1] and hist[i] >= hist[i + 1]:
                peaks.append((int(hist[i]), float(th), float(edges[i] + rho_bin / 2)))
    peaks.sort(reverse=True)
    used = np.zeros(len(pts), dtype=bool)
    walls = []
    for cnt, th, rho in peaks:
        nv = np.array([np.cos(th), np.sin(th)])
        d = np.abs(pts @ nv - rho)
        m = d <= 0.06
        new = m & ~used
        if new.sum() < 0.5 * cnt:      # duplicate of an accepted line
            continue
        w = refine_line(pts, th, rho)
        if w is None:
            continue
        walls.append(w)
        used |= m
        if len(walls) >= 20:
            break
    return walls


def analyze(path, res=0.05):
    arr = read_pgm(path)
    h, w = arr.shape
    free = arr >= 250
    occ = arr <= 10
    unk = ~free & ~occ
    n = arr.size
    print('=== %s ===' % path)
    print('  size: %d x %d cells = %.2f x %.2f m (res %.3f)'
          % (w, h, w * res, h * res, res))
    print('  unknown %.1f%%  free %.1f%%  occupied %.2f%%'
          % (100.0 * unk.sum() / n, 100.0 * free.sum() / n, 100.0 * occ.sum() / n))

    lab, _ = ndimage.label(free, structure=np.ones((3, 3)))
    sizes = np.bincount(lab.ravel())[1:]
    big = sizes[sizes >= 50]
    print('  free components: %d (>=50 cells: %d), largest free component = %.1f%% of free'
          % (len(sizes), len(big),
             100.0 * (sizes.max() if len(sizes) else 0) / max(1, free.sum())))

    ys, xs = np.nonzero(occ)
    pts = np.stack([xs * res, (h - 1 - ys) * res], axis=1)
    if len(pts) < 100:
        print('  too few occupied cells')
        return
    walls = detect_walls(pts, res, min_peak=18)
    walls.sort(key=lambda x: -x['length'])
    print('  walls (len>0.8 m), best %d:' % min(8, len(walls)))
    long_clean = 0
    for x in walls[:8]:
        tag = ''
        if x['length'] > 1.5 and x['cont'] > 0.8 and x['flat'] < 0.04:
            tag = '  <-- clean'
            long_clean += 1
        print('    len=%5.2f m  theta=%6.1f  flatness=%.3f m  continuity=%.2f  inliers=%4d%s'
              % (x['length'], x['theta'], x['flat'], x['cont'], x['n'], tag))
    total_len = sum(x['length'] for x in walls)
    print('  total wall length: %.1f m  |  long clean walls (>1.5m, cont>0.8, flat<4cm): %d'
          % (total_len, long_clean))

    # double walls
    doub = []
    for i in range(len(walls)):
        for j in range(i + 1, len(walls)):
            a, b = walls[i], walls[j]
            if a['length'] < 0.8 or b['length'] < 0.8:
                continue
            dth = abs((a['theta'] - b['theta'] + 90) % 180 - 90)
            if dth > 3.0:
                continue
            da = a['center'] @ a['normal']
            db = b['center'] @ a['normal']
            off = abs(da - db)
            if not (0.04 <= off <= 0.20):
                continue
            # projection overlap
            pa = np.array([a['center'] @ a['dir'] - a['length'] / 2,
                           a['center'] @ a['dir'] + a['length'] / 2])
            pb = np.array([b['center'] @ a['dir'] - b['length'] / 2,
                           b['center'] @ a['dir'] + b['length'] / 2])
            ov = min(pa[1], pb[1]) - max(pa[0], pb[0])
            if ov > 1.0:
                doub.append((off, ov, a['length'], b['length']))
    print('  double walls (parallel 4-20 cm apart, overlap>1 m): %d' % len(doub))
    for d in doub[:6]:
        print('    offset=%.3f m overlap=%.2f m (len %.1f / %.1f)' % d)
    print()


if __name__ == '__main__':
    for p in sys.argv[1:]:
        try:
            analyze(p)
        except Exception as e:  # noqa: BLE001
            print('ERROR %s: %s' % (p, e))
