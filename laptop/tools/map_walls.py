#!/usr/bin/env python3
"""Perimeter-wall quality check for trinary slam_toolbox maps.

For a roughly rectangular room, take the four sides of the free-space bounding
box, fit a line through the occupied cells forming each side and report:
  length, angle, flatness (residual std), thickness (perpendicular spread),
  continuity (fraction of 10 cm bins covered).
A drifted / ghosted map shows doubled lines (large thickness) or high residual.
"""
import sys

import numpy as np
from scipy import ndimage

sys.path.insert(0, '.')
from map_analyze import read_pgm  # noqa: E402


def fit(pts):
    c = pts.mean(axis=0)
    q = pts - c
    cov = q.T @ q / max(1, len(q))
    evals, evecs = np.linalg.eigh(cov)
    dirv = evecs[:, np.argmax(evals)]
    normal = np.array([-dirv[1], dirv[0]])
    proj = q @ dirv
    perp = q @ normal
    length = proj.max() - proj.min()
    nb = max(1, int(length / 0.10))
    hist, _ = np.histogram(proj, bins=nb, range=(proj.min(), proj.max()))
    cont = (hist > 0).sum() / nb
    return dict(length=float(length), flat=float(np.std(perp)),
                thick=float(np.percentile(perp, 95) - np.percentile(perp, 5)),
                cont=float(cont), angle=float(np.degrees(np.arctan2(dirv[1], dirv[0])) % 180))


def analyze(path, res=0.05, band=0.3):
    arr = read_pgm(path)
    h, w = arr.shape
    free = arr >= 250
    occ = arr <= 10
    unk = ~free & ~occ
    n = arr.size
    print('=== %s ===' % path)
    print('  %.2f x %.2f m | unknown %.1f%%  free %.1f%%  occupied %.2f%%'
          % (w * res, h * res, 100 * unk.sum() / n, 100 * free.sum() / n,
             100 * occ.sum() / n))
    lab, _ = ndimage.label(free, structure=np.ones((3, 3)))
    sizes = np.bincount(lab.ravel())[1:]
    print('  free components %d | largest %.1f%% of free'
          % (len(sizes), 100 * (sizes.max() if len(sizes) else 0) / max(1, free.sum())))
    # occupied component noise
    olab, on = ndimage.label(occ, structure=np.ones((3, 3)))
    osz = np.bincount(olab.ravel())[1:]
    osz = osz[osz > 0]
    if len(osz):
        specks = (osz <= 3).sum()
        print('  occupied components %d | specks(<=3 cells) %d | median size %.0f cells'
              % (len(osz), specks, np.median(osz)))
    ys, xs = np.nonzero(free)
    x0, x1 = xs.min() * res, xs.max() * res
    y0, y1 = (h - 1 - ys.max()) * res, (h - 1 - ys.min()) * res
    print('  free bbox: x %.2f..%.2f  y %.2f..%.2f' % (x0, x1, y0, y1))
    oy, ox = np.nonzero(occ)
    pts = np.stack([ox * res, (h - 1 - oy) * res], axis=1)
    # thin structures only (walls), local thickness <= 0.15 m
    dist = ndimage.distance_transform_edt(occ)
    thin = occ & ((2.0 * dist * res) <= 0.15)
    pts_thin = np.stack([np.nonzero(thin)[1] * res,
                         (h - 1 - np.nonzero(thin)[0]) * res], axis=1)
    print('  occupied cells %d | thin(wall-like) %d (%.0f%%)'
          % (len(pts), len(pts_thin), 100.0 * len(pts_thin) / max(1, len(pts))))
    sides = {
        'left  (x~x0)': pts[(pts[:, 0] < x0 + band)],
        'right (x~x1)': pts[(pts[:, 0] > x1 - band)],
        'bottom(y~y0)': pts[(pts[:, 1] < y0 + band)],
        'top   (y~y1)': pts[(pts[:, 1] > y1 - band)],
    }
    print('  perimeter walls (band %.1f m, thin cells only):' % band)
    thin_sides = {
        'left  (x~x0)': pts_thin[(pts_thin[:, 0] < x0 + band)],
        'right (x~x1)': pts_thin[(pts_thin[:, 0] > x1 - band)],
        'bottom(y~y0)': pts_thin[(pts_thin[:, 1] < y0 + band)],
        'top   (y~y1)': pts_thin[(pts_thin[:, 1] > y1 - band)],
    }
    for name, p in thin_sides.items():
        if len(p) < 30:
            print('    %-13s too few cells (%d)' % (name, len(p)))
            continue
        f = fit(p)
        print('    %-13s len=%5.2f m  angle=%6.1f  flatness=%.3f m  thickness=%.3f m  cont=%.2f  cells=%d'
              % (name, f['length'], f['angle'], f['flat'], f['thick'], f['cont'], len(p)))
    print()


if __name__ == '__main__':
    for p in sys.argv[1:]:
        try:
            analyze(p)
        except Exception as e:  # noqa: BLE001
            print('ERROR %s: %s' % (p, e))
