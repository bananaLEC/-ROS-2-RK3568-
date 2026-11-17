#!/usr/bin/env python3
"""Scan-to-scan ICP: measure the robot's REAL displacement during the drive test
and compare it with the odometry displacement."""
import math
import struct
import sys

import numpy as np


def load_scans(path):
    scans = []
    with open(path, 'rb') as f:
        n = struct.unpack('<i', f.read(4))[0]
        for _ in range(n):
            t, amin, ainc = struct.unpack('<ddd', f.read(24))
            m = struct.unpack('<i', f.read(4))[0]
            r = struct.unpack('<%df' % m, f.read(4 * m))
            scans.append((t, amin, ainc, np.array(r, dtype=np.float64)))
    return scans


def to_points(scan, rmin=0.20, rmax=9.0):
    t, amin, ainc, r = scan
    ang = amin + ainc * np.arange(len(r))
    ok = np.isfinite(r) & (r > rmin) & (r < rmax)
    return np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])], axis=1)


def grid_of(points, res=0.05, half=8.0):
    n = int(2 * half / res)
    g = np.zeros((n, n), dtype=np.uint8)
    ix = ((points[:, 0] + half) / res).astype(int)
    iy = ((points[:, 1] + half) / res).astype(int)
    ok = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
    g[iy[ok], ix[ok]] = 1
    # dilate one cell
    d = g.copy()
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            d |= np.roll(np.roll(g, dy, axis=0), dx, axis=1)
    return d, res, half


def score(A, Bg, res, half, dx, dy, dyaw_deg):
    c, s = math.cos(math.radians(dyaw_deg)), math.sin(math.radians(dyaw_deg))
    # rotate A by -dyaw (A expressed in B-ish frame), then shift by -d
    x = c * A[:, 0] + s * A[:, 1]
    y = -s * A[:, 0] + c * A[:, 1]
    ix = ((x - dx + half) / res).astype(int)
    iy = ((y - dy + half) / res).astype(int)
    n = Bg.shape[0]
    ok = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
    if ok.sum() == 0:
        return 0.0
    return float(Bg[iy[ok], ix[ok]].mean())


def main():
    scans = load_scans(sys.argv[1] if len(sys.argv) > 1 else 'gt_scans.bin')
    t0 = scans[0][0]
    # scan A: ~1.5 s after start (idle-before), scan B: ~9.5 s (idle-after)
    A = min(scans, key=lambda s: abs(s[0] - t0 - 1.5))
    B = min(scans, key=lambda s: abs(s[0] - t0 - 9.5))
    print('scanA t=%.2f n=%d   scanB t=%.2f n=%d' % (A[0] - t0, len(A[3]),
                                                     B[0] - t0, len(B[3])))
    pa = to_points(A)
    pb = to_points(B)
    print('points A=%d B=%d' % (len(pa), len(pb)))
    # deterministic subsample for speed
    idx = np.linspace(0, len(pa) - 1, 350).astype(int)
    A350 = pa[idx]
    Bg, res, half = grid_of(pb)

    best = None
    # coarse search: dx,dy in [-0.9, 0.9] step 0.03 ; yaw in [-3,3] step 0.5
    for dyaw in np.arange(-3.0, 3.01, 0.5):
        for dx in np.arange(-0.9, 0.901, 0.03):
            for dy in np.arange(-0.9, 0.901, 0.03):
                sc = score(A350, Bg, res, half, dx, dy, dyaw)
                if best is None or sc > best[0]:
                    best = (sc, dx, dy, dyaw)
    print('coarse best: score=%.3f dx=%.3f dy=%.3f dyaw=%.2f' % best)
    # fine
    s0, dx0, dy0, dy0a = best
    for dyaw in np.arange(dy0a - 0.4, dy0a + 0.41, 0.1):
        for dx in np.arange(dx0 - 0.03, dx0 + 0.031, 0.005):
            for dy in np.arange(dy0 - 0.03, dy0 + 0.031, 0.005):
                sc = score(A350, Bg, res, half, dx, dy, dyaw)
                if sc > best[0]:
                    best = (sc, dx, dy, dyaw)
    s0, dx, dy, dyaw = best
    print('fine best  : score=%.3f dx=%.3f dy=%.3f dyaw=%.2f' % best)
    print('ICP displacement in laser frame: %.3f m (dx=%.3f dy=%.3f, yaw %.2f deg)'
          % (math.hypot(dx, dy), dx, dy, dyaw))
    return 0


if __name__ == '__main__':
    sys.exit(main())
