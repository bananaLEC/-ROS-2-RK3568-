#!/usr/bin/env python3
"""Offline global scan-to-map localization (reference pose for AMCL accuracy checks).

Loads a trinary PGM map and a recorded /scan capture and brute-force correlates
the scan against a blurred occupancy likelihood field over (x, y, yaw).

Output pose is in the map frame == what map_server publishes.

Usage: python3 global_localize.py <map.pgm> <origin_x> <origin_y> <res> <scans.bin> [scan_index]
"""
import math
import struct
import sys

import numpy as np
from scipy import ndimage

sys.path.insert(0, '.')
from map_analyze import read_pgm  # noqa: E402


def load_scan(path, idx=-1):
    with open(path, 'rb') as f:
        n = struct.unpack('<i', f.read(4))[0]
        scans = []
        for _ in range(n):
            st, amin, ainc, arr = struct.unpack('<dddd', f.read(32))
            m = struct.unpack('<i', f.read(4))[0]
            r = np.array(struct.unpack('<%df' % m, f.read(4 * m)))
            scans.append((st, amin, ainc, r))
    return scans[idx]


def scan_points(scan, rmin=0.25, rmax=11.0, n_out=130):
    st, amin, ainc, r = scan
    ang = amin + ainc * np.arange(len(r))
    ok = np.isfinite(r) & (r > rmin) & (r < rmax)
    p = np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])], axis=1)
    if len(p) > n_out:
        p = p[np.linspace(0, len(p) - 1, n_out).astype(int)]
    return p


def score_pose(field, H, ox, oy, res, pts, X, Y, yaw):
    """field[rows, cols]; rows grow downward (image convention)."""
    c, s = math.cos(yaw), math.sin(yaw)
    qx = c * pts[:, 0] - s * pts[:, 1]
    qy = s * pts[:, 0] + c * pts[:, 1]
    cols = (X - ox + qx) / res
    rows = (H - 1) - (Y - oy + qy) / res
    ci = cols.astype(np.int32)
    ri = rows.astype(np.int32)
    valid = (ci >= 0) & (ci < field.shape[1]) & (ri >= 0) & (ri < H)
    ci = np.clip(ci, 0, field.shape[1] - 1)
    ri = np.clip(ri, 0, H - 1)
    return float(np.where(valid, field[ri, ci], 0.0).mean())


def main():
    map_pgm, ox, oy, res, scan_bin = sys.argv[1], float(sys.argv[2]), \
        float(sys.argv[3]), float(sys.argv[4]), sys.argv[5]
    idx = int(sys.argv[6]) if len(sys.argv) > 6 else -1
    arr = read_pgm(map_pgm)
    H, W = arr.shape
    occ = arr <= 10
    field = ndimage.gaussian_filter(occ.astype(np.float32), sigma=1.2)
    scan = load_scan(scan_bin, idx)
    pts = scan_points(scan)
    print('map %dx%d res=%.3f origin=(%.3f,%.3f)' % (W, H, res, ox, oy))
    print('scan points: %d' % len(pts))

    yaws = np.deg2rad(np.arange(0, 360, 3.0))
    step = 0.15
    nX = int(round((W - 1) * res / step))
    nY = int(round((H - 1) * res / step))
    Xg = ox + np.arange(nX) * step
    Yg = oy + np.arange(nY) * step
    XX, YY = np.meshgrid(Xg, Yg)          # (nY, nX)
    best = []
    for yaw in yaws:
        c, s = math.cos(yaw), math.sin(yaw)
        qx = c * pts[:, 0] - s * pts[:, 1]
        qy = s * pts[:, 0] + c * pts[:, 1]
        cols = (XX[None, :, :] - ox + qx[:, None, None]) / res
        rows = (H - 1) - (YY[None, :, :] - oy + qy[:, None, None]) / res
        ci = cols.astype(np.int32)
        ri = rows.astype(np.int32)
        valid = (ci >= 0) & (ci < W) & (ri >= 0) & (ri < H)
        ci = np.clip(ci, 0, W - 1)
        ri = np.clip(ri, 0, H - 1)
        sc = np.where(valid, field[ri, ci], 0.0).mean(axis=0)
        k = np.unravel_index(np.argmax(sc), sc.shape)
        best.append((float(sc[k]), yaw, float(Xg[k[1]]), float(Yg[k[0]])))
    best.sort(reverse=True)
    seen = []
    for sc, yaw, X, Y in best:
        if all(math.hypot(X - a, Y - b) > 0.6 or
               abs((yaw - c + math.pi) % (2 * math.pi) - math.pi) > 0.2
               for _, a, b, c in seen):
            seen.append((sc, X, Y, yaw))
        if len(seen) >= 5:
            break
    print('\ncoarse top candidates:')
    for sc, X, Y, yaw in seen:
        print('  score=%.3f  x=%7.2f y=%7.2f yaw=%6.1f' % (sc, X, Y, math.degrees(yaw)))

    print('\nrefined:')
    out = []
    for sc0, X0, Y0, yaw0 in seen:
        bestr = (sc0, X0, Y0, yaw0)
        for dyaw in np.arange(-0.06, 0.061, 0.01):
            yaw = yaw0 + dyaw
            c, s = math.cos(yaw), math.sin(yaw)
            qx = c * pts[:, 0] - s * pts[:, 1]
            qy = s * pts[:, 0] + c * pts[:, 1]
            for dX in np.arange(-0.3, 0.301, 0.025):
                for dY in np.arange(-0.3, 0.301, 0.025):
                    sc = score_pose(field, H, ox, oy, res, pts, X0 + dX, Y0 + dY, yaw)
                    if sc > bestr[0]:
                        bestr = (sc, X0 + dX, Y0 + dY, yaw)
        out.append(bestr)
        print('  score=%.3f  x=%7.3f y=%7.3f yaw=%7.2f deg'
              % (bestr[0], bestr[1], bestr[2], math.degrees(bestr[3])))
    out.sort(reverse=True)
    sc, X, Y, yaw = out[0]
    print('\nBEST: x=%.3f y=%.3f yaw=%.4f rad (%.2f deg) score=%.3f'
          % (X, Y, yaw, math.degrees(yaw), sc))
    if len(out) > 1:
        print('second: x=%.3f y=%.3f yaw=%.2f score=%.3f'
              % (out[1][1], out[1][2], math.degrees(out[1][3]), out[1][0]))


if __name__ == '__main__':
    main()
