#!/usr/bin/env python3
"""Clean a slam_toolbox trinary PGM map for navigation use.

Operations (all conservative, map frame / origin / resolution unchanged):
  1. remove isolated occupied blobs (connected components <= N cells) -> set free
  2. close 1-cell gaps in walls (morphological closing on occupied) -> set occupied
  3. optionally fill tiny unknown holes fully enclosed by free space -> free

Usage: python3 map_clean.py in.pgm out.pgm [--min-blob 3]
"""
import sys

import numpy as np
from scipy import ndimage

sys.path.insert(0, '.')
from map_analyze import read_pgm  # noqa: E402


def write_pgm(path, arr):
    h, w = arr.shape
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(arr.astype(np.uint8).tobytes())


def clean(arr, min_blob=3, close_gaps=True, fill_holes=2):
    out = arr.copy()
    occ = out <= 10
    free = out >= 250
    unk = ~occ & ~free

    # 1) remove tiny occupied blobs
    lab, n = ndimage.label(occ, structure=np.ones((3, 3)))
    sizes = np.bincount(lab.ravel())
    remove = np.zeros(n + 1, dtype=bool)
    for i in range(1, n + 1):
        if sizes[i] <= min_blob:
            remove[i] = True
    removed = remove[lab]
    out[removed] = 254
    occ = out <= 10
    n_removed = int(removed.sum())

    # 2) close 1-cell gaps between wall cells (keep free/unknown untouched in gaps
    #    only where both sides are occupied)
    n_closed = 0
    if close_gaps:
        st = np.ones((3, 3), dtype=bool)
        closed = ndimage.binary_closing(occ, structure=st, iterations=1)
        add = closed & ~occ & ~free
        out[add] = 0
        n_closed = int(add.sum())

    # 3) fill small unknown holes enclosed by free space
    n_filled = 0
    if fill_holes > 0:
        unk2 = out == 205
        lab2, n2 = ndimage.label(unk2, structure=np.ones((3, 3)))
        sizes2 = np.bincount(lab2.ravel())
        for i in range(1, n2 + 1):
            if sizes2[i] <= fill_holes:
                # only fill if surrounded by free (not on map border)
                ys, xs = np.nonzero(lab2 == i)
                if ys.min() == 0 or xs.min() == 0 or ys.max() == out.shape[0] - 1 \
                        or xs.max() == out.shape[1] - 1:
                    continue
                neigh = out[max(0, ys.min() - 1):ys.max() + 2,
                            max(0, xs.min() - 1):xs.max() + 2]
                if (neigh == 254).sum() > (neigh == 205).sum():
                    out[ys, xs] = 254
                    n_filled += len(ys)
    return out, dict(removed_specks=n_removed, closed_cells=n_closed,
                     filled_unknown=n_filled)


def main():
    src = sys.argv[1]
    dst = sys.argv[2]
    min_blob = 3
    for i, a in enumerate(sys.argv):
        if a == '--min-blob' and i + 1 < len(sys.argv):
            min_blob = int(sys.argv[i + 1])
    arr = read_pgm(src)
    out, info = clean(arr, min_blob=min_blob)
    write_pgm(dst, out)
    for k, v in info.items():
        print('  %-16s %d' % (k, v))
    # summary
    for name, a in (('before', arr), ('after ', out)):
        occ = (a <= 10).sum()
        free = (a >= 250).sum()
        unk = a.size - occ - free
        print('  %s: unknown %.1f%%  free %.1f%%  occupied %.2f%%'
              % (name, 100 * unk / a.size, 100 * free / a.size, 100 * occ / a.size))
    print('  saved %s' % dst)


if __name__ == '__main__':
    main()
