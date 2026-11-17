#!/usr/bin/env python3
"""Convert an XWD dump (from xwd -root) to PNG (minimal parser, TrueColor only)."""
import struct
import sys

import numpy as np
from PIL import Image


def convert(src, dst):
    with open(src, 'rb') as f:
        data = f.read()
    hdr = struct.unpack('>25I', data[:100])
    (header_size, file_version, pixmap_format, pixmap_depth,
     width, height, xoffset, byte_order, bitmap_unit, bitmap_bit_order,
     bitmap_pad, bits_per_pixel, bytes_per_line, visual_class,
     red_mask, green_mask, blue_mask, bits_per_rgb, colormap_entries,
     ncolors, window_width, window_height, window_x, window_y,
     window_bdrwidth) = hdr
    off = header_size
    # skip colormap
    off += ncolors * 12
    bpl = bytes_per_line
    raw = data[off:off + bpl * height]
    if bits_per_pixel not in (24, 32):
        raise SystemExit('unsupported bits_per_pixel=%d' % bits_per_pixel)
    if byte_order == 0:      # LSBFirst: B,G,R,(X)
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(height, bpl)
        img = arr[:, :width * 4].reshape(height, width, 4)[:, :, :3]
        img = img[:, :, ::-1]  # BGR -> RGB
    else:                    # MSBFirst: X,R,G,B
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(height, bpl)
        img = arr[:, :width * 4].reshape(height, width, 4)[:, :, 1:4]
    Image.fromarray(np.ascontiguousarray(img)).save(dst)
    print('converted %s -> %s (%dx%d)' % (src, dst, width, height))


if __name__ == '__main__':
    convert(sys.argv[1], sys.argv[2])
