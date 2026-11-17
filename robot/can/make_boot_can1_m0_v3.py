#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""启用 CAN1_M0 —— 同时补丁 FIT fdt 组件 和 resource 组件里的 DTB

实测发现 U-Boot 有时用 FIT 的 fdt，有时回退到 resource 里的 DTB，
所以两处都改，保证无论走哪条路都能生效。

改动:
  can@fe580000  status: disabled -> okay
  can@fe580000  pinctrl-0: 改为 can1m0 (GPIO1_A0/A1 = 排针 pin3/5)
  i2c@fe5c0000  status: okay -> disabled   (腾出上述引脚)

用法: sudo python3 make_boot_can1_m0_v3.py
输出: ~/can/boot_can1_m0_v3.img
"""
import hashlib
import os
import re
import struct
import subprocess

HOME = '/home/kickpi'
WORK = '/tmp/canv3'
OUT = os.path.join(HOME, 'can/boot_can1_m0_v3.img')
BACKUP = os.path.join(HOME, 'can/boot_backup_p3.img')
FDT_MAGIC = b'\xd0\x0d\xfe\xed'
RES_SIZE = 1448960


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True)


def patch_dtb(blob, tag):
    """对单个 DTB 打补丁，返回等长的新 DTB（或 None 表示无需/失败）"""
    orig_size = len(blob)
    open('t.dtb', 'wb').write(blob)
    r = run(['dtc', '-I', 'dtb', '-O', 'dts', 't.dtb', '-o', 't.dts'])
    if r.returncode != 0:
        print('    %s: 解编译失败' % tag)
        return None
    dts = open('t.dts', encoding='utf-8').read()

    m = re.search(r'\tcan@fe580000 \{(.*?)\n\t\};', dts, re.S)
    if not m:
        print('    %s: 没有 can@fe580000，跳过' % tag)
        return None
    blk = m.group(1)
    if 'status = "okay"' in blk and '0x23e' in blk:
        print('    %s: 已经是目标状态' % tag)
        return None

    pc = re.search(r'pinctrl-0 = <([^>]*)>;', blk)
    newblk = blk.replace('status = "disabled";', 'status = "okay";', 1)
    if pc:
        newblk = newblk.replace('pinctrl-0 = <%s>;' % pc.group(1),
                                'pinctrl-0 = <0x23e>;', 1)
    dts = dts[:m.start(1)] + newblk + dts[m.end(1):]

    # i2c3 关掉
    m2 = re.search(r'\ti2c@fe5c0000 \{(.*?)\n\t\};', dts, re.S)
    if m2 and 'status = "okay"' in m2.group(1):
        nb = m2.group(1).replace('status = "okay";', 'status = "disabled";', 1)
        dts = dts[:m2.start(1)] + nb + dts[m2.end(1):]

    open('t_new.dts', 'w', encoding='utf-8').write(dts)
    r = run(['dtc', '-I', 'dts', '-O', 'dtb', 't_new.dts', '-o', 't_new.dtb'])
    if r.returncode != 0 or not os.path.isfile('t_new.dtb'):
        print('    %s: 重编译失败' % tag)
        return None
    new = open('t_new.dtb', 'rb').read()
    ntot = struct.unpack('>I', new[4:8])[0]
    if ntot > orig_size:
        print('    %s: 新 DTB 更大 (%d > %d)，跳过' % (tag, ntot, orig_size))
        return None
    if ntot < orig_size:
        new = bytearray(new + b'\x00' * (orig_size - ntot))
        struct.pack_into('>I', new, 4, orig_size)
        new = bytes(new)
    print('    %s: 已打补丁（原 %d -> 新 %d，补零到 %d）' % (tag, ntot, ntot, orig_size))
    return new


def main():
    os.makedirs(WORK, exist_ok=True)
    os.chdir(WORK)
    os.chmod(WORK, 0o777)

    img = bytearray(open(BACKUP, 'rb').read())
    print('[1/5] boot 镜像 %d 字节' % len(img))

    # 定位所有 k1Mini DTB
    blobs = []
    i = 0
    while True:
        i = img.find(FDT_MAGIC, i)
        if i < 0:
            break
        if i + 8 <= len(img):
            tot = struct.unpack('>I', img[i + 4:i + 8])[0]
            if 1000 < tot < 4 * 1024 * 1024 and i + tot <= len(img) and b'k1Mini' in img[i:i + tot]:
                blobs.append((i, tot))
        i += 4
    print('[2/5] 找到 %d 个 k1Mini DTB: %s' % (len(blobs), ['0x%X/%d' % b for b in blobs]))

    # 记录旧的组件哈希
    fdt_off = 0x800
    fdt_size = struct.unpack('>I', img[fdt_off + 4:fdt_off + 8])[0]
    old_fdt_hash = hashlib.sha256(img[fdt_off:fdt_off + fdt_size]).digest()

    first_dtb = img.find(FDT_MAGIC, 0x02700000)
    res_off = img.rfind(b'RSCE', first_dtb - 0x8000, first_dtb)
    print('    FIT fdt: 0x%X (%d)   resource: 0x%X' % (fdt_off, fdt_size, res_off))
    old_res_hash = hashlib.sha256(img[res_off:res_off + RES_SIZE]).digest()

    # 逐个打补丁
    print('[3/5] 逐个打补丁...')
    n_patched = 0
    for off, tot in blobs:
        new = patch_dtb(bytes(img[off:off + tot]), '0x%X' % off)
        if new:
            img[off:off + tot] = new
            n_patched += 1
    print('    共修改 %d 个 DTB' % n_patched)

    if n_patched == 0:
        raise SystemExit('!! 没有 DTB 被修改，中止')

    # 更新两个组件哈希
    print('[4/5] 更新组件哈希...')
    new_fdt_hash = hashlib.sha256(img[fdt_off:fdt_off + fdt_size]).digest()
    new_res_hash = hashlib.sha256(img[res_off:res_off + RES_SIZE]).digest()
    cnt = 0
    for old, new in ((old_fdt_hash, new_fdt_hash), (old_res_hash, new_res_hash)):
        if old == new:
            continue
        idx = 0
        while True:
            p = img.find(old, idx, res_off)
            if p < 0:
                break
            img[p:p + 32] = new
            cnt += 1
            idx = p + 32
    print('    替换哈希 %d 处' % cnt)
    print('    fdt 新哈希: %s' % new_fdt_hash.hex())
    print('    res 新哈希: %s' % new_res_hash.hex())

    open(OUT, 'wb').write(img)
    print('[5/5] 已生成 %s' % OUT)

    # 自检
    print()
    print('=== 自检 ===')
    r = run(['dumpimage', '-l', OUT])
    print('  FIT: %s' % ('正常' if 'FIT description' in r.stdout else '异常'))
    data = open(OUT, 'rb').read()
    for off, tot in blobs:
        open('c.dtb', 'wb').write(data[off:off + tot])
        r = run(['dtc', '-I', 'dtb', '-O', 'dts', 'c.dtb'])
        if r.returncode != 0:
            print('  0x%X: 解析失败' % off)
            continue
        m = re.search(r'\tcan@fe580000 \{(.*?)\n\t\};', r.stdout, re.S)
        st = re.search(r'status = "(\w+)"', m.group(1)) if m else None
        pc = re.search(r'pinctrl-0 = <([^>]*)>', m.group(1)) if m else None
        m2 = re.search(r'\ti2c@fe5c0000 \{(.*?)\n\t\};', r.stdout, re.S)
        st2 = re.search(r'status = "(\w+)"', m2.group(1)) if m2 else None
        print('  0x%-10X can1=%-7s pinctrl=%-6s i2c3=%s'
              % (off, st.group(1) if st else '?', pc.group(1) if pc else '-',
                 st2.group(1) if st2 else '?'))


if __name__ == '__main__':
    main()
