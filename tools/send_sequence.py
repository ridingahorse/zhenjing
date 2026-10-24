#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
上位机测试脚本：运行一次，连续下发一批目标点，发完即退出。

序列（每两个点一个周期）：
    关激光靠近草根旁 (x, -y_off, laser=0)
    开激光切过根茎   (x, +y_off, laser=1)   -> 到达后固件自动关激光
下一周期的 x 依次平移，形成一条扫描带。

默认发送间隔为 0（背靠背），用于观察环形缓冲积压。
"""

import argparse
import functools
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("需要 pyserial: pip install pyserial")


def build_frame(x, y, laser):
    """AA 55 | XL XH | YL YH | LASER | XOR"""
    f = bytes([0xAA, 0x55]) + struct.pack('<hh', x, y) + bytes([1 if laser else 0])
    return f + bytes([functools.reduce(int.__xor__, f, 0)])


def clamp(v):
    return max(-32000, min(32000, int(v)))


def main():
    ap = argparse.ArgumentParser(description="振镜目标点批量下发")
    ap.add_argument('-p', '--port', default='COM5', help='串口名，如 COM3 或 /dev/ttyUSB0')
    ap.add_argument('-n', '--count', type=int, default=200, help='本次发送的目标点总数')
    ap.add_argument('-s', '--step', type=int, default=200, help='相邻草茎的 X 间距')
    ap.add_argument('-y', '--yoff', type=int, default=400, help='Y 方向偏移幅度')
    ap.add_argument('-t', '--interval', type=float, default=0.001, help='帧间延时(秒)，0=不等')
    ap.add_argument('-b', '--baud', type=int, default=115200)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(0.1)

    x0 = -2000
    t0 = time.perf_counter()
    sent = 0

    for i in range(args.count):
        j = i // 2
        x = clamp(x0 + j * args.step)

        if i % 2 == 0:
            frame = build_frame(x, -args.yoff, 0)   # 靠近
        else:
            frame = build_frame(x, +args.yoff, 1)   # 切割

        ser.write(frame)
        sent += 1

        if args.interval > 0:
            time.sleep(args.interval)

        if sent % 50 == 0:
            print(f"  已发送 {sent}/{args.count}")

    ser.flush()
    dt = time.perf_counter() - t0
    ser.close()

    print(f"完成: {sent} 帧, {sent * 8} 字节, 用时 {dt:.3f}s, "
          f"平均 {dt / sent * 1000:.3f} ms/帧")
    if dt > 0:
        print(f"发送速率约 {sent / dt:.0f} 点/s")
    print(f"理论执行时长约 {sent * 0.8:.0f} ms（每点约 800us）")


if __name__ == '__main__':
    main()
