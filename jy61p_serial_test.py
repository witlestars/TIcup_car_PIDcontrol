# -*- coding: utf-8 -*-
"""============================================================
JY61P 串口直读测试 - 通过 CH340 (COM7) 和 IMU 通信
------------------------------------------------------------
1. 读 2 秒数据流, 确认 IMU 在发数据 (帧头 0x55)
2. 解析加速度/角速度/角度数据包
3. 尝试通过串口读寄存器 (发查询帧)

JY61P 串口协议:
  数据帧: 0x55 <type> <8 bytes data> <sum>
    type 0x51: 加速度
    type 0x52: 角速度
    type 0x53: 角度 (Roll/Pitch/Yaw)
  写寄存器: 0xFF 0xAA <reg> <dataL> <dataH>
============================================================"""
import serial
import time
import sys

PORT = "COM7"
BAUD = 9600   # JY61P 默认 9600

def parse_frame(frame):
    """解析一帧数据"""
    if len(frame) < 11 or frame[0] != 0x55:
        return None
    ftype = frame[1]
    # 校验和 (低 8 位)
    s = sum(frame[0:10]) & 0xFF
    if s != frame[10]:
        return f"BAD SUM (calc={s:02X} got={frame[10]:02X})"

    if ftype == 0x51:
        # 加速度
        ax = (frame[3] << 8 | frame[2]) / 32768.0 * 16
        ay = (frame[5] << 8 | frame[4]) / 32768.0 * 16
        az = (frame[7] << 8 | frame[6]) / 32768.0 * 16
        return f"ACC  ax={ax:.2f} ay={ay:.2f} az={az:.2f} g"
    elif ftype == 0x52:
        # 角速度
        wx = (frame[3] << 8 | frame[2]) / 32768.0 * 2000
        wy = (frame[5] << 8 | frame[4]) / 32768.0 * 2000
        wz = (frame[7] << 8 | frame[6]) / 32768.0 * 2000
        return f"GYRO wx={wx:.1f} wy={wy:.1f} wz={wz:.1f} deg/s"
    elif ftype == 0x53:
        # 角度
        roll  = (frame[3] << 8 | frame[2]) / 32768.0 * 180
        pitch = (frame[5] << 8 | frame[4]) / 32768.0 * 180
        yaw   = (frame[7] << 8 | frame[6]) / 32768.0 * 180
        return f"ANG  R={roll:.2f} P={pitch:.2f} Y={yaw:.2f} deg"
    else:
        return f"TYPE 0x{ftype:02X} raw: {frame[2:10].hex()}"


def main():
    print(f"打开 {PORT} @ {BAUD} ...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
    except Exception as e:
        print(f"打开失败: {e}")
        sys.exit(1)

    # 清空缓冲区
    ser.reset_input_buffer()
    time.sleep(0.2)

    print("\n=== 第1步: 读 2 秒数据流 ===")
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < 2.0:
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))
        else:
            time.sleep(0.02)

    print(f"收到 {len(buf)} 字节")
    if len(buf) == 0:
        print("!!! 没收到任何数据: 检查 CH340 的 RX 是否接到 IMU 的 TX")
        ser.close()
        return

    # 显示前 32 字节十六进制
    print(f"前 32 字节: {buf[:32].hex(' ')}")

    # 找 0x55 帧头解析
    frames_parsed = 0
    i = 0
    while i < len(buf) - 10 and frames_parsed < 5:
        if buf[i] == 0x55:
            frame = buf[i:i+11]
            result = parse_frame(frame)
            if result:
                print(f"  [{i}] {result}")
                frames_parsed += 1
                i += 11
                continue
        i += 1

    if frames_parsed == 0:
        print("!!! 没解析到有效帧: 可能波特率不对, 或校验失败")
        print(f"原始数据 (前 100 字节): {buf[:100].hex(' ')}")
    else:
        print(f"\n>>> IMU 数据流正常! 共解析 {frames_parsed} 帧")

    print("\n=== 第2步: 尝试读版本/配置寄存器 ===")
    # 维特模块有些支持主动返回寄存器值, 但需要上位机协议
    # 这里只做个简单测试: 发 0xFF 0xAA 0x?? 读 (JY61P 串口读寄存器需要特定协议)
    # 维特标准: 写 0xFF 0xAA <reg> <0x00 0x00> 会触发模块返回该寄存器值
    # 但这会改写寄存器为 0, 危险, 不做

    # 改成只统计 2 秒内数据包类型分布
    type_count = {0x51: 0, 0x52: 0, 0x53: 0, 0x54: 0, 0x55: 0, 0x56: 0, 0x57: 0, 0x58: 0, 0x59: 0, 0x5A: 0}
    ser.reset_input_buffer()
    t0 = time.time()
    total = 0
    while time.time() - t0 < 2.0:
        n = ser.in_waiting
        if n:
            data = ser.read(n)
            total += len(data)
            for j in range(len(data) - 1):
                if data[j] == 0x55 and data[j+1] in type_count:
                    type_count[data[j+1]] += 1
        else:
            time.sleep(0.02)

    print(f"\n2 秒内收到 {total} 字节, 数据包类型分布:")
    type_names = {0x51:"加速度", 0x52:"角速度", 0x53:"角度", 0x54:"磁场",
                  0x55:"端口", 0x56:"气压", 0x57:"经纬", 0x58:"高度", 0x59:"GPS", 0x5A:"版本"}
    for t, c in type_count.items():
        if c > 0:
            print(f"  0x{t:02X} {type_names.get(t, '?')}: {c} 包")

    print("\n=== 结论 ===")
    if total > 0:
        print("IMU 串口通信正常, 模块活着")
        print("问题确认在 I2C 侧: IMU 的 I2C 输出可能没启用")
        print("下一步: 用维特上位机配置启用 I2C 输出")
    else:
        print("串口没数据, 可能 RX/TX 接反或波特率不对")

    ser.close()


if __name__ == "__main__":
    main()
