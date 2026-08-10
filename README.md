# TIcup_car_PIDcontrol

> TI 杯平衡车 / 巡线车 PID 控制固件 — 基于 MSPM0G3507 LaunchPad

本项目是 TI 杯智能车竞赛练习与正赛的代码仓库，跑在 **MSPM0G3507**（TI LP-MSPM0G3507）上，使用 DriverLib + SysConfig 实现。当前主线在 `fix-i2c-hardware` 分支，正在做平衡 PID、电机角度反馈读取、视觉数据解析与 VOFA+ 调参。

---

## 功能概览

- **三环串级平衡 PID**：位置外环 → 速度中环 → 角度内环，含 ZDT 电机角度反馈与回零逻辑
- **灰度巡线**：离心 PD → 差速 → 驱动板速度指令；直道 / 弯道分段 PD 切换，丢线保持 250ms
- **航位推算（Odometry）**：编码器 + IMU yaw 融合，输出 `(x, y, θ)` 与累计里程
- **视觉通信协议**：接收 K230 视觉模块下发的小球位置 / 速度（115200bps，自定义 `$O...*CS` 帧）
- **IMU 解析**：JY61P 串口数据（Roll/Pitch/Yaw + 加速度 + 角速度）
- **VOFA+ 调参**：上位机实时改 PID 参数、收波形
- **OLED 显示**：里程、Yaw、PD 预设等运行信息
- **模式管理**：5 种底盘任务（`CAR1`~`CAR5`）+ 4 种平衡任务（`Balance_0`~`Balance_2`）

---

## 硬件平台

| 模块 | 型号 / 说明 |
|------|------------|
| 主控 | TI LP-MSPM0G3507（MSPM0G3507，Cortex-M0+） |
| IMU | JY61P（串口，115200bps） |
| 电机驱动 | ZDT X42S 驱动板（I2C/UART 双通道） |
| 电机 + 编码器 | 直流减速电机，轮径 65mm，减速比 40，磁环 11 线 |
| 视觉模块 | K230（UART 下发位置/速度） |
| 灰度 | 8 路灰度传感器阵列 |
| 显示 | 0.96" OLED（I2C） |
| 调参 | VOFA+ 上位机（UART） |

---

## 目录结构

```
TIcup/
├── main.c                     # 主程序 + 中断 ISR
├── BSP/
│   ├── balance.c/h            # 三环串级 PID 平衡控制
│   ├── balance_segmented.c/h  # 分段平衡方案
│   ├── track.c/h              # 灰度巡线 (离心 PD + 分段)
│   ├── imu.c/h                # JY61P IMU 解析
│   ├── odometry.c/h           # 编码器+IMU 融合航位推算
│   ├── vision_protocol.c/h    # K230 视觉协议解析
│   ├── vofa.c/h               # VOFA+ 上位机通信
│   ├── oled.c/h               # OLED 显示
│   ├── ZDT_X42S_Driver.c/h    # ZDT 电机驱动板驱动
│   ├── task.c/h               # 任务调度 + 模式管理
│   └── template/              # 早期模板代码 (I2C/电机/灰度/延时)
├── jy61p_serial_test.py       # IMU 串口测试脚本
├── save_params.py             # 参数保存脚本
└── empty.syscfg               # SysConfig 配置
```

---

## 核心算法

### 三环串级平衡 PID（[BSP/balance.c](BSP/balance.c)）

```
位置环 (g_pos_pid)  →  速度环 (g_vel_pid)  →  角度环 (g_angle_pid)  →  电机
   ↑ 小球位置mm         ↑ 小球速度mm/s         ↑ 电机角度°              ↑ PWM
   │ 视觉反馈            │ 视觉反馈              │ ZDT 反馈
```

- `PID_Calcula()` 通用 PID，含前馈 `Kff`、积分限幅、输出限幅
- `Balance_MotorFeedbackTask()` 在主循环中轮询电机角度反馈
- 角度限位 `[-35°, +50°]`，回零速度 15，零位容差 0.2°

### 灰度巡线（[BSP/track.c](BSP/track.c)）

- 离心值 `centroid` 范围 `-3.5~+3.5`（8 路灰度归一化）
- 直道用 `turn_p/turn_d`，弯道用 `curve_p/curve_d` 自动切换
- 可选 IMU 辅助转弯（`g_imu_assist=1` 时启用 yaw 闭环）
- 槽口型赛道：每圈计 2 个半圆弯，`g_corner_count` 跟踪圈数

### 航位推算（[BSP/odometry.c](BSP/odometry.c)）

- 10ms 周期读左右轮编码器脉冲
- 1 脉冲位移实测 ≈ 4.64mm（π×65/(40×11)×10 修正）
- IMU yaw 作为朝向，积分得 `(x, y, θ)`
- 坐标系：θ=0°→+x(东)，θ=90°→+y(北)，逆时针为正

---

## 编译与烧录

1. **工具链**：Code Composer Studio (CCS) + MSPM0 SDK + SysConfig
2. 打开 `.project` / `.ccsproject` 导入工程
3. `empty.syscfg` 配置外设引脚（UART×4、I2C、GPIO、定时器）
4. 编译生成 `.out`，通过 XDS-110 烧录

> 依赖 `.ccsproject`、`.cproject`、`.project`、`targetConfigs/MSPM0G3507.ccxml` 等工程文件。

---

## 分支说明

| 分支 | 说明 |
|------|------|
| `fix-i2c-hardware` | 当前主线，硬件 I2C 修复 + 平衡 PID 调试 |
| `main` | 稳定版本 |
| `26_main` | 2026 赛季主线（组织仓库 `kk-diansai-2026`） |
| `dev-26` | 2026 赛季开发分支 |
| `dev-softuart` | 软串口方案（PB8/PB10） |

---

## 调试工具

- **VOFA+**：上位机实时改 PID 参数、波形观测
- **[jy61p_serial_test.py](jy61p_serial_test.py)**：IMU 数据串口测试
- **[save_params.py](save_params.py)**：参数保存
- **ESP32 WiFi 透传桥**（配套仓库）：MSPM0 软串 ↔ TCP，无线调参

---

## 致谢

- 队友协作（`organization` remote: `kk-diansai-2026`）
- TI MSPM0 SDK + DriverLib
- VOFA+ 上位机
