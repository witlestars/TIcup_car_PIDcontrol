/**
 * @file    cmd.c
 * @brief   UART 命令解析 — 从 ESP32 接收并修改全局参数
 * 
/* 命令格式 (通过 WiFi→ESP32→UART_BLUETOOTH):
 *   b200    基础速度 = 200 (驱动板单位)
 *   p18     TURN_GAIN_P = 18
 *   d1.5    TURN_GAIN_D = 1.5
 *   o200    原地转弯速度 = 200 (直角弯PIVOT用)
 *   f500    直角弯前冲时间 = 500ms (可调)
 *   k1.5    驱动板 PID kp = 1.5 (发 $MPID 指令)
 *   i0.03   驱动板 PID ki = 0.03
 *   j0.1    驱动板 PID kd = 0.1
 *   s       急停
 *   g       恢复 (同时复位直角弯超时锁定)
 *   m0/m1/m3 巡线/空转/不倒翁(IMU yaw自稳, 切到m3时锁当前朝向, 手转车会自动回正)
 *   t200    空转目标速度
 *   E0/E1   IMU 解析开关 (改串口后不再与OLED互斥): E1=启用IMU解析, E0=暂停IMU解析
 *   L3      设置目标圈数 = 3 (替代原 LAP_DN 按钮, 范围1~9)
 *   e0/e1   切换 IMU 辅助转弯
 *   U       切换 UART_DEBUG 原始字节回显 (诊断 JY61P 串口, 开关型)
 *   ?       回传当前参数
 *   B       回显启动状态 (各模块 + 引脚 + 模式)
 */

#include "cmd.h"
#include "ti_msp_dl_config.h"
#include "track.h"
#include "motor.h"
#include "imu_uart.h"
#include "uart_bluetooth.h"
#include "imu.h"
#include "odometry.h"
#include "oled.h"
#include "IOI2C.h"
#include "bsp_motor_iic.h"
#include "motor_iic.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define CMD_BUF_SIZE 32

static char  cmd_buf[CMD_BUF_SIZE];
static uint8_t cmd_idx = 0;

uint8_t g_mode      = 0;     /* 0=巡线, 1=空转, 3=不倒翁(IMU yaw自稳) */
float   g_target_rpm = 200;  /* 空转目标速度 (驱动板单位) */
uint8_t g_running    = 0;    /* 上电默认停止, 发 'g' 启动, 's' 停止 */
uint8_t g_imu_uart_echo = 0;  /* 1=把 IMU 串口收到的原始字节回显到 PC */

/* 驱动板 PID 缓存 (cmd修改后发给驱动板) */
static float drv_kp = 0.8f;
static float drv_ki = 0.06f;
static float drv_kd = 0.5f;

/* ─── 文本回传: 通过 UART_BLUETOOTH 发文本给 ESP32 → 电脑 ───
 * 文本帧格式: 以 \n 结尾 (电脑端按行解析)
 * 与 JustFloat 二进制帧区分: JustFloat 帧尾 00 00 80 7F, 文本不会有这个序列
 */
void CMD_SendText(const char *text)
{
    const char *p = text;
    while (*p) {
        /* 带超时的 TX FIFO 等待, 防止 TX 没使能时死循环卡死主循环 */
        uint16_t timeout = 60000;
        while (DL_UART_Main_isTXFIFOFull(UART_BLUETOOTH_INST) && timeout--);
        if (timeout == 0) break;   /* TX FIFO 持续满, 放弃发送避免卡死 */
        DL_UART_Main_transmitDataBlocking(UART_BLUETOOTH_INST, (uint8_t)*p);
        p++;
    }
}

void CMD_Init(void)
{
    cmd_idx = 0;
    /* 清空 UART RX FIFO 里的上电残留垃圾 */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_BLUETOOTH_INST)) {
        (void)DL_UART_Main_receiveData(UART_BLUETOOTH_INST);
    }
    /* 注意: cmd_receiving 是 CMD_Poll 的 static 变量, 上电默认 0, 不用这里初始化 */
}

/**
 * @brief 简易 atof
 */
static float my_atof(const char *s)
{
    float val = 0, frac = 0;
    uint8_t dot = 0;
    int sign = 1;
    if (*s == '-') { sign = -1; s++; }
    while (*s) {
        if (*s == '.') { dot = 1; s++; continue; }
        if (*s < '0' || *s > '9') break;
        if (dot) { frac = frac * 10 + (*s - '0'); dot *= 10; }
        else val = val * 10 + (*s - '0');
        s++;
    }
    if (dot > 1) val += frac / dot;
    return sign * val;
}

/**
 * @brief 参数状态 → UART_BLUETOOTH → ESP32 → 电脑 (6通道 JustFloat)
 */
static void CMD_Report(void)
{
    float rpt[6] = { g_base_speed, g_turn_p, g_turn_d,
                     drv_kp, drv_ki, drv_kd };
    UART_Bluetooth_Send(rpt, 6);
}

/**
 * @brief 执行命令, 每条命令执行后通过 UART 回传文本确认
 */
static void CMD_Exec(void)
{
    char ack[256];   /* 加大到256, 避免 ? 命令的多行回传被截断 */

    if (cmd_idx == 0) return;
    cmd_buf[cmd_idx] = '\0';

    char c = cmd_buf[0];
    float v = my_atof(cmd_buf + 1);

    switch (c) {
    case 'b':
        g_base_speed  = v; g_base_rpm = v;
        snprintf(ack, sizeof(ack), "[MSPM0] base_speed=%.1f\n", g_base_speed);
        CMD_SendText(ack);
        break;
    case 'p':
        g_turn_p = v;
        snprintf(ack, sizeof(ack), "[MSPM0] turn_p=%.2f\n", g_turn_p);
        CMD_SendText(ack);
        break;
    case 'd':
        g_turn_d = v;
        snprintf(ack, sizeof(ack), "[MSPM0] turn_d=%.2f\n", g_turn_d);
        CMD_SendText(ack);
        break;
    case 'o':
        g_pivot_speed = v;
        snprintf(ack, sizeof(ack), "[MSPM0] pivot_speed=%.1f\n", g_pivot_speed);
        CMD_SendText(ack);
        break;
    case 'f':   /* 直角弯前冲时间 (ms) */
        g_corner_fwd_ms = v;
        snprintf(ack, sizeof(ack), "[MSPM0] corner_fwd_ms=%.0f\n", g_corner_fwd_ms);
        CMD_SendText(ack);
        break;

    /* k/i/j → 修改驱动板 PID 并发送指令 */
    case 'k':
        drv_kp = v;
        Motor_Set_PID(drv_kp, drv_ki, drv_kd);
        snprintf(ack, sizeof(ack), "[MSPM0] drv_kp=%.2f\n", drv_kp);
        CMD_SendText(ack);
        break;
    case 'i':
        drv_ki = v;
        Motor_Set_PID(drv_kp, drv_ki, drv_kd);
        snprintf(ack, sizeof(ack), "[MSPM0] drv_ki=%.3f\n", drv_ki);
        CMD_SendText(ack);
        break;
    case 'j':
        drv_kd = v;
        Motor_Set_PID(drv_kp, drv_ki, drv_kd);
        snprintf(ack, sizeof(ack), "[MSPM0] drv_kd=%.2f\n", drv_kd);
        CMD_SendText(ack);
        break;

    case 's': /* stop */
        g_running = 0;
        Motor_Stop();
        CMD_SendText("[MSPM0] STOPPED (g_running=0)\n");
        break;
    case 'g': /* go */
        g_running = 1;
        Track_Reset();   /* 复位圈数/弯数/锁定, 防止 LAPS DONE 残留 */
        CMD_SendText("[MSPM0] RUNNING (g_running=1)\n");
        break;
    case '?':
        snprintf(ack, sizeof(ack),
                 "[MSPM0] ? mode=%d run=%d base=%.1f p=%.2f d=%.2f pivot=%.1f fwd=%.0f target=%.1f\n"
                 "       yaw=%.2f target=%.2f err=%.2f assist=%d x=%.0f y=%.0f total=%.0f edge=%d\n"
                 "       imu=%d oled=%d use_imu=%d laps=%d/%d (1=present,0=not)\n",
                 g_mode, g_running, g_base_speed, g_turn_p, g_turn_d,
                 g_pivot_speed, g_corner_fwd_ms, g_target_rpm,
                 IMU_Get_Yaw_Cached(), g_yaw_target, g_yaw_err, g_imu_assist,
                 Odom_Get_X(), Odom_Get_Y(), Odom_Get_Total_Dist(), Odom_Get_Edge_Index(),
                 g_imu_present, g_oled_present, g_use_imu, g_current_lap, g_target_laps);
        CMD_SendText(ack);
        CMD_Report();   /* 同时发 JustFloat 帧 */
        break;
    case 'O':   /* OLED 开关: O0=断开(软件层禁用) O1=启用 */
        if (v < 0.5f) {
            g_oled_present = 0;
            CMD_SendText("[MSPM0] OLED disabled (g_oled_present=0)\n");
        } else {
            OLED_Init();
            if (g_oled_present) {
                OLED_Clear();
                OLED_PrintfAt(0, 0, "OLED ON");
                CMD_SendText("[MSPM0] OLED enabled & re-init OK\n");
            } else {
                CMD_SendText("[MSPM0] OLED init FAIL\n");
            }
        }
        break;
    case 'J':   /* IMU 串口诊断: 显示 UART_DEBUG (PA10/PA11) 是否收到 JY61P 数据
                 * IMU 已改串口, 原 I2C 探测无意义, 此命令读当前缓存角度并报告状态 */
        CMD_SendText("[MSPM0] === IMU UART Debug (PA10/PA11, 9600bps) ===\n");
        {
            float r, p, y;
            uint8_t ret = IMU_Read_RPY(&r, &p, &y);
            snprintf(ack, sizeof(ack),
                "[MSPM0] g_imu_present=%d  g_use_imu=%d  IMU_Read_RPY=%d\n"
                "       cached: Roll=%.2f Pitch=%.2f Yaw=%.2f (deg)\n",
                g_imu_present, g_use_imu, ret, r, p, y);
            CMD_SendText(ack);
            if (!g_imu_present) {
                CMD_SendText("[MSPM0] IMU no frame received yet.\n"
                             "       check wiring: JY61P TX→PA11, RX→PA10, VCC→3.3V, GND→GND\n"
                             "       check JY61P baudrate (default 9600), check UART_DEBUG baudrate=9600\n");
            }
        }
        break;
    case 'm':
        if (v > 2.5f) {
            g_mode = 3;   /* m3=正方形行进 (纯 IMU + 编码器, 非灰度) */
            /* 初始化正方形状态机: 锁当前 yaw 为第 0 条边的目标朝向,
             * 复位里程计 (起点 0,0), 状态=直行, 边数=0 */
            extern uint8_t g_square_state;
            extern uint8_t g_square_edge;
            extern float   g_square_yaw_base;
            g_square_state = 0;
            g_square_edge = 0;
            g_square_yaw_base = IMU_Get_Yaw_Cached();
            Odom_Reset();
            g_yaw_target = g_square_yaw_base;
        } else {
            g_mode = (v > 0.5f) ? 1 : 0;
        }
        snprintf(ack, sizeof(ack), "[MSPM0] mode=%d (%s) yaw_target=%.1f\n",
                 g_mode,
                 g_mode == 0 ? "TRACK" : g_mode == 1 ? "IDLE" : "SQUARE",
                 g_yaw_target);
        CMD_SendText(ack);
        break;
    case 't':
        g_target_rpm = v;
        snprintf(ack, sizeof(ack), "[MSPM0] target_rpm=%.1f\n", g_target_rpm);
        CMD_SendText(ack);
        break;
    case 'r':
        g_target_rpm = 400;
        CMD_SendText("[MSPM0] step: target=400\n");
        break;

    /* ─── IMU / 里程计命令 ─── */
    case 'N':   /* 重新初始化驱动板 (排查 OLED 干扰问题)
                 * 用法: N=只重发电机类型, N1=完整Motor_Init, N2=读电池电压 */
        if (v < 0.5f) {
            /* 只重发电机类型=1 (520电机, 带编码器) */
            Set_motor_type(1);
            CMD_SendText("[MSPM0] re-set motor_type=1 (520)\n");
        } else if (v < 1.5f) {
            /* 完整重新初始化驱动板 */
            CMD_SendText("[MSPM0] Motor_Init() start...\n");
            Motor_Init();
            CMD_SendText("[MSPM0] Motor_Init() done\n");
        } else {
            /* 读电池电压 (0x08 寄存器, uint16, /10 = V) */
            {
                uint8_t buf[2] = {0};
                int ret = motor_i2cRead(Motor_model_ADDR, 0x08, 2, buf);
                if (ret == 0) {
                    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
                    snprintf(ack, sizeof(ack),
                        "[MSPM0] battery=%.1fV (raw=0x%04X)\n", raw/10.0f, raw);
                } else {
                    snprintf(ack, sizeof(ack),
                        "[MSPM0] battery read FAIL (ret=%d)\n", ret);
                }
                CMD_SendText(ack);
            }
        }
        break;
    case 'y':   /* 查询 yaw + 转弯状态 */
        snprintf(ack, sizeof(ack),
            "[MSPM0] yaw=%.2f target=%.2f err=%.2f done_by=%d assist=%d\n",
            g_yaw_now, g_yaw_target, g_yaw_err, g_corner_done_by, g_imu_assist);
        CMD_SendText(ack);
        break;
    case 'x':   /* 查询位置 */
        snprintf(ack, sizeof(ack),
            "[MSPM0] x=%.1fmm y=%.1fmm theta=%.2f deg total=%.1fmm edge=%d edist=%.1fmm\n",
            Odom_Get_X(), Odom_Get_Y(), Odom_Get_Theta(),
            Odom_Get_Total_Dist(), Odom_Get_Edge_Index(), Odom_Get_Edge_Dist());
        CMD_SendText(ack);
        break;
    case 'z':   /* 清零里程计 + Z轴归零 */
        Odom_Reset();
        CMD_SendText("[MSPM0] odom reset (x=y=0, theta=now)\n");
        break;
    case 'c':   /* Z轴归零 (仅IMU, 不清里程计) */
        if (IMU_Calibrate_Z() == 0) {
            CMD_SendText("[MSPM0] IMU Z-axis calibrated to 0\n");
        } else {
            CMD_SendText("[MSPM0] IMU calibrate FAILED\n");
        }
        break;
    case 'e':   /* 切换 IMU 辅助转弯: e0关 e1开 */
        g_imu_assist = (v > 0.5f) ? 1 : 0;
        snprintf(ack, sizeof(ack), "[MSPM0] imu_assist=%d\n", g_imu_assist);
        CMD_SendText(ack);
        break;
    case 'E':   /* IMU 解析开关 (改串口后不再与OLED互斥): E1=启用IMU解析, E0=暂停
                 * 和 BTN_MODE 按钮走同一个 switch_mode(), 切完会发通知 */
        switch_mode((v > 0.5f) ? 1 : 0);
        break;
    case 'L':   /* 设置目标圈数 (替代原 LAP_DN 按钮): L3=3圈, L1=1圈 */
        if (v < 1) v = 1;
        if (v > 9) v = 9;
        g_target_laps = (uint8_t)v;
        snprintf(ack, sizeof(ack), "[MSPM0] target_laps=%d\n", g_target_laps);
        CMD_SendText(ack);
        break;
    case 'U':   /* 切换 IMU 串口原始字节回显 (诊断 JY61P 串口通不通) */
        g_imu_uart_echo = !g_imu_uart_echo;
        snprintf(ack, sizeof(ack), "[MSPM0] IMU UART raw echo %s\n",
                 g_imu_uart_echo ? "ON (spamming hex)" : "OFF");
        CMD_SendText(ack);
        break;
    case 'B':   /* 回显启动状态 (Boot log): 各模块初始化结果 */
        snprintf(ack, sizeof(ack),
            "[MSPM0] === Boot Status ===\n"
            "       track: OK\n"
            "       cmd:   OK\n"
            "       motor: OK ( drv_pid=%.2f/%.3f/%.2f )\n"
            "       imu:   %s (g_imu_present=%d, via UART_DEBUG 9600bps)\n"
            "       odom:  OK\n"
            "       oled:  %s (g_oled_present=%d, via I2C PA17/PA15)\n"
            "       button:OK\n"
            "       imu_parse: %s (g_use_imu=%d, 1=on/0=paused, OLED unaffected)\n"
            "       pins:  MOTOR_I2C=PB11/PB12, OLED_I2C=PA17/PA15, IMU_UART=PA10/PA11, GREY_OUT=PA1\n"
            "       btns:  START=PA7, LAP_UP=PA18, MODE=PB1, RESET=PB14\n",
            drv_kp, drv_ki, drv_kd,
            g_imu_present ? "OK" : "FAIL/SKIPPED", g_imu_present,
            g_oled_present ? "OK" : "FAIL/SKIPPED", g_oled_present,
            g_use_imu ? "ON" : "PAUSED", g_use_imu);
        CMD_SendText(ack);
        break;
    case 'M':   /* 读电机编码器 + 直接发 PWM 测试驱动板输出 */
        CMD_SendText("[MSPM0] === Motor Test ===\n");
        {
            /* 读当前 10ms 编码器值 (电机不动时应为 0) */
            int16_t m1 = Motor_Read_Encoder_L();
            int16_t m2 = Motor_Read_Encoder_R();
            snprintf(ack, sizeof(ack),
                "       enc_L=%d enc_R=%d (电机不动时应为0)\n", m1, m2);
            CMD_SendText(ack);

            /* 直接发 PWM=2000 给 M1 (左轮), 持续 500ms 看电机动不动 */
            CMD_SendText("       send PWM=2000 to M1 (500ms)...\n");
            control_pwm(2000, 0, 0, 0);
            delay_ms(500);
            control_pwm(0, 0, 0, 0);
            delay_ms(50);

            /* 再读编码器, 如果电机转了编码器应该有值 */
            m1 = Motor_Read_Encoder_L();
            m2 = Motor_Read_Encoder_R();
            snprintf(ack, sizeof(ack),
                "       after PWM: enc_L=%d enc_R=%d (有值=电机转了)\n", m1, m2);
            CMD_SendText(ack);
        }
        break;
    case 'I':   /* I2C 总线扫描 (仅 OLED, PA17/PA15; IMU 已改串口不在 I2C 上) */
        CMD_SendText("[MSPM0] I2C scan (PA17/PA15 OLED bus)...\n");
        {
            uint8_t found = 0;
            for (uint8_t addr = 1; addr < 0x80; addr++) {
                IIC_Start();
                IIC_Send_Byte((addr << 1) | 0);
                if (IIC_Wait_Ack() == 0) {
                    snprintf(ack, sizeof(ack), "[MSPM0] found: 0x%02X (%d)\n", addr, addr);
                    CMD_SendText(ack);
                    found++;
                }
                IIC_Stop();
            }
            snprintf(ack, sizeof(ack), "[MSPM0] scan done, %d device(s)\n", found);
            CMD_SendText(ack);
        }
        break;
    case 'D':   /* 驱动板 I2C 扫描 (PB11/PB12 专用总线) */
        CMD_SendText("[MSPM0] I2C scan (PB11/PB12 MOTOR bus)...\n");
        {
            uint8_t found = 0;
            for (uint8_t addr = 1; addr < 0x80; addr++) {
                Motor_IIC_Start();
                Motor_IIC_Send_Byte((addr << 1) | 0);
                if (Motor_IIC_Wait_Ack() == 0) {
                    snprintf(ack, sizeof(ack), "[MSPM0] found: 0x%02X (%d)\n", addr, addr);
                    CMD_SendText(ack);
                    found++;
                }
                Motor_IIC_Stop();
            }
            snprintf(ack, sizeof(ack), "[MSPM0] scan done, %d device(s)\n", found);
            CMD_SendText(ack);
        }
        break;
    default:
        snprintf(ack, sizeof(ack), "[MSPM0] unknown cmd '%c' (0x%02X) idx=%d\n",
                 (c >= 0x20 && c < 0x7F) ? c : '?', (unsigned char)c, cmd_idx);
        CMD_SendText(ack);
        break;
    }
    cmd_idx = 0;
}

/**
 * @brief 每周期轮询 UART_BLUETOOTH
 *        清除 overrun 错误, 防止 UART RX 卡死
 *        每秒发一次心跳, 方便诊断通信状态
 *
 * 调试模式 (CMD_ECHO=1): 收到每个字节都回传 <XX> 十六进制
 *                        方便看 MSPM0 到底收到什么
 */
#define CMD_ECHO  0   /* 1=回显收到的字节(调试用), 0=正常模式 */

void CMD_Poll(void)
{
    /* 清除 overrun / framing / parity 等错误标志 */
    DL_UART_clearInterruptStatus(UART_BLUETOOTH_INST,
        DL_UART_INTERRUPT_OVERRUN_ERROR |
        DL_UART_INTERRUPT_RX_TIMEOUT_ERROR);

    while (!DL_UART_Main_isRXFIFOEmpty(UART_BLUETOOTH_INST)) {
        char ch = (char)DL_UART_Main_receiveData(UART_BLUETOOTH_INST);

        /* 调试回显: 每个字节都回传 <XX> 十六进制 */
#if CMD_ECHO
        {
            char echo[8];
            uint8_t b = (uint8_t)ch;
            const char hex[] = "0123456789ABCDEF";
            echo[0] = '<';
            echo[1] = hex[(b >> 4) & 0x0F];
            echo[2] = hex[b & 0x0F];
            echo[3] = '>';
            echo[4] = '\0';
            CMD_SendText(echo);
        }
#endif

        if (ch == '\n' || ch == '\r') {
            if (cmd_idx > 0) CMD_Exec();
        }
        /* 命令过滤: 丢弃 ESP-01S 混入的 WiFi 日志字节
         * 合法命令: 首字符字母(b/p/d/o/f/k/i/j/s/g/m/t/r) 或 ?
         *          后续字符: 数字/小数点/负号
         * 其他字节 (WiFi日志的非ASCII字节/控制字符) 清空缓冲区 */
        else if (cmd_idx == 0) {
            /* 首字符: 只接受字母或? */
            if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || ch == '?') {
                cmd_buf[cmd_idx++] = ch;
            }
            /* 其他字符丢弃 (WiFi日志) */
        }
        else {
            /* 后续字符: 只接受数字、小数点、负号 */
            if ((ch >= '0' && ch <= '9') || ch == '.' || ch == '-') {
                if (cmd_idx < CMD_BUF_SIZE - 1) {
                    cmd_buf[cmd_idx++] = ch;
                }
            } else {
                /* 非法字符: 清空缓冲区 (被 WiFi 日志污染了) */
                cmd_idx = 0;
            }
        }
    }

    /* 调试模式下不发波形数据, 避免干扰回显 */
#if !CMD_ECHO
    /* $D 调试数据发送已关闭
     * 原因: 每50ms持续发送导致 ESP32 client.write() 丢字节, 回复乱码
     * 现在只在收到命令时才回复, 数据量小不会丢
     * 需要调试数据时用 ? 命令查询当前状态 */
#endif
}
