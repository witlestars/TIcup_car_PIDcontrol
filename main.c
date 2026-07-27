/**
 * @file    main.c
 * @brief   巡线小车主程序 — MSPM0G3507 + 四路驱动板
 *          灰度巡线 + IMU + 编码器里程 + 蓝牙调参
 *          非阻塞调度: IMU解析每轮 + 巡线/电机/按钮 10ms
 */

#include "ti_msp_dl_config.h"
#include "BSP/track.h"
#include "BSP/template/motor.h"
#include "BSP/template/cmd.h"
#include "BSP/template/uart_bluetooth.h"
#include "BSP/imu.h"
#include "BSP/odometry.h"
#include "BSP/oled.h"
#include "BSP/button.h"
#include "delay.h"

/* SysTick 1ms 时基 (非阻塞调度用; delay.c 用 delay_cycles 不冲突) */
volatile uint32_t g_sys_tick = 0;
void SysTick_Handler(void) { g_sys_tick++; }

/* IMU 解析启用/暂停切换 (按钮和 'E' 命令共用; IMU已改串口, 与OLED不互斥) */
void switch_mode(uint8_t use_imu)
{
    g_use_imu = use_imu;
    if (use_imu) {
        CMD_SendText("[MSPM0] IMU enabled (parsing on)\n");
    } else {
        CMD_SendText("[MSPM0] IMU paused (parsing off, OLED unaffected)\n");
    }
}

/* m3 正方形行进状态机: 0=直行 1=转弯 2=完成 3=刹车 */
uint8_t g_square_state = 0, g_square_edge = 0;
float   g_square_yaw_base = 0.0f, g_turn_start_yaw = 0.0f;
uint8_t g_one_shot_turn = 0;
float   g_one_shot_angle = 0.0f;
uint32_t g_brake_start = 0;

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Config(32000);   /* 1ms 时基 (32MHz/1000) */

    CMD_SendText("[MSPM0] init: track\n");  Track_Init();
    CMD_SendText("[MSPM0] init: cmd\n");    CMD_Init();
    CMD_SendText("[MSPM0] init: motor\n");  Motor_Init();

    /* IMU: UART_DEBUG 9600bps, 等500ms让JY61P冷启动, 启用RX中断 */
    delay_ms(500);
    IMU_EnableRxIRQ();
    CMD_SendText("[MSPM0] init: imu\n");
    CMD_SendText(IMU_Init() == 0 ? "[MSPM0] IMU JY61P OK\n" : "[MSPM0] IMU JY61P FAIL\n");

    CMD_SendText("[MSPM0] init: odom\n");   Odom_Init();
    CMD_SendText("[MSPM0] init: oled\n");   OLED_Init();
    if (g_oled_present) {
        OLED_Clear();
        OLED_PrintfAt(0, 0, "==TI CUP==");
        OLED_PrintfAt(1, 0, "ready");
    }

    CMD_SendText("[MSPM0] init: button\n"); Button_Init();
    CMD_SendText("[MSPM0] init done\n");

    uint32_t last_10ms = 0;
    while (1) {
        CMD_Poll();
        IMU_EchoTick();
        if (g_use_imu) IMU_Poll();   /* 每轮解析降低 yaw 延迟 */

        /* 10ms 节拍: 按钮 + 巡线PID + 电机指令 */
        if ((uint32_t)(g_sys_tick - last_10ms) >= 10) {
            last_10ms = g_sys_tick;

            Button_HandleEvents();   /* ISR 设标志, 这里执行业务 */

            if (g_laps_done && g_running) {
                g_running = 0;
                Motor_Stop();
                CMD_SendText("[MSPM0] LAPS DONE! auto-stop\n");
            }

            /* ── 运行/停止/模式控制 ── */
            if (!g_running) {
                g_motor_l_speed = 0;
                g_motor_r_speed = 0;
            } else if (g_mode == 0) {
                Track_Loop();
                Odom_Update();   /* 巡线模式也推里程计 (过弯检测/圈数用) */
            } else if (g_mode == 3) {
                /* m3 正方形行进 / T 命令单次转弯 (纯 IMU + 编码器)
                 * 状态机: 0=直行 1=转弯 2=完成 3=刹车(停车200ms消惯性)
                 * 左转=yaw+ (JY61P倒扣: 从车顶看左转=模块顶面顺时针=维特yaw+)
                 * T命令(g_one_shot_turn=1)优先级最高: 单次转弯后停车 */
                extern uint8_t g_square_state, g_square_edge, g_one_shot_turn;
                extern float   g_square_yaw_base, g_one_shot_angle;
                extern uint32_t g_brake_start;
                #define SQ_EDGE_LEN    1500.0f   /* 边长 mm */
                #define SQ_TURN_THRESH 5.0f      /* 转弯到位阈值 (度) */
                #define SQ_BASE_SPD    80        /* 直行速度 */
                #define SQ_TURN_SPD    60        /* 转弯速度 */
                #define SQ_YAW_KP       15.0f    /* 直行yaw修正P */
                #define SQ_YAW_CLIP     40        /* 修正最大差速 */
                #define SQ_BRAKE_MS     200       /* 刹车时间 ms */

                if (!g_imu_present) {  /* IMU离线保护 */
                    g_motor_l_speed = 0;
                    g_motor_r_speed = 0;
                    if (g_running) {
                        g_running = 0;
                        Motor_Stop();
                        CMD_SendText("[MSPM0] SQUARE: IMU offline, stop\n");
                    }
                } else {
                    float yaw_now = IMU_Get_Yaw_Cached();
                    g_yaw_now = yaw_now;

                    if (g_one_shot_turn) {
                        /* T命令单次转弯: delta=now-start, err=target-delta */
                        float delta = yaw_now - g_turn_start_yaw;
                        if (delta > 180.0f)  delta -= 360.0f;
                        if (delta < -180.0f) delta += 360.0f;
                        float err = g_one_shot_angle - delta;
                        g_yaw_err = err;
                        if (err > SQ_TURN_THRESH) {
                            g_motor_l_speed = -SQ_TURN_SPD;   /* 左转 */
                            g_motor_r_speed =  SQ_TURN_SPD;
                        } else if (err < -SQ_TURN_THRESH) {
                            g_motor_l_speed =  SQ_TURN_SPD;   /* 右转 */
                            g_motor_r_speed = -SQ_TURN_SPD;
                        } else {
                            g_one_shot_turn = 0; g_running = 0; Motor_Stop();
                            CMD_SendText("[MSPM0] TURN done\n");
                        }
                    } else if (g_square_state == 2) {
                        g_motor_l_speed = 0; g_motor_r_speed = 0;
                        g_running = 0; Motor_Stop();
                        CMD_SendText("[MSPM0] SQUARE DONE! auto-stop\n");
                    } else if (g_square_state == 3) {
                        /* 刹车: 停车消惯性再切直行 */
                        g_motor_l_speed = 0; g_motor_r_speed = 0;
                        if ((uint32_t)(g_sys_tick - g_brake_start) >= SQ_BRAKE_MS) {
                            g_square_yaw_base = IMU_Get_Yaw_Cached();
                            Odom_Reset_Edge();
                            g_square_edge++;
                            if (g_square_edge >= 4) {
                                g_square_state = 2;
                                CMD_SendText("[MSPM0] SQUARE: all edges done\n");
                            } else {
                                g_square_state = 0;
                                CMD_SendText("[MSPM0] SQUARE: next edge\n");
                            }
                        }
                    } else if (g_square_state == 0) {
                        /* 直行: yaw误差做差速保持直线 */
                        float err = g_square_yaw_base - yaw_now;
                        if (err > 180.0f)  err -= 360.0f;
                        if (err < -180.0f) err += 360.0f;
                        g_yaw_err = err;
                        float correction = err * SQ_YAW_KP;
                        if (correction > SQ_YAW_CLIP)  correction = SQ_YAW_CLIP;
                        if (correction < -SQ_YAW_CLIP) correction = -SQ_YAW_CLIP;
                        g_motor_l_speed = (int16_t)(SQ_BASE_SPD - correction);
                        g_motor_r_speed = (int16_t)(SQ_BASE_SPD + correction);
                        if (Odom_Get_Edge_Dist() >= SQ_EDGE_LEN) {
                            g_square_state = 1;
                            g_turn_start_yaw = yaw_now;
                            CMD_SendText("[MSPM0] SQUARE: edge done, turning\n");
                        }
                    } else {
                        /* state==1 转弯: 原地左转90°, delta=now-start 目标90° */
                        float delta = yaw_now - g_turn_start_yaw;
                        if (delta > 180.0f)  delta -= 360.0f;
                        if (delta < -180.0f) delta += 360.0f;
                        float err = 90.0f - delta;
                        g_yaw_err = err;
                        if (err > SQ_TURN_THRESH) {
                            g_motor_l_speed = -SQ_TURN_SPD;   /* 左转 */
                            g_motor_r_speed =  SQ_TURN_SPD;
                        } else if (err < -SQ_TURN_THRESH) {
                            g_motor_l_speed =  SQ_TURN_SPD;   /* 过冲, 右转修正 */
                            g_motor_r_speed = -SQ_TURN_SPD;
                        } else {
                            g_square_state = 3;  /* 到位进刹车 */
                            g_brake_start = g_sys_tick;
                            CMD_SendText("[MSPM0] SQUARE: turn done, braking\n");
                        }
                    }
                }
            } else {
                g_motor_l_speed = (int16_t)g_target_rpm;
                g_motor_r_speed = (int16_t)g_target_rpm;
            }

            Motor_Send_Speed(0, -g_motor_r_speed, 0, -g_motor_l_speed);
        }
    }
}
