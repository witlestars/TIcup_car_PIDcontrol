#include "bsp_motor_iic.h"

int Encoder_Offset[4];   /* 10ms 实时脉冲差值 */
int Encoder_Now[4];      /* 累计脉冲总数 */

/**
 * float 转 bytes（小端）
 */
static void float_to_bytes(float f, uint8_t *bytes)
{
    memcpy(bytes, &f, sizeof(float));
}

/**
 * 设置电机类型: 1=520, 2=310, 3=TT编码器, 4=TT无编码器
 */
void Set_motor_type(uint8_t data)
{
    motor_i2cWrite(Motor_model_ADDR, MOTOR_TYPE_REG, 1, &data);
}

/**
 * 配置死区 (0-3600)
 */
void Set_motor_deadzone(uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (data >> 8) & 0xff;
    buf[1] = data & 0xff;
    motor_i2cWrite(Motor_model_ADDR, MOTOR_DeadZONE_REG, 2, buf);
}

/**
 * 配置磁环线数
 */
void Set_Pluse_line(uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (data >> 8) & 0xff;
    buf[1] = data & 0xff;
    motor_i2cWrite(Motor_model_ADDR, MOTOR_PluseLine_REG, 2, buf);
}

/**
 * 配置减速比
 */
void Set_Pluse_Phase(uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (data >> 8) & 0xff;
    buf[1] = data & 0xff;
    motor_i2cWrite(Motor_model_ADDR, MOTOR_PlusePhase_REG, 2, buf);
}

/**
 * 配置轮径 (mm)
 */
void Set_Wheel_dis(float data)
{
    uint8_t bytes[4];
    float_to_bytes(data, bytes);
    motor_i2cWrite(Motor_model_ADDR, WHEEL_DIA_REG, 4, bytes);
}

/**
 * 速度控制: 同时设置 4 个电机的速度 (-1000~1000)
 */
void control_speed(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    uint8_t speed[8];
    speed[0] = (m1 >> 8) & 0xff;  speed[1] = m1 & 0xff;
    speed[2] = (m2 >> 8) & 0xff;  speed[3] = m2 & 0xff;
    speed[4] = (m3 >> 8) & 0xff;  speed[5] = m3 & 0xff;
    speed[6] = (m4 >> 8) & 0xff;  speed[7] = m4 & 0xff;
    motor_i2cWrite(Motor_model_ADDR, SPEED_Control_REG, 8, speed);
}

/**
 * PWM 控制: 同时设置 4 个电机的 PWM (-3600~3600)
 * 配合 Read_10_Enconder 可自行实现闭环，但驱动板已自带 PID，一般用 control_speed 即可
 */
void control_pwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    uint8_t pwm[8];
    pwm[0] = (m1 >> 8) & 0xff;  pwm[1] = m1 & 0xff;
    pwm[2] = (m2 >> 8) & 0xff;  pwm[3] = m2 & 0xff;
    pwm[4] = (m3 >> 8) & 0xff;  pwm[5] = m3 & 0xff;
    pwm[6] = (m4 >> 8) & 0xff;  pwm[7] = m4 & 0xff;
    motor_i2cWrite(Motor_model_ADDR, PWM_Control_REG, 8, pwm);
}

/**
 * 读取 10ms 周期内的编码器脉冲差值（4 个电机）
 * 结果存入 Encoder_Offset[0~3]
 */
void Read_10_Enconder(void)
{
    uint8_t buf[2];

    motor_i2cRead(Motor_model_ADDR, READ_TEN_M1Enconer_REG, 2, buf);
    Encoder_Offset[0] = buf[0] << 8 | buf[1];

    motor_i2cRead(Motor_model_ADDR, READ_TEN_M2Enconer_REG, 2, buf);
    Encoder_Offset[1] = buf[0] << 8 | buf[1];

    motor_i2cRead(Motor_model_ADDR, READ_TEN_M3Enconer_REG, 2, buf);
    Encoder_Offset[2] = buf[0] << 8 | buf[1];

    motor_i2cRead(Motor_model_ADDR, READ_TEN_M4Enconer_REG, 2, buf);
    Encoder_Offset[3] = buf[0] << 8 | buf[1];
}

/**
 * 读取电机累计脉冲总数（4 个电机）
 * 结果存入 Encoder_Now[0~3]
 */
void Read_ALL_Enconder(void)
{
    uint8_t buf[2];
    uint8_t buf2[2];

    /* M1 */
    motor_i2cRead(Motor_model_ADDR, READ_ALLHigh_M1_REG, 2, buf);
    motor_i2cRead(Motor_model_ADDR, READ_ALLLOW_M1_REG, 2, buf2);
    Encoder_Now[0] = buf[0] << 24 | buf[1] << 16 | buf2[0] << 8 | buf2[1];

    /* M2 */
    motor_i2cRead(Motor_model_ADDR, READ_ALLHigh_M2_REG, 2, buf);
    motor_i2cRead(Motor_model_ADDR, READ_ALLLOW_M2_REG, 2, buf2);
    Encoder_Now[1] = buf[0] << 24 | buf[1] << 16 | buf2[0] << 8 | buf2[1];

    /* M3 */
    motor_i2cRead(Motor_model_ADDR, READ_ALLHigh_M3_REG, 2, buf);
    motor_i2cRead(Motor_model_ADDR, READ_ALLLOW_M3_REG, 2, buf2);
    Encoder_Now[2] = buf[0] << 24 | buf[1] << 16 | buf2[0] << 8 | buf2[1];

    /* M4 */
    motor_i2cRead(Motor_model_ADDR, READ_ALLHigh_M4_REG, 2, buf);
    motor_i2cRead(Motor_model_ADDR, READ_ALLLOW_M4_REG, 2, buf2);
    Encoder_Now[3] = buf[0] << 24 | buf[1] << 16 | buf2[0] << 8 | buf2[1];
}
