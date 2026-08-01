#include "balance.h"

#include "ZDT_X42S_Driver.h"
#include "delay.h"
#include "ti_msp_dl_config.h"

static ZDT_MotorTypeDef balance_motor;

volatile Balance_PID_t g_pos_pid;
volatile Balance_PID_t g_vel_pid;
volatile Balance_PID_t g_angle_pid;


volatile float g_balance_motor_angle_deg = 0.0f;
volatile bool g_balance_angle_limit_active = false;
volatile bool g_balance_motor_feedback_valid = false;
volatile uint32_t g_balance_motor_feedback_tick = 0;

extern volatile uint32_t g_sys_tick;

static float s_min_angle_deg = BALANCE_ANGLE_MIN_DEFAULT_DEG;
static float s_max_angle_deg = BALANCE_ANGLE_MAX_DEFAULT_DEG;
static bool s_returning_to_zero = false;
static int16_t s_return_speed = BALANCE_RETURN_SPEED_DEFAULT;
static uint32_t s_feedback_query_tick = 0;
static uint8_t s_feedback_rx[8];
static uint8_t s_feedback_rx_index = 0;

#define BALANCE_FEEDBACK_QUERY_PERIOD_MS  (20U)
#define BALANCE_FEEDBACK_TIMEOUT_MS       (100U)

void Balance_Init(void)
{
    ZDT_Motor_Init(&balance_motor, UART_Motor_INST, DMA_CH4_CHAN_ID, 2);
    delay_ms(500);
    ZDT_Motor_Enable(&balance_motor, true);
    delay_ms(500);

    g_balance_motor_angle_deg = 0.0f;
    g_balance_angle_limit_active = false;
    s_feedback_query_tick = g_sys_tick;
    s_feedback_rx_index = 0;
    g_balance_motor_feedback_valid = false;
    g_balance_motor_feedback_tick = 0;

    /* 
     * ========================================================
     * 1. 初始化位置外环 (Position Loop)
     * 输入：小球位置偏差 (mm)
     * 输出：期望的小球速度 (mm/s)
     * ========================================================
     */
    g_pos_pid.Kp = 0.0f;
    g_pos_pid.Ki = 0.0f;
    g_pos_pid.Kd = 0.0f;
    g_pos_pid.Kff = 0.0f;
    g_pos_pid.error_sum = 0.0f;
    g_pos_pid.last_error = 0.0f;
    g_pos_pid.out_max = 1500.0f;       // 外环调参时先限制目标速度为 +/-50 mm/s
    g_pos_pid.integral_max = 300.0f;  // 积分限幅防饱和

    /* 
     * ========================================================
     * 2. 初始化速度中环 (Velocity Loop)
     * 输入：小球速度偏差 (mm/s)
     * 输出：期望的平板倾角 (度)
     * ========================================================
     */
    g_vel_pid.Kp = 35.0f;
    g_vel_pid.Ki = 0.0f;
    g_vel_pid.Kd = 0.0f;
    g_vel_pid.Kff = 0.0f;
    g_vel_pid.error_sum = 0.0f;
    g_vel_pid.last_error = 0.0f;
    g_vel_pid.out_max = 14.0f;       // 最终在 Balance_Task 中限制为 -6 到 +14 度
    g_vel_pid.integral_max = 5.0f;   // 速度环一般不加积分，或者积分极小

    /* 
     * ========================================================
     * 3. 初始化角度内环 (Angle Loop)
     * 输入：平板角度偏差 (度)
     * 输出：步进电机的目标转速 (RPM / 脉冲频率)
     * ========================================================
     */
    g_angle_pid.Kp = 1.6f;
    g_angle_pid.Ki = 0.05f;
    g_angle_pid.Kd = 0.0f;
    g_angle_pid.Kff = 0.0f;
    g_angle_pid.error_sum = 0.0f;
    g_angle_pid.last_error = 0.0f;
    g_angle_pid.out_max = 300.0f;    // 限幅：电机的最大转速，根据你的 ZDT_X42S 步进电机性能设置
    g_angle_pid.integral_max = 50.0f;// 积分限幅


}

int Balance_MotorSetSpeed(int16_t speed)
{
    g_balance_angle_limit_active = false;

    if (!g_balance_motor_feedback_valid && speed != 0)
    {
        speed = 0;
    }

    if ((g_balance_motor_angle_deg >= s_max_angle_deg && speed > 0) ||
        (g_balance_motor_angle_deg <= s_min_angle_deg && speed < 0))
    {
        speed = 0;
        g_balance_angle_limit_active = true;
        
        // 触发限幅时，清空串级三环的积分防饱和
        g_pos_pid.error_sum = 0.0f;
        g_vel_pid.error_sum = 0.0f;
        g_angle_pid.error_sum = 0.0f;
    }

    return ZDT_Motor_SetSpeed(&balance_motor, speed);
}

void Balance_SetAngleLimits(float min_angle_deg, float max_angle_deg)
{
    if (min_angle_deg >= max_angle_deg)
    {
        return;
    }

    s_min_angle_deg = min_angle_deg;
    s_max_angle_deg = max_angle_deg;
}

float Balance_GetMotorAngle(void)
{
    return g_balance_motor_angle_deg;
}

void Balance_MotorRXByteCallback(uint8_t data)
{
    uint32_t raw_position;
    float angle_deg;
    uint8_t i;

    if (s_feedback_rx_index < sizeof(s_feedback_rx))
    {
        s_feedback_rx[s_feedback_rx_index++] = data;
    }
    else
    {
        for (i = 0U; i < (sizeof(s_feedback_rx) - 1U); i++)
        {
            s_feedback_rx[i] = s_feedback_rx[i + 1U];
        }
        s_feedback_rx[sizeof(s_feedback_rx) - 1U] = data;
    }

    if (s_feedback_rx_index < sizeof(s_feedback_rx) ||
        s_feedback_rx[0] != balance_motor.motor_id ||
        s_feedback_rx[1] != 0x36U ||
        s_feedback_rx[7] != 0x6BU)
    {
        return;
    }

    raw_position = ((uint32_t)s_feedback_rx[3] << 24) |
                   ((uint32_t)s_feedback_rx[4] << 16) |
                   ((uint32_t)s_feedback_rx[5] << 8) |
                   (uint32_t)s_feedback_rx[6];

    /* Emm firmware reports 65536 position counts per motor revolution. */
    angle_deg = (float)raw_position * (360.0f / 65536.0f);
    if (s_feedback_rx[2] != 0U)
    {
        angle_deg = -angle_deg;
    }

    g_balance_motor_angle_deg = angle_deg;
    g_balance_motor_feedback_tick = g_sys_tick;
    g_balance_motor_feedback_valid = true;
}

void Balance_MotorFeedbackTask(void)
{
    uint32_t now_tick = g_sys_tick;

    if (g_balance_motor_feedback_valid &&
        (uint32_t)(now_tick - g_balance_motor_feedback_tick) >
            BALANCE_FEEDBACK_TIMEOUT_MS)
    {
        g_balance_motor_feedback_valid = false;
    }

    if ((uint32_t)(now_tick - s_feedback_query_tick) <
        BALANCE_FEEDBACK_QUERY_PERIOD_MS)
    {
        return;
    }

    if (ZDT_Motor_ReadPosition(&balance_motor) == 0)
    {
        s_feedback_query_tick = now_tick;
    }
}

void Balance_StartReturnToZero(int16_t speed)
{
    if (speed < 0)
    {
        speed = -speed;
    }
    if (speed == 0)
    {
        speed = BALANCE_RETURN_SPEED_DEFAULT;
    }

    s_return_speed = speed;
    s_returning_to_zero = true;
    
    // 启动归零时，清空串级三环的积分
    g_pos_pid.error_sum = 0.0f;
    g_vel_pid.error_sum = 0.0f;
    g_angle_pid.error_sum = 0.0f;
}

void Balance_CancelReturnToZero(void)
{
    s_returning_to_zero = false;
    Balance_MotorSetSpeed(0);
}

bool Balance_IsReturningToZero(void)
{
    return s_returning_to_zero;
}

bool Balance_ReturnToZeroTask(void)
{
    float angle;

    if (!s_returning_to_zero)
    {
        return false;
    }

    angle = Balance_GetMotorAngle();
    if (angle >= -BALANCE_ZERO_TOLERANCE_DEG &&
        angle <= BALANCE_ZERO_TOLERANCE_DEG)
    {
        if (Balance_MotorSetSpeed(0) == 0)
        {
            s_returning_to_zero = false;
        }
        return true;
    }

    Balance_MotorSetSpeed((angle > 0.0f) ? -s_return_speed : s_return_speed);
    return true;
}

float PID_Calcula(volatile Balance_PID_t *pid, float target,
                  float feedback, float feedback_rate,
                  float feedforward, float dt_s)
{
    float error;
    float output;

    if (pid == NULL || dt_s <= 0.0f)
    {
        return 0.0f;
    }

    error = target - feedback;
    pid->error_sum += error * dt_s;

    if (pid->error_sum > pid->integral_max)
    {
        pid->error_sum = pid->integral_max;
    }
    else if (pid->error_sum < -pid->integral_max)
    {
        pid->error_sum = -pid->integral_max;
    }

    output = pid->Kp * error +
             pid->Ki * pid->error_sum -
             pid->Kd * feedback_rate +
             pid->Kff * feedforward;

    if (output > pid->out_max)
    {
        output = pid->out_max;
    }
    else if (output < -pid->out_max)
    {
        output = -pid->out_max;
    }

    pid->last_error = error;
    return output;
}
