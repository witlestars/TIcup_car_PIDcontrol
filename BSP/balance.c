#include "balance.h"

#include "ZDT_X42S_Driver.h"
#include "delay.h"
#include "ti_msp_dl_config.h"

static ZDT_MotorTypeDef balance_motor;

volatile Balance_PID_t g_balance_pid;
volatile float g_balance_motor_angle_deg = 0.0f;
volatile bool g_balance_angle_limit_active = false;

extern volatile uint32_t g_sys_tick;

static float s_min_angle_deg = BALANCE_ANGLE_MIN_DEFAULT_DEG;
static float s_max_angle_deg = BALANCE_ANGLE_MAX_DEFAULT_DEG;
static bool s_returning_to_zero = false;
static int16_t s_return_speed = BALANCE_RETURN_SPEED_DEFAULT;
static int16_t s_commanded_speed_rpm = 0;
static uint32_t s_angle_update_tick = 0;

static void Balance_UpdateEstimatedAngle(void)
{
    uint32_t now_tick = g_sys_tick;
    uint32_t elapsed_ms = (uint32_t)(now_tick - s_angle_update_tick);

    if (elapsed_ms == 0U)
    {
        return;
    }

    s_angle_update_tick = now_tick;
    g_balance_motor_angle_deg +=
        (float)s_commanded_speed_rpm * 0.006f * (float)elapsed_ms;
}

void Balance_Init(void)
{
    ZDT_Motor_Init(&balance_motor, UART_Motor_INST, DMA_CH4_CHAN_ID, 2);
    delay_ms(500);
    ZDT_Motor_Enable(&balance_motor, true);
    delay_ms(500);

    g_balance_motor_angle_deg = 0.0f;
    g_balance_angle_limit_active = false;
    s_commanded_speed_rpm = 0;
    s_angle_update_tick = g_sys_tick;

    g_balance_pid.Kp = 0.0f;
    g_balance_pid.Ki = 0.0f;
    g_balance_pid.Kd = 0.0f;
    g_balance_pid.Kff = 0.0f;
    g_balance_pid.error_sum = 0.0f;
    g_balance_pid.last_error = 0.0f;
    g_balance_pid.out_max = 200.0f;
    g_balance_pid.integral_max = 50.0f;
}

int Balance_MotorSetSpeed(int16_t speed)
{
    int result;

    Balance_UpdateEstimatedAngle();
    g_balance_angle_limit_active = false;

    if ((g_balance_motor_angle_deg >= s_max_angle_deg && speed > 0) ||
        (g_balance_motor_angle_deg <= s_min_angle_deg && speed < 0))
    {
        speed = 0;
        g_balance_angle_limit_active = true;
        g_balance_pid.error_sum = 0.0f;
    }

    result = ZDT_Motor_SetSpeed(&balance_motor, speed);
    if (result == 0)
    {
        s_commanded_speed_rpm = speed;
    }

    return result;
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

void Balance_AngleEstimateTask(void)
{
    Balance_UpdateEstimatedAngle();
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
    g_balance_pid.error_sum = 0.0f;
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

float Balance_PID(volatile Balance_PID_t *pid, float target,
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
