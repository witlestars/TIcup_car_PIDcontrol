#include "balance.h"
#include "ti_msp_dl_config.h" 
#include "ZDT_X42S_Driver.h"  
#include "delay.h"        
#include "vision_protocol.h"     

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
    uint32_t now_tick;
    uint32_t elapsed_ms;

    now_tick = g_sys_tick;
    elapsed_ms = (uint32_t)(now_tick - s_angle_update_tick);
    if (elapsed_ms == 0)
    {
        return;
    }

    s_angle_update_tick = now_tick;
    g_balance_motor_angle_deg += (float)s_commanded_speed_rpm * 0.006f *
                                 (float)elapsed_ms;
}

void Balance_Init(void)
{
    ZDT_Motor_Init(&balance_motor, UART_Motor_INST, DMA_CH4_CHAN_ID, 2);
    delay_ms(500);
    ZDT_Motor_Enable(&balance_motor, true);
    delay_ms(500);

    /* 主控上电时以电机上位机中已经设置好的零点作为 0 度。 */
    g_balance_motor_angle_deg = 0.0f;
    g_balance_angle_limit_active = false;
    s_commanded_speed_rpm = 0;
    s_angle_update_tick = g_sys_tick;

    g_balance_pid.Kp = 0.0f;
    g_balance_pid.Ki = 0.0f;
    g_balance_pid.Kd = 0.0f;  // 注意：引入真实速度后，Kd的量级可能需要重新调整
    g_balance_pid.Kff = 0.0f; 
    
    g_balance_pid.error_sum = 0.0f;
    g_balance_pid.last_error = 0.0f; // 在新算法中不再作为核心微分依据
    
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
    if (speed < 0) {
        speed = -speed;
    }
    if (speed == 0) {
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

void Balance_PID(float target_pos, float vision_pos, float vision_vel, uint16_t age_ms, float gyro_rate)
{
    float current_pos_predicted;
    float error, p_term, i_term, d_term, feedforward, output;
    int16_t motor_speed;

    /* 1. 航位推算与通信超时保护  */
    // 计算从收到这帧数据到现在，经过了多少毫秒
    uint32_t local_delay_ms = g_sys_tick - g_vision_data.last_update_tick;

    // 视觉协议建议：连续 250ms 未收到有效帧，视为失锁安全状态
    if (local_delay_ms > 250 || g_vision_data.target_found == 0) 
    {
        // 视觉丢失或超时，电机停转
        Balance_MotorSetSpeed(0);
        return; 
    }

    // 真正的总延时 = K230内部延时 + 单片机本地等待延时
    float total_delay_s = (age_ms + local_delay_ms) / 1000.0f;

    // 动态推算当前时刻小球的真实位置！
    current_pos_predicted = vision_pos + vision_vel * total_delay_s;

    /* 2. 计算预测偏差 */
    error = target_pos - current_pos_predicted;
    
    // 静止死区判断
    // 如果位置误差小于 1.0mm 且 速度小于 8.0mm/s (阈值根据你 VOFA+ 观察到的噪声峰值来定)
    if (error > -3.0f && error < 3.0f && vision_vel > -10.0f && vision_vel < 10.0f) 
    {
        error = 0.0f;
        vision_vel = 0.0f;
        // 这会让计算出的 p_term 和 d_term 直接归零，系统进入绝对安静状态
    }


    /* 3. 积分计算 (抗积分饱和) */
    g_balance_pid.error_sum += error;
    if (g_balance_pid.error_sum > g_balance_pid.integral_max) {
        g_balance_pid.error_sum = g_balance_pid.integral_max;
    } else if (g_balance_pid.error_sum < -g_balance_pid.integral_max) {
        g_balance_pid.error_sum = -g_balance_pid.integral_max;
    }

    /* 4. 比例与积分项 */
    p_term = g_balance_pid.Kp * error;
    i_term = g_balance_pid.Ki * g_balance_pid.error_sum;

    /* 5. 微分计算 (直接使用视觉真实速度) 
     * 目标速度为 0，所以误差变化率就是 -vision_vel 
     */
    d_term = g_balance_pid.Kd * (-vision_vel);

    /* 6. 前馈计算 (基于陀螺仪角速度) */
    feedforward = g_balance_pid.Kff * gyro_rate;

    /* 7. 复合输出 */
    output = p_term + i_term + d_term + feedforward;

    /* 8. 输出量限幅 */
    if (output > g_balance_pid.out_max) {
        output = g_balance_pid.out_max;
    } else if (output < -g_balance_pid.out_max) {
        output = -g_balance_pid.out_max;
    }

    /* 9. 转换类型并下发指令给电机 */
    motor_speed = (int16_t)output;
    Balance_MotorSetSpeed(motor_speed);
}
