#include "balance_segmented.h"

#include "balance.h"
#include "vision_protocol.h"

extern volatile uint32_t g_sys_tick;

static SegmentedBalanceStatus s_status;
static uint32_t s_challenge_start_tick;
static uint32_t s_stable_start_tick;
static bool s_stable_timer_running;

static float SegmentedBalance_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float SegmentedBalance_Sign(float value)
{
    if (value > 0.0f) {
        return 1.0f;
    }
    if (value < 0.0f) {
        return -1.0f;
    }
    return 0.0f;
}

static float SegmentedBalance_GetDesiredVelocity(float position_error_mm)
{
    float error_abs = SegmentedBalance_Abs(position_error_mm);
    float velocity_abs;

    if (error_abs > SEG_BALANCE_POSITION_FAR_MM) {
        velocity_abs = SEG_BALANCE_VELOCITY_FAR_MM_S;
    } else if (error_abs > SEG_BALANCE_POSITION_MID_MM) {
        velocity_abs = SEG_BALANCE_VELOCITY_MID_MM_S;
    } else if (error_abs > SEG_BALANCE_POSITION_NEAR_MM) {
        velocity_abs = SEG_BALANCE_VELOCITY_NEAR_MM_S;
    } else {
        velocity_abs = 0.0f;
    }

    return SegmentedBalance_Sign(position_error_mm) * velocity_abs;
}

static float SegmentedBalance_GetTargetRodAngle(float speed_error_mm_s)
{
    float speed_error_abs = SegmentedBalance_Abs(speed_error_mm_s);
    float angle_abs;

    if (speed_error_abs > SEG_BALANCE_SPEED_ERROR_LARGE_MM_S) {
        angle_abs = SEG_BALANCE_ROD_ANGLE_LARGE_DEG;
    } else if (speed_error_abs > SEG_BALANCE_SPEED_ERROR_MID_MM_S) {
        angle_abs = SEG_BALANCE_ROD_ANGLE_MID_DEG;
    } else if (speed_error_abs > SEG_BALANCE_SPEED_ERROR_SMALL_MM_S) {
        angle_abs = SEG_BALANCE_ROD_ANGLE_SMALL_DEG;
    } else {
        angle_abs = 0.0f;
    }

    return SEG_BALANCE_ROD_ANGLE_SIGN *
           SegmentedBalance_Sign(speed_error_mm_s) * angle_abs;
}

static void SegmentedBalance_TrackRodAngle(float target_angle_deg)
{
    float angle_error_deg = target_angle_deg - Balance_GetMotorAngle();
    float angle_error_abs = SegmentedBalance_Abs(angle_error_deg);
    int16_t motor_speed_rpm;

    if (angle_error_abs <= SEG_BALANCE_ANGLE_DEADBAND_DEG) {
        motor_speed_rpm = 0;
    } else if (angle_error_abs > SEG_BALANCE_ANGLE_ERROR_LARGE_DEG) {
        motor_speed_rpm = SEG_BALANCE_MOTOR_SPEED_FAST_RPM;
    } else if (angle_error_abs > SEG_BALANCE_ANGLE_ERROR_MID_DEG) {
        motor_speed_rpm = SEG_BALANCE_MOTOR_SPEED_MID_RPM;
    } else {
        motor_speed_rpm = SEG_BALANCE_MOTOR_SPEED_SLOW_RPM;
    }

    if (angle_error_deg < 0.0f) {
        motor_speed_rpm = -motor_speed_rpm;
    }

    Balance_MotorSetSpeed(motor_speed_rpm);
}

static void SegmentedBalance_UpdateChallenge3(float position_mm,
                                              float velocity_mm_s,
                                              uint32_t now_tick)
{
    if ((uint32_t)(now_tick - s_challenge_start_tick) >=
        SEG_BALANCE_CHALLENGE_TIMEOUT_MS) {
        s_status.timed_out = true;
        s_status.stage = SEG_BALANCE_STAGE_TIMEOUT;
        return;
    }

    switch (s_status.stage) {
        case SEG_BALANCE_STAGE_TO_PLUS_50:
            s_status.target_position_mm = SEG_BALANCE_CHALLENGE_PLUS_MM;
            if (position_mm >= SEG_BALANCE_PLUS_SWITCH_MM) {
                s_status.stage = SEG_BALANCE_STAGE_TO_MINUS_50;
                s_status.target_position_mm = SEG_BALANCE_CHALLENGE_MINUS_MM;
            }
            break;

        case SEG_BALANCE_STAGE_TO_MINUS_50:
            s_status.target_position_mm = SEG_BALANCE_CHALLENGE_MINUS_MM;
            if (SegmentedBalance_Abs(position_mm - SEG_BALANCE_CHALLENGE_MINUS_MM) <=
                    SEG_BALANCE_FINAL_TOLERANCE_MM &&
                SegmentedBalance_Abs(velocity_mm_s) <=
                    SEG_BALANCE_FINAL_SPEED_MM_S) {
                s_status.stage = SEG_BALANCE_STAGE_HOLD_MINUS_50;
                s_stable_start_tick = now_tick;
                s_stable_timer_running = true;
            }
            break;

        case SEG_BALANCE_STAGE_HOLD_MINUS_50:
            s_status.target_position_mm = SEG_BALANCE_CHALLENGE_MINUS_MM;
            if (SegmentedBalance_Abs(position_mm - SEG_BALANCE_CHALLENGE_MINUS_MM) <=
                    SEG_BALANCE_FINAL_TOLERANCE_MM &&
                SegmentedBalance_Abs(velocity_mm_s) <=
                    SEG_BALANCE_FINAL_SPEED_MM_S) {
                if (s_stable_timer_running &&
                    (uint32_t)(now_tick - s_stable_start_tick) >=
                        SEG_BALANCE_FINAL_STABLE_MS) {
                    s_status.stage = SEG_BALANCE_STAGE_COMPLETE;
                    s_status.complete = true;
                }
            } else {
                s_status.stage = SEG_BALANCE_STAGE_TO_MINUS_50;
                s_stable_timer_running = false;
            }
            break;

        case SEG_BALANCE_STAGE_COMPLETE:
            s_status.target_position_mm = SEG_BALANCE_CHALLENGE_MINUS_MM;
            break;

        case SEG_BALANCE_STAGE_TIMEOUT:
            break;

        default:
            s_status.stage = SEG_BALANCE_STAGE_TO_PLUS_50;
            s_status.target_position_mm = SEG_BALANCE_CHALLENGE_PLUS_MM;
            break;
    }
}

void SegmentedBalance_Init(void)
{
    s_status.mode = SEG_BALANCE_MODE_STOP;
    s_status.stage = SEG_BALANCE_STAGE_IDLE;
    s_status.target_position_mm = 0.0f;
    s_status.desired_ball_velocity_mm_s = 0.0f;
    s_status.target_rod_angle_deg = 0.0f;
    s_status.vision_valid = false;
    s_status.complete = false;
    s_status.timed_out = false;
    s_challenge_start_tick = 0;
    s_stable_start_tick = 0;
    s_stable_timer_running = false;
}

void SegmentedBalance_StartHold(float target_position_mm)
{
    s_status.mode = SEG_BALANCE_MODE_HOLD;
    s_status.stage = SEG_BALANCE_STAGE_IDLE;
    s_status.target_position_mm = target_position_mm;
    s_status.desired_ball_velocity_mm_s = 0.0f;
    s_status.target_rod_angle_deg = 0.0f;
    s_status.complete = false;
    s_status.timed_out = false;
    s_stable_timer_running = false;
}

void SegmentedBalance_StartChallenge3(void)
{
    s_status.mode = SEG_BALANCE_MODE_CHALLENGE_3;
    s_status.stage = SEG_BALANCE_STAGE_TO_PLUS_50;
    s_status.target_position_mm = SEG_BALANCE_CHALLENGE_PLUS_MM;
    s_status.desired_ball_velocity_mm_s = 0.0f;
    s_status.target_rod_angle_deg = 0.0f;
    s_status.complete = false;
    s_status.timed_out = false;
    s_challenge_start_tick = g_sys_tick;
    s_stable_timer_running = false;
}

void SegmentedBalance_Stop(void)
{
    s_status.mode = SEG_BALANCE_MODE_STOP;
    s_status.stage = SEG_BALANCE_STAGE_IDLE;
    s_status.desired_ball_velocity_mm_s = 0.0f;
    s_status.target_rod_angle_deg = 0.0f;
    s_status.vision_valid = false;
    Balance_MotorSetSpeed(0);
}

void SegmentedBalance_Task(void)
{
    uint32_t now_tick = g_sys_tick;
    float position_mm;
    float velocity_mm_s;
    float position_error_mm;
    float speed_error_mm_s;

    if (s_status.mode == SEG_BALANCE_MODE_STOP) {
        Balance_MotorSetSpeed(0);
        return;
    }

    s_status.vision_valid =
        (g_vision_data.target_found != 0U) &&
        ((uint32_t)(now_tick - g_vision_data.last_update_tick) <=
         SEG_BALANCE_VISION_TIMEOUT_MS);

    if (!s_status.vision_valid) {
        s_status.desired_ball_velocity_mm_s = 0.0f;
        s_status.target_rod_angle_deg = 0.0f;
        SegmentedBalance_TrackRodAngle(0.0f);
        return;
    }

    position_mm = (float)g_vision_data.position_01mm / 10.0f;
    velocity_mm_s = (float)g_vision_data.velocity_mm_s;

    if (s_status.mode == SEG_BALANCE_MODE_CHALLENGE_3) {
        SegmentedBalance_UpdateChallenge3(position_mm, velocity_mm_s, now_tick);
        if (s_status.timed_out) {
            s_status.desired_ball_velocity_mm_s = 0.0f;
            s_status.target_rod_angle_deg = 0.0f;
            SegmentedBalance_TrackRodAngle(0.0f);
            return;
        }
    }

    position_error_mm = s_status.target_position_mm - position_mm;
    s_status.desired_ball_velocity_mm_s =
        SegmentedBalance_GetDesiredVelocity(position_error_mm);

    speed_error_mm_s =
        s_status.desired_ball_velocity_mm_s - velocity_mm_s;
    s_status.target_rod_angle_deg =
        SegmentedBalance_GetTargetRodAngle(speed_error_mm_s);

    SegmentedBalance_TrackRodAngle(s_status.target_rod_angle_deg);
}

bool SegmentedBalance_IsActive(void)
{
    return s_status.mode != SEG_BALANCE_MODE_STOP;
}

bool SegmentedBalance_IsComplete(void)
{
    return s_status.complete;
}

bool SegmentedBalance_HasTimedOut(void)
{
    return s_status.timed_out;
}

SegmentedBalanceStatus SegmentedBalance_GetStatus(void)
{
    return s_status;
}
