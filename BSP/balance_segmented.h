#ifndef BALANCE_SEGMENTED_H
#define BALANCE_SEGMENTED_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Minimal integration:
 *
 *     Balance_Init();
 *     SegmentedBalance_Init();
 *     SegmentedBalance_StartChallenge3();
 *
 * Then call SegmentedBalance_Task() once every 10 ms.
 * Do not run Balance_Task() at the same time.
 */

/*
 * Flip this sign if a positive rod angle makes the ball move in the
 * negative position direction.
 */
#define SEG_BALANCE_ROD_ANGLE_SIGN             (1.0f)

/* Position bands and commanded ball velocities. */
#define SEG_BALANCE_POSITION_NEAR_MM           (4.0f)
#define SEG_BALANCE_POSITION_MID_MM            (10.0f)
#define SEG_BALANCE_POSITION_FAR_MM            (30.0f)
#define SEG_BALANCE_VELOCITY_NEAR_MM_S         (20.0f)
#define SEG_BALANCE_VELOCITY_MID_MM_S          (50.0f)
#define SEG_BALANCE_VELOCITY_FAR_MM_S          (100.0f)

/* Speed-error bands and discrete rod angles used for acceleration/braking. */
#define SEG_BALANCE_SPEED_ERROR_SMALL_MM_S     (12.0f)
#define SEG_BALANCE_SPEED_ERROR_MID_MM_S       (30.0f)
#define SEG_BALANCE_SPEED_ERROR_LARGE_MM_S     (60.0f)
#define SEG_BALANCE_ROD_ANGLE_SMALL_DEG        (1.5f)
#define SEG_BALANCE_ROD_ANGLE_MID_DEG          (3.0f)
#define SEG_BALANCE_ROD_ANGLE_LARGE_DEG        (6.0f)

/* Motor speeds used to track the discrete rod-angle command. */
#define SEG_BALANCE_ANGLE_DEADBAND_DEG         (0.35f)
#define SEG_BALANCE_ANGLE_ERROR_MID_DEG        (1.5f)
#define SEG_BALANCE_ANGLE_ERROR_LARGE_DEG      (3.5f)
#define SEG_BALANCE_MOTOR_SPEED_SLOW_RPM       (6)
#define SEG_BALANCE_MOTOR_SPEED_MID_RPM        (15)
#define SEG_BALANCE_MOTOR_SPEED_FAST_RPM       (30)

/* Vision safety and challenge-3 state-machine settings. */
#define SEG_BALANCE_VISION_TIMEOUT_MS          (250U)
#define SEG_BALANCE_CHALLENGE_PLUS_MM          (50.0f)
#define SEG_BALANCE_CHALLENGE_MINUS_MM         (-50.0f)
#define SEG_BALANCE_PLUS_SWITCH_MM             (45.0f)
#define SEG_BALANCE_FINAL_TOLERANCE_MM         (8.0f)
#define SEG_BALANCE_FINAL_SPEED_MM_S           (20.0f)
#define SEG_BALANCE_FINAL_STABLE_MS            (500U)
#define SEG_BALANCE_CHALLENGE_TIMEOUT_MS       (5000U)

typedef enum {
    SEG_BALANCE_MODE_STOP = 0,
    SEG_BALANCE_MODE_HOLD,
    SEG_BALANCE_MODE_CHALLENGE_3
} SegmentedBalanceMode;

typedef enum {
    SEG_BALANCE_STAGE_IDLE = 0,
    SEG_BALANCE_STAGE_TO_PLUS_50,
    SEG_BALANCE_STAGE_TO_MINUS_50,
    SEG_BALANCE_STAGE_HOLD_MINUS_50,
    SEG_BALANCE_STAGE_COMPLETE,
    SEG_BALANCE_STAGE_TIMEOUT
} SegmentedBalanceStage;

typedef struct {
    SegmentedBalanceMode mode;
    SegmentedBalanceStage stage;
    float target_position_mm;
    float desired_ball_velocity_mm_s;
    float target_rod_angle_deg;
    bool vision_valid;
    bool complete;
    bool timed_out;
} SegmentedBalanceStatus;

/* Call after Balance_Init(). */
void SegmentedBalance_Init(void);

/* One-key entry points. */
void SegmentedBalance_StartHold(float target_position_mm);
void SegmentedBalance_StartChallenge3(void);
void SegmentedBalance_Stop(void);

/*
 * Call once every 10 ms. Do not call the original Balance_Task() at the
 * same time, because both functions command the same motor.
 */
void SegmentedBalance_Task(void);

bool SegmentedBalance_IsActive(void);
bool SegmentedBalance_IsComplete(void);
bool SegmentedBalance_HasTimedOut(void);
SegmentedBalanceStatus SegmentedBalance_GetStatus(void);

#endif
