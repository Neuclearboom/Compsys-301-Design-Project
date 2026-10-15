/* ==========================================================================
 * Straight-Line Follower
 * ==========================================================================
 * Follows a straight black line on white. Uses ONLY:
 *   frontLeft  (S3)   frontRight (S2)   mid (S6)   back (S4)
 * The left/right axle sensors (S5, S1) are not used.
 *
 * Sensors go HIGH over white, so a sensor over the black line reads LOW.
 *
 * The front pair straddles the line (a bit wider than the line), so when the
 * robot is centred NEITHER front sensor sees it:
 *   line under frontLeft   -> robot has drifted right -> steer left
 *   line under frontRight  -> robot has drifted left  -> steer right
 *   neither / both         -> drive straight
 * If every sensor reads white for LINE_LOST_MS the robot stops (LED on) and
 * resumes when any sensor sees the line again.
 * ========================================================================== */

#include "project.h"
#include <stdint.h>

/* ---------------- Configuration ---------------- */

#define LOOP_MS                5u

/* Sensor mapping - swap the two front macros if it steers AWAY from the line */
#define SENSOR_ACTIVE_LEVEL    0u
#define RAW_FRONT_LEFT()       S3_Read()
#define RAW_FRONT_RIGHT()      S2_Read()
#define RAW_MID()              S6_Read()
#define RAW_BACK()             S4_Read()

/* Wait after power-up (LED blinking) so the robot can be placed on the line */
#define START_DELAY_MS         3000u

/* Speeds (PWM compare, 0-255) */
#define PWM_MAX_COMPARE        255u
#define SPEED_BASE             30u
#define STEER_CUT              6u     /* how much slower the inside wheel runs
                                       * when correcting. Raise if it can't hold
                                       * the line, lower if it zig-zags. */

/* Left motor trim: left wheel commands are scaled up by this many percent to
 * stop the robot leaning left. Raise if it still leans left, lower if it now
 * leans right. */
#define LEFT_MOTOR_TRIM_PERCENT  7

#define LINE_LOST_MS           200u

/* ---------------- Sensors ---------------- */

typedef struct {
    uint8_t frontLeft;
    uint8_t frontRight;
    uint8_t mid;
    uint8_t back;
} SensorState;

static uint8_t OnLine(uint8_t raw)
{
    return (raw == SENSOR_ACTIVE_LEVEL) ? 1u : 0u;
}

static void Sensors_Read(SensorState *s)
{
    s->frontLeft  = OnLine(RAW_FRONT_LEFT());
    s->frontRight = OnLine(RAW_FRONT_RIGHT());
    s->mid        = OnLine(RAW_MID());
    s->back       = OnLine(RAW_BACK());
}

/* ---------------- Motors (positive = forward, negative = reverse) ---------------- */

static int16_t Clamp(int16_t v)
{
    if (v >  (int16_t)PWM_MAX_COMPARE) { return  (int16_t)PWM_MAX_COMPARE; }
    if (v < -(int16_t)PWM_MAX_COMPARE) { return -(int16_t)PWM_MAX_COMPARE; }
    return v;
}

static void Motor_Left_SetSpeed(int16_t speed)
{
    speed = Clamp((int16_t)(((int32_t)speed * (100 + LEFT_MOTOR_TRIM_PERCENT)) / 100));
    if (speed >= 0) {
        PWM_1_WriteCompare1((uint8_t)speed);
        PWM_1_WriteCompare2(0u);
    } else {
        PWM_1_WriteCompare1(0u);
        PWM_1_WriteCompare2((uint8_t)(-speed));
    }
}

static void Motor_Right_SetSpeed(int16_t speed)
{
    speed = Clamp(speed);
    if (speed >= 0) {
        PWM_2_WriteCompare1((uint8_t)speed);
        PWM_2_WriteCompare2(0u);
    } else {
        PWM_2_WriteCompare1(0u);
        PWM_2_WriteCompare2((uint8_t)(-speed));
    }
}

static void Motor_SetSpeed(int16_t leftSpeed, int16_t rightSpeed)
{
    Motor_Left_SetSpeed(leftSpeed);
    Motor_Right_SetSpeed(rightSpeed);
}

static void Motor_Stop(void)
{
    Motor_SetSpeed(0, 0);
}

/* ---------------- Line following ---------------- */

static uint16_t lineLostTicks = 0;
static uint8_t  lost          = 0;

static void RunLineFollower(const SensorState *s)
{
    uint8_t anyOnLine = s->frontLeft | s->frontRight | s->mid | s->back;

    /* ---- line-lost handling ---- */
    if (anyOnLine) {
        lineLostTicks = 0;
        if (lost) {
            lost = 0;
            LED_Write(0u);
        }
    } else if (!lost) {
        if (++lineLostTicks >= (LINE_LOST_MS / LOOP_MS)) {
            lost = 1;
            LED_Write(1u);
        }
    }

    if (lost) {
        Motor_Stop();
        return;
    }

    /* ---- steering ---- */
    if (s->frontLeft && !s->frontRight) {
        /* line under the left sensor -> drifted right -> steer left */
        Motor_SetSpeed((int16_t)SPEED_BASE - (int16_t)STEER_CUT, (int16_t)SPEED_BASE);
    } else if (s->frontRight && !s->frontLeft) {
        /* line under the right sensor -> drifted left -> steer right */
        Motor_SetSpeed((int16_t)SPEED_BASE, (int16_t)SPEED_BASE - (int16_t)STEER_CUT);
    } else {
        /* centred (neither), both, or a brief gap: keep going straight */
        Motor_SetSpeed((int16_t)SPEED_BASE, (int16_t)SPEED_BASE);
    }
}

/* ---------------- Main ---------------- */

int main(void)
{
    CyGlobalIntEnable;

    Clock_1_Start();
    Clock_2_Start();
    PWM_1_Start();
    PWM_2_Start();
    Motor_Stop();
    LED_Write(0u);

    /* countdown: LED blinks so the robot can be placed on the line */
    {
        uint16_t waited;
        uint8_t  ledOn = 0u;
        for (waited = 0u; waited < START_DELAY_MS; waited += 250u) {
            ledOn = (uint8_t)!ledOn;
            LED_Write(ledOn);
            CyDelay(250u);
        }
        LED_Write(0u);
    }

    for (;;) {
        SensorState sensors;
        Sensors_Read(&sensors);
        RunLineFollower(&sensors);
        CyDelay(LOOP_MS);
    }
}