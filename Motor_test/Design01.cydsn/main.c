/* ==========================================================================
 * Straight-Line Follower
 * ==========================================================================
 * Follows a straight black line on white. Uses ONLY:
 *   frontLeft, frontRight, mid, back
 * The left/right axle sensors (S5, S1) are not used.
 *
 * Sensors go HIGH over white, so a sensor over the black line reads LOW.
 *
 * The front pair straddles the line (a bit wider than the line), so when the
 * robot is centred NEITHER front sensor sees it:
 *   line under frontLeft   -> robot has drifted right -> steer left
 *   line under frontRight  -> robot has drifted left  -> steer right
 *   neither / both         -> drive straight
 *
 * CENTRING: when a sensor clears, the line is only just off that sensor's
 * edge. So the correction is HELD for STEER_HOLD_MS after the sensor clears,
 * which carries the line across into the middle of the gap instead of leaving
 * it hugging one sensor.
 *
 * If every sensor reads white for LINE_LOST_MS the robot stops (LED on) and
 * resumes when any sensor sees the line again.
 * ========================================================================== */

#include "project.h"
#include <stdint.h>

/* ---------------- Configuration ---------------- */

#define LOOP_MS                5u

/* Sensor mapping (front pair swapped as confirmed on the bench) */
#define SENSOR_ACTIVE_LEVEL    0u
#define RAW_FRONT_LEFT()       S2_Read()
#define RAW_FRONT_RIGHT()      S3_Read()
#define RAW_MID()              S6_Read()
#define RAW_BACK()             S4_Read()

/* Wait after power-up (LED blinking) so the robot can be placed on the line */
#define START_DELAY_MS         3000u

/* Speeds (PWM compare, 0-255) */
#define PWM_MAX_COMPARE        255u
#define SPEED_BASE             30u
#define STEER_CUT              6u     /* how much slower the inside wheel runs
                                       * while correcting */

/* How long a correction keeps going AFTER the sensor clears (TUNE).
 *   Line still hugs one sensor -> raise it (try 40, 50, ...)
 *   Robot now zig-zags side to side across the gap -> lower it */
#define STEER_HOLD_MS          30u

/* Left motor trim in PERMILLE (10 = 1 %). Positive speeds the left wheel up.
 * Fine-grained: the fraction is dithered between neighbouring PWM values, so
 * even 10 permille has an effect at speed 30. TUNE:
 *   still leaning left  -> raise it (try 20, 30, ...)
 *   leaning right       -> lower it / go negative */
#define LEFT_TRIM_PERMILLE     0

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

static int32_t leftRemainder = 0;   /* carries the fractional part of the trim */

static int16_t Clamp(int16_t v)
{
    if (v >  (int16_t)PWM_MAX_COMPARE) { return  (int16_t)PWM_MAX_COMPARE; }
    if (v < -(int16_t)PWM_MAX_COMPARE) { return -(int16_t)PWM_MAX_COMPARE; }
    return v;
}

static void Motor_Left_SetSpeed(int16_t speed)
{
    int16_t out;

    if (speed == 0) {
        leftRemainder = 0;
        out = 0;
    } else {
        int32_t scaled = (int32_t)speed * (1000 + LEFT_TRIM_PERMILLE) + leftRemainder;
        out           = (int16_t)(scaled / 1000);
        leftRemainder = scaled % 1000;
    }
    out = Clamp(out);

    if (out >= 0) {
        PWM_1_WriteCompare1((uint8_t)out);
        PWM_1_WriteCompare2(0u);
    } else {
        PWM_1_WriteCompare1(0u);
        PWM_1_WriteCompare2((uint8_t)(-out));
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
static int8_t   steerDir      = 0;   /* -1 = steering left, +1 = steering right */
static uint16_t holdTicks     = 0;   /* ticks left to keep steering after clear  */

static void RunLineFollower(const SensorState *s)
{
    uint8_t anyOnLine = s->frontLeft | s->frontRight | s->mid | s->back;
    int8_t  want      = 0;

    /* ---- line-lost handling ---- */
    if (anyOnLine) {
        lineLostTicks = 0;
        if (lost) {
            lost = 0;
            steerDir = 0;
            holdTicks = 0;
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

    /* ---- which way does the line want us to go? ---- */
    if (s->frontLeft && !s->frontRight)      { want = -1; }   /* line on left  -> steer left  */
    else if (s->frontRight && !s->frontLeft) { want = +1; }   /* line on right -> steer right */

    if (want != 0) {
        steerDir  = want;                                 /* sensor is on the line: correct */
        holdTicks = (uint16_t)(STEER_HOLD_MS / LOOP_MS);  /* and keep going after it clears */
    } else if (s->frontLeft && s->frontRight) {
        steerDir  = 0;                                    /* both lit: just go straight */
        holdTicks = 0;
    } else if (holdTicks > 0u) {
        holdTicks--;                                      /* sensor cleared: carry the line to the middle */
    } else {
        steerDir = 0;
    }

    /* ---- drive ---- */
    if (steerDir < 0) {
        Motor_SetSpeed((int16_t)SPEED_BASE - (int16_t)STEER_CUT, (int16_t)SPEED_BASE);
    } else if (steerDir > 0) {
        Motor_SetSpeed((int16_t)SPEED_BASE, (int16_t)SPEED_BASE - (int16_t)STEER_CUT);
    } else {
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