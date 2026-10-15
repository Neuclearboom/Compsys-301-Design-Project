/* ==========================================================================
 * Line-Following Robot - RIGHT-HAND PRIORITY  (rev 2)
 * ==========================================================================
 * Hardware (from top design):
 *   PWM_1 (In1/IN2)       -> left  motor      QuadDec_L (ENCD3/ENCD4)
 *   PWM_2 (In1_1/IN2_1)   -> right motor      QuadDec_R (ENCD1/ENCD2)
 *   S1..S6 digital inputs, SW (start switch), LED
 *   D1 / D1_1 are constant-0 outputs in the schematic - no code needed.
 *
 * Map: black line on white. Sensors go HIGH over white, so a sensor that
 * sees the black line reads LOW  (SENSOR_ACTIVE_LEVEL = 0).
 *
 * Sensor layout:
 *   Front pair   : frontLeft, frontRight - straddle the line (a bit wider
 *                  than the line), so when centred NEITHER sees it.
 *   Axle row     : left, mid, right - just behind the axle.
 *   Rear         : back - centre of the tail.
 *
 * Junction rule (right-hand priority):
 *   1. Right branch available  -> turn right
 *   2. else straight available -> go straight
 *   3. else left branch        -> turn left
 *   4. else dead end           -> spin 180 (to the right)
 *
 * TURN PROCEDURE (every turn, 90 or 180):
 *   ALIGN   reverse so the axle sits over the junction centre
 *   SEEK    slow spin. Blind (coarse speed) until the minimum angle is
 *           reached, then fine speed. The leading front sensor hits the new
 *           line first, then the SECOND front sensor hits it.
 *   RIGHT   the moment the second front sensor hits the line: stop, then
 *   ITSELF  creep back the other way at fine speed until that sensor lets go
 *           of the line, i.e. the line is back between the front pair.
 *   Then normal line following resumes.
 *
 * Everything marked TUNE needs bench calibration for your robot.
 * ========================================================================== */

#include "project.h"
#include <stdint.h>

/* ==========================================================================
 * Configuration
 * ========================================================================== */

#define LOOP_MS                5u
#define MS(x)                  ((uint16_t)((x) / LOOP_MS))   /* ms -> loop ticks */

/* ---- Sensor mapping: swap these if a sensor is on the wrong side ---------
 * Carried over from the previous main.c (S5/S1 were swapped on the bench). */
#define SENSOR_ACTIVE_LEVEL    0u
#define RAW_FRONT_LEFT()       S3_Read()
#define RAW_FRONT_RIGHT()      S2_Read()
#define RAW_MID()              S6_Read()
#define RAW_BACK()             S4_Read()
#define RAW_LEFT()             S5_Read()
#define RAW_RIGHT()            S1_Read()

/* ---- Start switch ---------------------------------------------------------
 * The switch's idle level is read at power-up (assumes it is NOT held down at
 * boot); "pressed" is whatever level differs from that, so polarity doesn't
 * matter. Set USE_START_SWITCH to 0 to start straight after the self-test. */
#define USE_START_SWITCH       1

/* ---- Startup motor self-test: both wheels forward, then backward ---------- */
#define SELFTEST_SPEED         60u
#define SELFTEST_MS            300u

/* ---- Speeds (PWM compare, 0-255) ---------------------------------------- */
#define PWM_MAX_COMPARE        255u
#define SPEED_BASE             30u

#define SPEED_TURN_COARSE      45u   /* spin speed before the minimum angle - nothing
                                      * is being detected yet, so this can be quicker */
#define SPEED_TURN_FINE        30u   /* spin speed while hunting for the line (TUNE:
                                      * lower = less overshoot, but too low and the
                                      * wheels stall - raise if the robot stops) */
#define SPEED_RIGHTING         30u   /* counter-spin that re-centres after overshoot */

/* ---- Line following ------------------------------------------------------
 * A single front sensor must stay lit this long before we steer. A side
 * branch crossing under one front sensor is brief; real drift persists. */
#define DRIFT_CONFIRM_TICKS    MS(15)
#define DRIFT_CUT              6u    /* how much slower the inside wheel runs */

/* ---- Junction detection ------------------------------------------------- */
#define SIDE_CONFIRM_TICKS     MS(10)   /* side sensor must hold this long   */
#define REARM_CLEAR_TICKS      MS(20)   /* both side sensors clear to re-arm */
#define DEAD_END_CONFIRM_TICKS MS(30)   /* back-only must hold this long     */
#define LINE_LOST_TICKS        MS(200)  /* all-white this long = lost        */

/* ---- Encoder distances (ticks, direction-independent).  TUNE -------------
 * Distances are measured as (|dL| + |dR|)/2 so encoder count direction does
 * not matter.
 * ticks_per_mm   = encoder counts per wheel rev / (pi * wheel_diameter_mm)
 * JUNCTION_CREEP = (line width + ~5 mm) * ticks_per_mm
 * ALIGN_BACKUP   = (axle-to-side-sensor distance - line_width/2) * ticks_per_mm
 * TURN_90_*      = per-wheel arc for a 90 deg pivot = pi * track_width / 4
 *                  MIN ~ 60 % of it, MAX ~ 150 % of it. 180 deg doubles both. */
#define JUNCTION_CREEP_TICKS   40
#define ALIGN_BACKUP_TICKS     60
#define TURN_90_MIN_TICKS      150
#define TURN_90_MAX_TICKS      600
#define TURN_180_MIN_TICKS     350
#define TURN_180_MAX_TICKS     1100

/* ---- Safety timeouts ---------------------------------------------------- */
#define JUNCTION_TIMEOUT_TICKS MS(400)
#define ALIGN_TIMEOUT_TICKS    MS(600)
#define TURN_90_TIMEOUT_TICKS  MS(3000)
#define TURN_180_TIMEOUT_TICKS MS(5000)
#define RIGHTING_TIMEOUT_TICKS MS(600)

#define BRAKE_MS               50u   /* pause after stopping, before reversing */
#define SETTLE_MS              40u   /* pause to let the wheels stop spinning  */

/* ==========================================================================
 * Sensors
 * ========================================================================== */

typedef struct {
    uint8_t frontLeft;
    uint8_t frontRight;
    uint8_t mid;
    uint8_t back;
    uint8_t left;
    uint8_t right;
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
    s->left       = OnLine(RAW_LEFT());
    s->right      = OnLine(RAW_RIGHT());
}

/* ==========================================================================
 * Motors  (positive = forward, negative = reverse)
 * ========================================================================== */

static int16_t Clamp(int16_t v)
{
    if (v >  (int16_t)PWM_MAX_COMPARE) { return  (int16_t)PWM_MAX_COMPARE; }
    if (v < -(int16_t)PWM_MAX_COMPARE) { return -(int16_t)PWM_MAX_COMPARE; }
    return v;
}

static void Motor_Left_SetSpeed(int16_t speed)
{
    speed = Clamp(speed);
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

/* dir > 0 spins right (left wheel fwd, right wheel back), dir < 0 spins left */
static void Motor_Spin(int8_t dir, int16_t speed)
{
    if (dir > 0) { Motor_SetSpeed(speed, -speed); }
    else         { Motor_SetSpeed(-speed, speed); }
}

/* ==========================================================================
 * Encoders - distance travelled since the current state began
 * ========================================================================== */

static int32_t encL = 0;
static int32_t encR = 0;
static int32_t stateStartL = 0;
static int32_t stateStartR = 0;

static int32_t Abs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

static void Enc_Update(void)
{
    encL = (int32_t)QuadDec_L_GetCounter();
    encR = (int32_t)QuadDec_R_GetCounter();
}

/* ticks moved since EnterState(), whichever way the wheels turned */
static int32_t Travel(void)
{
    return (Abs32(encL - stateStartL) + Abs32(encR - stateStartR)) / 2;
}

/* ==========================================================================
 * State machine
 * ========================================================================== */

typedef enum {
    ST_FOLLOW,        /* following the line, watching for junctions        */
    ST_JUNCTION,      /* left-only branch seen: roll on to check for straight */
    ST_ALIGN,         /* reverse so the axle is over the junction          */
    ST_TURN_SEEK,     /* slow spin until the second front sensor hits line */
    ST_TURN_RIGHTING, /* creep back until the line is between the front pair */
    ST_LINE_LOST
} State;

static State    state       = ST_FOLLOW;
static uint16_t stateTicks  = 0;

/* turn parameters, set by BeginTurn() */
static int8_t   turnDir        = +1;   /* +1 right, -1 left */
static int32_t  turnMinTicks   = 0;
static int32_t  turnMaxTicks   = 0;
static uint16_t turnTimeout    = 0;

/* pending 90-degree turn and how far to reverse before it */
static int8_t   pendingDir     = +1;
static int32_t  alignTarget    = 0;

/* junction latches */
static uint8_t  seenLeft       = 0;
static uint8_t  seenRight      = 0;

/* debounce / arming */
static uint16_t sideLeftTicks  = 0;
static uint16_t sideRightTicks = 0;
static uint16_t clearTicks     = 0;
static uint8_t  sideArmed      = 1;
static uint16_t deadEndTicks   = 0;
static uint16_t lineLostTicks  = 0;

/* drift correction */
static int8_t   driftDir       = 0;   /* -1 line on left, +1 line on right */
static uint16_t driftTicks     = 0;

static uint16_t Inc(uint16_t v)
{
    return (v < 0xFFFFu) ? (uint16_t)(v + 1u) : v;
}

static void EnterState(State next)
{
    state       = next;
    stateTicks  = 0;
    stateStartL = encL;
    stateStartR = encR;
}

/* Back to line-following. Side sensors must clear before the next junction
 * can be detected (stops one junction being counted twice). */
static void ResumeFollow(void)
{
    Motor_Stop();
    sideArmed      = 0;
    clearTicks     = 0;
    sideLeftTicks  = 0;
    sideRightTicks = 0;
    deadEndTicks   = 0;
    lineLostTicks  = 0;
    driftDir       = 0;
    driftTicks     = 0;
    EnterState(ST_FOLLOW);
}

/* Stop, pause, then reverse (alignBackTicks) to put the axle over the
 * junction centre before pivoting. */
static void BeginAlign(int8_t dir, int32_t alignBackTicks)
{
    Motor_Stop();
    CyDelay(BRAKE_MS);
    pendingDir  = dir;
    alignTarget = alignBackTicks;
    EnterState(ST_ALIGN);
}

static void BeginTurn(int8_t dir, int32_t minT, int32_t maxT, uint16_t timeoutTicks)
{
    turnDir      = dir;
    turnMinTicks = minT;
    turnMaxTicks = maxT;
    turnTimeout  = timeoutTicks;
    EnterState(ST_TURN_SEEK);
}

/* Straight-line following with the front pair.
 * Line under frontLeft  -> robot has drifted right -> steer left.
 * Line under frontRight -> robot has drifted left  -> steer right.
 * A lone front sensor must persist for DRIFT_CONFIRM_TICKS first, so a side
 * branch sliding under one sensor doesn't make the robot swerve. */
static void Steer(const SensorState *s)
{
    int8_t want = 0;

    if (s->frontLeft && !s->frontRight)      { want = -1; }
    else if (s->frontRight && !s->frontLeft) { want = +1; }

    if (want != 0 && want == driftDir) { driftTicks = Inc(driftTicks); }
    else                               { driftTicks = 0; }
    driftDir = want;

    if (want == 0 || driftTicks < DRIFT_CONFIRM_TICKS) {
        Motor_SetSpeed((int16_t)SPEED_BASE, (int16_t)SPEED_BASE);
    } else if (want < 0) {
        Motor_SetSpeed((int16_t)SPEED_BASE - (int16_t)DRIFT_CUT, (int16_t)SPEED_BASE);
    } else {
        Motor_SetSpeed((int16_t)SPEED_BASE, (int16_t)SPEED_BASE - (int16_t)DRIFT_CUT);
    }
}

static void RunLineFollower(const SensorState *s)
{
    uint8_t anyOnLine = s->left | s->frontLeft | s->mid | s->frontRight | s->right | s->back;
    uint8_t onlyBack  = s->back && !s->left && !s->frontLeft && !s->mid
                        && !s->frontRight && !s->right;

    Enc_Update();
    stateTicks = Inc(stateTicks);

    switch (state) {

    /* ------------------------------------------------------------------ */
    case ST_FOLLOW:
        /* line-lost watchdog: short all-white reads are treated as gaps */
        if (!anyOnLine) {
            lineLostTicks = Inc(lineLostTicks);
            if (lineLostTicks >= LINE_LOST_TICKS) {
                Motor_Stop();
                EnterState(ST_LINE_LOST);
                break;
            }
        } else {
            lineLostTicks = 0;
        }

        sideLeftTicks  = s->left  ? Inc(sideLeftTicks)  : 0u;
        sideRightTicks = s->right ? Inc(sideRightTicks) : 0u;
        deadEndTicks   = onlyBack ? Inc(deadEndTicks)   : 0u;

        /* re-arm once both outer sensors have been off the line for a while */
        if (!sideArmed) {
            if (!s->left && !s->right) {
                clearTicks = Inc(clearTicks);
                if (clearTicks >= REARM_CLEAR_TICKS) { sideArmed = 1u; }
            } else {
                clearTicks = 0;
            }
        }

        if (sideArmed && (sideRightTicks >= SIDE_CONFIRM_TICKS ||
                          sideLeftTicks  >= SIDE_CONFIRM_TICKS)) {
            seenRight = (sideRightTicks >= SIDE_CONFIRM_TICKS) ? 1u : 0u;
            seenLeft  = (sideLeftTicks  >= SIDE_CONFIRM_TICKS) ? 1u : 0u;

            if (seenRight) {
                BeginAlign(+1, ALIGN_BACKUP_TICKS);      /* priority 1: right */
            } else {
                EnterState(ST_JUNCTION);                 /* left only: straight too? */
            }
        }
        else if (deadEndTicks >= DEAD_END_CONFIRM_TICKS) {
            Motor_Stop();
            CyDelay(BRAKE_MS);
            deadEndTicks = 0;
            BeginTurn(+1, TURN_180_MIN_TICKS, TURN_180_MAX_TICKS, TURN_180_TIMEOUT_TICKS);
        }
        else {
            Steer(s);
        }
        break;

    /* ------------------------------------------------------------------ */
    case ST_JUNCTION:
        /* Roll forward just past the branch's width. A right branch showing
         * up late (robot not square) still wins. After that, if mid still
         * sees the line then the line continues straight. */
        Motor_SetSpeed((int16_t)SPEED_BASE, (int16_t)SPEED_BASE);
        if (s->right) { seenRight = 1u; }
        if (s->left)  { seenLeft  = 1u; }

        if (seenRight) {
            /* we've rolled on by Travel() since the trigger - reverse that too */
            BeginAlign(+1, ALIGN_BACKUP_TICKS + Travel());
        }
        else if (Travel() >= JUNCTION_CREEP_TICKS || stateTicks >= JUNCTION_TIMEOUT_TICKS) {
            if (s->mid) {
                ResumeFollow();                          /* priority 2: straight */
            } else if (seenLeft) {
                BeginAlign(-1, ALIGN_BACKUP_TICKS + Travel());   /* priority 3: left */
            } else {
                ResumeFollow();                          /* false alarm */
            }
        }
        break;

    /* ------------------------------------------------------------------ */
    case ST_ALIGN:
        Motor_SetSpeed(-(int16_t)SPEED_BASE, -(int16_t)SPEED_BASE);
        if (Travel() >= alignTarget || stateTicks >= ALIGN_TIMEOUT_TICKS) {
            Motor_Stop();
            CyDelay(SETTLE_MS);
            BeginTurn(pendingDir, TURN_90_MIN_TICKS, TURN_90_MAX_TICKS, TURN_90_TIMEOUT_TICKS);
        }
        break;

    /* ------------------------------------------------------------------ */
    case ST_TURN_SEEK: {
        /* The front pair straddles the line. Spinning right, the RIGHT front
         * sensor (the leading one) meets the new line first, then the LEFT
         * one (the second/trailing sensor). Spinning left it's mirrored.
         * Nothing is accepted before the minimum angle so the old line can't
         * fool us. Slow down once past the minimum angle. */
        int32_t progress = Travel();
        uint8_t trail    = (turnDir > 0) ? s->frontLeft : s->frontRight;

        Motor_Spin(turnDir, (progress < turnMinTicks) ? (int16_t)SPEED_TURN_COARSE
                                                      : (int16_t)SPEED_TURN_FINE);

        if (progress >= turnMaxTicks || stateTicks >= turnTimeout) {
            /* never found the line - give up and let FOLLOW / LINE_LOST cope */
            Motor_Stop();
            CyDelay(SETTLE_MS);
            ResumeFollow();
        }
        else if (progress >= turnMinTicks && trail) {
            /* second front sensor just hit the line -> stop and right ourselves */
            Motor_Stop();
            CyDelay(SETTLE_MS);
            EnterState(ST_TURN_RIGHTING);
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case ST_TURN_RIGHTING: {
        /* We're a touch past centre: the trailing front sensor is on the line.
         * Creep back the other way until it lets go - the line is then back
         * between the pair. If the leading sensor picks it up we've come back
         * through centre, so that's also done. */
        uint8_t trail = (turnDir > 0) ? s->frontLeft  : s->frontRight;
        uint8_t lead  = (turnDir > 0) ? s->frontRight : s->frontLeft;

        if (!trail || lead || stateTicks >= RIGHTING_TIMEOUT_TICKS) {
            Motor_Stop();
            CyDelay(SETTLE_MS);
            ResumeFollow();
        } else {
            Motor_Spin((int8_t)-turnDir, (int16_t)SPEED_RIGHTING);
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case ST_LINE_LOST:
        Motor_Stop();
        LED_Write(1u);
        if (anyOnLine) {
            LED_Write(0u);
            ResumeFollow();
        }
        break;
    }
}

/* ==========================================================================
 * Main
 * ========================================================================== */

static void Motor_SelfTest(void)
{
    LED_Write(1u);
    Motor_SetSpeed((int16_t)SELFTEST_SPEED, (int16_t)SELFTEST_SPEED);      /* forward  */
    CyDelay(SELFTEST_MS);
    Motor_Stop();
    CyDelay(150u);
    Motor_SetSpeed(-(int16_t)SELFTEST_SPEED, -(int16_t)SELFTEST_SPEED);    /* backward */
    CyDelay(SELFTEST_MS);
    Motor_Stop();
    LED_Write(0u);
    CyDelay(300u);
}

int main(void)
{
    CyGlobalIntEnable;

    Clock_1_Start();
    Clock_2_Start();
    PWM_1_Start();
    PWM_2_Start();
    QuadDec_L_Start();
    QuadDec_R_Start();
    QuadDec_L_SetCounter(0);
    QuadDec_R_SetCounter(0);
    Motor_Stop();
    LED_Write(0u);

    CyDelay(200u);          /* let the supply and inputs settle */
    Motor_SelfTest();

#if USE_START_SWITCH
    {
        /* idle level = what the switch reads right now; pressed = the opposite */
        uint8_t idle = (uint8_t)(SW_Read() ? 1u : 0u);
        uint8_t blink = 0u;
        uint8_t ledOn = 0u;

        /* wait for press (LED blinks to show we're armed and waiting) */
        while ((SW_Read() ? 1u : 0u) == idle) {
            CyDelay(10u);
            if (++blink >= 25u) { blink = 0u; ledOn = (uint8_t)!ledOn; LED_Write(ledOn); }
        }
        LED_Write(0u);

        /* wait for release, then give the user time to let go */
        while ((SW_Read() ? 1u : 0u) != idle) { CyDelay(10u); }
        CyDelay(500u);
    }
#endif

    for (;;) {
        SensorState sensors;
        Sensors_Read(&sensors);
        RunLineFollower(&sensors);
        CyDelay(LOOP_MS);
    }
}