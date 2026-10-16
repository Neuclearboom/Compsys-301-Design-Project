/* ==========================================================================
 * Line Follower - straight following + RIGHT-BIASED turning
 * ==========================================================================
 * Follows a black line on white.
 *
 * STRAIGHT FOLLOWING uses frontLeft, frontRight, mid, back.
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
 *
 * TURNING (right-hand priority) uses the outer axle sensors (left / right):
 *   1. right branch seen              -> turn right
 *   2. left branch only, line goes on -> go straight
 *   3. left branch only, no line on   -> turn left
 *   4. only the back sensor sees line -> dead end, spin 180 (to the right)
 *
 * Every turn: ALIGN (reverse so the axle is over the junction) -> SEEK (slow
 * spin until the SECOND front sensor hits the new line) -> RIGHTING (creep
 * back until that sensor lets go, so the line is between the front pair).
 * ========================================================================== */

#include "project.h"
#include <stdint.h>

/* ---------------- Configuration ---------------- */

<<<<<<< Updated upstream
#define LOOP_MS                5u

/* Sensor mapping (front pair swapped as confirmed on the bench) */
#define SENSOR_ACTIVE_LEVEL    0u
#define RAW_FRONT_LEFT()       S2_Read()
#define RAW_FRONT_RIGHT()      S3_Read()
#define RAW_MID()              S6_Read()
#define RAW_BACK()             S4_Read()
=======
#define SENSOR_ACTIVE_LEVEL   0u
#define PWM_MAX_COMPARE       255u
#define SPEED_BASE            30u
#define SPEED_TURN            40u
#define SPEED_TURN_AROUND     40u         /* spin speed for the dead-end 180 only - the normal left/right
                                            * turns keep using SPEED_TURN */

#define LINE_LOST_THRESHOLD   20u
#define TURN_TIMEOUT          60u
#define TURN_COOLDOWN         30u
#define POST_TURN_COOLDOWN    30u
>>>>>>> Stashed changes

/* Outer axle-row sensors, used ONLY to spot junctions. Swapped (like the front
 * pair): S1 is physically on the left, S5 on the right. If a right turn
 * happens at a LEFT branch, swap these two back.                            */
#define RAW_LEFT()             S1_Read()   /* left outer sensor pin */
#define RAW_RIGHT()            S5_Read()   /* right outer sensor pin */

<<<<<<< Updated upstream
/* Wait after power-up (LED blinking) so the robot can be placed on the line */
#define START_DELAY_MS         500u
=======
#define TURN_BACKUP_BRAKE_MS      50u
#define TURN_BACKUP_ITERATIONS    10u
#define POST_TURN_ADJUST_ITERATIONS  6u  /* safety cap only now - see STATE_ADJUST_LEFT/RIGHT below */
>>>>>>> Stashed changes

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
#define LEFT_TRIM_PERMILLE     2

#define LINE_LOST_MS           200u

/* ---------------- Turning configuration ---------------- */

#define TICKS(ms)              ((uint16_t)((ms) / LOOP_MS))   /* ms -> loop ticks */

/* Spin speeds. Slow = less overshoot, but too slow and the wheels stall. */
#define SPEED_TURN_COARSE      45u   /* spin speed BEFORE the minimum angle (nothing is detected yet) */
#define SPEED_TURN_FINE        30u   /* spin speed while hunting for the line (lower = less overshoot) */
#define SPEED_RIGHTING         30u   /* creep-back speed that re-centres after overshoot */

/* Junction detection timing */
#define SIDE_CONFIRM_MS        10u   /* outer sensor must hold this long to count as a branch */
#define REARM_CLEAR_MS         20u   /* both outer sensors clear this long before the next junction is accepted */
#define DEAD_END_CONFIRM_MS    30u   /* back-only must hold this long to count as a dead end */

/* Encoder distances in ticks (direction-independent). PLACEHOLDERS - TUNE.
 *   ticks_per_mm   = encoder counts per wheel rev / (pi * wheel_diameter_mm)
 *   JUNCTION_CREEP = (line width + ~5 mm) * ticks_per_mm
 *   ALIGN_BACKUP   = (axle-to-outer-sensor distance - line_width/2) * ticks_per_mm
 *   TURN_90_*      = per-wheel arc of a 90 deg pivot = pi * track_width / 4
 *                    MIN ~ 60 % of it, MAX ~ 150 % of it. 180 deg doubles both. */
#define JUNCTION_CREEP_TICKS   40    /* roll-on distance after a left-only branch, to see if the line goes straight */
#define ALIGN_BACKUP_TICKS     4     /* reverse distance so the axle sits over the junction */
#define TURN_90_MIN_TICKS      150   /* 90 deg turn - ignore the front sensors until this much rotation */
#define TURN_90_MAX_TICKS      600   /* 90 deg turn - give up after this much rotation */
#define TURN_180_MIN_TICKS     350   /* dead-end spin - ignore the front sensors until this much rotation */
#define TURN_180_MAX_TICKS     1100  /* dead-end spin - give up after this much rotation */

/* Safety timeouts so no turning state can hang forever */
#define JUNCTION_TIMEOUT_MS    400u  /* roll-on check timeout */
#define ALIGN_TIMEOUT_MS       600u  /* reverse timeout */
#define TURN_90_TIMEOUT_MS     3000u /* 90 deg spin timeout */
#define TURN_180_TIMEOUT_MS    5000u /* 180 deg spin timeout */
#define RIGHTING_TIMEOUT_MS    600u  /* re-centring timeout */

#define BRAKE_MS               50u   /* pause after stopping, before reversing */
#define SETTLE_MS              40u   /* pause to let the wheels stop spinning */

/* ---------------- Sensors ---------------- */

typedef struct {
    uint8_t frontLeft;
    uint8_t frontRight;
    uint8_t mid;
    uint8_t back;
    uint8_t left;     /* outer left sensor */
    uint8_t right;    /* outer right sensor */
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

/* in-place spin: dir > 0 = right (left wheel fwd, right wheel back), dir < 0 = left */
static void Motor_Spin(int8_t dir, int16_t speed)
{
    if (dir > 0) { Motor_SetSpeed(speed, -speed); }
    else         { Motor_SetSpeed(-speed, speed); }
}

/* ---------------- Encoders (turning only) ---------------- */

static int32_t encL        = 0;   /* latest left encoder count */
static int32_t encR        = 0;   /* latest right encoder count */
static int32_t stateStartL = 0;   /* left count when the current state began */
static int32_t stateStartR = 0;   /* right count when the current state began */

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

/* ---------------- Line following state ---------------- */

typedef enum {
    ST_FOLLOW,          /* normal line following (+ junction watching)          */
    ST_JUNCTION,        /* left-only branch seen: roll on to check for straight */
    ST_ALIGN,           /* reverse so the axle is over the junction             */
    ST_TURN_SEEK,       /* slow spin until the second front sensor hits line    */
    ST_TURN_RIGHTING    /* creep back until the line is between the front pair  */
} State;   /* robot state (everything except ST_FOLLOW is turning) */

static State    state      = ST_FOLLOW;   /* current state */
static uint16_t stateTicks = 0;           /* loop ticks in state */

static uint16_t lineLostTicks = 0;
static uint8_t  lost          = 0;
static int8_t   steerDir      = 0;   /* -1 = steering left, +1 = steering right */
static uint16_t holdTicks     = 0;   /* ticks left to keep steering after clear  */

/* turn parameters, set by BeginTurn() */
static int8_t   turnDir          = +1;  /* +1 = turning right, -1 = turning left */
static int32_t  turnMinTicks     = 0;   /* rotation before front sensors are trusted */
static int32_t  turnMaxTicks     = 0;   /* rotation limit */
static uint16_t turnTimeoutTicks = 0;   /* time limit */

/* pending 90 deg turn and how far to reverse before it */
static int8_t   pendingDir  = +1;  /* direction of the turn waiting behind ST_ALIGN */
static int32_t  alignTarget = 0;   /* reverse distance for ST_ALIGN */

/* junction latches and debounce */
static uint8_t  seenLeft       = 0;   /* left branch latched */
static uint8_t  seenRight      = 0;   /* right branch latched */
static uint16_t sideLeftTicks  = 0;   /* how long left outer has been on the line */
static uint16_t sideRightTicks = 0;   /* how long right outer has been on the line */
static uint16_t clearTicks     = 0;   /* how long both outers have been clear */
static uint8_t  sideArmed      = 1;   /* 1 = junction detection enabled */
static uint16_t deadEndTicks   = 0;   /* how long ONLY the back sensor has seen line */

static uint16_t Inc(uint16_t v)   /* saturating ++ (used by turning counters) */
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

/* Back to line-following. The outer sensors must clear before the next
 * junction is accepted, so one junction isn't counted twice. */
static void ResumeFollow(void)
{
    Motor_Stop();
    sideArmed      = 0;
    clearTicks     = 0;
    sideLeftTicks  = 0;
    sideRightTicks = 0;
    deadEndTicks   = 0;
    lineLostTicks  = 0;
    lost           = 0;
    steerDir       = 0;
    holdTicks      = 0;
    EnterState(ST_FOLLOW);
}

/* Stop, pause, then reverse alignBackTicks so the axle is over the junction. */
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
    turnDir          = dir;
    turnMinTicks     = minT;
    turnMaxTicks     = maxT;
    turnTimeoutTicks = timeoutTicks;
    EnterState(ST_TURN_SEEK);
}

/* ---------------- Normal line following (+ junction detection) ---------------- */

static void FollowStep(const SensorState *s)
{
    uint8_t anyOnLine = s->frontLeft | s->frontRight | s->mid | s->back;
    int8_t  want      = 0;
    uint8_t onlyBack  = s->back && !s->frontLeft && !s->frontRight && !s->mid
                        && !s->left && !s->right;   /* line seen only by the back sensor */

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

    /* ---- junction / dead-end detection (turning) ---- */
    sideLeftTicks  = s->left  ? Inc(sideLeftTicks)  : 0u;
    sideRightTicks = s->right ? Inc(sideRightTicks) : 0u;
    deadEndTicks   = onlyBack ? Inc(deadEndTicks)   : 0u;

    /* re-arm once both outer sensors have been off the line for a while */
    if (!sideArmed) {
        if (!s->left && !s->right) {
            clearTicks = Inc(clearTicks);
            if (clearTicks >= TICKS(REARM_CLEAR_MS)) { sideArmed = 1u; }
        } else {
            clearTicks = 0;
        }
    }

    if (sideArmed && (sideRightTicks >= TICKS(SIDE_CONFIRM_MS) ||
                      sideLeftTicks  >= TICKS(SIDE_CONFIRM_MS))) {
        seenRight = (sideRightTicks >= TICKS(SIDE_CONFIRM_MS)) ? 1u : 0u;
        seenLeft  = (sideLeftTicks  >= TICKS(SIDE_CONFIRM_MS)) ? 1u : 0u;

        if (seenRight) {
            BeginAlign(+1, ALIGN_BACKUP_TICKS);     /* priority 1: right */
        } else {
            EnterState(ST_JUNCTION);                /* left only: does the line go straight too? */
        }
        return;
    }
    if (deadEndTicks >= TICKS(DEAD_END_CONFIRM_MS)) {
        Motor_Stop();
        CyDelay(BRAKE_MS);
        deadEndTicks = 0;
        BeginTurn(+1, TURN_180_MIN_TICKS, TURN_180_MAX_TICKS, TICKS(TURN_180_TIMEOUT_MS));
        return;                                     /* priority 4: dead end, spin right */
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

/* ---------------- Turning states ---------------- */

static void RunLineFollower(const SensorState *s)
{
    Enc_Update();
    stateTicks = Inc(stateTicks);

    switch (state) {

    case ST_FOLLOW:
        FollowStep(s);
        break;

    /* Left-only branch: roll forward past the branch width. A right branch
     * showing up late (robot not square) still wins. Then, if mid still sees
     * the line, the line continues straight. */
    case ST_JUNCTION:
        Motor_SetSpeed((int16_t)SPEED_BASE, (int16_t)SPEED_BASE);
        if (s->right) { seenRight = 1u; }
        if (s->left)  { seenLeft  = 1u; }

        if (seenRight) {
            BeginAlign(+1, ALIGN_BACKUP_TICKS + Travel());          /* reverse the roll-on too */
        }
        else if (Travel() >= JUNCTION_CREEP_TICKS
                 || stateTicks >= TICKS(JUNCTION_TIMEOUT_MS)) {
            if (s->mid) {
                ResumeFollow();                                     /* priority 2: straight */
            } else if (seenLeft) {
                BeginAlign(-1, ALIGN_BACKUP_TICKS + Travel());      /* priority 3: left */
            } else {
                ResumeFollow();                                     /* false alarm */
            }
        }
        break;

    /* Reverse so the axle sits over the junction centre, then pivot. */
    case ST_ALIGN:
        Motor_SetSpeed(-(int16_t)SPEED_BASE, -(int16_t)SPEED_BASE);
        if (Travel() >= alignTarget || stateTicks >= TICKS(ALIGN_TIMEOUT_MS)) {
            Motor_Stop();
            CyDelay(SETTLE_MS);
            BeginTurn(pendingDir, TURN_90_MIN_TICKS, TURN_90_MAX_TICKS,
                      TICKS(TURN_90_TIMEOUT_MS));
        }
        break;

    /* Slow spin. The leading front sensor meets the new line first, then the
     * SECOND (trailing) front sensor. Nothing is accepted before the minimum
     * angle so the old line can't fool us. */
    case ST_TURN_SEEK: {
        int32_t progress = Travel();
        uint8_t trail    = (turnDir > 0) ? s->frontLeft : s->frontRight;        /* the second front sensor to hit the line */

        Motor_Spin(turnDir, (progress < turnMinTicks) ? (int16_t)SPEED_TURN_COARSE
                                                      : (int16_t)SPEED_TURN_FINE);

        if (progress >= turnMaxTicks || stateTicks >= turnTimeoutTicks) {
            Motor_Stop();                 /* never found the line - give up */
            CyDelay(SETTLE_MS);
            ResumeFollow();
        }
        else if (progress >= turnMinTicks && trail) {
            Motor_Stop();                 /* second front sensor hit the line: right ourselves */
            CyDelay(SETTLE_MS);
            EnterState(ST_TURN_RIGHTING);
        }
        break;
    }

    /* Slightly past centre: the trailing front sensor is on the line. Creep
     * back the other way until it lets go - the line is then between the
     * pair. If the leading sensor picks it up we've come back through centre,
     * which also counts as done. */
    case ST_TURN_RIGHTING: {
        uint8_t trail = (turnDir > 0) ? s->frontLeft  : s->frontRight;
        uint8_t lead  = (turnDir > 0) ? s->frontRight : s->frontLeft;

        if (!trail || lead || stateTicks >= TICKS(RIGHTING_TIMEOUT_MS)) {
            Motor_Stop();
            CyDelay(SETTLE_MS);
            ResumeFollow();
        } else {
            Motor_Spin((int8_t)-turnDir, (int16_t)SPEED_RIGHTING);
        }
        break;
    }
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
    QuadDec_L_Start();            /* encoders only needed for turning */
    QuadDec_R_Start();
    QuadDec_L_SetCounter(0);
    QuadDec_R_SetCounter(0);
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