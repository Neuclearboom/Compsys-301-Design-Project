/* ==========================================================================
<<<<<<< HEAD
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
=======
 * Line-Following Robot
 * ==========================================================================
 * Sensor pins:
 *   S1 = P1.5    S2 = P1.6    S3 = P1.7
 *   S4 = P2.5    S5 = P2.6    S6 = P2.7
 *
 * Physical role assignment:
 *   S3 = front-left     S2 = front-right
 *   S6 = mid            S4 = back
 *   S5 = left           S1 = right   (swapped - matched turning the correct
 *                                     direction on the bench)
>>>>>>> 6448ee8 (added working version of project. Must be built locally to work)
 * ========================================================================== */

#include "project.h"
#include <stdint.h>

/* ---------------- Configuration ---------------- */

<<<<<<< HEAD
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
=======
#define SENSOR_ACTIVE_LEVEL   0u
#define PWM_MAX_COMPARE       255u
#define SPEED_BASE            30u
#define SPEED_TURN            60u
#define SPEED_TURN_AROUND     40u         /* spin speed for the dead-end 180 only - the normal left/right
                                            * turns keep using SPEED_TURN */

#define LINE_LOST_THRESHOLD   20u
#define TURN_TIMEOUT          60u
#define TURN_COOLDOWN         30u
#define POST_TURN_COOLDOWN    20u

#define DEAD_END_CONFIRM_ITERATIONS  3u   /* back-only must hold this many ticks in a row before the 180 starts (1 = immediate) */
#define U_TURN_TIMEOUT        120u        /* safety cap for the 180 - roughly 2x TURN_TIMEOUT since it covers about twice the angle */

#define TURN_BACKUP_BRAKE_MS      50u
#define TURN_BACKUP_ITERATIONS    15u
#define POST_TURN_ADJUST_ITERATIONS  6u  /* safety cap only now - see STATE_ADJUST_LEFT/RIGHT below */

#define DRIFT_CORRECTION_OFFSET   5u  /* how much slower the inside wheel runs during a drift-correction
                                        * nudge (straight-line AND post-turn adjust). Small on purpose -
                                        * this used to be a 33% cut (SPEED_BASE / 1.5), which was strong
                                        * enough to overshoot past center and oscillate. Bump it up only a
                                        * little at a time if corrections now feel too weak/slow. */

/* ---------------- Sensor reading ---------------- */
>>>>>>> 6448ee8 (added working version of project. Must be built locally to work)

typedef struct {
    uint8_t frontLeft;
    uint8_t frontRight;
    uint8_t mid;
    uint8_t back;
<<<<<<< HEAD
} SensorState;

static uint8_t OnLine(uint8_t raw)
{
    return (raw == SENSOR_ACTIVE_LEVEL) ? 1u : 0u;
=======
    uint8_t left;
    uint8_t right;
} SensorState;

static uint8_t ReadSensor(uint8_t rawPinValue)
{
    return (rawPinValue == SENSOR_ACTIVE_LEVEL) ? 1u : 0u;
>>>>>>> 6448ee8 (added working version of project. Must be built locally to work)
}

static void Sensors_Read(SensorState *s)
{
<<<<<<< HEAD
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
=======
    s->frontLeft  = ReadSensor(S3_Read());
    s->frontRight = ReadSensor(S2_Read());
    s->mid        = ReadSensor(S6_Read());
    s->back       = ReadSensor(S4_Read());
    s->left       = ReadSensor(S5_Read());
    s->right      = ReadSensor(S1_Read());
}

/* ---------------- Motor drive ---------------- */

/* positive = forward, negative = reverse */
static void Motor_Left_SetSpeed(int16_t speed)
{
    if (speed > (int16_t)PWM_MAX_COMPARE)  { speed = (int16_t)PWM_MAX_COMPARE; }
    if (speed < -(int16_t)PWM_MAX_COMPARE) { speed = -(int16_t)PWM_MAX_COMPARE; }

    if (speed >= 0) {
        PWM_1_WriteCompare1((uint8_t)speed);
        PWM_1_WriteCompare2(0u);
    } else {
        PWM_1_WriteCompare1(0u);
        PWM_1_WriteCompare2((uint8_t)(-speed));
>>>>>>> 6448ee8 (added working version of project. Must be built locally to work)
    }
}

static void Motor_Right_SetSpeed(int16_t speed)
{
<<<<<<< HEAD
    speed = Clamp(speed);
=======
    if (speed > (int16_t)PWM_MAX_COMPARE)  { speed = (int16_t)PWM_MAX_COMPARE; }
    if (speed < -(int16_t)PWM_MAX_COMPARE) { speed = -(int16_t)PWM_MAX_COMPARE; }

>>>>>>> 6448ee8 (added working version of project. Must be built locally to work)
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

<<<<<<< HEAD
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
=======
/* ---------------- Line-following state machine ---------------- */

typedef enum {
    STATE_DRIVE_STRAIGHT,
    STATE_TURN_BACKUP,   /* brief brake + reverse, entered right before a turn */
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_ADJUST_LEFT,   /* brief re-centering nudge after STATE_TURN_LEFT's sensor trigger */
    STATE_ADJUST_RIGHT,  /* same, mirrored, after STATE_TURN_RIGHT's sensor trigger */
    STATE_TURN_AROUND,   /* dead-end 180 - entered when the line is seen only on the back sensor */
    STATE_LINE_LOST
} RobotState;

static RobotState state             = STATE_DRIVE_STRAIGHT;
static RobotState pendingTurnState  = STATE_DRIVE_STRAIGHT;  /* which turn to enter once backup finishes */
static uint16_t   lineLostCounter   = 0;
static uint16_t   turnCounter       = 0;
static uint16_t   backupCounter     = 0;
static uint16_t   adjustCounter     = 0;
static uint16_t   suppressSideCounter = 0;  /* set on a timed-out turn, forces a clean straight period */
static uint16_t   deadEndCounter    = 0;    /* consecutive ticks where ONLY the back sensor sees the line */

static void RunLineFollower(const SensorState *s)
{
    uint8_t anyOnLine = s->left | s->frontLeft | s->mid | s->frontRight | s->right | s->back;
    uint8_t onlyBack  = s->back && !s->left && !s->frontLeft && !s->mid && !s->frontRight && !s->right;

    /* dead-end detector: counts consecutive ticks of "back sensor only" while driving straight */
    deadEndCounter = (onlyBack && state == STATE_DRIVE_STRAIGHT) ? (deadEndCounter + 1u) : 0u;

    if (state != STATE_TURN_LEFT && state != STATE_TURN_RIGHT && state != STATE_TURN_BACKUP
        && state != STATE_ADJUST_LEFT && state != STATE_ADJUST_RIGHT && state != STATE_TURN_AROUND) {
        if (!anyOnLine) {
            lineLostCounter++;
            if (lineLostCounter >= LINE_LOST_THRESHOLD) {
                state = STATE_LINE_LOST;
            }
            /* else treat a short all-off read as a gap, not a loss */
        } else {
            lineLostCounter = 0;
        }
    } else {
        lineLostCounter = 0;
    }

    switch (state) {

        case STATE_DRIVE_STRAIGHT:
            if (suppressSideCounter > 0) {
                /* just forced out of a turn that never cleared on its own -
                 * ignore left/right for a bit so a chronically-triggered
                 * sensor can't instantly throw us back into the same turn */
                suppressSideCounter--;
                Motor_SetSpeed(SPEED_BASE, SPEED_BASE);
            } else if (s->left) {
                /* brake, then back up before attempting the turn - by the
                 * time this side sensor sees the branch the robot has
                 * usually already driven past the actual junction */
                Motor_Stop();
                CyDelay(TURN_BACKUP_BRAKE_MS);
                pendingTurnState = STATE_TURN_LEFT;
                backupCounter = 0;
                state = STATE_TURN_BACKUP;
            } else if (s->right) {
                Motor_Stop();
                CyDelay(TURN_BACKUP_BRAKE_MS);
                pendingTurnState = STATE_TURN_RIGHT;
                backupCounter = 0;
                state = STATE_TURN_BACKUP;
            } else if (deadEndCounter >= DEAD_END_CONFIRM_ITERATIONS) {
                /* line seen ONLY on the back sensor - dead end, spin around */
                state = STATE_TURN_AROUND;
                turnCounter = 0;
                deadEndCounter = 0;
            } else if (s->frontLeft && !s->frontRight) {
                Motor_SetSpeed(SPEED_BASE, SPEED_BASE - DRIFT_CORRECTION_OFFSET); /* line drifted right -> nudge right */
            } else if (s->frontRight && !s->frontLeft) {
                Motor_SetSpeed(SPEED_BASE - DRIFT_CORRECTION_OFFSET, SPEED_BASE); /* line drifted left -> nudge left */
            } else {
                Motor_SetSpeed(SPEED_BASE, SPEED_BASE); /* centered, or only mid/back still seeing it */
            }
            break;

        case STATE_TURN_BACKUP:
            Motor_SetSpeed(-(int16_t)SPEED_BASE, -(int16_t)SPEED_BASE);
            backupCounter++;
            if (backupCounter >= TURN_BACKUP_ITERATIONS) {
                state = pendingTurnState;   /* backed up - now actually attempt the turn */
                turnCounter = 0;
                backupCounter = 0;
            }
            break;

        case STATE_TURN_LEFT:
            Motor_SetSpeed(-(int16_t)SPEED_TURN, (int16_t)SPEED_TURN);
            turnCounter++;
            /* exit as soon as the OPPOSITE front sensor sees the line -
             * doesn't wait for mid too, so it can still be off-center at
             * the exact instant this fires. STATE_ADJUST_LEFT closes that
             * remaining gap before handing off to straight driving. */
            if (s->frontRight) {
                state = STATE_ADJUST_LEFT;
                turnCounter = 0;
                adjustCounter = 0;
                suppressSideCounter = POST_TURN_COOLDOWN;
            } else if (turnCounter >= TURN_TIMEOUT) {
                /* frontRight never lit up - force a clean break instead of
                 * hanging here forever */
                state = STATE_DRIVE_STRAIGHT;
                turnCounter = 0;
                suppressSideCounter = TURN_COOLDOWN;
            }
            break;

        case STATE_TURN_RIGHT:
            Motor_SetSpeed((int16_t)SPEED_TURN, -(int16_t)SPEED_TURN);
            turnCounter++;
            if (s->frontLeft) {
                state = STATE_ADJUST_RIGHT;
                turnCounter = 0;
                adjustCounter = 0;
                suppressSideCounter = POST_TURN_COOLDOWN;
            } else if (turnCounter >= TURN_TIMEOUT) {
                state = STATE_DRIVE_STRAIGHT;
                turnCounter = 0;
                suppressSideCounter = TURN_COOLDOWN;
            }
            break;

        case STATE_TURN_AROUND:
            /* in-place spin left; exits as soon as the OPPOSITE front sensor
             * (frontRight) sees the line, straight into driving - no adjust
             * step */
            Motor_SetSpeed(-(int16_t)SPEED_TURN_AROUND, (int16_t)SPEED_TURN_AROUND);
            turnCounter++;
            if (s->frontRight) {
                state = STATE_DRIVE_STRAIGHT;
                turnCounter = 0;
                suppressSideCounter = POST_TURN_COOLDOWN;
            } else if (turnCounter >= U_TURN_TIMEOUT) {
                /* frontRight never lit up - force a clean break */
                state = STATE_DRIVE_STRAIGHT;
                turnCounter = 0;
                suppressSideCounter = TURN_COOLDOWN;
            }
            break;

        case STATE_ADJUST_LEFT:
            /* same direction as the turn just taken, but gentle and still
             * moving forward. Sensor-reactive: exits the instant mid or
             * BOTH front sensors confirm it's actually centered, instead of
             * blindly running the full duration and driving past center -
             * POST_TURN_ADJUST_ITERATIONS is now just a backstop in case
             * neither condition is ever met. */
            Motor_SetSpeed(SPEED_BASE - DRIFT_CORRECTION_OFFSET, SPEED_BASE);
            adjustCounter++;
            if (s->mid || (s->frontLeft && s->frontRight) || (adjustCounter >= POST_TURN_ADJUST_ITERATIONS)) {
                state = STATE_DRIVE_STRAIGHT;
                adjustCounter = 0;
            }
            break;

        case STATE_ADJUST_RIGHT:
            Motor_SetSpeed(SPEED_BASE, SPEED_BASE - DRIFT_CORRECTION_OFFSET);
            adjustCounter++;
            if (s->mid || (s->frontLeft && s->frontRight) || (adjustCounter >= POST_TURN_ADJUST_ITERATIONS)) {
                state = STATE_DRIVE_STRAIGHT;
                adjustCounter = 0;
            }
            break;

        case STATE_LINE_LOST:
            Motor_Stop();
            if (anyOnLine) {
                state = STATE_DRIVE_STRAIGHT;
                lineLostCounter = 0;
            }
            break;
>>>>>>> 6448ee8 (added working version of project. Must be built locally to work)
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
<<<<<<< HEAD
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
=======
>>>>>>> 6448ee8 (added working version of project. Must be built locally to work)

    for (;;) {
        SensorState sensors;
        Sensors_Read(&sensors);
        RunLineFollower(&sensors);
<<<<<<< HEAD
        CyDelay(LOOP_MS);
=======
        CyDelay(10); /* ~100 Hz control loop */
>>>>>>> 6448ee8 (added working version of project. Must be built locally to work)
    }
}