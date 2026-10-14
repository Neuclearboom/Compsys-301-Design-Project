/* ==========================================================================
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
 * ========================================================================== */

#include "project.h"
#include <stdint.h>

/* ---------------- Configuration ---------------- */

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

typedef struct {
    uint8_t frontLeft;
    uint8_t frontRight;
    uint8_t mid;
    uint8_t back;
    uint8_t left;
    uint8_t right;
} SensorState;

static uint8_t ReadSensor(uint8_t rawPinValue)
{
    return (rawPinValue == SENSOR_ACTIVE_LEVEL) ? 1u : 0u;
}

static void Sensors_Read(SensorState *s)
{
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
    }
}

static void Motor_Right_SetSpeed(int16_t speed)
{
    if (speed > (int16_t)PWM_MAX_COMPARE)  { speed = (int16_t)PWM_MAX_COMPARE; }
    if (speed < -(int16_t)PWM_MAX_COMPARE) { speed = -(int16_t)PWM_MAX_COMPARE; }

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

    for (;;) {
        SensorState sensors;
        Sensors_Read(&sensors);
        RunLineFollower(&sensors);
        CyDelay(10); /* ~100 Hz control loop */
    }
}