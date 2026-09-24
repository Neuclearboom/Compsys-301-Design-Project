/* ==========================================================================
 * Line-Following Robot Control — matched to your existing TopDesign
 *   (Motor test / Design01.cydsn, CY8C5888LTI-LP097)
 * ==========================================================================
 * Sensor pins (already in your TopDesign, PSoC pins per your schematic trace):
 *   S1 = P1.5    S2 = P1.6    S3 = P1.7
 *   S4 = P2.5    S5 = P2.6    S6 = P2.7
 *
 * Physical role assignment (best-effort from your board photos — confirm
 * on the bench, easy to swap below if a role is wrong):
 *   S3 = front-left     S2 = front-right
 *   S6 = mid            S4 = back 
 *   S1 = left           S5 = right
 *
 * ========================================================================== */

#include "project.h"
#include <stdint.h>

/* ---------------- Configuration ---------------- */

#define SENSOR_ACTIVE_LEVEL   1u     /* 1 = HIGH means "on line" */
#define PWM_MAX_COMPARE       255u   /* match PWM_1 / PWM_2's configured Period */
#define SPEED_BASE            40u    /* forward speed while driving straight - tune on the bench */
#define SPEED_TURN            42u   /* speed used on BOTH wheels while turning */

#define LINE_LOST_THRESHOLD   20u    /* consecutive empty reads before giving up, ~100 Hz -> 200 ms */
#define TURN_TIMEOUT          40u    /* max loop iterations stuck in one turn, ~100 Hz -> 400 ms - hard cap */
#define TURN_COOLDOWN         30u    /* forced-straight loops after a TIMED-OUT turn, ignoring left/right, ~300 ms */

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

/* positive = forward, negative = reverse, magnitude clamped to PWM_MAX_COMPARE */
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
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_LINE_LOST
} RobotState;

static RobotState state = STATE_DRIVE_STRAIGHT;
static uint16_t lineLostCounter = 0;
static uint16_t turnCounter = 0;
static uint16_t suppressSideCounter = 0;  /* set on a timed-out turn, forces a clean straight period */

static void RunLineFollower(const SensorState *s)
{
    uint8_t anyOnLine = s->left | s->frontLeft | s->mid | s->frontRight | s->right | s->back;

    if (!anyOnLine) {
        lineLostCounter++;
        if (lineLostCounter >= LINE_LOST_THRESHOLD) {
            state = STATE_LINE_LOST;
        }
        /* else treat a short all-off read as a gap, not a loss */
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
                state = STATE_TURN_LEFT;
                turnCounter = 0;
            } else if (s->right) {
                state = STATE_TURN_RIGHT;
                turnCounter = 0;
            } else if (s->frontLeft && !s->frontRight) {
                Motor_SetSpeed(SPEED_BASE, SPEED_BASE / 2); /* line drifted right -> nudge right */
            } else if (s->frontRight && !s->frontLeft) {
                Motor_SetSpeed(SPEED_BASE / 2, SPEED_BASE); /* line drifted left -> nudge left */
            } else {
                Motor_SetSpeed(SPEED_BASE, SPEED_BASE); /* centered, or only mid/back still seeing it */
            }
            break;

        case STATE_TURN_LEFT:
            Motor_SetSpeed(-(int16_t)SPEED_TURN, (int16_t)SPEED_TURN);
            turnCounter++;
            /* exit once the line has swept under to the OPPOSITE front
             * sensor and the mid sensor - i.e. the robot has pivoted far
             * enough that the line now sits under frontRight+mid */
            if (s->mid && s->frontRight) {
                state = STATE_DRIVE_STRAIGHT;
                turnCounter = 0;
            } else if (turnCounter >= TURN_TIMEOUT) {
                /* mid+frontRight never lined up - force a clean break
                 * instead of hanging here forever */
                state = STATE_DRIVE_STRAIGHT;
                turnCounter = 0;
                suppressSideCounter = TURN_COOLDOWN;
            }
            break;

        case STATE_TURN_RIGHT:
            Motor_SetSpeed((int16_t)SPEED_TURN, -(int16_t)SPEED_TURN);
            turnCounter++;
            if (s->mid && s->frontLeft) {
                state = STATE_DRIVE_STRAIGHT;
                turnCounter = 0;
            } else if (turnCounter >= TURN_TIMEOUT) {
                state = STATE_DRIVE_STRAIGHT;
                turnCounter = 0;
                suppressSideCounter = TURN_COOLDOWN;
            }
            break;

        case STATE_LINE_LOST:
            Motor_Stop();
            /* Recovery is a judgement call - natural place to hand off to a
             * search rotation or your BFS turn logic later. Stops for now. */
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