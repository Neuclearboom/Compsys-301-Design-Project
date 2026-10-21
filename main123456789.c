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
 *
 * Path following: BFS on mazeMap from START through every food stop to the
 * GOAL (see "Path planning inputs" below). The robot stops FOOD_PAUSE_MS at
 * every food stop and then carries on. FOLLOW_PLANNED_PATH 0u = plain
 * reactive behaviour, as before.
 * ========================================================================== */

#include "project.h"
#include <stdint.h>

/* ---------------- Configuration ---------------- */

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

#define DEAD_END_CONFIRM_ITERATIONS  3u   /* back-only must hold this many ticks in a row before the 180 starts (1 = immediate) */
#define U_TURN_TIMEOUT        120u        /* safety cap for the 180 - roughly 2x TURN_TIMEOUT since it covers about twice the angle */

/* ---- Path planning inputs ---- */
#define FOLLOW_PLANNED_PATH   1u          /* 1u = BFS path following, 0u = old reactive behaviour */

/* row/col on mazeMap below (row 0 = top, col 0 = left). Coordinates given as (x,y) from
 * the bottom-left corner (1,1) convert as row = 15 - y, col = x - 1. */
#define START_ROW             7u          /* start_pos {7,1} from generated_route.h = {row, col}: a dead end whose only exit is east */
#define START_COL             1u
#define GOAL_ROW              4u          /* the finish is the last fruit, {4,7} - the robot stops there for good */
#define GOAL_COL              7u
#define START_HEADING         HEADING_EAST  /* direction the robot faces at boot: HEADING_NORTH (up the map),
                                            * HEADING_EAST, HEADING_SOUTH or HEADING_WEST. Has to match how
                                            * the robot is actually placed on the start cell. The start is a
                                            * dead end whose only exit is east. */

/* Food (fruit) to pick up, visited in this order, before finishing at the GOAL above.
 * Rows are {row, col} straight from food_list in generated_route.h.
 * NUM_FOOD says how many rows of food_list are used (0 to MAX_FOOD);
 * rows past that are ignored. {row, col} on mazeMap. Every food stop must be a
 * dead end, corner or junction - the sensors need something to recognise it
 * by - otherwise the plan is refused and the robot stays put. */
#define MAX_FOOD              5u
#define NUM_FOOD              4u          /* F5 is the finish (GOAL above), so it is not in this list */
static const uint8_t food_list[MAX_FOOD][2] = {
    {  4u, 5u },    /* F1: junction (T)                       - sensor stop */
    { 11u, 3u },    /* F2: STRAIGHT corridor                  - encoder stop */
    {  1u, 1u },    /* F3: dead end                           - pause, then 180 */
    {  5u, 9u },    /* F4: corner                             - sensor stop */
    {  4u, 7u }     /* F5: junction (T) = FINISH, not used (NUM_FOOD is 4) */
};
#if NUM_FOOD > MAX_FOOD
#error NUM_FOOD can be at most MAX_FOOD (5)
#endif

#define PATH_TURNS_MIRRORED   1u         /* 1u = on this robot the route's LEFT turns came out as right turns on the
                                            * bench, so the route's left and right are swapped (and the 180 sweeps the
                                            * other side). Only affects BFS path following. 0u = turns go the way
                                            * they are named. */

#define FOOD_PAUSE_MS         2000u       /* stand still this long at every food stop, then carry on
                                            * (the GOAL is not a food stop - the robot stops there for good) */

/* ---- Encoder distance: food stops in the MIDDLE of a straight corridor ----
 * Junctions, corners and dead ends are found by the side/back sensors. A food cell on a plain
 * straight corridor has nothing the sensors can see, so the robot counts encoder ticks from the
 * last thing it DID see (junction passed, turn finished) and stops after COUNTS_PER_NODE x nodes.
 *
 * Hardware needed in the PSoC schematic: two Quadrature Decoder components (counter resolution x4),
 * one per wheel, fed by the DE3 encoder Channel A / Channel B pins - here QuadDec_L and QuadDec_R.
 * The names are mapped in the ENC_* macros below - change them there if yours are called something else.
 * (A 32-bit counter is fine: only the low 16 bits of the DIFFERENCE are used, which wraps correctly.)
 * At x4 this motor gives 228 counts per wheel revolution (57 cycles x 4 edges, from the spec sheet). */
#define USE_ENCODER_FOOD_STOPS  1u        /* 0u = old behaviour: a food stop must be a junction/corner/dead end */

#define COUNTS_PER_NODE         97       /* encoder counts for the robot to drive from one node to the next.
                                           * Calculated: node 90 mm, wheel 65 mm, 228 counts per wheel turn (x4):
                                           *   distance per count = pi x 65 / 228 = 0.896 mm
                                           *   counts per node    = 90 / 0.896     = 100.5  ->  100
                                           * (The motor is really 3249/169 = 19.2:1 not 19:1, which would give ~102.)
                                           * Fine-tune from a real food-stop run: stops SHORT -> raise it, stops
                                           * PAST the cell -> lower it. */

#define ENC_L_START()           QuadDec_L_Start()
#define ENC_R_START()           QuadDec_R_Start()
#define ENC_L_GET()             QuadDec_L_GetCounter()
#define ENC_R_GET()             QuadDec_R_GetCounter()

#define ODO_CALIBRATION_MODE    0u        /* 1u = motors stay OFF and the robot just reports odoDebugCounts. Push the
                                           * robot along the line BY HAND and read the count in the debugger. Optional:
                                           * use it to check COUNTS_PER_NODE, then set back to 0u. */

#define ODO_OFFSET_AFTER_STRAIGHT  0      /* counts already travelled past the node centre at the moment the robot
                                           * decides a junction it drives STRAIGHT through. Can be negative. */
#define ODO_OFFSET_AFTER_TURN      0      /* same, but at the moment a turn / 180 has finished and driving resumes.
                                           * Tune these two until the robot stops dead centre on the food cell. */
#define MIDSTOP_BRAKE_COUNTS       0      /* straight-fruit calibration: positive stops earlier */

/* Safety nets so a bad encoder can't make the robot drive blind past its junctions. */
#define MIDSTOP_STALL_TICKS     150u      /* while waiting for a plain-corridor food stop, if the robot has driven this many
                                           * ticks (10 ms each = 1.5 s) ... */
#define MIDSTOP_STALL_COUNTS    20        /* ... and the encoders have counted fewer than this, they are not working.
                                           * The fruit is SKIPPED (no pause) and the robot carries on with the route;
                                           * encoderFault is set to 2 so you can see it happened. */
#define BOOT_SETTLE_MS          500u      /* stand still this long after power-up so sensors and encoders settle */
#define BOOT_IGNORE_TICKS       15u       /* then ignore junction / dead-end detection for this many ticks (10 ms each) */
#define ODO_CAL_USE_LED         1u        /* in ODO_CALIBRATION_MODE the board LED flips every COUNTS_PER_NODE of travel,
                                           * so you can test the encoders with no debugger. Set 0u if LED_Write() doesn't
                                           * exist in your project. */

#define SIDE_CLEAR_ITERATIONS 5u          /* both side sensors must read off this many ticks in a row before
                                            * the next junction can be detected - stops one junction being
                                            * counted twice */

#define TURN_BACKUP_BRAKE_MS      50u
#define TURN_BACKUP_ITERATIONS    7u
#define U_TURN_BACKUP_ITERATIONS  20u         /* reverse time before a 180, in 10 ms ticks (90-degree turns use TURN_BACKUP_ITERATIONS) */
#define POST_TURN_ADJUST_ITERATIONS  12u  /* safety cap only now - see STATE_ADJUST_LEFT/RIGHT below */

#define DRIFT_CORRECTION_OFFSET   10u  /* how much slower the inside wheel runs during a drift-correction
                                        * nudge (straight-line AND post-turn adjust). Small on purpose -
                                        * this used to be a 33% cut (SPEED_BASE / 1.5), which was strong
                                        * enough to overshoot past center and oscillate. Bump it up only a
                                        * little at a time if corrections now feel too weak/slow. */

#define TURN_EXIT_NEEDS_WHITE_FIRST 1u   /* 1u = a turn only ends once its exit front sensor has read WHITE and then BLACK.
                                          * A sensor already on black when the turn starts does not end it instantly. */

#define BACK_UP_BEFORE_180 1u   /* 1u = every 180 first brakes and reverses a little (the same backup the 90-degree turns use) */

#define IGNORE_SIDE_DURING_180 1u   /* 1u = for the whole 180 - the reverse before it and the spin, until the front sensors end the
                                     * turn - the left and right side sensors are read as OFF and junction counting is paused.
                                     * 0u = they are read normally. */

#define SKIP_FIRST_LINE_AT_T_180 1u   /* 1u = a 180 at a T junction or crossing (both sides open, so the spin has to sweep past a side
                                       * branch) ignores the FIRST line the front sensors find - that is the side branch, about 90 degrees
                                       * round - and only ends on the SECOND one, the line the robot is turning back onto. 180s at dead
                                       * ends and corners are not affected. 0u = off. */

/* ---- Turn finishing (stops the robot ending a turn pointing off the line) ----
 * The old rule ended a turn the moment the FIRST front sensor touched the new line. That sensor is the
 * one leading the rotation, so it touches the line while the robot is still roughly 20-35 degrees short of
 * pointing along it - and then the robot drove off at that angle. Now the turn only ends once BOTH front
 * sensors have found the line (each reading white, then black), which happens when the robot is lined up
 * with it. Once one has found it the spin slows to SPEED_TURN_FINAL so it doesn't sail past. */
#define TURN_EXIT_BOTH_FRONT  1u   /* 0u = old behaviour: end the turn when the first front sensor sees the line */
#define SPEED_TURN_FINAL      30u  /* spin speed after the first front sensor has found the line */
#define STEER_DURING_COOLDOWN 1u   /* 1u = the front sensors keep steering during the post-turn cooldown (only the
                                    * side-sensor junction detection is suppressed). 0u = old: drive dead straight. */
#define TURN_USE_ADJUST       0u   /* 1u = after a 90-degree turn, do the gentle forward curve (STATE_ADJUST_*) first.
                                    * Off by default: that curve kept steering the same way the turn went. */

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

/* ---------------- Debug: why did the robot stop? ----------------
 * Add these to the debugger's Watch window. After a surprise stop, stopReason says which
 * rule fired and routeIndex / routeLen say where in the plan the robot thought it was. */
typedef enum {
    STOP_NONE = 0,
    STOP_PLAN_FAILED,          /* Route_Plan refused the inputs at boot */
    STOP_GOAL,                 /* reached the goal - normal */
    STOP_ROUTE_RAN_OUT,        /* a junction was seen after the last route command (a junction got counted twice, or one was missed) */
    STOP_DEAD_END_UNEXPECTED,  /* only the back sensor saw the line and the plan did not expect a dead end here */
    STOP_LINE_LOST             /* every sensor off for LINE_LOST_THRESHOLD ticks (clears itself if a sensor sees the line again) */
} StopReason;
static volatile StopReason stopReason = STOP_NONE;

/* ---------------- Encoder distance ---------------- */

static volatile uint8_t encoderFault = 0u;   /* 0 = fine, 1 = one wheel's encoder not counting (other one used),
                                              * 2 = encoders not counting at all, a fruit was skipped */
static uint16_t         midWatchTicks = 0u;  /* ticks driven while waiting for a CMD_MIDSTOP */

#if USE_ENCODER_FOOD_STOPS
static int16_t         odoMarkL = 0;
static int16_t         odoMarkR = 0;
static int32_t         odoOffset = 0;
static volatile int32_t odoDebugCounts = 0;   /* watch this to calibrate COUNTS_PER_NODE */
#endif

static void Odo_Start(void)
{
#if USE_ENCODER_FOOD_STOPS
    ENC_L_START();
    ENC_R_START();
#endif
}

/* Start counting from here. offset = counts the robot is already past the node centre. */
static void Odo_Mark(int32_t offset)
{
#if USE_ENCODER_FOOD_STOPS
    odoMarkL  = (int16_t)ENC_L_GET();
    odoMarkR  = (int16_t)ENC_R_GET();
    odoOffset = offset;
#else
    (void)offset;
#endif
}

/* Average distance moved by the two wheels since Odo_Mark(), in encoder counts. The int16 subtraction
 * wraps correctly if the counter rolls over. Only meant for driving straight ahead: the mark is always
 * taken AFTER a turn or reverse has finished, and the robot never reverses between a mark and a food stop. */
static int32_t Odo_Counts(void)
{
#if USE_ENCODER_FOOD_STOPS
    int32_t dL = (int16_t)((int16_t)ENC_L_GET() - odoMarkL);
    int32_t dR = (int16_t)((int16_t)ENC_R_GET() - odoMarkR);
    if (dL < 0) { dL = -dL; }   /* the motors face opposite ways, so one decoder counts down when driving */
    if (dR < 0) { dR = -dR; }   /* forward. Using the size of each change means the direction doesn't matter. */
    {
        int32_t hi = (dL > dR) ? dL : dR;
        int32_t lo = (dL > dR) ? dR : dL;
        int32_t dist = (dL + dR) / 2;
        if (hi >= 40 && (lo * 4) < hi) {
            dist = hi;             /* one wheel's encoder reads under a quarter of the other's: it is not counting.
                                    * Trust the working one instead of averaging in a zero. */
            encoderFault = 1u;
        }
        odoDebugCounts = dist + odoOffset;
    }
    return odoDebugCounts;
#else
    return 0;
#endif
}

/* ---------------- Path planning (BFS) ---------------- */

#define ROWS       15
#define COLS       19
#define MAX_NODES  (ROWS * COLS)

typedef struct {
    int8_t r, c;
} Cell;

/* Map from map_editor.py (generated_route.h). MAP_LINE_VALUE says which digit means "line here" -
 * the rest is wall. In this map 1 = line and the whole outer border is 0. */
#define MAP_LINE_VALUE  1u
static const uint8_t mazeMap[ROWS][COLS] = {
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0},
    {0,1,0,1,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0},
    {0,1,0,1,0,1,0,1,0,1,1,1,0,1,1,1,0,1,0},
    {0,1,0,1,0,1,1,1,0,0,0,1,0,0,0,0,0,1,0},
    {0,1,1,1,1,1,0,1,1,1,0,1,0,1,1,1,1,1,0},
    {0,0,0,0,0,0,0,1,0,1,0,1,0,1,0,0,0,1,0},
    {0,1,1,1,1,1,1,1,0,1,0,1,0,1,0,1,1,1,0},
    {0,0,0,1,0,1,0,0,0,0,0,1,0,1,0,1,0,0,0},
    {0,1,1,1,0,1,0,1,1,1,0,1,1,1,0,1,1,1,0},
    {0,1,0,0,0,1,0,1,0,1,0,1,0,0,0,0,0,1,0},
    {0,1,1,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,0},
    {0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0},
    {0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
};

typedef enum {
    HEADING_NORTH = 0,   /* toward row 0 (up the map) */
    HEADING_EAST  = 1,
    HEADING_SOUTH = 2,
    HEADING_WEST  = 3
} Heading;

/* values chosen so that (outHeading - inHeading) & 3 gives the command directly */
typedef enum {
    CMD_STRAIGHT = 0,
    CMD_RIGHT    = 1,
    CMD_AROUND   = 2,
    CMD_LEFT     = 3,
    CMD_STOP     = 4,
    CMD_AROUND_RIGHT = 5,  /* the 180 again, but spinning RIGHT - see Map_UTurnCmd */
    CMD_MIDSTOP  = 6       /* food stop on a plain corridor: drive midDist[] nodes (by encoder) from the last
                            * command, stand still FOOD_PAUSE_MS, then carry on. No junction event involved. */
} PlanCmd;

/* A route entry is a PlanCmd in the low bits. ROUTE_FOOD_FLAG marks the junction /
 * corner / dead end that IS a food stop: the robot stands still there for
 * FOOD_PAUSE_MS first, then carries out the command. */
#define ROUTE_CMD_MASK   0x07u
#define ROUTE_FOOD_FLAG  0x80u
#define ROUTE_IMMEDIATE_FLAG 0x40u   /* do this command straight after the CMD_MIDSTOP before it, not on the next
                                      * junction event (a 180 on the spot at a food cell in a plain corridor) */
#define ROUTE_SKIP_FLAG  0x20u       /* on a CMD_AROUND / CMD_AROUND_RIGHT: the spin sweeps a side branch, so it must ignore the
                                      * first line the front sensors find (see SKIP_FIRST_LINE_AT_T_180) */

typedef struct {
    Cell     q[MAX_NODES];
    uint16_t front, back;
} Queue;

static void q_init(Queue *q) { q->front = q->back = 0u; }
static uint8_t q_empty(Queue *q) { return (q->front == q->back) ? 1u : 0u; }
static void q_push(Queue *q, Cell v) { q->q[q->back++] = v; }
static Cell q_pop(Queue *q) { return q->q[q->front++]; }

/* Same BFS as bfs.c. The working arrays are static (not locals) - together
 * they are a few KB, which would overflow PSoC's default stack. */
static uint8_t bfs_path(const uint8_t m[ROWS][COLS],
                        Cell start,
                        Cell goal,
                        Cell out_path[MAX_NODES],
                        uint16_t *out_len)
{
    static uint8_t visited[ROWS][COLS];
    static Cell    parent[ROWS][COLS];
    static Queue   q;
    static const int8_t dr[4] = {-1, 1, 0, 0};
    static const int8_t dc[4] = {0, 0, -1, 1};
    uint8_t r, c, k;
    int16_t nr, nc;

    for (r = 0u; r < ROWS; r++) {
        for (c = 0u; c < COLS; c++) {
            visited[r][c] = 0u;
        }
    }
    q_init(&q);

    visited[start.r][start.c] = 1u;
    parent[start.r][start.c] = (Cell){-1, -1};
    q_push(&q, start);

    while (!q_empty(&q)) {
        Cell cur = q_pop(&q);

        if (cur.r == goal.r && cur.c == goal.c) {
            uint16_t len = 0u;
            uint16_t i;
            Cell p = goal;

            while (p.r != -1) {
                out_path[len++] = p;
                p = parent[p.r][p.c];
            }

            for (i = 0u; i < len / 2u; i++) {
                Cell tmp = out_path[i];
                out_path[i] = out_path[len - 1u - i];
                out_path[len - 1u - i] = tmp;
            }

            *out_len = len;
            return 1u;
        }

        for (k = 0u; k < 4u; k++) {
            nr = cur.r + dr[k];
            nc = cur.c + dc[k];

            if (nr < 0 || nr >= ROWS || nc < 0 || nc >= COLS) continue;
            if (m[nr][nc] != MAP_LINE_VALUE) continue;
            if (visited[nr][nc]) continue;

            visited[nr][nc] = 1u;
            parent[nr][nc] = cur;
            q_push(&q, (Cell){(int8_t)nr, (int8_t)nc});
        }
    }

    return 0u;
}

/* ---- Path -> junction commands ----
 * The robot has no way to count cells along a straight corridor, so the path
 * is reduced to the places the side sensors can actually see: every cell on
 * the path that is a corner or has 3+ open neighbours (a junction). One
 * command per such cell, in order:
 *   CMD_LEFT / CMD_RIGHT : turn at that junction/corner
 *   CMD_STRAIGHT         : junction with a side branch, but the path goes straight on
 *   CMD_AROUND           : the path turns back the way it came (180), spinning left
 *   CMD_AROUND_RIGHT     : same 180 spinning right - used when a branch of the line leaves
 *                          on the left, which a left spin would stop early on
 *   CMD_STOP             : the goal
 * Plain corridor cells (exactly 2 open neighbours, in a straight line) make no
 * command. With PATH_TURNS_MIRRORED the LEFT / RIGHT entries are stored swapped, so
 * they are what the robot's STATE_TURN_LEFT / STATE_TURN_RIGHT must be asked to do.
 *
 * Food stops: the route is built leg by leg - start -> food stop 1 -> food stop 2
 * -> ... -> GOAL - with a separate BFS for each leg, joined into one command
 * list. The command at a food stop's cell is flagged ROUTE_FOOD_FLAG so the
 * robot pauses there. That means every food stop has to be a dead end, a
 * corner or a junction (the sensors need something to recognise it by) - a
 * food stop on a plain straight corridor cell makes the plan be refused. If
 * the next leg leaves a food stop back the way it arrived (CMD_AROUND), the
 * robot pauses and then does a 180 there. */

#define MAX_ROUTE_CMDS  (MAX_NODES * (MAX_FOOD + 1u))

static const int8_t headingDr[4] = {-1, 0, 1, 0};   /* N E S W */
static const int8_t headingDc[4] = {0, 1, 0, -1};

static Cell     legPath[MAX_NODES];                 /* BFS result for the leg being built */
static uint16_t legLen         = 0u;
static uint8_t  route[MAX_ROUTE_CMDS];
static uint16_t routeLen       = 0u;
static uint16_t routeIndex     = 0u;
static PlanCmd  initialCmd     = CMD_STRAIGHT;   /* turn needed at the start cell to face the first move */
static uint8_t  initialSkip    = 0u;             /* 1 if that turn is a 180 that sweeps a side branch */
static uint8_t  midDist[MAX_FOOD];                /* nodes to drive for each CMD_MIDSTOP, in route order */
static uint8_t  midCount       = 0u;
static uint8_t  midIndex       = 0u;

static uint8_t Map_IsOpen(int16_t r, int16_t c)
{
    return (r >= 0 && r < ROWS && c >= 0 && c < COLS && mazeMap[r][c] == MAP_LINE_VALUE) ? 1u : 0u;
}

static uint8_t Map_OpenNeighbours(Cell p)
{
    uint8_t k;
    uint8_t n = 0u;
    for (k = 0u; k < 4u; k++) {
        n += Map_IsOpen(p.r + headingDr[k], p.c + headingDc[k]);
    }
    return n;
}

/* 0 for a plain straight corridor cell (exactly two open neighbours, opposite
 * each other) - nothing for the sensors to pick out. 1 for everything else:
 * dead end, corner, junction. */
static uint8_t Map_IsDetectable(Cell p)
{
    if (Map_OpenNeighbours(p) == 2u) {
        uint8_t north = Map_IsOpen(p.r - 1, p.c);
        uint8_t south = Map_IsOpen(p.r + 1, p.c);
        uint8_t east  = Map_IsOpen(p.r, p.c + 1);
        uint8_t west  = Map_IsOpen(p.r, p.c - 1);
        if ((north && south) || (east && west)) {
            return 0u;
        }
    }
    return 1u;
}

/* Which way to spin for a 180 on this cell, arriving facing hIn. The spin stops as soon
 * as the line is under the front sensors again, and on the way round it sweeps past
 * the side it turns toward - so if a branch of the line leaves on that side the spin
 * would stop early, 90 degrees round, on the branch. CMD_AROUND is the spin that
 * STATE_TURN_AROUND does (the one tested at dead ends). When a branch leaves on the side
 * that spin sweeps and the other side is clear, use CMD_AROUND_RIGHT (the opposite spin)
 * instead. A dead end has no branch at all, so it keeps CMD_AROUND. Both sides open, or
 * the line continuing straight ahead, can't be avoided by choosing a direction.
 * STATE_TURN_AROUND sweeps the robot's left side, or its right side if
 * PATH_TURNS_MIRRORED. */
static PlanCmd Map_UTurnCmd(Cell p, uint8_t hIn)
{
    uint8_t hl = (uint8_t)((hIn + 3u) & 3u);   /* heading of the robot's left side */
    uint8_t hr = (uint8_t)((hIn + 1u) & 3u);   /* heading of the robot's right side */
    uint8_t leftOpen  = Map_IsOpen(p.r + headingDr[hl], p.c + headingDc[hl]);
    uint8_t rightOpen = Map_IsOpen(p.r + headingDr[hr], p.c + headingDc[hr]);
    uint8_t sweptOpen = PATH_TURNS_MIRRORED ? rightOpen : leftOpen;   /* side STATE_TURN_AROUND sweeps */
    uint8_t otherOpen = PATH_TURNS_MIRRORED ? leftOpen : rightOpen;
    return (sweptOpen && !otherOpen) ? CMD_AROUND_RIGHT : CMD_AROUND;
}

/* The route entry for a path cell: rel = (heading out - heading in) & 3 is the turn as it
 * is on the map; PATH_TURNS_MIRRORED swaps left and right, and a 180 picks its spin. */
static PlanCmd Route_CmdFor(uint8_t rel, Cell p, uint8_t hIn)
{
    if (rel == (uint8_t)CMD_AROUND) {
        return Map_UTurnCmd(p, hIn);
    }
    if (PATH_TURNS_MIRRORED && (rel == (uint8_t)CMD_LEFT || rel == (uint8_t)CMD_RIGHT)) {
        return (rel == (uint8_t)CMD_LEFT) ? CMD_RIGHT : CMD_LEFT;
    }
    return (PlanCmd)rel;
}

/* 1 if the spin chosen for this 180 sweeps a side branch of the line. Map_UTurnCmd already picks the spin
 * away from a single branch, so this is left only when BOTH sides are open: a T junction (arriving on the
 * stem) or a crossing. The front sensors find that branch about 90 degrees round, well short of the real
 * target, so the 180 has to let the first line they find go by. */
static uint8_t Map_UTurnSweepsBranch(Cell p, uint8_t hIn, PlanCmd cmd)
{
    uint8_t hl = (uint8_t)((hIn + 3u) & 3u);   /* heading of the robot's left side */
    uint8_t hr = (uint8_t)((hIn + 1u) & 3u);   /* heading of the robot's right side */
    uint8_t leftOpen  = Map_IsOpen(p.r + headingDr[hl], p.c + headingDc[hl]);
    uint8_t rightOpen = Map_IsOpen(p.r + headingDr[hr], p.c + headingDc[hr]);
    /* same sides as Map_UTurnCmd: CMD_AROUND sweeps the left, or the right if PATH_TURNS_MIRRORED; CMD_AROUND_RIGHT the other */
    uint8_t sweepsLeft = (cmd == CMD_AROUND) ? (uint8_t)(PATH_TURNS_MIRRORED ? 0u : 1u) : (uint8_t)(PATH_TURNS_MIRRORED ? 1u : 0u);
    return sweepsLeft ? leftOpen : rightOpen;
}

/* The route byte for a path cell: the command from Route_CmdFor, plus ROUTE_SKIP_FLAG on a 180 that sweeps a branch. */
static uint8_t Route_EntryFor(uint8_t rel, Cell p, uint8_t hIn)
{
    PlanCmd cmd   = Route_CmdFor(rel, p, hIn);
    uint8_t entry = (uint8_t)cmd;
    if (SKIP_FIRST_LINE_AT_T_180 && (cmd == CMD_AROUND || cmd == CMD_AROUND_RIGHT)
        && Map_UTurnSweepsBranch(p, hIn, cmd)) {
        entry |= ROUTE_SKIP_FLAG;
    }
    return entry;
}

static uint8_t Heading_Between(Cell a, Cell b)
{
    if (b.r < a.r) { return (uint8_t)HEADING_NORTH; }
    if (b.c > a.c) { return (uint8_t)HEADING_EAST; }
    if (b.r > a.r) { return (uint8_t)HEADING_SOUTH; }
    return (uint8_t)HEADING_WEST;
}

/* One path cell, arrived at facing hIn and left facing hOut. Adds a command
 * if the sensors will see something here. Returns 0 if the plan can't work. */
static uint8_t Route_AddNode(Cell node, uint8_t hIn, uint8_t hOut)
{
    uint8_t rel = (uint8_t)((hOut - hIn) & 3);

    if (Map_OpenNeighbours(node) >= 3u || rel != 0u) {
        if (rel == (uint8_t)CMD_AROUND && !Map_IsDetectable(node)) {
            return 0u;   /* has to turn back in the middle of a plain corridor - can't be recognised */
        }
        if (routeLen >= MAX_ROUTE_CMDS) {
            return 0u;
        }
        route[routeLen++] = Route_EntryFor(rel, node, hIn);
    }
    return 1u;
}

/* stops[] = food stops in visiting order, then the goal last.
 * Returns 1 if a usable route was built, 0 if not (wall/out-of-map cell, no
 * path, nothing to do, or a final stop the line sensors could never recognise). */
static uint8_t Route_Plan(Cell start, const Cell *stops, uint8_t numStops, uint8_t heading)
{
    Cell     pos        = start;
    uint8_t  curHeading = heading;   /* direction the robot faces while standing on pos */
    uint8_t  moved      = 0u;
    uint8_t  k, hIn, hOut;
    uint16_t i;
    uint16_t lastCmdIdx;   /* last path cell of this leg the robot gets a command at (or the leg's start cell) */

    routeLen   = 0u;
    routeIndex = 0u;
    initialCmd = CMD_STRAIGHT;
    initialSkip = 0u;
    midCount   = 0u;
    midIndex   = 0u;

    if (numStops == 0u || !Map_IsOpen(start.r, start.c)) { return 0u; }
    for (k = 0u; k < numStops; k++) {
        if (!Map_IsOpen(stops[k].r, stops[k].c)) { return 0u; }
    }

    for (k = 0u; k < numStops; k++) {
        if (stops[k].r == pos.r && stops[k].c == pos.c) { continue; }   /* already standing on this stop */
        if (!bfs_path(mazeMap, pos, stops[k], legPath, &legLen)) { return 0u; }

        lastCmdIdx = 0u;
        for (i = 0u; i + 1u < legLen; i++) {
            hOut = Heading_Between(legPath[i], legPath[i + 1u]);
            hIn  = (i == 0u) ? curHeading : Heading_Between(legPath[i - 1u], legPath[i]);
            if (i == 0u && !moved) {
                /* the start cell itself: turn on the spot at boot, not a sensor event */
                uint8_t firstEntry = Route_EntryFor((uint8_t)((hOut - hIn) & 3), legPath[0], hIn);
                initialCmd  = (PlanCmd)(firstEntry & ROUTE_CMD_MASK);
                initialSkip = (firstEntry & ROUTE_SKIP_FLAG) ? 1u : 0u;
            } else if (i == 0u && USE_ENCODER_FOOD_STOPS && !Map_IsDetectable(legPath[0])) {
                /* this leg starts on a food stop in a plain corridor. The robot was already stopped here
                 * by its CMD_MIDSTOP, which did the pause. Carry on the same way = nothing to add; turn
                 * back the way it came = a 180 on the spot, done straight after the pause. */
                if (((hOut - hIn) & 3) == (uint8_t)CMD_AROUND) {
                    if (routeLen >= MAX_ROUTE_CMDS) { return 0u; }
                    route[routeLen++] = Route_EntryFor((uint8_t)CMD_AROUND, legPath[0], hIn) | ROUTE_IMMEDIATE_FLAG;
                }
            } else {
                uint16_t before = routeLen;
                if (!Route_AddNode(legPath[i], hIn, hOut)) {
                    return 0u;
                }
                if (i == 0u) {
                    /* this leg starts on a food stop: its command carries the pause */
                    if (routeLen == before) {
                        return 0u;   /* food stop on a plain corridor cell and USE_ENCODER_FOOD_STOPS is off */
                    }
                    route[routeLen - 1u] |= ROUTE_FOOD_FLAG;
                }
                if (routeLen != before) {
                    lastCmdIdx = i;
                }
            }
        }

        /* This leg ends on a food stop (not the goal). If it is a plain corridor cell the sensors can't
         * see it, so add a CMD_MIDSTOP: "drive this many nodes past the last command, then stop". */
        if (USE_ENCODER_FOOD_STOPS && (k + 1u < numStops) && !Map_IsDetectable(stops[k])) {
            if (midCount >= MAX_FOOD || routeLen >= MAX_ROUTE_CMDS) { return 0u; }
            midDist[midCount++] = (uint8_t)((legLen - 1u) - lastCmdIdx);
            route[routeLen++]   = (uint8_t)CMD_MIDSTOP;
        }
        curHeading = Heading_Between(legPath[legLen - 2u], legPath[legLen - 1u]);
        pos   = stops[k];
        moved = 1u;
    }

    if (!moved) { return 0u; }   /* already standing on every stop */

    /* The final stop has to be something the sensors can see: a dead end, a
     * corner or a junction. A goal in the middle of a plain straight corridor
     * gives the robot nothing to recognise it by (no encoder), so refuse it. */
    if (!Map_IsDetectable(pos) || routeLen >= MAX_ROUTE_CMDS) { return 0u; }
    route[routeLen++] = (uint8_t)CMD_STOP;
    return 1u;
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
    STATE_TURN_AROUND_RIGHT, /* the same 180 spinning right - for a 180 where a branch leaves on the left */
    STATE_DONE,          /* reached the goal (or the plan failed) - stopped for good */
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
static uint16_t   sideClearCounter  = 0;    /* consecutive ticks both side sensors have read off */
static uint8_t    foodCollectedCount = 0u;  /* increments once for each 5-second fruit pause */
static uint8_t    turnSawWhite      = 0;    /* this turn's exit sensor has read white at least once */
static uint8_t    turnSkipFinds     = 0;    /* a 180 that sweeps a side branch: how many line finds it still has to let go by */
static uint8_t    turnSkipped       = 0;    /* a find has just been let go by - the next one needs white first, whatever TURN_EXIT_NEEDS_WHITE_FIRST says */
static uint8_t    turnWhiteFL = 0, turnWhiteFR = 0;   /* each front sensor has read white during this turn */
static uint8_t    turnSeenFL  = 0, turnSeenFR  = 0;   /* each front sensor has since read black = found the new line */
static RobotState prevState         = STATE_DRIVE_STRAIGHT;  /* state on the previous tick - to spot "turn just finished" */

/* Plans the route at boot. If the robot has to start facing a different way
 * than the first move, the matching turn state is entered straight away. */
static void Route_Init(void)
{
    static const uint8_t numFood = NUM_FOOD;
    Cell    start = { (int8_t)START_ROW, (int8_t)START_COL };
    Cell    stops[MAX_FOOD + 1u];   /* food stops in order, then the goal */
    uint8_t k;

    if (!FOLLOW_PLANNED_PATH) {
        return;
    }

    for (k = 0u; k < numFood; k++) {
        stops[k].r = (int8_t)food_list[k][0];
        stops[k].c = (int8_t)food_list[k][1];
    }
    stops[numFood].r = (int8_t)GOAL_ROW;
    stops[numFood].c = (int8_t)GOAL_COL;

    if (!Route_Plan(start, stops, (uint8_t)(numFood + 1u), (uint8_t)START_HEADING)) {
        Motor_Stop();
        stopReason = STOP_PLAN_FAILED;
        state = STATE_DONE;      /* bad inputs / no path / undetectable stop - stay put */
        return;
    }

    sideClearCounter = 0;        /* the start cell's own branches must clear before a junction counts */
    turnCounter = 0;
    if (initialCmd == CMD_LEFT) {
        state = STATE_TURN_LEFT;
    } else if (initialCmd == CMD_RIGHT) {
        state = STATE_TURN_RIGHT;
    } else if (initialCmd == CMD_AROUND) {
        state = STATE_TURN_AROUND;
    } else if (initialCmd == CMD_AROUND_RIGHT) {
        state = STATE_TURN_AROUND_RIGHT;
    }
    turnSkipFinds = initialSkip;   /* only a 180 state ever reads this */
}

/* Food stop reached: stand still for FOOD_PAUSE_MS, then the route carries on. */
static void Food_Pause(void)
{
    Motor_Stop();
    CyDelay(FOOD_PAUSE_MS);
    if (foodCollectedCount < NUM_FOOD) {
        foodCollectedCount++;
    }
}

/* Start a 180 (spinState = STATE_TURN_AROUND or STATE_TURN_AROUND_RIGHT). With
 * BACK_UP_BEFORE_180 the robot brakes and reverses a little first, exactly like
 * before a 90-degree turn, then spins. */
static void Begin_UTurn(RobotState spinState, uint8_t skipFinds)
{
    turnSkipFinds = skipFinds;   /* 1 = this 180 sweeps a side branch: let the first line the front sensors find go by */
    if (BACK_UP_BEFORE_180) {
        Motor_Stop();
        CyDelay(TURN_BACKUP_BRAKE_MS);
        pendingTurnState = spinState;
        backupCounter = 0;
        state = STATE_TURN_BACKUP;
    } else {
        state = spinState;
    }
    turnCounter = 0;
}

/* Called once per tick while a turn is running: records which front sensors have found the new line
 * (white first, then black - so a sensor that starts the turn sitting on the OLD line doesn't count). */
static void Turn_Track(const SensorState *s)
{
    if (!s->frontLeft)  { turnWhiteFL = 1u; } else if (turnWhiteFL || (!TURN_EXIT_NEEDS_WHITE_FIRST && !turnSkipped)) { turnSeenFL = 1u; }
    if (!s->frontRight) { turnWhiteFR = 1u; } else if (turnWhiteFR || (!TURN_EXIT_NEEDS_WHITE_FIRST && !turnSkipped)) { turnSeenFR = 1u; }
}

/* Called by a 180 state when its exit test has just passed. Returns 1 if this find has to be let go by (the side
 * branch of a T junction): the "found the line" memory starts again, so the exit needs white and then black again. */
static uint8_t Turn_SkipFind(void)
{
    if (turnSkipFinds == 0u) {
        return 0u;
    }
    turnSkipFinds--;
    turnSkipped = 1u;
    turnSawWhite = 0u;
    turnWhiteFL = turnWhiteFR = 0u;
    turnSeenFL  = turnSeenFR  = 0u;
    return 1u;
}

/* Turn exit test, called once per tick by the four turn states. */
static uint8_t Turn_ExitSeen(uint8_t exitSensor)
{
    if (TURN_EXIT_BOTH_FRONT) {
        return (turnSeenFL && turnSeenFR) ? 1u : 0u;
    }
    if (!exitSensor) {
        turnSawWhite = 1u;
    }
    return (exitSensor && (turnSawWhite || !TURN_EXIT_NEEDS_WHITE_FIRST)) ? 1u : 0u;
}

/* Spin speed: full speed until a front sensor has found the new line, then slower for the last bit. */
static int16_t Turn_Speed(uint16_t full)
{
    return (int16_t)((TURN_EXIT_BOTH_FRONT && (turnSeenFL || turnSeenFR)) ? SPEED_TURN_FINAL : full);
}

static RobotState Turn_NextState(RobotState adjustState)
{
    return TURN_USE_ADJUST ? adjustState : STATE_DRIVE_STRAIGHT;
}

static void RunLineFollower(const SensorState *s)
{
    SensorState noSides;   /* copy of this tick's readings with the side sensors forced off, used during a 180 */
    uint8_t     sidesOff = 0u;
    if (IGNORE_SIDE_DURING_180
        && (state == STATE_TURN_AROUND || state == STATE_TURN_AROUND_RIGHT
            || (state == STATE_TURN_BACKUP
                && (pendingTurnState == STATE_TURN_AROUND || pendingTurnState == STATE_TURN_AROUND_RIGHT)))) {
        sidesOff = 1u;
        noSides = *s;
        noSides.left  = 0u;
        noSides.right = 0u;
        s = &noSides;
    }

    uint8_t anyOnLine = s->left | s->frontLeft | s->mid | s->frontRight | s->right | s->back;
    uint8_t onlyBack  = s->back && !s->left && !s->frontLeft && !s->mid && !s->frontRight && !s->right;

    (void)Odo_Counts();   /* keeps odoDebugCounts live for calibration */

    /* A turn / 180 / adjust has just finished and the robot is driving straight again: start the
     * encoder distance count from here. (Not after LINE_LOST: the robot hasn't moved to a new reference.) */
    if (state != prevState) {
        if (state == STATE_DRIVE_STRAIGHT && prevState != STATE_LINE_LOST) {
            Odo_Mark(ODO_OFFSET_AFTER_TURN);
        }
        prevState = state;
    }

    /* the white-then-black turn exit starts fresh for every turn */
    if (state != STATE_TURN_LEFT && state != STATE_TURN_RIGHT
        && state != STATE_TURN_AROUND && state != STATE_TURN_AROUND_RIGHT) {
        turnSawWhite = 0u;
        turnSkipped  = 0u;
        turnWhiteFL = turnWhiteFR = 0u;
        turnSeenFL  = turnSeenFR  = 0u;
    } else {
        Turn_Track(s);
    }

    /* dead-end detector: counts consecutive ticks of "back sensor only" while driving straight */
    deadEndCounter = (onlyBack && state == STATE_DRIVE_STRAIGHT) ? (deadEndCounter + 1u) : 0u;

    /* junction detector: a side sensor going active after both have been off
     * for SIDE_CLEAR_ITERATIONS ticks = one new junction/corner (fires once) */
    uint8_t junctionEvent = 0u;
    if (sidesOff) {
        /* during a 180 the side sensors are off: no junction can be seen and the count is paused */
    } else if (s->left || s->right) {
        if (sideClearCounter >= SIDE_CLEAR_ITERATIONS) {
            junctionEvent = 1u;
        }
        sideClearCounter = 0;
    } else if (sideClearCounter < SIDE_CLEAR_ITERATIONS) {
        sideClearCounter++;
    }

    /* A CMD_MIDSTOP is next in the plan: the food cell is a plain corridor cell, so there is no
     * junction before it. Ignore side-sensor events until the encoder says we are on the cell. */
    uint8_t midStopDue = 0u;
    if (FOLLOW_PLANNED_PATH && USE_ENCODER_FOOD_STOPS && state == STATE_DRIVE_STRAIGHT
        && routeIndex < routeLen && (route[routeIndex] & ROUTE_CMD_MASK) == (uint8_t)CMD_MIDSTOP) {
        int32_t counts = Odo_Counts();
        junctionEvent = 0u;
        if (counts >= ((int32_t)midDist[midIndex] * (int32_t)COUNTS_PER_NODE) - (int32_t)MIDSTOP_BRAKE_COUNTS) {
            midStopDue = 1u;
        } else {
            if (midWatchTicks < 0xFFFFu) { midWatchTicks++; }
            if (midWatchTicks >= MIDSTOP_STALL_TICKS && counts < (int32_t)MIDSTOP_STALL_COUNTS) {
                /* driving for 1.5 s and the encoders have barely moved: they are not working. Waiting any longer
                 * would leave the robot ignoring every junction, so skip this fruit and stay on the route. */
                encoderFault = 2u;
                midIndex++;
                routeIndex++;
                midWatchTicks = 0u;
            }
        }
    }

    if (state != STATE_TURN_LEFT && state != STATE_TURN_RIGHT && state != STATE_TURN_BACKUP
        && state != STATE_ADJUST_LEFT && state != STATE_ADJUST_RIGHT && state != STATE_TURN_AROUND
        && state != STATE_TURN_AROUND_RIGHT
        && state != STATE_DONE) {
        if (!anyOnLine) {
            lineLostCounter++;
            if (lineLostCounter >= LINE_LOST_THRESHOLD) {
                stopReason = STOP_LINE_LOST;
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
                /* just finished (or forced out of) a turn - ignore left/right for a bit so a
                 * chronically-triggered sensor can't instantly throw us back into the same turn.
                 * The FRONT sensors still steer: a robot that finishes a turn slightly to one side
                 * of the line (the wheels pivot a few mm short of the junction) used to drive
                 * blind for this whole period and could lose the line before steering started. */
                suppressSideCounter--;
                if (STEER_DURING_COOLDOWN && s->frontLeft && !s->frontRight) {
                    Motor_SetSpeed(SPEED_BASE, SPEED_BASE - DRIFT_CORRECTION_OFFSET);
                } else if (STEER_DURING_COOLDOWN && s->frontRight && !s->frontLeft) {
                    Motor_SetSpeed(SPEED_BASE - DRIFT_CORRECTION_OFFSET, SPEED_BASE);
                } else {
                    Motor_SetSpeed(SPEED_BASE, SPEED_BASE);
                }
            } else if (midStopDue) {
                /* on the food cell in a plain corridor: stand still, then carry on */
                Food_Pause();
                midIndex++;
                routeIndex++;
                midWatchTicks = 0u;
                Odo_Mark(0);   /* we are on the cell centre by definition - next count starts here */
                if (routeIndex < routeLen && (route[routeIndex] & ROUTE_IMMEDIATE_FLAG)) {
                    /* the next leg leaves back the way we came: 180 on the spot */
                    uint8_t uentry = route[routeIndex++];
                    uint8_t uturn  = (uint8_t)(uentry & ROUTE_CMD_MASK);
                    Begin_UTurn((uturn == (uint8_t)CMD_AROUND_RIGHT) ? STATE_TURN_AROUND_RIGHT : STATE_TURN_AROUND,
                                (uentry & ROUTE_SKIP_FLAG) ? 1u : 0u);
                }
            } else if (FOLLOW_PLANNED_PATH && junctionEvent) {
                /* new junction/corner: carry out the next command from the
                 * BFS route. Running out of commands is treated as STOP. */
                uint8_t ranOut = (routeIndex < routeLen) ? 0u : 1u;
                uint8_t entry = (routeIndex < routeLen) ? route[routeIndex++] : (uint8_t)CMD_STOP;
                PlanCmd cmd   = (PlanCmd)(entry & ROUTE_CMD_MASK);
                if (entry & ROUTE_FOOD_FLAG) {
                    Food_Pause();   /* food stop: stand still, then carry on with the command below */
                }
                if (cmd == CMD_LEFT || cmd == CMD_RIGHT) {
                    Motor_Stop();
                    CyDelay(TURN_BACKUP_BRAKE_MS);
                    pendingTurnState = (cmd == CMD_LEFT) ? STATE_TURN_LEFT : STATE_TURN_RIGHT;
                    backupCounter = 0;
                    state = STATE_TURN_BACKUP;
                } else if (cmd == CMD_AROUND) {
                    Begin_UTurn(STATE_TURN_AROUND, (entry & ROUTE_SKIP_FLAG) ? 1u : 0u);   /* next leg heads back the way we came */
                } else if (cmd == CMD_AROUND_RIGHT) {
                    Begin_UTurn(STATE_TURN_AROUND_RIGHT, (entry & ROUTE_SKIP_FLAG) ? 1u : 0u);   /* same, spinning away from a branch on the left */
                } else if (cmd == CMD_STOP) {
                    Motor_Stop();
                    stopReason = ranOut ? STOP_ROUTE_RAN_OUT : STOP_GOAL;
                    state = STATE_DONE;
                } else {
                    Motor_SetSpeed(SPEED_BASE, SPEED_BASE);   /* CMD_STRAIGHT: drive on through the junction */
                    Odo_Mark(ODO_OFFSET_AFTER_STRAIGHT);       /* a plain-corridor food stop after this is counted from here */
                }
            } else if (!FOLLOW_PLANNED_PATH && s->left) {
                /* brake, then back up before attempting the turn - by the
                 * time this side sensor sees the branch the robot has
                 * usually already driven past the actual junction */
                Motor_Stop();
                CyDelay(TURN_BACKUP_BRAKE_MS);
                pendingTurnState = STATE_TURN_LEFT;
                backupCounter = 0;
                state = STATE_TURN_BACKUP;
            } else if (!FOLLOW_PLANNED_PATH && s->right) {
                Motor_Stop();
                CyDelay(TURN_BACKUP_BRAKE_MS);
                pendingTurnState = STATE_TURN_RIGHT;
                backupCounter = 0;
                state = STATE_TURN_BACKUP;
            } else if (deadEndCounter >= DEAD_END_CONFIRM_ITERATIONS) {
                /* line seen ONLY on the back sensor - dead end */
                if (FOLLOW_PLANNED_PATH) {
                    /* a dead end on the route is either a food stop the next leg
                     * turns back from (pause, then 180), or the goal. Anything
                     * else means the robot is lost - stop. */
                    uint8_t next = (routeIndex < routeLen) ? (uint8_t)(route[routeIndex] & ROUTE_CMD_MASK) : (uint8_t)CMD_STOP;
                    if (next == (uint8_t)CMD_AROUND || next == (uint8_t)CMD_AROUND_RIGHT) {
                        uint8_t deSkip = (route[routeIndex] & ROUTE_SKIP_FLAG) ? 1u : 0u;
                        if (route[routeIndex] & ROUTE_FOOD_FLAG) {
                            Food_Pause();
                        }
                        routeIndex++;
                        Begin_UTurn((next == (uint8_t)CMD_AROUND) ? STATE_TURN_AROUND : STATE_TURN_AROUND_RIGHT, deSkip);
                        deadEndCounter = 0;
                    } else {
                        Motor_Stop();
                        if (next == (uint8_t)CMD_STOP) {
                            stopReason = (routeIndex < routeLen) ? STOP_GOAL : STOP_ROUTE_RAN_OUT;
                        } else {
                            stopReason = STOP_DEAD_END_UNEXPECTED;   /* the plan wanted a junction/turn here, not a dead end */
                        }
                        state = STATE_DONE;
                    }
                } else {
                    Begin_UTurn(STATE_TURN_AROUND, 0u);
                    deadEndCounter = 0;
                }
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
            if (backupCounter >= ((pendingTurnState == STATE_TURN_AROUND || pendingTurnState == STATE_TURN_AROUND_RIGHT)
                                  ? U_TURN_BACKUP_ITERATIONS : TURN_BACKUP_ITERATIONS)) {
                state = pendingTurnState;   /* backed up - now actually attempt the turn */
                turnCounter = 0;
                backupCounter = 0;
            }
            break;

        case STATE_TURN_LEFT:
            Motor_SetSpeed(-Turn_Speed(SPEED_TURN), Turn_Speed(SPEED_TURN));
            turnCounter++;
            /* exit as soon as the OPPOSITE front sensor sees the line -
             * doesn't wait for mid too, so it can still be off-center at
             * the exact instant this fires. STATE_ADJUST_LEFT closes that
             * remaining gap before handing off to straight driving. */
            if (Turn_ExitSeen(s->frontRight)) {
                state = Turn_NextState(STATE_ADJUST_LEFT);
                turnCounter = 0;
                adjustCounter = 0;
                suppressSideCounter = POST_TURN_COOLDOWN;
            } else if (turnCounter >= TURN_TIMEOUT) {
                /* frontRight never lit up - force a clean break instead of
                 * hanging here forever */
                state = STATE_ADJUST_LEFT;
                turnCounter = 0;
                adjustCounter = 0;
                suppressSideCounter = POST_TURN_COOLDOWN;
            }
            break;

        case STATE_TURN_RIGHT:
            Motor_SetSpeed(Turn_Speed(SPEED_TURN), -Turn_Speed(SPEED_TURN));
            turnCounter++;
            if (Turn_ExitSeen(s->frontLeft)) {
                state = Turn_NextState(STATE_ADJUST_RIGHT);
                turnCounter = 0;
                adjustCounter = 0;
                suppressSideCounter = POST_TURN_COOLDOWN;
            } else if (turnCounter >= TURN_TIMEOUT) {
                state = STATE_ADJUST_RIGHT;
                turnCounter = 0;
                adjustCounter = 0;
                suppressSideCounter = POST_TURN_COOLDOWN;
            }
            break;

        case STATE_TURN_AROUND:
            /* in-place spin left; exits as soon as the OPPOSITE front sensor
             * (frontRight) sees the line, straight into driving - no adjust
             * step */
            Motor_SetSpeed(-Turn_Speed(SPEED_TURN_AROUND), Turn_Speed(SPEED_TURN_AROUND));
            turnCounter++;
            /* at a T junction the first line found is the side branch (Turn_SkipFind lets it go by) */
            if (Turn_ExitSeen(s->frontRight) && !Turn_SkipFind()) {
                state = STATE_DRIVE_STRAIGHT;
                turnCounter = 0;
                turnSkipFinds = 0u;
                suppressSideCounter = POST_TURN_COOLDOWN;
            } else if (turnCounter >= U_TURN_TIMEOUT) {
                /* frontRight never lit up - force a clean break */
                state = STATE_DRIVE_STRAIGHT;
                turnCounter = 0;
                turnSkipFinds = 0u;
                suppressSideCounter = TURN_COOLDOWN;
            }
            break;

        case STATE_TURN_AROUND_RIGHT:
            /* mirror of STATE_TURN_AROUND: in-place spin RIGHT; exits as soon as
             * the OPPOSITE front sensor (frontLeft) sees the line, straight into
             * driving - no adjust step. Used for a 180 where a branch of the line
             * leaves on the left, so the spin turns away from it */
            Motor_SetSpeed(Turn_Speed(SPEED_TURN_AROUND), -Turn_Speed(SPEED_TURN_AROUND));
            turnCounter++;
            if (Turn_ExitSeen(s->frontLeft) && !Turn_SkipFind()) {
                state = STATE_DRIVE_STRAIGHT;
                turnCounter = 0;
                turnSkipFinds = 0u;
                suppressSideCounter = POST_TURN_COOLDOWN;
            } else if (turnCounter >= U_TURN_TIMEOUT) {
                /* frontLeft never lit up - force a clean break */
                state = STATE_DRIVE_STRAIGHT;
                turnCounter = 0;
                turnSkipFinds = 0u;
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
            if ((s->frontLeft && s->frontRight) || (s->mid && adjustCounter >= 3u) || (adjustCounter >= POST_TURN_ADJUST_ITERATIONS)) {
                state = STATE_DRIVE_STRAIGHT;
                adjustCounter = 0;
            }
            break;

        case STATE_ADJUST_RIGHT:
            Motor_SetSpeed(SPEED_BASE, SPEED_BASE - DRIFT_CORRECTION_OFFSET);
            adjustCounter++;
            if ((s->frontLeft && s->frontRight) || (s->mid && adjustCounter >= 3u) || (adjustCounter >= POST_TURN_ADJUST_ITERATIONS)) {
                state = STATE_DRIVE_STRAIGHT;
                adjustCounter = 0;
            }
            break;

        case STATE_DONE:
            Motor_Stop();
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
    Odo_Start();
    Odo_Mark(0);
#if ODO_CALIBRATION_MODE
    for (;;) {                       /* motors off: just keep odoDebugCounts live while you push the robot by hand */
        (void)Odo_Counts();
        CyDelay(10);
    }
#endif
    Route_Init();   /* BFS runs once here, before the loop starts */

    for (;;) {
        SensorState sensors;
        Sensors_Read(&sensors);
        RunLineFollower(&sensors);
        CyDelay(10); /* ~100 Hz control loop */
    }
}