/* Tests for:
 *  Startup, Trajectory, Kinematics
 *  Plug-in is optional.
 */
#define DEBUG

#include <Leg.h>
#include <Timer.h>
RServoDriver servo_driver;
DogLeg leg_ur;

#define LEG_UR_C_ANG_OFS  (0)
#define LEG_UR_S_ANG_OFS  (-3)
#define LEG_UR_E_ANG_OFS  (6)
    
#define DEFAULT_LEG_HEIGHT 100 //mm
#define WIDTH2 60 // half total width
#define LENGTH2 112.5 // half total length
Point mounting_point = Point( LENGTH2, -WIDTH2, 0);
Point starting_height = Point(0, 0, DEFAULT_LEG_HEIGHT);
Point default_position = mounting_point - starting_height;

Timer timer;

void setup() {
//  // put your setup code here, to run once:
    Serial.begin(9600);
    while (!Serial) {}
    servo_driver.defaultStartup();
    
    leg_ur = DogLeg(&servo_driver,  0,  1,  2, mounting_point, default_position); // in future to enforce level starting, find way to set default_position.z = default_height conveniently.
    leg_ur.flipLR();
    leg_ur.calibrateServos(LEG_UR_C_ANG_OFS, LEG_UR_S_ANG_OFS, LEG_UR_E_ANG_OFS);

    // Startup Checks
    // Default position, accessors
    Serial.println("Checking Startup position...");
    Serial.print("Access default: "); Serial.println(leg_ur.getDefaultPosition_oBfB() == default_position);
    Serial.print("Current is at default: "); Serial.println(leg_ur.getPosition_oBfB() == default_position);

    // Trajectory check: checks that position is correctly computed by the trajectory.
    Serial.println("Trajectory Check... ");
    // Instant move: position after 1 tick is the goal position
    Point goal_position = default_position + Point(10, 10, 10);
    leg_ur.moveToPositionFromBody(goal_position, TIME_INSTANT);
    leg_ur.operate();
    Serial.print("Instant Move: "); Serial.println(leg_ur.getPosition_oBfB() == goal_position);
    // Reset
    leg_ur.moveToPositionFromBody(default_position, TIME_INSTANT);
    leg_ur.operate();
    // Slow move
    timer.usePrecision();
    float time_traj = 1.0; // 1 sec
    timer.reset(time_traj/2);
    leg_ur.moveToPositionFromBody(goal_position, time_traj);
    while (!timer.timeOut()) {leg_ur.operate();}
    Point mid_position = leg_ur.getPosition_oBfB();
    Point mid_position_expected = (default_position + goal_position)/2;
    Serial.print("Timed motion midway: "); Serial.println((mid_position - mid_position_expected).norm() < 0.1);
    timer.reset(time_traj/2 + 0.1);
    while (!timer.timeOut()) {leg_ur.operate();}
    Serial.print("Timed motion final: "); Serial.println(leg_ur.getPosition_oBfB() == goal_position);

    // Kinematics Check: verifies that kinematics are correctly computed for given angles
    Serial.println("Kinematics Checks...");
    leg_ur.moveToPositionFromBody(default_position, TIME_INSTANT);
    leg_ur.operate();
    Point goal_position2 = Point(0, 13.97, -106.06) + mounting_point;
    leg_ur.moveToPositionFromBody(goal_position2, TIME_INSTANT);
    leg_ur.solveMotion();
    leg_ur.printIkinAngles();
    leg_ur.operate();
    Serial.print("Match To: C: "); Serial.print(0.0);
    Serial.print(" S: "); Serial.print(-45.0);
    Serial.print(" E: "); Serial.print(-110.0);
    Serial.println();

    // TODO: Trajectory Adjustment checks: timing check and goal change check

    
    // Live checks: Plug in the servo driver then comment out the desired check.
    leg_ur.moveToPositionFromBody(default_position, TIME_INSTANT);

}

void loop() {
}
