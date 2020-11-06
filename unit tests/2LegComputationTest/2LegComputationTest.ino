/* Tests for:
 *  Startup, Trajectory, Kinematics
 *  Plug-in is optional.
 */
#define DEBUG

#include <Leg.h>
#include <Timer.h>
RServoDriver servo_driver;
DogLeg leg;

#define DEFAULT_LEG_HEIGHT 100 //mm
#define WIDTH2 60 // half total width
#define LENGTH2 112.5 // half total length
Point mounting_point = Point( LENGTH2, WIDTH2, 0);
Point starting_height = Point(0, 0, DEFAULT_LEG_HEIGHT);
Point default_position = mounting_point - starting_height;

Timer timer;

void setup() {
//  // put your setup code here, to run once:
    Serial.begin(9600);
    while (!Serial) {}
    servo_driver.defaultStartup();
    
    mounting_point = Point( LENGTH2, WIDTH2, 0);
    leg = DogLeg(&servo_driver, 12, 13, 14, mounting_point, default_position);

    // Startup Checks
    // Default position, accessors
    Serial.println("Checking Startup position...");
    Serial.print("Access default: "); Serial.println(leg.getDefaultPosition_oBfB() == default_position);
    Serial.print("Current is at default: "); Serial.println(leg.getPosition_oBfB() == default_position);
    // Reset to default position
    leg.moveToDefaultPosition();
    leg.operate();

    // Trajectory check: checks that position is correctly computed by the trajectory.
    Serial.println("Trajectory Check... ");
    // Instant move: position after 1 tick is the goal position
    Point goal_position = default_position + Point(10, 10, 10);
    leg.moveToPositionFromBody(goal_position, TIME_INSTANT);
    leg.operate();
    Serial.print("Instant Move: "); Serial.println(leg.getPosition_oBfB() == goal_position);
    // Reset
    leg.moveToPositionFromBody(default_position, TIME_INSTANT);
    leg.operate();
    // Slow move
    timer.usePrecision();
    float time_traj = 1.0; // 1 sec
    timer.reset(time_traj/2);
    leg.moveToPositionFromBody(goal_position, time_traj);
    while (!timer.timeOut()) {leg.operate();}
    Point mid_position = leg.getPosition_oBfB();
    Point mid_position_expected = (default_position + goal_position)/2;
    Serial.print("Timed motion midway: "); Serial.println((mid_position - mid_position_expected).norm() < 0.1);
    timer.reset(time_traj/2 + 0.1);
    while (!timer.timeOut()) {leg.operate();}
    Serial.print("Timed motion final: "); Serial.println(leg.getPosition_oBfB() == goal_position);

    // Kinematics Check: verifies that kinematics are correctly computed for given angles
    Serial.println("Kinematics Checks...");
    leg.moveToPositionFromBody(default_position, TIME_INSTANT);
    leg.operate();
    Point goal_position2 = Point(0, 13.97, -106.06) + mounting_point;
    leg.moveToPositionFromBody(goal_position2, TIME_INSTANT);
    leg.solveMotion();
    leg.printIkinAngles();
    leg.sendSignals();
    Serial.print("Match To   : C: "); Serial.print(0.0);
    Serial.print(" S: "); Serial.print(45.0);
    Serial.print(" E: "); Serial.print(110.0);
    Serial.println();

    Point goal_position3 = Point(30, 0, -106.06) + mounting_point;
    leg.moveToPositionFromBody(goal_position3, TIME_INSTANT);
    leg.solveMotion();
    leg.printIkinAngles();
    leg.sendSignals();
    Serial.print("Match To   : C: "); Serial.print(0.0);
    Serial.print(" S: "); Serial.print(0.0);
    Serial.print(" E: "); Serial.print(0.0);
    Serial.println();


    // TODO: Trajectory Adjustment checks: timing check and goal change check

    
    // Live checks: Plug in the servo driver then comment out the desired check.
    leg.moveToPositionFromBody(default_position, TIME_INSTANT);

}

void loop() {
}
