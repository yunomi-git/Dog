/* Tests for:
 *  (General): Accessors, 
 *  (Feet planted): trajectory (orientation, translation), Frame conversion, Anchor Points, centroid
 *  (Feet are lifted): stance changes, indiv foot movement, centroid/anchor point adjustment
 */

#define DEBUG
#define DEBUG_COMPUTATION

#include <Dog.h>
#include <Timer.h>

#define NUM_LEGS 4

RobotDog dog;

Point default_starting_position;
Point default_leg_positions[NUM_LEGS];

bool temp_bool; // use this for checking truth value of multiple things
Timer timer;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    while (!Serial) {}
    dog.begin();

    // Startup Values
    Serial.println("Checking Startup position...");
    Serial.print("Default height: "); Serial.println(dog.getStartingHeight());
    default_starting_position = Point(0, 0, dog.getStartingHeight());
    Serial.println("Mounting point of legs: "); 
    for (int i = 0; i < NUM_LEGS; i++) {
        default_leg_positions[i] = dog.getLeg(i)->getPosition_oBfB();
        Serial.print("Leg "); Serial.print(i); Serial.print(": "); default_leg_positions[i].print();       
    }

    // Check accessor values
    Serial.println(); Serial.println("Checking Accessors...");
    Serial.print("Body Position: "); Serial.println(dog.getBodyPosition_oC(Frame::FLOOR) == default_starting_position);
    Serial.print("Leg Positions: ");
    temp_bool = true;
    for (int i = 0; i < NUM_LEGS; i++) {
        temp_bool = temp_bool && (dog.getLegPosition_oB(i, Frame::FLOOR) == default_leg_positions[i]);
    }
    Serial.println(temp_bool);
    Serial.print("Kine Orientation: "); Serial.println(dog.getBodyKinematicOrientation_fF2B() == ROT_ZERO);
    Serial.print("IMU  Orientation: "); Serial.println(dog.getBodyIMUOrientation_fG2B() == ROT_ZERO);
    Serial.print("Anchor Points: ");
    temp_bool = true;
    for (int i = 0; i < NUM_LEGS; i++) {
        temp_bool = temp_bool && (dog.getAnchorPoint_oC(i, Frame::FLOOR) == default_leg_positions[i] + default_starting_position);
    }
    Serial.println(temp_bool);
    Serial.print("Stances: ");
    temp_bool = true;
    for (int i = 0; i < NUM_LEGS; i++) {
        temp_bool = temp_bool && (dog.getFootStance(i) == FootStance::PLANTED);
    }
    Serial.println(temp_bool);

    // Check Trajectory
    checkTrajectoryComputational();
    
    // Check Stances
    checkStanceChangeComputational();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void checkTrajectoryComputational() {
    dog.resetDefaultStance();
    Serial.println();
    Serial.println("Trajectory Check... ");
    // Instant move: position after 1 tick is the goal position
    Point goal_position = default_starting_position + Point(10, 10, 10);
    Rot goal_orientation = Rot(5, 5, 5);
    dog.moveToPosition(goal_position, Frame::FLOOR, TIME_INSTANT);
    dog.moveToOrientation(goal_orientation, TIME_INSTANT);
    dog.feedIMU(goal_orientation);
    dog.operate();
    Serial.println("Instant Move: "); 
    Serial.print("_Position Floor: "); Serial.println(dog.getBodyPosition_oC(Frame::FLOOR) == goal_position);
    Serial.print("_Position Body: "); Serial.println(dog.getBodyPosition_oC(Frame::BODY) == goal_position / goal_orientation);
    Serial.print("_Kine Orientation: "); Serial.println(dog.getBodyKinematicOrientation_fF2B() == goal_orientation);
    // Reset
    dog.resetDefaultStance();
    // Slow move
    timer.usePrecision();
    float time_traj = 1.0; // 1 sec
    timer.reset(time_traj/2);
    dog.moveToPosition(goal_position, Frame::FLOOR, time_traj);
    dog.moveToOrientation(goal_orientation, time_traj);
    while (!timer.timeOut()) {dog.operate();}
    Point mid_position_expected = (default_starting_position + goal_position)/2;
    Rot mid_orientation_expected = goal_orientation/2;
    Serial.print("Timed motion midway: "); Serial.println();
    Serial.print("_Position Floor: "); Serial.println((dog.getBodyPosition_oC(Frame::FLOOR) - mid_position_expected).norm() < 0.1);
    Serial.print("_Kine Orientation: "); Serial.println((dog.getBodyKinematicOrientation_fF2B() - mid_orientation_expected).norm() < 0.1);
    timer.reset(time_traj/2 + 0.1);
    while (!timer.timeOut()) {dog.operate();}
    Serial.print("Timed motion final: "); Serial.println(dog.getBodyPosition_oC(Frame::FLOOR) == goal_position);
}

void checkStanceChangeComputational() {
    int foot_to_switch = 0;
    dog.resetDefaultStance();
    Serial.println();
    Serial.println("Checking Stances...");
    Point expected_centroid;
    
    Serial.println("Plant->Set");
    dog.switchFootStance(foot_to_switch, FootStance::SET);
    expected_centroid = POINT_ZERO;
    for (int i = 1; i < NUM_LEGS; i++) {
      expected_centroid += default_leg_positions[i]/3;
    }
    Serial.print("_Centroid: "); Serial.println(dog.getBodyPosition_oC(Frame::FLOOR) == -expected_centroid);
    Serial.print("_altered anchor: "); Serial.println(dog.getAnchorPoint_oC(0) == default_leg_positions[0] - expected_centroid);
    Serial.print("_else anchor: "); 
    temp_bool = true;
    for (int i = 1; i < NUM_LEGS; i++) {
        temp_bool = temp_bool && (dog.getAnchorPoint_oC(i) == default_leg_positions[i] - expected_centroid);
    }
    Serial.println(temp_bool);
    
    Serial.println("Set->Lift");
    dog.switchFootStance(foot_to_switch, FootStance::LIFTED);
    Serial.print("_Centroid: "); Serial.println(dog.getBodyPosition_oC(Frame::FLOOR) == -expected_centroid);
    Serial.print("_altered no anchor: "); Serial.println(dog.getAnchorPoint_oC(0) == POINT_NULL);
    Serial.print("_else anchor: "); 
    temp_bool = true;
    for (int i = 1; i < NUM_LEGS; i++) {
        temp_bool = temp_bool && (dog.getAnchorPoint_oC(i) == default_leg_positions[i] - expected_centroid);
    }
    Serial.println(temp_bool);
    
    Serial.println("Lift->Set");
    dog.switchFootStance(foot_to_switch, FootStance::SET);
    Serial.print("_Centroid: "); Serial.println(dog.getBodyPosition_oC(Frame::FLOOR) == -expected_centroid);
    Serial.print("_altered anchor: "); Serial.println(dog.getAnchorPoint_oC(0) == default_leg_positions[0] - expected_centroid);
    Serial.print("_else anchor: "); 
    temp_bool = true;
    for (int i = 1; i < NUM_LEGS; i++) {
        temp_bool = temp_bool && (dog.getAnchorPoint_oC(i) == default_leg_positions[i] - expected_centroid);
    }
    Serial.println(temp_bool);
    
    Serial.println("Set->Plant");
    dog.switchFootStance(foot_to_switch, FootStance::PLANTED);
    expected_centroid = -default_starting_position;
    Serial.print("_Centroid: "); Serial.println(dog.getBodyPosition_oC(Frame::FLOOR) == default_starting_position);
    Serial.print("_anchors: "); 
    temp_bool = true;
    for (int i = 0; i < NUM_LEGS; i++) {
        temp_bool = temp_bool && (dog.getAnchorPoint_oC(i) == default_leg_positions[i] - expected_centroid);
    }
    Serial.println(temp_bool);

    Serial.println("Plant->Lift");
    dog.switchFootStance(foot_to_switch, FootStance::LIFTED);
    expected_centroid = POINT_ZERO;
    for (int i = 1; i < NUM_LEGS; i++) {
      expected_centroid += default_leg_positions[i]/3;
    }
    Serial.print("_Centroid: "); Serial.println(dog.getBodyPosition_oC(Frame::FLOOR) == -expected_centroid);
    Serial.print("_altered no anchor: "); Serial.println(dog.getAnchorPoint_oC(0) == POINT_NULL);
    Serial.print("_else anchors: "); 
    temp_bool = true;
    for (int i = 1; i < NUM_LEGS; i++) {
        temp_bool = temp_bool && (dog.getAnchorPoint_oC(i) == default_leg_positions[i] - expected_centroid);
    }
    Serial.println(temp_bool);

    Serial.println("Lift->Plant");
    dog.switchFootStance(foot_to_switch, FootStance::PLANTED);
    expected_centroid = -default_starting_position;
    Serial.print("_Centroid: "); Serial.println(dog.getBodyPosition_oC(Frame::FLOOR) == default_starting_position);
    Serial.print("_anchors: "); 
    temp_bool = true;
    for (int i = 0; i < NUM_LEGS; i++) {
        temp_bool = temp_bool && (dog.getAnchorPoint_oC(i) == default_leg_positions[i] - expected_centroid);
    }
    Serial.println(temp_bool);
}
