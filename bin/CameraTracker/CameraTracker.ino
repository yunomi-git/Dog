#define FAST_CREEP

#include <Dog.h>
#include "CreepGaitCoordinator.h"
#include "BalanceHandler.h"
#include "SoftwareSerial.h"
#include "FunctionGenerator.h"
#include "Timer.h"
#include "PixyTracker.h"
#include "SensorHistory.h"

#include "math2.h"

// LEDs
#define start_LED_pin 22 // 21 20
#define serial_LED_pin 21
#define ground_mode_LED_pin 20



#define DESIRED_OBJECT_DEPTH  200
#define MOTION_DEPTH_THRESHOLD  50
#define FORWARD_SCALING 0.5
#define FORWARD_MOTION_LIMIT 40

#define MOTION_ROTATION_THRESHOLD 19
#define ROTATION_SCALING 1 // should increase
#define ROTATION_MOTION_LIMIT 10


#define TILT_INPUT_SCALING 0.2
#define PAN_INPUT_SCALING 0.2
#define MAX_PAN 20
#define MAX_TILT 20

#define MAX_ERROR 10
float pan_error_integrator = 0;
float tilt_error_integrator = 0;
float prev_pan_error = 0;
float prev_tilt_error = 0;
Timer error_timer;
#define PAN_D_SCALING 0//-0.1
#define TILT_D_SCALING 0//-0.1
#define PAN_I_SCALING 0.0001
#define TILT_I_SCALING 0.0001
#define DEBUG

RobotDog dog;
CreepGaitCoordinator creep_gait_coordinator(&dog);
BalanceHandler balancer(&dog);

bool in_ball_detection_mode = true;

Pixy2SPI_SS pixy;
ObjectPositionTracker tracker(&pixy);

// COORDINATION INFORMATION
struct MotionCommand {
    float x_motion;
    float y_motion;
    float yaw_motion;
    float pan;
    float tilt;
    bool send_motion;

    MotionCommand() {
        x_motion = 0;
        y_motion = 0;
        yaw_motion = 0;
        pan = 0;
        tilt = 0;
        send_motion = false;
    }
};

PixyObject tracked_pixy_object;
TrackedObjectDynamics tracked_object;

MotionCommand desired_motion;
bool has_returned = true;
Timer command_check_timer;
#define COMMAND_CHECK_TIMER_PERIOD 0.5



SensorHistory<float> object_depth_history(5);

// PARAMETERS


float temp_pan_save;

void setup() {
    Serial.begin(9600);  
  
    pinMode(start_LED_pin, OUTPUT);
    pinMode(ground_mode_LED_pin, OUTPUT);
    pinMode(serial_LED_pin, OUTPUT);

    digitalWrite(start_LED_pin, HIGH);
    //while (!Serial) {}
    Serial.println("Beginning...");
    dog.begin();
    dog.resetDefaultStance();
    digitalWrite(start_LED_pin, LOW);
    Serial.println("Taring IMU...");
    digitalWrite(start_LED_pin, HIGH);
    delay(1000);
    dog.tareIMU();
    Serial.println("Tare done");
    digitalWrite(start_LED_pin, LOW);
  
    digitalWrite(serial_LED_pin, LOW);
    
    pixy.init();
    Serial.println("-----------------");
    Serial.println("...Actually Starting");
    Serial.flush();

    creep_gait_coordinator.toggleFollowExternalOrientation();
    error_timer.reset();
}

void loop() {
    tracker.operate();

    if (in_ball_detection_mode) {
        receiveAndCheckPixyDetection();
        decideAndSendNextCommandToCoordinator();
    }

    creep_gait_coordinator.operate();
    dog.operate();

    flushInput();
}

void receiveAndCheckPixyDetection() {
    if (tracker.objectWasDetected()) {
        tracked_pixy_object = tracker.getPixyObject();
        tracked_object = tracker.getObjectDynamics();
        desired_motion = convertTrackedPositionToMotionCommand();
        digitalWrite(serial_LED_pin, HIGH);
    } else {
        digitalWrite(serial_LED_pin, LOW);
    }

}

void decideAndSendNextCommandToCoordinator() {
    if (tracker.objectWasDetected()) {
        // tilt to look at the ball
        Rot desired_orientation = Rot(0, desired_motion.tilt, desired_motion.pan);
        creep_gait_coordinator.modifyBodyOrientation(desired_orientation);

        // send movement command if appropriate
        if (creep_gait_coordinator.isWaiting()) {
            if (has_returned && desired_motion.send_motion) {
                creep_gait_coordinator.sendMotionCommand(desired_motion.x_motion, 
                                                         desired_motion.y_motion, 
                                                         desired_motion.yaw_motion);
                has_returned = false;
            } else if (!has_returned) {
                creep_gait_coordinator.sendReturnToCOMCommand();  
                has_returned = true;
                digitalWrite(ground_mode_LED_pin, LOW);
                digitalWrite(start_LED_pin, LOW);
            }
        }
    } 
    else {
        creep_gait_coordinator.sendReturnToCOMCommand();  
        has_returned = true;
    }
}

MotionCommand convertTrackedPositionToMotionCommand() {
    MotionCommand command;

    Point object_position = tracked_object.position;
    float object_x = object_position.x;
    float object_y = -object_position.y;
    float raw_object_z = object_position.z;

        object_depth_history.updateHistory(raw_object_z);
        float object_z = object_depth_history.getValue();
        float object_z_error = object_z - DESIRED_OBJECT_DEPTH;
        Serial.println(object_z);
        
    float dt = error_timer.dt();
    error_timer.reset();

    if (has_returned && creep_gait_coordinator.isWaiting()) {
        pan_error_integrator += dt * object_x;
        tilt_error_integrator += dt * object_y;
    }
    
    float dpan_error = (object_x - prev_pan_error) / dt;
    float dtilt_error = (object_y - prev_tilt_error) / dt;

    
    prev_pan_error = object_x;
    prev_tilt_error = object_y;

    command.pan = PAN_INPUT_SCALING * object_x + 
                  PAN_D_SCALING * dpan_error + 
                  PAN_I_SCALING * fbound(pan_error_integrator, -MAX_ERROR, MAX_ERROR);
    command.pan = fbound(command.pan, -MAX_PAN, MAX_PAN);
    command.tilt = TILT_INPUT_SCALING * object_y + 
                   TILT_D_SCALING * dtilt_error + 
                   TILT_I_SCALING * fbound(tilt_error_integrator, -MAX_ERROR, MAX_ERROR);
    command.tilt = fbound(command.tilt, -MAX_TILT, MAX_TILT);

//Serial.println(command.pan);
    // obtain motion
    if (has_returned && creep_gait_coordinator.isWaiting()) {

    
        if (fabs(command.pan) > MOTION_ROTATION_THRESHOLD) {
            command.yaw_motion = fbound(command.pan * ROTATION_SCALING, -ROTATION_MOTION_LIMIT, ROTATION_MOTION_LIMIT);
            command.send_motion = true;
            digitalWrite(ground_mode_LED_pin, HIGH);
        } 
        else if (fabs(object_z_error) > MOTION_DEPTH_THRESHOLD) { // object z > or < predetermined
            command.x_motion = fbound(object_z_error * FORWARD_SCALING, -FORWARD_MOTION_LIMIT, FORWARD_MOTION_LIMIT);
            command.send_motion = true;
            digitalWrite(start_LED_pin, HIGH);
        }
    }

    temp_pan_save = command.pan;

    return command;
}


void flushInput() {
    desired_motion = MotionCommand();
}
