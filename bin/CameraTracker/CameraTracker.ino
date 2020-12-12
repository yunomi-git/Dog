#include <Dog.h>
#include "CreepGaitCoordinator.h"
#include "BalanceHandler.h"
#include "SoftwareSerial.h"
#include "FunctionGenerator.h"
#include "Timer.h"
#include "PixyTracker.h"

// LEDs
#define start_LED_pin 22 // 21 20
#define serial_LED_pin 21
#define ground_mode_LED_pin 20

#define TILT_INPUT_SCALING 15
#define PAN_INPUT_SCALING -15

#define DEBUG

RobotDog dog;
CreepGaitCoordinator creep_gait_coordinator(&dog);
BalanceHandler balancer(&dog);

Pixy2SPI_SS pixy;
ObjectPositionTracker tracker(&pixy);

// COORDINATION INFORMATION
struct MotionCommand {
    float x_motion;
    float y_motion;
    float yaw_motion;
    float pan;
    float tilt;

    MotionCommand() {
        x_motion = 0;
        y_motion = 0;
        yaw_motion = 0;
        pan = 0;
        tilt = 0;
    }
};

enum CommandType {NONE, MOTION, RETURN_LEG, REORIENT};

CommandType command_recieved = NONE;
CommandType last_command_recieved = command_recieved;

bool leg_return_command_started = false;

PixyObject tracked_pixy_object;

MotionCommand desired_motion;
bool has_returned = true;
int leg_return_iterator = 0;
Timer command_check_timer;
#define COMMAND_CHECK_TIMER_PERIOD 0.5
Rot command_orientation = ROT_ZERO;
#define ORIENT_TIMER_PERIOD 0.005
Timer orient_command_timer;

// PARAMETERS

#define TRANSLATION_INPUT_SCALING 40
#define TRANSLATION_INPUT_THRESHOLD 15
#define ROTATION_INPUT_SCALING 8

#define MAX_X_ORIENTATION 20
#define MAX_Y_ORIENTATION 20
#define MAX_Z_ORIENTATION 20

FunctionGenerator generator;
float wave_magnitude = 0;
float rps = 0.5;
float magnitude = 10;

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

    orient_command_timer.reset(ORIENT_TIMER_PERIOD);
}

void loop() {
    tracker.operate();
    
    receiveAndCheckPixyDetection();
    decideAndSendNextCommandToCoordinator();

    creep_gait_coordinator.operate();
    dog.operate();

    flushInput();
}

void receiveAndCheckPixyDetection() {
    if (tracker.objectWasDetected()) {
        tracked_pixy_object = tracker.getPixyObject();
        digitalWrite(serial_LED_pin, HIGH);
    } else {
        digitalWrite(serial_LED_pin, LOW);
    }

}

void decideAndSendNextCommandToCoordinator() {
    if (creep_gait_coordinator.isWaiting()) {
        if (has_returned && tracker.objectWasDetected()) {
            desired_motion = convertTrackedPositionToMotionCommand();
//            creep_gait_coordinator.sendMotionCommand(desired_motion.x_motion, 
//                                                     desired_motion.y_motion, 
//                                                     desired_motion.yaw_motion);
            Rot desired_orientation = Rot(0, desired_motion.tilt, desired_motion.pan);
            //creep_gait_coordinator.modifyBodyOrientation(desired_orientation);
            //has_returned = false;
            dog.moveBodyToOrientationAtSpeed(desired_orientation, 40);
        } else if (!has_returned) {
            creep_gait_coordinator.sendReturnToCOMCommand();  
            has_returned = true;
        } 
    } 
}

MotionCommand convertTrackedPositionToMotionCommand() {
    float pan;
    float tilt;

    float object_x_position = tracked_pixy_object.x/PIXY_CAMERA_W_PIX;
    float object_y_position = tracked_pixy_object.y/PIXY_CAMERA_H_PIX;
    
    Rot current_kinematic_orientation = dog.getBodyKinematicOrientation_fF2B();
    float current_dog_pan = current_kinematic_orientation.z;
    float current_dog_tilt = current_kinematic_orientation.y;

    pan = current_dog_pan + PAN_INPUT_SCALING * object_x_position;
    tilt = current_dog_tilt + TILT_INPUT_SCALING * object_y_position;

    MotionCommand command;
    command.pan = pan;
    command.tilt = tilt;

    #ifdef DEBUG
      tracker.printPixyData();
    #endif
    Serial.print("Pan: "); Serial.print(pan); Serial.print(", ");
    Serial.print("Tilt: "); Serial.println(tilt);
    return command;
}

void flushInput() {
    
}
