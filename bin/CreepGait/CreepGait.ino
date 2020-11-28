#include <Dog.h>
#include "CreepGaitCoordinator.h"
#include "SoftwareSerial.h"
#include "Timer.h"

// Communication
// rx to pin9, tx to pin10
#define XBee Serial2
struct Joystick_t {
    float rx;
    float ry;
    float lx;
    float ly;

    Joystick_t() {
        rx = 0; ry = 0; lx = 0; ly = 0;
    }
};

Joystick_t joystick;
//float joystick[4]; // RY, RX, LY, LX
String buttons_read;

// LEDs
#define start_LED_pin 22 // 21 20
#define serial_LED_pin 21

RobotDog dog;
CreepGaitCoordinator creep_gait_coordinator(&dog);

// COORDINATION INFORMATION
struct MotionCommand {
    float x_motion;
    float y_motion;
    float yaw_motion;

    MotionCommand() {
        x_motion = 0;
        y_motion = 0;
        yaw_motion = 0;
    }
};

enum CommandType {NONE, MOTION, RETURN_LEG, REORIENT};

CommandType command_recieved = NONE;
CommandType last_command_recieved = command_recieved;

bool leg_return_command_started = false;

MotionCommand desired_motion;
bool has_returned = true;
int leg_return_iterator = 0;
Timer command_check_timer;
#define COMMAND_CHECK_TIMER_PERIOD 1.5
Rot command_orientation = ROT_ZERO;
#define ORIENT_TIMER_PERIOD 0.005
Timer orient_command_timer;

// PARAMETERS
#define TRANSLATION_INPUT_SCALING 40
#define TRANSLATION_INPUT_THRESHOLD 5
#define ROTATION_INPUT_SCALING 8

#define MAX_X_ORIENTATION 20
#define MAX_Y_ORIENTATION 20
#define MAX_Z_ORIENTATION 20

void setup() {
    Serial.begin(9600);  
    XBee.begin(19200);
  
    pinMode(start_LED_pin, OUTPUT);
    digitalWrite(start_LED_pin, HIGH);
    //while (!Serial) {}
    Serial.println("Beginning...");
    dog.begin();
    dog.resetDefaultStance();
    Serial.println("Taring IMU...");
    dog.tareIMU();
    Serial.println("Tare done");
    digitalWrite(start_LED_pin, LOW);
  
    pinMode(serial_LED_pin, OUTPUT);
    digitalWrite(serial_LED_pin, LOW);
    
    
    Serial.println("-----------------");
    Serial.println("...Actually Starting");
    Serial.flush();

    orient_command_timer.reset(ORIENT_TIMER_PERIOD);
}

void loop() {  
    receiveAndCheckSerialInput();
    processSerialInput();
    decideAndSendNextCommandToCoordinator();

    creep_gait_coordinator.operate();
    dog.operate();

    flushInput();
}

void receiveAndCheckSerialInput() {
    if (XBee.available()) {
        joystick.ry = XBee.parseFloat();
        joystick.rx = XBee.parseFloat();
        joystick.ly = XBee.parseFloat();
        joystick.lx = XBee.parseFloat();
        buttons_read = XBee.readStringUntil('\n');

        digitalWrite(serial_LED_pin, HIGH);
    } else {
        digitalWrite(serial_LED_pin, LOW);
    }
}

void processSerialInput() {
    if (buttons_read.indexOf("RZ2") > 0) { 
        float x_angle = joystick.rx * MAX_X_ORIENTATION;
        float y_angle = joystick.ry* MAX_Y_ORIENTATION;
        float z_angle = joystick.lx * MAX_Z_ORIENTATION;

        command_orientation = Rot(x_angle, y_angle, z_angle);

        command_recieved = REORIENT;
    } 
    else if (buttons_read.indexOf("B1") > 0) {
        command_recieved = RETURN_LEG;
        leg_return_command_started = true;
    }
    else {
        //orient_command_active = false;
        desired_motion.x_motion = joystick.ry * TRANSLATION_INPUT_SCALING;
        desired_motion.y_motion = -joystick.rx * TRANSLATION_INPUT_SCALING;
        desired_motion.yaw_motion = joystick.lx * -ROTATION_INPUT_SCALING;
        
        Point desired_translation = Point(desired_motion.x_motion, desired_motion.y_motion, 0);
        Rot desired_rotation = Rot(0, 0, desired_motion.yaw_motion);
        if ((desired_translation.norm() > TRANSLATION_INPUT_THRESHOLD) || (desired_rotation.norm() > 3)) {
            command_recieved = MOTION;
        } else {
            command_recieved = NONE;
        }
    }
}

void decideAndSendNextCommandToCoordinator() {
    if (creep_gait_coordinator.isWaiting()) {
        if (command_recieved == REORIENT) {
            if (orient_command_timer.timeOut()) {
                Rot current_orientation = creep_gait_coordinator.getCurrentOrientation();
                dog.moveBodyToOrientation(command_orientation + current_orientation, TIME_INSTANT);
                orient_command_timer.reset();
            }
        }
        else if (command_recieved == MOTION) {
            creep_gait_coordinator.sendMotionCommand(desired_motion.x_motion, 
                                                     desired_motion.y_motion, 
                                                     desired_motion.yaw_motion);
            command_check_timer.reset(COMMAND_CHECK_TIMER_PERIOD);
            has_returned = false;
            leg_return_command_started = false;
            leg_return_iterator = 0;
        }
        else if (leg_return_command_started) {
            has_returned = false;
            if (leg_return_iterator < 4) {
                creep_gait_coordinator.sendMotionCommandUsingFoot(leg_return_iterator, 0, 0, 0);
                leg_return_iterator++;
            } else if (!has_returned) {
                creep_gait_coordinator.sendReturnToCOMCommand();  
                has_returned = true;
                leg_return_command_started = false;
                leg_return_iterator = 0;
            }
            command_check_timer.reset(COMMAND_CHECK_TIMER_PERIOD);
        }
        else if (command_check_timer.timeOut()) {
            if (!has_returned) {
                creep_gait_coordinator.sendReturnToCOMCommand();  
                has_returned = true;
            } 
        } 
    } 
//    if (orientation_command_recieved) {
//        creep_gait_coordinator.modifyOrientation();
//    }   
}

void flushInput() {
    command_recieved = NONE;
}
