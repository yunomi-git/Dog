#include <Dog.h>
#include "CreepGaitCoordinator.h"
#include "SoftwareSerial.h"
#include "Timer.h"

// Communication
// rx to pin9, tx to pin10
#define XBee Serial2
float joystick[4];
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

enum CommandType {MOTION, RETURN_LEG, REORIENT};


MotionCommand desired_motion;
bool motion_command_recieved = false;

bool has_returned = true;
bool leg_return_command_started = false;
Timer command_check_timer;
#define COMMAND_CHECK_TIMER_PERIOD 3
int leg_return_iterator = 0;

// PARAMETERS
#define TRANSLATION_INPUT_SCALING 40
#define TRANSLATION_INPUT_THRESHOLD 5
#define ROTATION_INPUT_SCALING 8

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
        for (int i = 0; i < 4; i++) {
            // RY, RX, LY, LX
            joystick[i] = (XBee.parseFloat());
        }
        buttons_read = XBee.readStringUntil('\n');
        
        digitalWrite(serial_LED_pin, HIGH);
    } else {
        digitalWrite(serial_LED_pin, LOW);
    }
}

void processSerialInput() {
    desired_motion.x_motion = joystick[0] * TRANSLATION_INPUT_SCALING;
    desired_motion.y_motion = -joystick[1] * TRANSLATION_INPUT_SCALING;
    desired_motion.yaw_motion = joystick[3] * -ROTATION_INPUT_SCALING;
    
    Point desired_translation = Point(desired_motion.x_motion, desired_motion.y_motion, 0);
    Rot desired_rotation = Rot(0, 0, desired_motion.yaw_motion);
    if ((desired_translation.norm() > TRANSLATION_INPUT_THRESHOLD) || (desired_rotation.norm() > 3)) {
        motion_command_recieved = true;
    } else {
        motion_command_recieved = false;
    }

    if (buttons_read.indexOf("B1") > 0) {
        leg_return_command_started = true;
    }
}

void decideAndSendNextCommandToCoordinator() {
    if (creep_gait_coordinator.isWaiting()) {
        if (motion_command_recieved) {
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
                creep_gait_coordinator.sendMotionCommand(0, 0, 0);
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
    motion_command_recieved = false;
}
