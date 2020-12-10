#include <Dog.h>
#include "CreepGaitCoordinator.h"
#include "BalanceHandler.h"
#include "SoftwareSerial.h"
#include "FunctionGenerator.h"
#include "Timer.h"

#define DEBUG

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
#define ground_mode_LED_pin 20

RobotDog dog;
CreepGaitCoordinator creep_gait_coordinator(&dog);
BalanceHandler balancer(&dog);

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
#define COMMAND_CHECK_TIMER_PERIOD 0.5
Rot command_orientation = ROT_ZERO;
#define ORIENT_TIMER_PERIOD 0.005
Timer orient_command_timer;
bool do_balancing = false;
bool do_reorientation = false;
bool new_serial_read = false;

// PARAMETERS
bool use_balancer = true;

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
    XBee.begin(19200);
  
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
    
    
    Serial.println("-----------------");
    Serial.println("...Actually Starting");
    Serial.flush();

    balancer.setPIDGains(1, 0.02, 0.04);
    balancer.setBalancingMagnitudeLimits(Rot(30, 30, 30));

    orient_command_timer.reset(ORIENT_TIMER_PERIOD);
}

void loop() {
    balancer.setDesiredOrientation(creep_gait_coordinator.getCurrentOrientation());
    balancer.operate();
    doPIDBalancing();
    receiveAndCheckSerialInput();
    if (new_serial_read) {
        processSerialInput();
        decideAndSendNextCommandToCoordinator();
    }


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

        new_serial_read = true;
        digitalWrite(serial_LED_pin, HIGH);
    } else {
      new_serial_read = false;
        digitalWrite(serial_LED_pin, LOW);
    }
}

void processSerialInput() {
  if (buttons_read.indexOf("RZ1") > 0) {
      do_reorientation = !do_reorientation;
  }

    if (do_reorientation) { 
        float x_angle = joystick.rx * MAX_X_ORIENTATION;
        float y_angle = joystick.ry* MAX_Y_ORIENTATION;
        float z_angle = -joystick.lx * MAX_Z_ORIENTATION;

        command_orientation = Rot(x_angle, y_angle, z_angle);

        command_recieved = REORIENT;
    } 
    else if (buttons_read.indexOf("B1") > 0) {
        command_recieved = RETURN_LEG;
        leg_return_command_started = true;
    }
    else {
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

    if (buttons_read.indexOf("B2") > 0) {
        do_balancing = !do_balancing;
        creep_gait_coordinator.toggleMoveInGroundFrame();
    }
}

bool buttonsContains(String button) {
    return buttons_read.indexOf(button) > 0;
}

void doPIDBalancing() {
    digitalWrite(ground_mode_LED_pin, LOW); 
    if (do_balancing) {
        digitalWrite(ground_mode_LED_pin, HIGH);
        // balancer.setDesiredOrientation(creep_gait_coordinator.getCurrentOrientation());
        Rot balancing_orientation = balancer.getNextKinematicBalancingOrientation();
        creep_gait_coordinator.modifyBodyOrientation(balancing_orientation);
    }
}

void decideAndSendNextCommandToCoordinator() {
    if (!do_balancing && !do_reorientation){
         creep_gait_coordinator.modifyBodyOrientation(ROT_ZERO);
    }

    if (creep_gait_coordinator.isWaiting()) {
        if (do_reorientation) {
            if (orient_command_timer.timeOut()) {
                creep_gait_coordinator.modifyBodyOrientation(command_orientation);
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
}

void flushInput() {
    joystick.ry = 0;
    joystick.rx = 0;
    joystick.ly = 0;
    joystick.lx = 0;
    buttons_read = "";
    command_recieved = NONE;
}
