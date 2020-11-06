#include <Dog.h>
//#include <CreepGaitInfo.h>
#include "SoftwareSerial.h"
#include "Timer.h"

// Communication
// rx to pin9, tx to pin10
#define XBee Serial2

#define start_LED_pin 22 // 21 20
#define serial_LED_pin 21


RobotDog dog;

enum ActionMode {STARTUP, NORMAL};

struct CreepStateInfo {
    typedef void (*action_function)(ActionMode); // type for conciseness
    Timer state_end_timer;
    bool is_in_startup;
    bool end_flag;
    float period;
    action_function action;

    CreepStateInfo() = default;

    CreepStateInfo(float new_period, action_function new_action) {
        period = new_period;
        state_end_timer.reset(period * 1.1);
        action = new_action;
        is_in_startup = true;
        end_flag = false;
    }

    void start() {
        is_in_startup = false;
        state_end_timer.reset();
    }

    void reset() {
        is_in_startup = true;
        end_flag = false;
    }

    bool isTimeToEnd() {
        return (state_end_timer.timeOut()) || end_flag;
    }

    void end() {
        end_flag = true;
    }

    void startupAction() {
        action(STARTUP);
    }

    void normalAction() {
        action(NORMAL);
    }
};

// COORDINATION INFORMATION
int state = 0;
int foot_to_move = 0;
Point desired_translation = POINT_ZERO;
Rot desired_rotation = ROT_ZERO;
bool input_is_valid;
Point next_foot_anchor_oC;
Rot current_rotation = ROT_ZERO;
float joystick[4];

// PARAMETERS
#define NUM_CREEP_STATES 5
CreepStateInfo creep_state_info[NUM_CREEP_STATES];
Point default_body_position = Point(0, 0, 100);
#define TRANSLATION_INPUT_SCALING 30
#define TRANSLATION_INPUT_THRESHOLD 5
#define LIFT_HEIGHT 30
int step_order[4] = {0, 2, 3, 1};
int step_order_iterator = 3;


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
  
    creep_state_info[0] = CreepStateInfo(1, getNextMotion);
    creep_state_info[1] = CreepStateInfo(0.2, prepareCOM);
    creep_state_info[2] = CreepStateInfo(0.1, liftFoot);
    creep_state_info[3] = CreepStateInfo(0.2, plantFoot);
    creep_state_info[4] = CreepStateInfo(0.3, returnCOM);

    pinMode(serial_LED_pin, OUTPUT);
    digitalWrite(serial_LED_pin, LOW);
    
    
    Serial.println("-----------------");
    Serial.println("...Actually Starting");
    Serial.flush();
}

void loop() {  
    receiveSerialInput();
    if (creep_state_info[state].is_in_startup) {
        creep_state_info[state].startupAction();
        creep_state_info[state].start();
    }
    
    creep_state_info[state].normalAction();
    
    if (creep_state_info[state].isTimeToEnd()) {
        creep_state_info[state].reset();
        switchToNextState();
    }

    dog.operate();

//    flushSerialInput();
}


bool inputIsValid() {
    return (desired_translation.norm() > TRANSLATION_INPUT_THRESHOLD) || (desired_rotation.norm() > 3);
}

void getNextMotion(ActionMode mode) {
    if (mode == STARTUP) {
        desired_translation = POINT_ZERO;
        desired_rotation = ROT_ZERO;
    } else {
        float x_motion = joystick[0] * TRANSLATION_INPUT_SCALING;
        float y_motion = -joystick[1] * TRANSLATION_INPUT_SCALING;
        float yaw_motion = joystick[3] * -5;

        desired_translation = Point(x_motion, y_motion, 0);
        desired_rotation = Rot(0, 0, yaw_motion);

        if (inputIsValid()) {
            creep_state_info[state].end();
            input_is_valid = false;
        } else {
            creep_state_info[state].state_end_timer.reset();
        }
    } 
}

int chooseFootToMove() {
    step_order_iterator = (step_order_iterator + 1)%4;
    return step_order[step_order_iterator];
}

Point chooseNextFootAnchor_fF() {
    // assume foot_to_move has already been set 
    Point default_anchor = dog.getDefaultFootPosition(foot_to_move, Frame::BODY) + Point(0, 0, dog.getStartingHeight()); 
    return (default_anchor  + desired_translation) * (current_rotation + desired_rotation); // body frame movement
    //return default_anchor * (current_rotation + desired_rotation) + desired_translation; // ground frame movement
}

void prepareCOM(ActionMode mode) {
    if (mode == STARTUP) {
        foot_to_move = chooseFootToMove();
        next_foot_anchor_oC = chooseNextFootAnchor_fF();
        desired_rotation.print();
        (dog.getDefaultFootPosition(foot_to_move, Frame::BODY) + Point(0, 0, dog.getStartingHeight())).print(); 
        next_foot_anchor_oC.print();
        dog.switchFootStance(foot_to_move, FootStance::SET);
        
        float time = creep_state_info[state].period * 1.5;
        dog.moveBodyToPositionFromCentroid(default_body_position, Frame::FLOOR, time);
    } else {
    
    
    }
}


void liftFoot(ActionMode mode) {
    if (mode == STARTUP) {
        Point lift_height = Point(0,0,LIFT_HEIGHT);
        Point next_foot_position_oBfF = (dog.getFootPositionFromBody(foot_to_move, Frame::FLOOR) + lift_height);
        
        float time = creep_state_info[state].period;
        dog.moveFootToPositionFromBody(foot_to_move, next_foot_position_oBfF, Frame::FLOOR, time);        
    } else {

    }
}

void plantFoot(ActionMode mode) {
    if (mode == STARTUP) {
        Point centroid_position = -dog.getBodyPositionFromCentroid(Frame::FLOOR);
        Point next_foot_position_oBfF = (centroid_position + next_foot_anchor_oC);
        //next_foot_position_oBfF.print();
        float time = creep_state_info[state].period;
        dog.moveFootToPositionFromBody(foot_to_move, next_foot_position_oBfF, Frame::FLOOR, time);
    } else {
    
    }
}

void returnCOM(ActionMode mode) {
    if (mode == STARTUP) {
        dog.switchFootStance(foot_to_move, FootStance::PLANTED);
        Point default_height = Point(0,0,100);
        
        float time = creep_state_info[state].period;
        dog.moveBodyToPositionFromCentroid(default_height, Frame::FLOOR, time);
        dog.moveBodyToOrientation(current_rotation + desired_rotation, time);
        current_rotation += desired_rotation;        
    } else {
        
    }
}

void switchToNextState() {
    state = (state+1)%NUM_CREEP_STATES;
    Serial.print("State: "); Serial.println(state);
}

void receiveSerialInput() {
    if (XBee.available()) {
        // RY, RX, LY, LX
        // First recieve joystick values
        for (int i = 0; i < 4; i++) {
            joystick[i] = (XBee.parseFloat());
        }
        
        // Then retrieve button values
        String s = XBee.readStringUntil('\n');
    }
}
