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
Rot desired_leg_rotation = ROT_ZERO;
bool input_is_valid;
Point next_foot_anchor_oC;
Rot current_rotation = ROT_ZERO;
Point body_distance_from_original_centroid = POINT_ZERO;
float joystick[4];

// PARAMETERS
#define NUM_CREEP_STATES 4
CreepStateInfo creep_state_info[NUM_CREEP_STATES];
Point default_body_position = Point(0, 0, 100);
#define LIFT_HEIGHT 30

#define TRANSLATION_INPUT_SCALING 40
#define TRANSLATION_INPUT_THRESHOLD 5
#define ROTATION_INPUT_SCALING 8
#define ROTATION_LEG_INPUT_MULTIPLIER 3

#define STATE_PREPARE_TIME 0.5
#define STATE_PREPARE_OVERLAP_FACTOR 0.4
#define STATE_LIFT_TIME 0.1
#define STATE_LIFT_OVERLAP_FACTOR 0
#define STATE_PLANT_TIME 0.2
#define STATE_PLANT_OVERLAP_FACTOR 0.7
#define STATE_RETURN_TIME 0.25
#define STATE_RETURN_OVERLAP_FACTOR 0


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
    creep_state_info[1] = CreepStateInfo(STATE_PREPARE_TIME * (1-STATE_PREPARE_OVERLAP_FACTOR), prepareCOM);
    creep_state_info[2] = CreepStateInfo(STATE_LIFT_TIME * (1-STATE_LIFT_OVERLAP_FACTOR), liftFoot);
    creep_state_info[3] = CreepStateInfo(STATE_PLANT_TIME * (1-STATE_PLANT_OVERLAP_FACTOR), plantFoot);
    //creep_state_info[4] = CreepStateInfo(STATE_RETURN_TIME * (1-STATE_RETURN_OVERLAP_FACTOR), returnCOM);

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
      Serial.println("---");
      Serial.print("End Leg Position fF: "); dog.getFootPositionFromBody(foot_to_move, Frame::FLOOR).print();
      Serial.print("World Position: "); dog.getBodyPositionFromWorld_fF().print();
        desired_translation = POINT_ZERO;
        desired_rotation = ROT_ZERO;
        desired_leg_rotation = desired_rotation * ROTATION_LEG_INPUT_MULTIPLIER;
//        Serial.print("Body Position from centroid: "); dog.getBodyPositionFromCentroid(Frame::FLOOR).print();
//        Serial.println("Planted Foot Positions: ");
//        for (int i = 0; i < 4; i++) {
//            if (dog.getFootStance(i) == FootStance::PLANTED) {
//                Serial.print(dog.getFootID(i)); Serial.print(": "); dog.getFootPositionFromBody(i, Frame::FLOOR).print();
//            }
//        }
//        Serial.print("Calculated Centroid: "); dog.calculateCentroid_oBfF().print();
    } else {
        float x_motion = joystick[0] * TRANSLATION_INPUT_SCALING;
        float y_motion = -joystick[1] * TRANSLATION_INPUT_SCALING;
        float yaw_motion = joystick[3] * -ROTATION_INPUT_SCALING;

        desired_translation = Point(x_motion, y_motion, 0);
        desired_rotation = Rot(0, 0, yaw_motion);
        desired_leg_rotation = desired_rotation * ROTATION_LEG_INPUT_MULTIPLIER;

        if (inputIsValid()) {
            creep_state_info[state].end();
            input_is_valid = false;
        } else {
            creep_state_info[state].state_end_timer.reset();
        }
    } 
}


void prepareCOM(ActionMode mode) {
    if (mode == STARTUP) {
        dog.switchFootStance(foot_to_move, FootStance::PLANTED);
        Point old_body_distance_from_planted_centroid = -(dog.getBodyPositionFromCentroid(Frame::FLOOR) - default_body_position);;
        
        foot_to_move = chooseFootToMove();
        next_foot_anchor_oC = calculateNextFootAnchor_fF(foot_to_move);
        Serial.print("Next Anchor: "); next_foot_anchor_oC.print();
        
        dog.switchFootStance(foot_to_move, FootStance::SET);
        Point old_body_distance_from_set_centroid = -(dog.getBodyPositionFromCentroid(Frame::FLOOR) - default_body_position);
        body_distance_from_original_centroid = old_body_distance_from_set_centroid - old_body_distance_from_planted_centroid;
        
        float time = creep_state_info[state].period;
        dog.moveBodyToPositionFromCentroid(default_body_position, Frame::FLOOR, time);
        dog.moveBodyToOrientation(current_rotation + desired_rotation, time);
        current_rotation += desired_rotation;    
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
        Point next_foot_position_oBfF = (centroid_position + next_foot_anchor_oC) - body_distance_from_original_centroid;

        float time = creep_state_info[state].period;
        dog.moveFootToPositionFromBody(foot_to_move, next_foot_position_oBfF, Frame::FLOOR, time);
    } else {
    
    }
}



int chooseFootToMove() {
//    float max_distance = 0;
//    int foot_to_move = 0;
//    for (int i = 0; i < NUM_LEGS; i++) {
//        Point next_foot_position = dog.getFootPositionFromBody(i, FRAME::BODY) * desired_rotation + desired_translation;
//        Point deviation_from_anchor = next_foot_position - calculateNextFootAnchor_fF(i);// needs to be written
//        float distance = deviation_from_anchor.norm();
//        if (distance < max_distance) {
//            foot_to_move = i;
//            max_distance = distance;
//        }
//    }
//    return foot_to_move;
    step_order_iterator = (step_order_iterator + 1)%4;
    return step_order[step_order_iterator];
}

Point calculateNextFootAnchor_fF(int foot_i) {
    Point default_anchor = dog.getDefaultFootPosition(foot_i, Frame::BODY) + Point(0, 0, dog.getStartingHeight());
    Serial.print("Begin Leg Position fF: "); dog.getFootPositionFromBody(foot_i, Frame::FLOOR).print();
    Serial.print("Default Anchor: "); default_anchor.print();  
    return (default_anchor  + desired_translation) * (current_rotation + desired_leg_rotation); // body frame movement
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
        String buttons_read = XBee.readStringUntil('\n');
        digitalWrite(serial_LED_pin, HIGH);
    } else {
        digitalWrite(serial_LED_pin, LOW);
    }
}
