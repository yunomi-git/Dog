#include <Dog.h>
#include "BalanceHandler.h"

RobotDog dog;
BalanceHandler balancer(&dog);

#define start_LED_pin 22

void setup() {
    Serial.begin(9600);
    pinMode(start_LED_pin, OUTPUT);
    digitalWrite(start_LED_pin, HIGH);
    //while (!Serial) {}
    Serial.println("Beginning...");
    dog.begin();
    dog.resetDefaultStance();
    digitalWrite(start_LED_pin, LOW);
    Serial.println("Taring IMU...");
    delay(1000);
    digitalWrite(start_LED_pin, HIGH);
    dog.tareIMU();
    Serial.println("Tare done...Actually Starting");
    digitalWrite(start_LED_pin, LOW);

//    balancer.setIDGains(0.010, 0.04);
//    balancer.setDesiredOrientation(Rot(0, 0, 0));
//    balancer.setBalancingVelocityLimit(0.2);
//    balancer.setBalancingMagnitudeLimits(Rot(30, 30, 30));

    
    balancer.setIDGains(0.000, 0.04);
    balancer.setBalancingVelocityLimit(10);
    balancer.setBalancingMagnitudeLimits(Rot(60, 60, 60));
}

void loop() {
    balancer.operate();

    Rot desired_orientation = balancer.getNextKinematicBalancingOrientation();
    Point desired_position = Point(0, 0, dog.getStartingHeight());

    dog.moveBodyToOrientation(desired_orientation, TIME_INSTANT);        
    dog.moveBodyToPositionFromCentroid(desired_position, Frame::GROUND, TIME_INSTANT);

    dog.operate();
}
