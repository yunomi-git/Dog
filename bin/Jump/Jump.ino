#include <Dog.h>

RobotDog dog;

#define start_LED_pin 14

Timer measurement_update_timer;
float measurement_interval = 3.0/1000;

void setup() {
    Serial.begin(9600);
    pinMode(start_LED_pin, OUTPUT);
    digitalWrite(start_LED_pin, HIGH);
    //while (!Serial) {}
    Serial.println("Beginning...");
    dog.begin();
    dog.moveToPosition(Point(0,0,100), Frame::BODY, 2);
    digitalWrite(start_LED_pin, LOW);

    jump();
}

void loop() {

}

Timer jump_timer(1);

void spinJump() {
    Point crouch_height = Point(0, 0, 50);
    Point extension_height = Point(0,0,140);
    Rot extension_orientation = Rot(0,0,-17);
    Point reset_height = Point(0, 0, 60);
    //dog.resetDefaultStance();
    delay(2000);
    dog.moveToPosition(crouch_height, Frame::BODY, 1);
    jump_timer.reset(1.0);
    while(!jump_timer.timeOut()) {dog.operate();}
    delay(500);
    digitalWrite(start_LED_pin, HIGH);
    delay(270);
    dog.moveToPosition(extension_height, Frame::BODY, TIME_INSTANT);
    dog.moveToOrientation(extension_orientation, TIME_INSTANT);
    dog.operate();
    delay(70);
    dog.moveToPosition(reset_height, Frame::BODY, TIME_INSTANT);
    dog.moveToOrientation(ROT_ZERO, TIME_INSTANT);
    dog.operate();
}

void jump() {
    Point crouch_height = Point(0, 0, 50);
    Point extension_height = Point(0,0,140);
    Point reset_height = Point(0, 0, 60);
    //dog.resetDefaultStance();
    delay(2000);
    dog.moveToPosition(crouch_height, Frame::BODY, 1);
    jump_timer.reset(1.0);
    while(!jump_timer.timeOut()) {dog.operate();}
    delay(500);
    digitalWrite(start_LED_pin, HIGH);
    delay(270);
    dog.moveToPosition(extension_height, Frame::BODY, TIME_INSTANT);
    dog.operate();
    delay(70);
    dog.moveToPosition(reset_height, Frame::BODY, TIME_INSTANT);
    dog.operate();
}


//double kp = 1;
//double kd = 0.1;
//double ki = 0.001;
//
//int i = 1;
//
//double perr[3] = {0,0,0};
//double derr[3] = {0,0,0};
//double ierr[3] = {0,0,0};
//
//void loop() {
//  double dt = timer - millis();
//  timer = millis();
//  double err[3] = {-IMU_getAngY(), -IMU_getAngZ(), IMU_getAngX()};
//  for (int i = 0; i < 3; i++) {
//    derr[i] = (err[i] - perr[i])/dt;
//    perr[i] = err[i];
//    ierr[i] = ierr[i] - err[i]*dt; // Why is this -??
//  }
//  dog.moveToPose(kp*perr[0] + kd*derr[0] + ki*ierr[0], kp*perr[1] + kd*derr[1] + ki*ierr[1], kp*perr[2] + kd*derr[2] + ki*ierr[2], 0, 0, 0);  
//  
//  //dog.moveToPose(-k*IMU_getAngY(), -k*IMU_getAngZ(), -k*-IMU_getAngX(), 0, 0, 0);  
//
//}
