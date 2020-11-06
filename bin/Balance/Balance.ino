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
    dog.resetDefaultStance();
    digitalWrite(start_LED_pin, LOW);
    Serial.println("Taring IMU...");
    delay(1000);
    digitalWrite(start_LED_pin, HIGH);
    dog.tareIMU();
    Serial.println("Tare done...Actually Starting");
    digitalWrite(start_LED_pin, LOW);

    measurement_update_timer.usePrecision();
    measurement_update_timer.reset(measurement_interval);
}

void loop() {
    if (measurement_update_timer.timeOut()) {
        Rot IMU_orientation = dog.getBodyIMUOrientation_fG2B();
       // Rot Kine_orientation = dog.getBodyKinematicOrientation_fF2B();
      
        //Rot desired_orientation = Kine_orientation/3-IMU_orientation;
        Rot desired_orientation = -IMU_orientation;
        Point desired_position = Point(0, 0, dog.getStartingHeight());
        dog.moveToOrientation(desired_orientation, TIME_INSTANT);
        dog.moveToPosition(desired_position, Frame::GROUND, TIME_INSTANT);
        Serial.print("Desired: "); desired_orientation.print();
        Serial.print("Measured: "); IMU_orientation.print();
        measurement_update_timer.reset();
    }
  
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
