#include <Dog.h>

RobotDog dog;

#define start_LED_pin 14

void setup() {
    Serial.begin(9600);
    digitalWrite(start_LED_pin, HIGH);
    //while (!Serial) {}
    Serial.println("Beginning...");
    dog.begin();
    dog.resetDefaultStance();
    Serial.println("Taring IMU...");
    dog.tareIMU();
    Serial.println("Tare done...Actually Starting");
    digitalWrite(start_LED_pin, LOW);
}

void loop() {
  Rot IMU_orientation = dog.getBodyIMUOrientation_fG2B();
  Rot Kine_orientation = dog.getBodyKinematicOrientation_fF2B();

  //Rot desired_orientation = Kine_orientation/3-IMU_orientation;
  Rot desired_orientation = -IMU_orientation;
  dog.moveBodyToOrientation(desired_orientation, TIME_INSTANT);
  Serial.print("Desired: "); desired_orientation.print();
  Serial.print("Measured: "); IMU_orientation.print();
  //  Serial.print(-dog.getBodyIMUOrientation_fG2B().z);
 // Serial.println();
  dog.operate();
//  Serial.print(dog.getBodyKinematicOrientation_fF2B().y);
//  Serial.print(" ");

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
