#include <Dog.h>

RobotDog dog;

#define start_LED_pin 14


float rps = 0.5; 
float magnitude = 10;
float desired_orientation;
Timer control_timer;
Timer print_timer;

void setup() {
    Serial.begin(9600);
    pinMode(start_LED_pin, OUTPUT);
    digitalWrite(start_LED_pin, HIGH);
    while (!Serial) {}
    Serial.println("Beginning...");
    dog.begin();
    dog.resetDefaultStance();
    delay(2000);
    Serial.println("Taring IMU...");
    dog.tareIMU();
    Serial.println("Tare done...Actually Starting");
    digitalWrite(start_LED_pin, LOW);

    control_timer.reset();
    print_timer.reset(0.005);
}



void loop() {
  float desired_magnitude = magnitude * sin(2 * M_PI * rps * control_timer.dt());
  Rot desired_orientation = Rot(0, desired_magnitude, 0);
  dog.moveBodyToOrientationInTime(desired_orientation, TIME_INSTANT);
 
  Rot IMU_orientation = dog.getBodyIMUOrientation_fG2B();
//  Rot Kine_orientation = dog.getBodyKinematicOrientation_fF2B();
dog.operate();

  float desired_magnitude_delayed = magnitude * sin(2 * M_PI * rps * control_timer.dt()); // the desired signal after dog has ffinished computing
  if (print_timer.timeOut()) {
      Serial.print("Requested: "); Serial.print(desired_orientation.y);
      Serial.print(" Requestdelayed: "); Serial.print(desired_magnitude_delayed);
      //Serial.print(" Set: "); Serial.print(Kine_orientation.y);
      Serial.print(" Measured: "); Serial.print(IMU_orientation.y);
      Serial.println();
      print_timer.reset();
  }


  
}
