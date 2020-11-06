#include <Dog.h>

RobotDog dog;

#define start_LED_pin 14

void setup() {
    Serial.begin(9600);
    pinMode(start_LED_pin, OUTPUT);
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
  if (Serial.available()) {
      float input[3];
      for (int i = 0; i < 3; i++) {
          input[i] = (Serial.parseFloat());
      }
      Serial.read();
      Rot desired_orientation = Rot(input[0],input[1],input[2]);
      desired_orientation.print();
      Serial.println();
      dog.moveBodyToOrientation(desired_orientation, TIME_INSTANT);
  }
 
  Rot IMU_orientation = dog.getBodyIMUOrientation_fG2B();
  Rot Kine_orientation = dog.getBodyKinematicOrientation_fF2B();
  Serial.print("Set: "); Kine_orientation.print();
  Serial.print("Measured: "); IMU_orientation.print();
  Serial.println("---------");

  dog.operate();
}
