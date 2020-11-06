#include <Dog.h>

RobotDog dog;

#define start_LED_pin 14


float rps = 1; 
float magnitude = 5;
float desired_orientation;
Timer control_timer;
Timer calculation_timer;

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
    calculation_timer.usePrecision();
    calculation_timer.reset();
}



void loop() {
  float desired_magnitude = magnitude * sin(2 * M_PI * rps * control_timer.dt());
  Rot desired_orientation = Rot(0, desired_magnitude, 0);
  calculation_timer.reset();
  dog.moveBodyToOrientation(desired_orientation, TIME_INSTANT);


  
  dog.operate();
  Serial.println(calculation_timer.dt() * 1000);
}
