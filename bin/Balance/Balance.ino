#include <Dog.h>

RobotDog dog;

#define start_LED_pin 22

Timer measurement_update_timer;
float measurement_interval = 3.0/1000;
Rot error_integrator = ROT_ZERO;

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
        Rot IMU_rot_velocity = dog.getBodyIMURotVelocity_fG2B();
       // Rot Kine_orientation = dog.getBodyKinematicOrientation_fF2B();
        error_integrator += IMU_orientation;
      
        //Rot desired_orientation = Kine_orientation/3-IMU_orientation;
        Rot desired_orientation = -(IMU_orientation - IMU_rot_velocity * 0.04 + error_integrator * 0.015);
        Point desired_position = Point(0, 0, dog.getStartingHeight());
        dog.moveBodyToOrientation(desired_orientation, TIME_INSTANT);
        dog.moveBodyToPositionFromCentroid(desired_position, Frame::GROUND, TIME_INSTANT);
        
        Serial.print("velocity: "); IMU_rot_velocity.print();
        Serial.print("Desired: "); desired_orientation.print();
        Serial.print("Measured: "); IMU_orientation.print();
        measurement_update_timer.reset();
    }
  
    dog.operate();
}
