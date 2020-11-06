#include <Point.h>
#include <Rot.h>
#include <IMU.h>

Point foot_0 = Point(112.5, -60, -100); // Front Right
Point foot_1 = Point(-112.5, -60, -100); // Back Right
Point foot_2 = Point(-112.5, 60, -100); // Back Left
Point foot_3 = Point(112.5, 60, -100); // Front Left

Point foot_positions[4] = {foot_0, foot_1, foot_2, foot_3};

IMU bno_imu;
Rot imu_orientation;

void update_imu() {
  bno_imu.operate();
  imu_orientation = bno_imu.getOrientation();
}

void print_feet() {
    Serial.print("IMU: "); imu_orientation.print();
    for (int i = 0; i < 4; i++) {
        Point foot_pos_G = foot_positions[i] * imu_orientation;
        Serial.print("Leg "); Serial.print(i); Serial.print(": "); foot_pos_G.print();
    }
}

void setup() {
  Serial.begin(9600);  
  Serial.println("Starting...");
  bno_imu.setCollectionMode(IMU::CONTINUOUS);
  bno_imu.defaultStartup();


  Serial.println("Taring IMU...");
  delay(1000);
  bno_imu.tareOrientation();

  Serial.println("...Tared");
  delay(500);
  
  Serial.println("-----------------");
  Serial.println("Start Loop");

  while(1) {
      update_imu();
      print_feet();
  }
    
}

void loop() {
}
