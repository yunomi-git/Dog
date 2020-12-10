#include "PixyTracker.h"

// a good area is 4000

Pixy2SPI_SS pixy;
ObjectPositionTracker tracker(&pixy);

#define DEBUG

void setup() {
  Serial.begin(9600);
  pixy.init();
}

void loop() {
  tracker.operate();
  if (tracker.objectWasDetected()) {
      tracker.printPixyData();
//  tracker.printPosition();
  } else {
      Serial.println(".");
  }

}
