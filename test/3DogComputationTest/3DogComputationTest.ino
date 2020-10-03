/* Tests for:
 *  (General): Accessors, 
 *  (Feet planted): trajectory (orientation, translation), Frame conversion, Anchor Points, centroid
 *  (Feet are lifted): stance changes, indiv foot movement, centroid/anchor point adjustment
 */

#define DEBUG
#define DEBUG_COMPUTATION

#include <Dog.h>
#include <Timer.h>

RobotDog dog;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    dog.begin();

    // Startup Values
    Serial.println("Checking Startup position...");
    Serial.print("Default height: "); Serial.println(dog.getStartingHeight());
    Serial.println("Mounting point of legs: "); 
    for (int i = 0; i < NUM_LEGS; i++) {
        Serial.print("Leg i: "); dog.getLeg(i)->getPosition_oBfB().print(); Serial.println();
    }

}

void loop() {
  // put your main code here, to run repeatedly:

}
