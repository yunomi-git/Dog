#include "Triangle.h"
#include "Point.h"

#define WIDTH2 60 // half total width
#define LENGTH2 112.5 // half total length
#define OFFSET 13.97

Point p1 = Point(LENGTH2, WIDTH2 + OFFSET, 0);
Point p2 = Point(LENGTH2, -WIDTH2 - OFFSET, 0);
Point p3 = Point(-LENGTH2, WIDTH2 + OFFSET, 0);
Point p4 = Point(-LENGTH2, -WIDTH2 - OFFSET, 0);

void setup() {
  Serial.begin(9600);
  while (!Serial);
  // put your setup code here, to run once:
  Triangle triangle = Triangle(p1, p2, p3);
  Serial.println(Triangle::getArea(triangle)); // 16643.25
  Serial.println(Triangle::getInscribedCircleArea(triangle)); // 8439.56

}

void loop() {
  // put your main code here, to run repeatedly:

}
