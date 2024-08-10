#include <Servo.h> // Load the library, it is built-in and does not require


Servo myservo; // Create a SERVO object
void setup() {
  myservo.attach(4); // Set which PIN the servo motor will be connected to
}
void loop() {
  myservo.write(0); // Rotate to 0 degrees, which is commonly referred to as
  delay(1000);
  myservo.write(90); // Rotate to 90 degrees.
  delay(1000);
  myservo.write(180); // Rotate to 180 degrees.
  delay(1000);
  myservo.write(90);
  delay(1000);
}