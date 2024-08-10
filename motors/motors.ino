#include <Ultrasonic.h>

Ultrasonic ultrasonic(9, 10); // "Create an Ultrasonic object."
int distance_threshold = 50;

int In1 = 33; // set L298n PIN
int In2 = 32;
int In3 = 31;
int In4 = 30;
// Enable pins
int enA = 8;
int enB = 9;

bool box_detected = false;
void setup()
{
  Serial.begin(9600);
  pinMode(In1, OUTPUT); // Arduino control a car using output voltage
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);

  // Enable control
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
}
void loop()
{
  box_detected = detect_box();
  if (box_detected == true)
  {
    if (check_color(0))
    {
      if (measure_distance() <= distance_threshold)
      {
        stop_motors();
        pick();
      }
      else
      {
        go_forward();
      }
    }
    else
    {
      go_forward(); // Going forward and turning
      delay(1000);
      turn_right();
      delay(1000);

    }
  }
  else
  {
    go_forward(); // Going forward and turning
    delay(1000);
    if (should_stop()){
      go_backward();
      delay(1000);
      turn_right();
      delay(500);
      go_forward();
      delay(1000);
    }
  }
  if (should_stop()){
    go_backward();
    delay(1000);
    turn_right();
    delay(1000);
    go_forward();
    delay(1000);
  }
}
void stop_motors()
{ // stop motor
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
}
void forward_control(int left_speed, int right_speed)
{ // motor go front

  // Speed control
  analogWrite(enA, right_speed); // max speed
  analogWrite(enB, left_speed);  // max speed

  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
}
void backward_control(int left_speed, int right_speed)
{                                // motor go back
  analogWrite(enA, right_speed); // max speed
  analogWrite(enB, left_speed);  // max speed
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
}
void turn_right()
{
  forward_control(0, 255); // Going forward and turning
}
void turn_left()
{
  forward_control(255, 0); // Going forward and turning
}
void go_forward(){
  forward_control(255, 255);
}
void go_backward(){
  backward_control(255, 255);
}

bool check_color(int color)
{
  if (color == 0 || color == 1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

int measure_distance()
{
  // Mesaure the distance using ultrasonic sensor
  int distance = ultrasonic.read(); // Without parameters, it outputs in  return distance;
  return distance;
}

bool should_stop(){
  int distance = measure_distance();
  if (distance < distance_threshold){
    return true;
  }
  else{
    return false;
  }
}

void pick()
{
}

bool detect_box()
{
  return false;
}

