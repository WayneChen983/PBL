int In1 = 4; // set L298n PIN
int In2 = 5;
int In3 = 6;
int In4 = 7;
// Enable pins
int enA = 8;
int enB = 9;

int distance_threshold = 10; // Change the value here
bool box_detected = false;
void setup()
{
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
        mstop();
        pick();
      }
      else
      {
        forward_control(255, 255);
      }
    }
    else
    {
      forward_control(100, 255); // Going forward and turning
    }
  }
  else
  {
    forward_control(100, 255); // Going forward and turning
  }
}
void mstop()
{ // stop motor
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
}
void forward_control(int right_speed, int left_speed)
{ // motor go front

  // Speed control
  analogWrite(enA, right_speed); // max speed
  analogWrite(enB, left_speed);  // max speed

  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
}
void backward_control(int right_speed, int left_speed)
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
  int distance = 10;
  return distance;
}

void pick()
{
}

bool detect_box()
{
  return true;
}
