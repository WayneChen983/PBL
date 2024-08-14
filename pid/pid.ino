#include <Pixy2.h>
#include <SPI.h>
#include <NewPing.h>

#define SONAR_NUM 4 // Number of sensors.
#define MAX_DISTANCE 350

#define AVOIDANCE_TURNING_DELAY 800
#define DRIVE_DELAY 500

// Right is 2, center is 1, left is 0

Pixy2 pixy;

float x, y, z;
int t = 0;

float integrator = 0;
float kp = 6;
float ki = 0;

int In1 = 29; // set L298n PIN
int In2 = 28;
int In3 = 27;
int In4 = 26;
// Enable pins
int enA = 8;
int enB = 9;

int distance_threshold = 60;

int distances[3] = {0, 0, 0}; // Initialize ultrasonic sensors distances

// Sensor object array.
NewPing sonar[SONAR_NUM] = {
    NewPing(4, 5, MAX_DISTANCE), //(trig,echo)
    NewPing(2, 3, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
    NewPing(12, 13, MAX_DISTANCE),
    NewPing(10, 11, MAX_DISTANCE)};

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

  pixy.init();
}

void loop()
{
  get_camera_output();

  Serial.print("x: ");
  Serial.print(x);
  Serial.print(",");
  Serial.print("y: ");
  Serial.print(y);
  Serial.print(",");
  if (!compare_float(x, -0.92))
  {
    float control_variable = calculate_pid(0.0, x);
    control_variable_to_speed(control_variable);
  }
  else
  {
    stop_motors();
    Serial.println("motors stopped");
  }

  obstacle_avoidance();
}

float calculate_pid(float setpoint, float process_variable)
{
  float difference = setpoint - process_variable;
  integrator += difference;
  float control_variable = kp * difference + ki * integrator;

  return control_variable;
}

void control_variable_to_speed(float control_variable)
{
  float right_speed = 150;
  float left_speed = 150;

  Serial.print("CV:");
  Serial.print(control_variable);

  if (control_variable < -150)
  {
    control_variable = -150;
  }
  else if (control_variable > 150)
  {
    control_variable = 150;
  }

  if (control_variable < 0)
  {
    right_speed += control_variable;
  }
  else if (control_variable > 0)
  {
    left_speed -= control_variable;
  }
  else
  {
    right_speed = 150;
    left_speed = 150;
  }

  Serial.print("left_speed:");
  Serial.print(left_speed);
  Serial.print(",");
  Serial.print("right_speed:");
  Serial.print(right_speed);
  Serial.print("\n");
  forward_control(left_speed, right_speed);
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

  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
}
void backward_control(int left_speed, int right_speed)
{                                // motor go back
  analogWrite(enA, right_speed); // max speed
  analogWrite(enB, left_speed);  // max speed
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
}
void turn_right()
{
  forward_control(0, 255); // Going forward and turning
}
void turn_left()
{
  forward_control(255, 0); // Going forward and turning
}
void go_forward()
{
  forward_control(255, 255);
}
void go_backward()
{
  backward_control(120, 120);
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

void pick()
{
}

bool detect_box()
{
  return false;
}

void get_camera_output()
{
  int i;
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks)
  {

    float x1, y1, z1;
    // double x2, y2, z2;
    x1 = 0.26 * (pixy.ccc.blocks[0].m_x) - 42;
    y1 = 0.26 * (pixy.ccc.blocks[0].m_y) - 28;

    float h = 0.26 * (pixy.ccc.blocks[0].m_height);
    float w = 0.26 * (pixy.ccc.blocks[0].m_width);
    float temp = h * w;
    z1 = 62 - (sqrt(18 * 18 / temp)) * 58;

    if (t == 0)
    {
      x = x1;
      y = y1;
      z = z1;
      t++;
    }
    if (abs(x - x1) > 5)
    {
      x = x1;
    }
    if (abs(y - y1) > 5)
    {
      y = y1;
    }
    if (abs(z - z1) > 5)
    {
      z = z1;
    }

    z = 0;

    // Serial.print('z'); Serial.println(z);
    delay(10);
  }
}
bool should_stop()
{
  return false;
}

int compare_float(float f1, float f2)
{
  float precision = 0.00001;
  if (((f1 - precision) < f2) &&
      ((f1 + precision) > f2))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void obstacle_avoidance()
{
  read_ultrasonic_values();
  int choice = calculate_choice();
  avoid_wall(choice);
}

int calculate_choice(){
  int choice = 0;
  for (int i = 0; i < SONAR_NUM; i++)
  {
    int result = check_wall_collision(distances[i]);
    choice += result * round(pow(10, i));
    Serial.print(i);
    Serial.print("=");
    Serial.print(result);
    Serial.print(" ");
  }
  // choice += 1;
  Serial.print("Choice: ");
  Serial.println(choice);
  return choice;
}

void avoid_wall(int choice)
{
  if (choice == 1) // turn left 20 degree;
  {
    go_backward();
    delay(DRIVE_DELAY);
    turn_right();
    delay(AVOIDANCE_TURNING_DELAY);
    Serial.println("Case 001");
  }
  else if (choice == 10) // turn_right_20_degree;
  {
    go_backward();
    delay(DRIVE_DELAY);
    turn_left();
    delay(AVOIDANCE_TURNING_DELAY);
    Serial.println("Case 010");
  }
  // case 011:
  else if (choice == 100) // turn_right_90_degree;
  {
    go_backward();
    delay(DRIVE_DELAY);
    turn_left();
    delay(AVOIDANCE_TURNING_DELAY);
    Serial.println("Case 100");
  }
  else if (choice == 101) // turn_right_90_degree;
  {
    go_backward();
    delay(DRIVE_DELAY);
    turn_right();
    delay(AVOIDANCE_TURNING_DELAY);
    Serial.println("Case 101");
  }
  else if (choice == 110) // turn_left_90_degree;
  {
    go_backward();
    delay(DRIVE_DELAY);
    turn_right();
    delay(AVOIDANCE_TURNING_DELAY);
    Serial.println("Case 110");
  }
  else if (choice == 111) // turn_left_90_degree;
  {
    go_backward();
    delay(DRIVE_DELAY);
    turn_left();
    delay(AVOIDANCE_TURNING_DELAY);
    Serial.println("Case 111");
  }
  else // continue
  {
    go_forward();
    delay(DRIVE_DELAY);
    Serial.println("Else");
  }
}

int check_wall_collision(int x) // for ultrasonic
{
  if (x < distance_threshold && x != 0)
  {
    return 1;
  }
  else
    return 0;
}

void read_ultrasonic_values()
{
  for (uint8_t i = 0; i < SONAR_NUM; i++)
  {            // Loop through each sensor and display results.
    delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    Serial.print(i);
    Serial.print("=");
    Serial.print(sonar[i].ping_cm());
    Serial.print("cm ");
    distances[i] = sonar[i].ping_cm();
  }
  Serial.println();
}