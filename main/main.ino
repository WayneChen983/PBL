#include <Pixy2.h>
#include <SPI.h>
#include <NewPing.h>
#include <Servo.h>   //it's in the ide already.

#define SONAR_NUM 3 // Number of sensors.
#define MAX_DISTANCE 350

#define AVOIDANCE_TURNING_DELAY 800
#define DRIVE_DELAY 500
#define MAX_SPEED 200

#define AVOIDANCE_TURNING_DELAY 800
#define BACKWARD_DRIVE_DELAY 800

// Right is 2, center is 1, left is 0

Pixy2 pixy;

Servo myservo_grabber;  // create a servo object for gripper
Servo myservo_arm;  // create a servo object for robotic arm

float x, y, z;
float h = 0.0;
float w = 0.0;
int t = 0;
int detection_sensitivity = 3; // How many times cube needs to be detected for arm to move

float integrator = 0;
float kp = 10;
float ki = 0;

int In1 = 25; // set L298n PIN
int In2 = 24;
int In3 = 23;
int In4 = 22;
// Enable pins
int enA = 8;
int enB = 9;

int search_iterator = 0;

int cube_distance_threshold = 30; // Distance threshold to pick up the cube in cm

bool collecting = false;
int collecting_iterator = 0;
int collecting_iterator_threshold = 5;

int pick_up_iterator = 0;
int pick_up_threshold = 5;

int distance_threshold = 50;
float cube_size_threshold = 20.;

int distances[3] = {0, 0, 0}; // Initialize ultrasonic sensors distances

// Sensor object array.
NewPing sonar[SONAR_NUM] = {
    NewPing(4, 5, MAX_DISTANCE), //(trig,echo) left one
    NewPing(12, 13, MAX_DISTANCE), // right
    NewPing(10, 11, MAX_DISTANCE)}; // Bottom one

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

  myservo_grabber.attach(3);  //set its pin
  myservo_arm.attach(2);       //set its pin

  myservo_grabber.write(110);    // initial the grabber be open
  myservo_arm.write(0);        // initial the arm rasing
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
  pixy.ccc.getBlocks();
  Serial.print("Collecting iterator"); 
  Serial.println(collecting_iterator);
  Serial.println("H*W="); Serial.print(h*w);
  if (collecting){
    pick_or_not();
  }
  if (pixy.ccc.numBlocks)
  {
    float control_variable = calculate_pid(0.0, -x);
    control_variable_to_speed(control_variable);
    myservo_grabber.write(180);
    delay(50);
    stop_motors();
    collecting = true;
  }
  else
  {
    // stop_motors();
    Serial.println("motors stopped");
    obstacle_avoidance();
    myservo_grabber.write(110);
    collecting = false;
    Serial.println("Not collecting");
  }
    // search();

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
  float right_speed = MAX_SPEED;
  float left_speed = MAX_SPEED;

  Serial.print("CV:");
  Serial.print(control_variable);

  if (control_variable < -MAX_SPEED)
  {
    control_variable = -MAX_SPEED;
  }
  else if (control_variable > MAX_SPEED)
  {
    control_variable = MAX_SPEED;
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
    right_speed = MAX_SPEED;
    left_speed = MAX_SPEED;
  }

  Serial.print("left_speed:");
  Serial.print(left_speed);
  Serial.print(",");
  Serial.print("right_speed:");
  Serial.print(right_speed);
  Serial.print(",");
  Serial.print(distances[2]); Serial.print("cm");
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
  forward_control(0, MAX_SPEED); // Going forward and turning
}
void turn_left()
{
  forward_control(MAX_SPEED, 0); // Going forward and turning
}
void go_forward()
{
  forward_control(MAX_SPEED, MAX_SPEED);
}
void go_backward()
{
  backward_control(MAX_SPEED, MAX_SPEED);
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

    h = 0.26 * (pixy.ccc.blocks[0].m_height);
    w = 0.26 * (pixy.ccc.blocks[0].m_width);
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
    if (result){
      choice = round(pow(2, i));
    }
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
  if (choice == 1) 
  {
    go_backward();
    delay(DRIVE_DELAY);
    turn_left();
    delay(AVOIDANCE_TURNING_DELAY);
    Serial.println("Case 1");
  }
  else if (choice == 2) 
  {
    go_backward();
    delay(DRIVE_DELAY);
    turn_right();
    delay(AVOIDANCE_TURNING_DELAY);
    Serial.println("Case 2");
  }
  // else if (choice == 4) 
  // {
    // go_backward();
    // delay(DRIVE_DELAY);
    // turn_left();
    // delay(AVOIDANCE_TURNING_DELAY);
    // Serial.println("Case 4");
  // }
  else{
    go_forward();
    delay(50);
    if (search_iterator > 20){
      search_iterator = 0;
    }
    else if(search_iterator > 10){
      turn_right();
      search_iterator++;
    }
    else{
      search_iterator++;
    }
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

void search(){
  forward_control(140, 140);
  delay(3000);
  if (search_iterator < 3){
    turn_right();
    search_iterator++;
  }
  else{
    turn_left();
    search_iterator = 0;
  }
  delay(500);
}

void pick_or_not(){
  // read_ultrasonic_values();
  if (!pixy.ccc.numBlocks){
    pick_the_cube();
  }
}

bool check_cube_distance(){
  if (h*w < cube_size_threshold){
    t += 1;
    return true;
  }
  else{
    t = 0;
    return false;
  }
}

void pick_the_cube(){
  Serial.print("Picking up");
  go_forward();
  delay(300);
  stop_motors();
  myservo_grabber.write(110);      //2.grabber clamps
  delay(2000);                   //the arm is waiting the grabber clamping  for 2s
  // move_arm(180);  //3.the arm moves up
  go_backward();
  delay(500);
  stop_motors();
  myservo_arm.write(180);
  delay(2000); 
  myservo_grabber.write(180);    //4.grabber opens
  delay(1000);
  myservo_arm.write(0); 
}

void move_arm(int angle_setpoint){
  for (int i=0; i<angle_setpoint; i++){
    myservo_arm.write(i);  
    delay(2);
  }
}