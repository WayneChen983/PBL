#include <Pixy2.h>
#include <SPI.h>
#define MAX_SPEED 220
#define CONSTANT_SPEED 120
#define DISTANT_THRESHOLD 50
#define WHITE -1
#define RED 1
#define GREEN 2
#define FIRST_PRIORITY 1
#define SECOND_PRIORITY 2
Pixy2 pixy; // Declare pixy camera object

// Detecting objects variables 
float current_X, current_Y, current_Z = 0; //coordinates of detected objects
int color;
int detected_color;
int highest_priority;
int time = 0; 

// PID controller variables 
float control_variable;
float difference;
float integrator = 0;
float kp = 5; // Porpotional parameter10
float ki = 0; // Integral parameter
float center_coordinate = 0.0;
int chosen_block = 0;
int distance_threshold = DISTANT_THRESHOLD; // The distance thresh hold to capture the block or distance of the walls

//Motor speed variables
int constant_speed = CONSTANT_SPEED;
int control_right_speed = MAX_SPEED;
int control_left_speed = MAX_SPEED;

//Set LN298 pins
int In1 = 29; // Left upper wheels 
int In2 = 28; // Right upper wheels
int In3 = 27; // Left back wheels 
int In4 = 26; // Right back wheels
unsigned long startTime = millis();
// Enable pins
int enA = 8;
int enB = 9;
int i = 0;
//Declaring functions
void get_camera_output(); // detect color and coordinate of the objects
void update_camera_output(); // continuously update the cube's coordinate for the PID controller
void get_block_priority();
void get_block_coordinate();
void calculate_pid(float, float); // PID controller algorithm, using the P & I 
void control_variable_to_speed(float); // convert control variable to speed of the motors
void pid_speed_control_motor();
void go_forward(int); 
void go_backward(int); 
void turn_right(int);
void turn_left(int);
void rotate(int, bool);
void stop();
void forward_control(int, int); //spin the wheels to move forward, control the speed by passing int values
void backward_control(int, int); //spin the wheels to move backward, control the speed by passing int values

void setup() {
  Serial.begin(9600);

  pinMode(In1, OUTPUT); // Arduino control a car using output voltage
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  //pinMode(grabber,OUTPUT);


  // Enable control
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pixy.init();
}

void loop() {
    get_camera_output();
  // Act based on the highest priority color detected
    if (color == RED) {  // Red
        get_camera_output();
        //Serial.println("Red cube detected, picking up...");
        calculate_pid(center_coordinate, current_X);
        control_variable_to_speed(control_variable);
    } 
    else if (color == GREEN) {  // Green
      get_camera_output();
      Serial.println("Green cube detected, picking up...");
      calculate_pid(center_coordinate, current_X);
      control_variable_to_speed(control_variable);
    }
    else {
      go_forward(constant_speed);
      //wall_detection();
      get_camera_output();
    } 
}

void get_camera_output(){
  //int i;
  pixy.ccc.getBlocks(); //captures how many blocks in one recent frame
  get_block_priority();
  get_block_coordinate();
}

void get_block_priority() {
  if (pixy.ccc.numBlocks) {
    highest_priority = SECOND_PRIORITY;  //priority function, 1 is the highest 
    color = WHITE;

    // Loop through all detected blocks
    for (int block_index = 0; block_index < pixy.ccc.numBlocks; ++block_index) {
      detected_color = pixy.ccc.blocks[i].m_signature;

      // Update if a higher-priority color is detected
      if (detected_color == RED && highest_priority > FIRST_PRIORITY) {
        color = RED;  // Red has the highest priority
        highest_priority = FIRST_PRIORITY;
        chosen_block = block_index;
      } 
      else if (detected_color == GREEN && highest_priority >= SECOND_PRIORITY) {
        color = GREEN;  // Green has middle priority
        highest_priority = SECOND_PRIORITY;
        chosen_block = block_index;
      } 
    }
  }
}

void get_block_coordinate() {
  float detected_X = 0.26 * (pixy.ccc.blocks[chosen_block].m_x) - 42;
  float detected_Y = 0.26 * (pixy.ccc.blocks[chosen_block].m_y) - 28;

  float detected_height = 0.26 * (pixy.ccc.blocks[chosen_block].m_height);
  float detected_width = 0.26 * (pixy.ccc.blocks[chosen_block].m_width);
    //float temp = h * w;
    //float z1 = 62 - (sqrt(18 * 18 / temp)) * 58 ;

  if (time == 0) { //inital time
    current_X = detected_X;
    current_Y = detected_Y;
    time++;
  }
  if (abs(current_X - detected_X) > 5) {
    current_X = detected_X;
  }
  if (abs(current_Y - detected_Y) > 5) {
    current_Y = detected_Y;
  }

  Serial.print('x: '); Serial.print(current_X); Serial.print(",");
  Serial.print('y: '); Serial.print(current_Y); Serial.print(",");
    // Serial.print('z'); Serial.println(z);
  delay(10);
}

void calculate_pid(float setpoint, float process_variable){
  difference = setpoint - process_variable;
  integrator += difference;
  control_variable = kp*difference + ki*integrator;
}

void control_variable_to_speed(float control_variable){
  Serial.print("CV:"); Serial.print(control_variable);
  control_right_speed = MAX_SPEED;
  control_left_speed = MAX_SPEED;

  if (control_variable < -MAX_SPEED){
    control_variable = -MAX_SPEED;
  }
  else if (control_variable > MAX_SPEED){
    control_variable = MAX_SPEED;
  }

  if (control_variable < 0){
    control_right_speed += control_variable;
  }
  else if (control_variable > 0){
    control_left_speed -= control_variable;
  }
  else{
    control_right_speed = MAX_SPEED;
    control_left_speed = MAX_SPEED;
  }

  if (control_left_speed < 0){
    control_left_speed = 0;
  }
  if (control_right_speed < 0){
    control_right_speed = 0;
  }

  Serial.print("left_speed:"); Serial.print(control_left_speed); Serial.print(",");
  Serial.print("right_speed:"); Serial.print(control_right_speed); Serial.print("\n");
  forward_control(control_left_speed, control_right_speed);
}

void go_forward(int constant_speed){
  forward_control(constant_speed, constant_speed); // Going forward
}

void go_backward(int constant_speed){
  backward_control(constant_speed, constant_speed); // Going backward
}

void turn_right(int constant_speed)
{
  forward_control(0, constant_speed); // Going forward and turning right
}

void turn_left(int constant_speed)
{
  forward_control(constant_speed, 0); // Going forward and turning left
}

void rotate(int speed, bool clockwise) {
  if (clockwise) {
    analogWrite(enA, speed/2); // max speed
    analogWrite(enB, speed/2);  // max speed

    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);
  } else {
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
    digitalWrite(In3, LOW);
    digitalWrite(In4, HIGH);
  }
}

void stop()
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

// bool check_color(int color)
// {
//   if (color == 1 || color == 2)
//   {
//     return true;
//   }
//   else
//   {
//     return false;
//   }
// }

//   void arm_down(){
//     analogWrite(arm,100);
//   }
//   void arm_up(){
//     analogWrite(arm,-100);
//   }
  
//   void grabber_on(){
//     analogWrite(grabber,100);
//   }
//   void grabber_off(){
//     analogWrite(grabber,-100);
//   }

//   void pick(){
//     if(check_color(color))
//     {
//       arm_down();
//       grabber_on();
//       arm_up();
//       grabber_off();
//     }
//   }
