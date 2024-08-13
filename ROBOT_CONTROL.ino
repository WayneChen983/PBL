#include <Pixy2.h>
#include <SPI.h>
#define MAX_SPEED 255
#define CONSTANT_SPEED 120
#define DISTANT_THRESHOLD 50
#define RED 1
#define GREEN 2
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
float kp = 6; // Porpotional parameter
float ki = 0; // Integral parameter
float center_coordinate = 0.0;

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

// Enable pins
int enA = 8;
int enB = 9;

//Declaring functions
void get_camera_output(); // detect color and coordinate of the objects
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
  // Act based on the highest priority color detected
    if (color == RED) {  // Red
      Serial.println("Red cube detected, picking up...");
      calculate_pid(center_coordinate, current_X);
      control_variable_to_speed(control_variable);
      // arm_down();
      // grabber_on();
      // arm_up();
      // grabber_off();
      // pick();
    } 
    else if (color == GREEN) {  // Green
      Serial.println("Green cube detected, picking up...");
      calculate_pid(center_coordinate, current_X);
      control_variable_to_speed(control_variable);
      // arm_down();
      // grabber_on();
      // arm_up();
      // grabber_off();
      // pick();
    }
    else {
      go_forward(constant_speed);
      delay(1000);
      rotate(130, true);
      //wall_detection();
      get_camera_output();
    } 
}

void get_camera_output(){
  int i;
  pixy.ccc.getBlocks(); //captures how many blocks in one recent frame
   
 if (pixy.ccc.numBlocks) {
    highest_priority = 2;  //priority function, 1 is the highest 
    color = -1;

    // Loop through all detected blocks
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      detected_color = pixy.ccc.blocks[i].m_signature;

      // Update if a higher-priority color is detected
      if (detected_color == 1 && highest_priority > 1) {
        color = 1;  // Red has the highest priority
        highest_priority = 1;
      } 
      else if (detected_color == 2 && highest_priority >= 2) {
        color = 2;  // Green has middle priority
        highest_priority = 2;
      } 
    }
  }
  //double x2, y2, z2; 
    //x1 = 0.26 * ((pixy.ccc.blocks[0].m_x) );
    float detected_X = 0.26 * (pixy.ccc.blocks[0].m_x) - 42;
    float detected_Y = 0.26 * (pixy.ccc.blocks[0].m_y) - 28;

    float detected_height = 0.26 * (pixy.ccc.blocks[0].m_height);
    float detected_width = 0.26 * (pixy.ccc.blocks[0].m_width);
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

    Serial.print('x'); Serial.print(current_X); Serial.print(",");
    Serial.print('y'); Serial.print(current_Y); Serial.print(",");
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
