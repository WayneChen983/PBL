#include <Pixy2.h>
#include <SPI.h>
#include <Ultrasonic.h>

int DISTANCE_THRESHLD = 30;
//install the libary Ultrasonic by ErickSimoes.

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

int distance_threshold = 50;

//for ultrasonic sensor
Ultrasonic ultrasonic_1(4, 5);//(trig,echo)
Ultrasonic ultrasonic_2(2, 3);
Ultrasonic ultrasonic_3(12, 13);
int distance_1=0;
int distance_2=0;
int distance_3=0;
//--------------------------------------------------------------

void setup() {
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

void loop() {
  Serial.println("Main loop");
  get_camera_output();
  avoid_the_wall();

  Serial.print("x: "); Serial.print(x); Serial.print(",");
  Serial.print("y: "); Serial.print(y); Serial.print(",");
//  if (!compare_float(x,-0.92)){
  float control_variable = calculate_pid(0.0, x);
  control_variable_to_speed(control_variable);
//  }
//  else{
//    stop_motors();
//    Serial.println("motors stopped");
//  }
  // forward_control(0, 255);
  // delay(1000);

}


float calculate_pid(float setpoint, float process_variable){
  float difference = setpoint - process_variable;
  integrator += difference;
  float control_variable = kp*difference + ki*integrator;
  
  return control_variable;
}

void control_variable_to_speed(float control_variable){
  float right_speed = 150;
  float left_speed = 150;

  Serial.print("CV:"); Serial.print(control_variable);
  
  if (control_variable < -150){
    control_variable = -150;
  }
  else if (control_variable > 150){
    control_variable = 150;
  }

  if (control_variable < 0){
    right_speed += control_variable;
  }
  else if (control_variable > 0){
    left_speed -= control_variable;
  }
  else{
    right_speed = 150;
    left_speed = 150;
  }

  Serial.print("left_speed:"); Serial.print(left_speed); Serial.print(",");
  Serial.print("right_speed:"); Serial.print(right_speed); Serial.print("\n");
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
  forward_control(255, 0); // Going forward and turning
  
}
void turn_left()
{
  forward_control(0, 255);
// Going forward and turning
}
void go_forward(){
  forward_control(255, 255);
}
void go_backward(){
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

void get_camera_output(){
  int i;
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks)
  {

    float x1, y1, z1;
    //double x2, y2, z2;
    //x1 = 0.26 * ((pixy.ccc.blocks[0].m_x) );
    x1 = 0.26 * (pixy.ccc.blocks[0].m_x) - 42 ;
    y1 = 0.26 * (pixy.ccc.blocks[0].m_y) - 28;

    float h = 0.26 * (pixy.ccc.blocks[0].m_height);
    float w = 0.26 * (pixy.ccc.blocks[0].m_width);
    float temp = h * w;
    z1 = 62 - (sqrt(18 * 18 / temp)) * 58 ;

    if (t == 0) {
      x = x1;
      y = y1;
      z = z1;
      t++;
    }
    if (abs(x - x1) > 5) {
      x = x1;
    }
    if (abs(y - y1) > 5) {
      y = y1;
    }
    if (abs(z - z1) > 5) {
      z = z1;
    }

    z = 0;

    // Serial.print('z'); Serial.println(z);
   

  }
}
bool should_stop(){
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

 void avoid_the_wall() {
  
  distance_1 = ultrasonic_1.read(); 
  distance_2 = ultrasonic_2.read(); 
  distance_3 = ultrasonic_3.read(); 
  
  Serial.print("Distance in CM(1): ");
  Serial.println(distance_1);
  Serial.print("Distance in CM(2): ");
  Serial.println(distance_2);
  Serial.print("Distance in CM(3): ");
  Serial.println(distance_3);
  

  int a=close_or_not_1(distance_1);
  int b=close_or_not_1(distance_2);
  int c=close_or_not_1(distance_3);
  int choice=100*a+10*b+c;
  Serial.print("1.close?:");
  Serial.print(a);
  Serial.print("\n");
  Serial.print("2.close?:");
  Serial.print(b);
  Serial.print("\n");
  Serial.print("3.close?:");
  Serial.print(c);
  switch(choice){
  //turn right---------------------------------------------------------------
    case 100:
    {
      go_backward();
      delay(1000);
      turn_right();
      delay(1500);
      go_forward();
      delay(500);
    };
    case 010://turn_right
    {
      go_backward();
      delay(1000);
      turn_right();
      delay(1500);
      go_forward();
      delay(500);
    }
    case 110://turn_right
    {
      go_backward();
      delay(1000);
      turn_right();
      delay(1500);
      go_forward();
      delay(500);
    }
     case 111://turn_right
    {
      go_backward();
      delay(1000);
      turn_right();
      delay(1500);
      go_forward();
      delay(500);
    }
  //-----------------------------------------------------
   //turn left
    case 001://turn_right_90_degree;
    {
      go_backward();
      delay(1000);
      turn_left();
      delay(1500);
      go_forward();
      delay(500);
    }
    case 011://turn_right_90_degree;
    {
      go_backward();
      delay(1000);
      turn_left();
      delay(1500);
      go_forward();
      delay(500);
    }
    default://continue
    {
      go_forward();
      delay(500);
    };
  }

}

 int close_or_not_1(int x)//for ultrasonic
 {
    if(x<DISTANCE_THRESHLD){
      return 1;
    }
    else return 0;
  }
