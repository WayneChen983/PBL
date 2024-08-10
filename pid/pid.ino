#include <Pixy2.h>
#include <SPI.h>

Pixy2 pixy;

float x, y, z;
int t = 0;

float integrator = 0;
float kp = 5;
float ki = 0;

int In1 = 4; // set L298n PIN
int In2 = 5;
int In3 = 6;
int In4 = 7;
// Enable pins
int enA = 8;
int enB = 9;

int distance_threshold = 50;

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
  get_camera_output();

  float control_variable = calculate_pid(0.0, x);
  control_variable_to_speed(control_variable);
  forward_control(255, 255);
  

}

float calculate_pid(float setpoint, float process_variable){
  float difference = setpoint - process_variable;
  integrator += difference;
  float control_variable = kp*difference + ki*integrator;
  
  return control_variable;
}

void control_variable_to_speed(float control_variable){
  float right_speed = 255;
  float left_speed = 255;

  Serial.print("CV:"); Serial.print(control_variable);
  
  if (control_variable < -255){
    control_variable = -255;
  }
  else if (control_variable > 255){
    control_variable = 255;
  }

  if (control_variable < 0){
    right_speed += control_variable;
  }
  else if (control_variable > 0){
    left_speed -= control_variable;
  }
  else{
    right_speed = 255;
    left_speed = 255;
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
  forward_control(0, 120); // Going forward and turning
}
void turn_left()
{
  forward_control(120, 0); // Going forward and turning
}
void go_forward(){
  forward_control(120, 120);
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


    Serial.print('x'); Serial.print(x); Serial.print(",");
    Serial.print('y'); Serial.print(y); Serial.print(",");
    // Serial.print('z'); Serial.println(z);
    delay(10);

  }
}
