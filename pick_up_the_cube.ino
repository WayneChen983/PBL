#include <Servo.h>   //it's in the ide already.
#include <Ultrasonic.h>

int DISTANCE_THRESHLD_CUBE = 20;
Ultrasonic ultrasonic_4(10,11);//(trig,echo)

int t=0;//avoid the sensor too sensetive
Servo myservo_grabber;  // create a servo object
Servo myservo_arm;  // create a servo object

void setup() {
  
  Serial.begin(9600);
  myservo_grabber.attach(3);  //set its pin
  myservo_arm.attach(2);       //set its pin
  
  myservo_grabber.write(100);    // initial the grabber be open
  myservo_arm.write(180);        // initial the arm rasing
}
//range of the grabber is 30~100
//range of the arm 0~180
void loop() {   
  int distance_cube = ultrasonic_4.read(); 
  Serial.println(distance_cube);
  if(cube_close_or_not(distance_cube)){
    pick();
  }
}

bool cube_close_or_not(int x){
  
  if(x<DISTANCE_THRESHLD_CUBE)t++;
  if(t>100){
    return true;
  }
  else return false;
}

void pick(){
myservo_arm.write(0);        //1.the arm moves down
delay(2000);                   //the grabber is waiting the arm moving down for 2s
myservo_grabber.write(25);      //2.grabber clamps
delay(2000);                   //the arm is waiting the grabber clamping  for 2s
myservo_arm.write(180);          //3.the arm moves up
delay(2000);                   //the grabber is waiting the arm moving down for 2s
myservo_grabber.write(100);    //4.grabber opens
t=0;
}
