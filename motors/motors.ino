int In1 = 4; //set L298n PIN
int In2 = 5;
int In3 = 6;
int In4 = 7;
// Enable pins
int enA = 8;
int enB = 9;

void setup() {
  pinMode(In1, OUTPUT); // Arduino control a car using output voltage
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);

  // Enable control
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
}
void loop() {
  mfront();
  delay(2000);
  mstop();
  delay(500);
  mback();
  delay(2000);
  mstop();
  delay(500);
  }
void mstop() { // stop motor
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
}
void mfront() { // motor go front

  // Speed control
  analogWrite(enA, 255); // max speed
  analogWrite(enB, 255); // max speed
  
  
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
}
void mback() { // motor go back
  analogWrite(enA, 255); // max speed
  analogWrite(enB, 255); // max speed
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
}
