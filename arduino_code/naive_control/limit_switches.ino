#include <Streaming.h>

int serialAv;

//limit switch pins
const int leftLimitPin = 18;
const int rightLimitPin = 17;

bool atLimit = false;

void setup() {
  DDRD & B11111000; // direction register for Port A (some range of arduino digital pins); 0 = input, 1 = output... faster
  DDRA = B11111111; // sets PAO PA4 (step, dir, m0, ml, m2) to Outputs

  pinMode(leftLimitPin, INPUT);
  pinMode(rightLimitPin, INPUT);
  
  digitalWrite(leftLimitPin, HIGH);
  digitalWrite(rightLimitPin, HIGH);

  Serial.begin(250000);
  Serial.println("hi");
}

void loop() {
  
  if (Serial.available() != serialAv) {
    return;
  }
  
  //LIMIT SWITCHES
  if (atLimit) {
    return;
  }
}

void isrLimit() {
  atLimit = true;
}