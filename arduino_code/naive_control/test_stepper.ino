#include <Streaming.h>

int serialAv;

//Stepper motor pins
const int dirPin = 25;
const int steppin = 26;
const int mOPin = 22;
const int mlPin = 23;
const int m2Pin = 24;

// uint8_t stepSize AndDir = B00000000; //00000001 is m0, 00000010 is ml, 00000100 is m2, 00001000 is dir.
unsigned int stepPulseWidth = 1000; //in milliseconds

void setup() {
  DDRD = B11111000; //direction register for Port A (some range of arduino digital pins); 0 = input, 1 = output... faster
  //sets PAO PA4 (step, dir, m0, ml, m2) to Outputs
  DDRA = B11111111;
  PORTA = B00000000;

  //&= is for setting 0
  // is for setting 1

  // digitalWrite(26, LOW);
  
  Serial.begin(250000);
  Serial.println("hi");
  
  while ((serialAv = Serial.available()) == 0) { }
}

void loop() {
  
  // if (Serial.available() != serialAv) {
  //   return;
  // }

  uint8_t encodval = 11;

  //STEPPER MOTOR ******
  PORTA &= B11111000;
  // switch (encodval) { //SETS STEP SIZE
  //   case 6: //NOSTEP THOUGH
  //     return; // and exits the switch
  //     break;
  //   case 5:
  //   case 7:
  //     PORTA |= B00000101;
  //     break;
  //   case 4:
  //   case 8:
  //     PORTA |= B00000100;
  //     break;
  //   case 3:
  //   case 9:
  //     PORTA |= B00000011;
  //     break;
  //   case 2:
  //   case 10:
  //     PORTA |= B00000010;
  //     break;
  //   case 1:
  //   case 11: 
  //     PORTA |= B00000001;
  //     break;
  //   case 0:
  //   case 12: 
  //     PORTA |= B00000001;
  //     break;
  // }
  PORTA |= B00000001;

  //SETS DIRECTION
  if (encodval > 6){
    PORTA |= B00001000;
  }else{
    PORTA &= B11110111;
  }

  // Set the step pin high
  PORTA |= B00010000;
  delayMicroseconds(stepPulseWidth);
  PORTA &= B11101111;
  delayMicroseconds(stepPulseWidth);
}