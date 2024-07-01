#include <Streaming.h>

int serialAv;

//encoder isr pins
const int ISR_FOR_ENCODER_B = 20;
const int A = 21;
const int Z = 19;

//limit switch pins

const int leftLimitPin = 18;
const int rightLimitPin = 19;

//Stepper motor pins
const int dirPin = 25;
const int steppin = 26;
const int mOPin = 22;
const int mlPin = 23;
const int m2Pin = 24;

// uint8_t stepSize AndDir = B00000000; //00000001 is m0, 00000010 is ml, 00000100 is m2, 00001000 is dir.
unsigned long stepPulseWidth = 2; //in milliseconds

volatile int16_t count = -2000;

bool atLimit = false;

int lastCount = -2000;

void setup() {
  // put your setup code here, to run once:
  
  //pinMode (ISR_FOR_ENCODER_B, INPUT);
  //pinMode (A, INPUT);
  //pinMode (Z, INPUT);
  DDRD & B11111000; //direction register for Port A (some range of arduino digital pins); 0 = input, 1 = output... faster
  
  //sets PAO PA4 (step, dir, m0, ml, m2) to Outputs
  DDRA = B11111111;

  pinMode(leftLimitPin, INPUT);
  pinMode(rightLimitPin, INPUT);
  
  digitalWrite(leftLimitPin, HIGH);
  digitalWrite(rightLimitPin, HIGH);

  digitalWrite(A, HIGH);
  digitalWrite(ISR_FOR_ENCODER_B, HIGH);
  digitalWrite(Z, HIGH);
  //PORTD |=B00000111; //digitalWrites to high but faster
  
  PORTA & B00000000;

  //&= is for setting 0
  // is for setting 1

  digitalWrite(26, LOW);

  attachInterrupt(digitalPinToInterrupt(ISR_FOR_ENCODER_B), isrB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ISR_FOR_ENCODER B), isrA, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(leftLimitPin), isrLimit, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightLimitPin), isrLimit, CHANGE);
  
  
  Serial.begin(250000);
  Serial.println("hi");
  
  while ((serialAv = Serial.available()) == 0) { }
}

void loop() {
  
  if (Serial.available() != serialAv) {
    return;
  }
  
  // put your main code here, to run repeatedly:
  //LIMIT SWITCHES + ENCODERVAL SET ******
  if (atLimit) {
    return;
  }

  cli(); //notInterrupts()
  int16_t rncount = count;
  sei (); //interrupts()
  
  //STEPPER MOTOR ******
  
  /*uint8_t encodVal = 4;
  if (rncount >= 3){
    encodVal = 7;
  }else if (rncount <= -4){
    encodVal = 0;
  }else{
    encodVal + rncount;
  }*/
  uint8_t encodVal = 6;
  if (rncount >= 6) {
    encodVal = 12;
  }else if (rncount <= -6) {
    encodVal = 0;
  }else{
    encodVal += rncount;  
  }
  PORTA & B11111000;
  switch (encodval) { //SETS STEP SIZE
    case 6 //NOSTEP THOUGH
      return; // and exits the switch
      break;
    case 5:
    case 7:
      PORTA | B00000101;
      break;
    case 4:
    case 8:
      PORTA | B00000100;
      break;
    case 3:
    case 9:
      PORTA | B00000011;
      break;
    case 2:
    case 10:
      PORTA | B00000010;
      break;
    case 1:
    case 11: 
      PORTA | B00000001;
      //PORTA |= B00000001;
      break;
    case 0:
    case 12: 
      PORTA | B00000001;
      //PORTA |=B00000000;
      break;
  }

  //SETS DIRECTION
  if (encodVal > 6){
    PORTA |= B00001000;
  }else{
    PORTA &= B11110111;
  }

  //STEP
  PORTA |= B00010000;
  delay (stepPulseWidth);
  PORTA &= B11101111;
  delay (stepPulseWidth);
}

void isrA() {
  byte a = (PIND & B00000011);
  if (a & B00000001 == 1){ // if pin 0 (phase A) goes from low to high
    if (a == B00000000 || a == B00000011){
      --count;
    }else{
      ++count;
    }
  }else{ // phase A goes from high to low
    if (a == B00000000 || a == B00000011){
      --count;
    }else{
      ++count;
    }
  }
  if (count < -2000) {
    count += 4000;
  } else if (count > 1999) {
    count -= 4000;
  }
}

void isrB() {
  byte b = (PIND & B00000011);
  if(b & B00000010 == 1){ //if pin 0 (phase B) goes from low to high
    if (b == B00000000 || b B00000011){
      ++count;
    }else{
      --count;
    }
  }else{ // phase B goes from high to low
    if (b == B00000000 || b == B00000011){
      ++count;
    }else{
      --count;      
    }
  }

  if (count < -2000){
    count + 4000;
  } else if (count > 1999) {
    count -= 4000;
  }
}

void isrLimit() {
  atLimit = true;
}