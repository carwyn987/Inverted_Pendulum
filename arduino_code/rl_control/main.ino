#include <Streaming.h>

int serialAv;

//Stepper motor pins
const int dirPin = 25;
const int steppin = 26;
const int mOPin = 22;
const int mlPin = 23;
const int m2Pin = 24;

// uint8_t stepSize AndDir = B00000000; //00000001 is m0, 00000010 is ml, 00000100 is m2, 00001000 is dir.
unsigned int stepPulseWidth = 2000; //in milliseconds

void setup() {
  DDRD = B11111000; // Direction register for Port A (some range of arduino digital pins); 0 = input, 1 = output...
  DDRA = B11111111; // sets PAO PA4 (step, dir, m0, ml, m2) to Outputs
  PORTA = B00000000; // Set all pins on Port A to low
  
  Serial.begin(250000);
  Serial.println("Press a key to begin execution ...");
  while ((serialAv = Serial.available()) == 0) { } // Wait for first serial input
}

void step_stepper_motor(float move, int numDelayMicroseconds) {
  /*
  Assuming positive for clockwise and negative for counter-clockwise, takes a float representing the movement desired
  by the stepper motor, takes a step.
  */

  // Reset step options
  PORTA &= B11111000;

  // Set the direction of the stepper motor
  if (move > 0){
    PORTA |= B00001000; // Set the direction pin high
  }else{
    PORTA &= B11110111; // Set the direction pin low
  }
  move = abs(move);

  // Set the step size
  if (move >= 1.0) {
    // do nothing
  } else if (move <= 1.0/32.0) {
    PORTA |= B00000101;
  } else {
    // step size choices: {1.0/32.0, 1.0/16.0, 1.0/8.0, 1.0/4.0, 1.0/2.0, 1.0}
    float cutoffs[] = {1.0, 0.75, 0.375, 0.1875, 0.09375, 0.046875, 0.03125}; // These are the midpoints between step sizes

    
    int indx = 0;
    for (int i = 0; i < 6; i++){
      if (move < cutoffs[i] && move >= cutoffs[i+1]){
        indx = i;
        break;
      }
    }

    float applied_movement = 0.0;

    // Binary Step Repr.: 000  100   010   110  001    111 or 011  101
    // Actual Step Size:  1    1/2   1/4   1/8  1/16   1/32
    switch (indx) { //SETS STEP SIZE
      case 0: //NOSTEP THOUGH
        applied_movement = 1.0;
        break;
      case 1:
        PORTA |= B00000001;
        applied_movement = 1.0/2.0;
        break;
      case 2:
        PORTA |= B00000010;
        applied_movement = 1.0/4.0;
        break;
      case 3:
        PORTA |= B00000011;
        applied_movement = 1.0/8.0;
        break;
      case 4:
        PORTA |= B00000100;
        applied_movement = 1.0/16.0;
        break;
      case 5:
        PORTA |= B00000110;
        applied_movement = 1.0/32.0;
        break;
    }
  }

  // PORTA |= B00000000; // 0.75
  // PORTA |= B00000001; // 1.5
  // PORTA |= B00000010; // 3
  // PORTA |= B00000011; // 6
  // PORTA |= B00000100; // 12
  // PORTA |= B00000110; // 26

  // Set the step pin high
  PORTA |= B00010000;
  delayMicroseconds(numDelayMicroseconds);
  PORTA &= B11101111;
  delayMicroseconds(numDelayMicroseconds);
}

void loop() {
  
  // if (Serial.available() != serialAv) {
  //   return;
  // }

  // Set up communication with computer
  

  // Send encoder value to computer


  
  // Read in the desired movement from the computer



  // Move the stepper motor
  float move = 0.0;
  for(float i = 0; i < 100; i+= 0.001){
    // set move equal to sin(i)
    move = sin(i);
    // print to serial the move
    // Serial << "Move: " << move << endl;
    // call the step_stepper_motor function
    step_stepper_motor(move, stepPulseWidth);
  }
}