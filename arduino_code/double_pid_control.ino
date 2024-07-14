#include <Streaming.h>

int serialAv; // Variable to store the serial availability

//Stepper motor pins
const int dirPin = 25;
const int steppin = 26;
const int mOPin = 22;
const int mlPin = 23;
const int m2Pin = 24;

//limit switch pins
const int leftLimitPin = 15;
const int rightLimitPin = 18;

// Define constants for encoder ISR pins
const int ISR_FOR_ENCODER_B = 20; // Pin for encoder ISR B
const int A = 21; // Pin for encoder phase A
const int Z = 19; // Pin for encoder phase Z (index)

// Define variables for encoder count
volatile int16_t count = -4000; // Variable to store encoder count
int lastCount = -2000; // Variable to store previous encoder count

float pos = 0.0;
int left_to_right = 0;

// Limit Switch variable
bool atLimit = false;
bool prevAtLimit = false;

// uint8_t stepSize AndDir = B00000000; //00000001 is m0, 00000010 is ml, 00000100 is m2, 00001000 is dir.
unsigned int stepPulseWidth = 2600; //in milliseconds

unsigned long start_ts = millis() / 1000;

void setup() {
  DDRD = B11111000; // Direction register for Port A (some range of arduino digital pins); 0 = input, 1 = output...
  DDRA = B11111111; // sets PAO PA4 (step, dir, m0, ml, m2) to Outputs
  PORTA = B00000000; // Set all pins on Port A to low
  
  // Encoder direction
  pinMode(A, INPUT);
  pinMode(Z, INPUT);
  pinMode(ISR_FOR_ENCODER_B, INPUT);

  // Limit switch direction
  pinMode(leftLimitPin, INPUT);
  pinMode(rightLimitPin, INPUT);

  // Encoder pullup resistor
  digitalWrite(A, HIGH);
  digitalWrite(ISR_FOR_ENCODER_B, HIGH);
  digitalWrite(Z, HIGH); // Set encoder phase Z (index) pin high

  // limit switch pullup resistor
  digitalWrite(leftLimitPin, HIGH);
  digitalWrite(rightLimitPin, HIGH);

  PORTA &= B00000000; // Clear Port A (some range of Arduino digital pins)

  // Attach interrupts for encoder
  attachInterrupt(digitalPinToInterrupt(ISR_FOR_ENCODER_B), isrB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(A), isrA, CHANGE);

  // Attach interrupts for limit switches
  attachInterrupt(digitalPinToInterrupt(leftLimitPin), isrLimit, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightLimitPin), isrLimit, CHANGE);
  
  Serial.begin(115200);
  // Serial.println("Press a key to begin execution ...");
  // while ((serialAv = Serial.available()) == 0) { } // Wait for first serial input
  calibrate(5000);
}

bool alwaysExit = false;

void loop() {
  // if (digitalRead(rightLimitPin)) {
  //   for (int i = 0; i<left_to_right/2; i++){
  //     step_stepper_motor(-1.0, stepPulseWidth, true);  
  //   }
  //   atLimit = false;
  // }
  // if (digitalRead(leftLimitPin)){
  //   for (int i = 0; i<left_to_right/2; i++){
  //     step_stepper_motor(1.0, stepPulseWidth, true);  
  //   }
  //   atLimit = false;
  // }
  // if ((digitalRead(rightLimitPin) && prevAtLimit) || (digitalRead(leftLimitPin) && prevAtLimit) || (atLimit && prevAtLimit)){
  //   return;
  // }
  if (alwaysExit) {
    return;
  }
  if (atLimit || digitalRead(rightLimitPin) || digitalRead(leftLimitPin)){
    delayMicroseconds(1000);
    if (digitalRead(rightLimitPin) || digitalRead(leftLimitPin)){
      alwaysExit = true;
      return;
    }else{
      atLimit = false;
    }
  }


  cli(); // Disable interrupts
  int16_t rncount = count; // Read encoder count
  sei(); // Enable interrupts

  // Send encoder value to computer
  Serial.print("a");
  Serial.println(rncount);
  
  // Read in the desired movement from the computer
  if (Serial.available() > 0) {
    float move_x = read_float_from_serial();
    float move = move_x; // / (2 * 3.14 * 0.028 * 1.8 / 360.0); // ~num steps

    // Move the stepper motor
    step_stepper_motor(move, stepPulseWidth, false);
  }
}

float read_float_from_serial() {
  String input = "";
  char incomingByte;
  while (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (incomingByte == '\n') {
      break;
    }
    input += incomingByte;
  }
  return input.toFloat();
}

void isrA() {
    // Interrupt service routine for encoder phase A
    byte a = (PIND & B00000011);
    if (a & B00000001 == 1){ // If pin 0 (phase A) goes from low to high
        if (a == B00000000 || a == B00000011){
            --count;
        }else{
            ++count;
        }
    }else{ // Phase A goes from high to low
        if (a == B00000000 || a == B00000011){
            --count;
        }else{
            ++count;
        }
    }
    // Handle overflow and underflow of encoder count
    if (count < -4000) {
        count += 8000;
    } else if (count > 3999) {
        count -= 8000;
    }
}

void isrB() {
    // Interrupt service routine for encoder phase B
    byte b = (PIND & B00000011);
    if(b & B00000010 == 1){ // If pin 0 (phase B) goes from low to high
        if (b == B00000000 || b == B00000011){
            ++count;
        }else{
            --count;
        }
    }else{ // Phase B goes from high to low
        if (b == B00000000 || b == B00000011){
            ++count;
        }else{
            --count;
        }
    }
    // Handle overflow and underflow of encoder count
    if (count < -4000){
        count += 8000;
    } else if (count > 3999) {
        count -= 8000;
    }
}

void isrLimit() {
  atLimit = true;
}


// called during
void step_stepper_motor(float move, int numDelayMicroseconds, bool override_limit) {
  /*
  Assuming positive for clockwise and negative for counter-clockwise, takes a float representing the movement desired
  by the stepper motor, takes a step.
  */

  // Reset step options
  PORTA &= B11111000;

  int direction = 0;

  // Set the direction of the stepper motor
  if (move > 0){
    PORTA |= B00001000; // Set the direction pin high
    direction = 1;
  }else{
    PORTA &= B11110111; // Set the direction pin low
    direction = -1;
  }
  move = abs(move);

  float applied_movement = 0.0;

  // Set the step size
  // step size choices: {1.0/32.0, 1.0/16.0, 1.0/8.0, 1.0/4.0, 1.0/2.0, 1.0}
  // float cutoffs[] = {1.0, 0.75, 0.375, 0.1875, 0.09375, 0.046875, 0.03125}; // These are the midpoints between step sizes
  float cutoffs[] = {999999, 1.0, 0.8, 0.6, 0.4, 0.2, 0.2, 0.0};
  
  int indx = 0;
  for (int i = 0; i < 7; i++){
    if (move < cutoffs[i] && move >= cutoffs[i+1]){
      indx = i;
      break;
    }
  }

  // Binary Step Repr.: 000  100   010   110  001    111 or 011  101
  // Actual Step Size:  1    1/2   1/4   1/8  1/16   1/32
  switch (indx) { //SETS STEP SIZE
    case 0: //MAX STEP
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
    case 6:
      PORTA |= B00000110;
      applied_movement = 1.0/32.0;
    default:
      PORTA |= B00000110;
      applied_movement = 1.0/32.0;
  }

  // PORTA |= B00000000; // 0.75
  // PORTA |= B00000001; // 1.5
  // PORTA |= B00000010; // 3
  // PORTA |= B00000011; // 6
  // PORTA |= B00000100; // 12
  // PORTA |= B00000110; // 26

  pos += direction * applied_movement * 1.8 * 2 * 3.14159 * 0.028 / 360;
  Serial.print("x");
  Serial.println(pos);

  if(!atLimit || override_limit){
    // Set the step pin high
    PORTA |= B00010000;
    delayMicroseconds(numDelayMicroseconds);
    PORTA &= B11101111;
    delayMicroseconds(numDelayMicroseconds/4); 
  }
}

int move(int step, int limitPin){
  
  bool last_limit_reached = false;
  int count_steps = 0;
  
  // move
  while (!atLimit && !(digitalRead(limitPin) == HIGH && last_limit_reached)){
    step_stepper_motor(step, stepPulseWidth, false);
    
    if(!atLimit){
      count_steps += 1;
    }
    if(digitalRead(limitPin) == HIGH){
      last_limit_reached = true;
    }
  }

  return count_steps;
}

void calibrate(float step_time){

  // set time
  float save_step_time = stepPulseWidth;
  stepPulseWidth = step_time;
  
  int count_steps = 0;

  float stepsize_tmp = 1.0;

  // move right
  count_steps = move(stepsize_tmp, rightLimitPin);

  int backup_from_limit = 20;
  int left_steps = count_steps - backup_from_limit;
  // Serial.print("Steps from start to left = ");
  // Serial.println(left_steps);
  count_steps = 0;

  // backup
  for (int i = 0; i < backup_from_limit; i++){
    step_stepper_motor(-1 * stepsize_tmp, stepPulseWidth, true); // move off limit switch
  }
  atLimit = false;

  // move left
  count_steps = move(-1.0 * stepsize_tmp, leftLimitPin);

  left_to_right = count_steps - backup_from_limit;
  // Serial.print("Steps from left to right = ");
  // Serial.println(left_to_right);

  // backup
  for (int i = 0; i < backup_from_limit; i++){
    step_stepper_motor(stepsize_tmp, stepPulseWidth, true); // move off limit switch
  }
  atLimit = false;

  // move to the center
  for (int i = 0; i<int(1.35 * left_to_right/2); i++){
    step_stepper_motor(stepsize_tmp, stepPulseWidth, true);  
  }
  
  pos = 0;
  atLimit = false;

  stepPulseWidth = save_step_time;
}