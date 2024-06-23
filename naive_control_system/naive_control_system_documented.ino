/*
Stepper Motor:
https://www.pololu.com/product/1477/faqs
Pololu Stepper Motor: Unipolar/Bipolar, 200 Steps/Rev, 57×76mm, 8.6V, 1 A/Phase Item 1477 SOYO

Likely Drivers:
https://www.pololu.com/category/120/stepper-motor-drivers
https://www.pololu.com/category/212/tic-stepper-motor-controllers 

Encoder:
https://www.ia.omron.com/product/item/2463/

*/

#include <Streaming.h> // Include the Streaming library for serial communication

// Define global variables
int serialAv; // Variable to store the serial availability

// Define constants for encoder ISR pins
const int ISR_FOR_ENCODER_B = 20; // Pin for encoder ISR B
const int A = 21; // Pin for encoder phase A
const int Z = 19; // Pin for encoder phase Z (index)

// Define constants for limit switch pins
const int leftLimitPin = 18; // Pin for left limit switch
const int rightLimitPin = 19; // Pin for right limit switch

// Define constants for stepper motor pins
const int dirPin = 25; // Pin for stepper motor direction
const int steppin = 26; // Pin for stepper motor step
const int mOPin = 22; // Pin for stepper motor microstep 0
const int mlPin = 23; // Pin for stepper motor microstep 1
const int m2Pin = 24; // Pin for stepper motor microstep 2

// Define variables for stepper motor control
uint8_t stepSizeAndDir = B00000000; // Variable to store step size and direction
unsigned long stepPulseWidth = 2; // Variable to store step pulse width in milliseconds

// Define variables for encoder count and limit switch state
volatile int16_t count = -2000; // Variable to store encoder count
bool atLimit = false; // Variable to store limit switch state

int lastCount = -2000; // Variable to store previous encoder count

void setup() {
    // Set up pins as inputs or outputs
    DDRD &= B11111000; // Set direction register for Port D (some range of Arduino digital pins) to input
    DDRA = B11111111; // Set direction register for Port A (some range of Arduino digital pins) to output

    pinMode(leftLimitPin, INPUT);
    pinMode(rightLimitPin, INPUT);

    // Writing HIGH to INPUT pins enables the internal pull-up resistor!
    digitalWrite(leftLimitPin, HIGH);
    digitalWrite(rightLimitPin, HIGH);

    digitalWrite(A, HIGH);
    digitalWrite(ISR_FOR_ENCODER_B, HIGH);
    digitalWrite(Z, HIGH); // Set encoder phase Z (index) pin high

    PORTA &= B00000000; // Clear Port A (some range of Arduino digital pins)

    digitalWrite(26, LOW); // Set stepper motor step pin low

    // Attach interrupts for encoder and limit switches
    attachInterrupt(digitalPinToInterrupt(ISR_FOR_ENCODER_B), isrB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(A), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(leftLimitPin), isrLimit, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rightLimitPin), isrLimit, CHANGE);

    Serial.begin(250000); // Initialize serial communication at 250000 baud
    Serial.println("hi"); // Print "hi" to serial monitor

    while ((serialAv = Serial.available()) == 0) { } // Wait for serial data to be available
}

void loop() {
    // Check if serial data is available
    if (Serial.available() != serialAv) {
        return;
    }

    // Check if limit switch is triggered
    if (atLimit) {
        return;
    }

    // Disable interrupts
    cli();

    // Read encoder count
    int16_t rncount = count;

    // Enable interrupts
    sei();

    // Calculate encoder value based on count
    uint8_t encodVal = 6;
    if (rncount >= 6) {
        encodVal = 12;
    } else if (rncount <= -6) {
        encodVal = 0;
    } else {
        encodVal += rncount;
    }

    // Set stepper motor step size and direction based on encoder value
    PORTA &= B11111000;
    switch (encodVal) {
    case 6:
        return;
    case 5:
    case 7:
        PORTA |= B00000101;
        break;
    case 4:
    case 8:
        PORTA |= B00000100;
        break;
    case 3:
    case 9:
        PORTA |= B00000011;
        break;
    case 2:
    case 10:
        PORTA |= B00000010;
        break;
    case 1:
    case 11:
        PORTA |= B00000001;
        break;
    case 0:
    case 12:
        PORTA |= B00000001;
        //PORTA |=B00000000;
        break;
    }

    // Set stepper motor direction based on encoder value
    if (encodVal > 6){
        PORTA |= B00001000;
    }else{
        PORTA &= B11110111;
    }

    // Step the stepper motor
    PORTA |= B00010000;
    delay(stepPulseWidth);
    PORTA &= B11101111;
    delay(stepPulseWidth);
}

/*
The isrA() and isrB() functions are called whenever there is a change in the state of 
the encoder phases A and B, respectively. They update the encoder count based on the 
direction of the change.

Based on https://www.youtube.com/watch?v=_YI_UJ910UY
I can probably replace some of the logic here by specifying RISING or FALLING in the 
attachInterrupt() function, and creating simpler ISR functions.
*/

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
    if (count < -2000) {
        count += 4000;
    } else if (count > 1999) {
        count -= 4000;
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
    if (count < -2000){
        count += 4000;
    } else if (count > 1999) {
        count -= 4000;
    }
}

/*
The isrLimit() function is called whenever a limit switch is triggered. It sets the 
atLimit variable to true, indicating that the system has reached a limit.
*/

void isrLimit() {
    // Interrupt service routine for limit switches
    atLimit = true;
}