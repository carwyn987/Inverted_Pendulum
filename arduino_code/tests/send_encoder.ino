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

// Define variables for encoder count and limit switch state
volatile int16_t count = -4000; // Variable to store encoder count
int lastCount = -2000; // Variable to store previous encoder count

void setup() {
    // Set up pins as inputs or outputs
    DDRD &= B11111000; // Set direction register for Port D (some range of Arduino digital pins) to input
    // DDRA = B11111111; // Set direction register for Port A (some range of Arduino digital pins) to output

    pinMode(A, INPUT);
    pinMode(Z, INPUT);
    pinMode(ISR_FOR_ENCODER_B, INPUT);

    digitalWrite(A, HIGH);
    digitalWrite(ISR_FOR_ENCODER_B, HIGH);
    digitalWrite(Z, HIGH); // Set encoder phase Z (index) pin high

    PORTA &= B00000000; // Clear Port A (some range of Arduino digital pins)

    // Attach interrupts for encoder and limit switches
    attachInterrupt(digitalPinToInterrupt(ISR_FOR_ENCODER_B), isrB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(A), isrA, CHANGE);

    Serial.begin(9600);  // Start serial communication at 9600 baud rate

    // while ((serialAv = Serial.available()) == 0) { } // Wait for serial data to be available
}

void loop() {
    // Check if serial data is available
    if (Serial.available() != serialAv) {
        return;
    }

    // Disable interrupts
    cli();

    // Read encoder count
    int16_t rncount = count;

    // Enable interrupts
    sei();

    // Calculate encoder value based on count
    // uint8_t encodVal = 6;
    // if (rncount >= 6) {
    //     encodVal = 12;
    // } else if (rncount <= -6) {
    //     encodVal = 0;
    // } else {
    //     encodVal += rncount;
    // }

    // Print encoder value to serial monitor
    // Serial << "Encoder Value: " << rncount << endl;
    Serial.println(rncount);
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