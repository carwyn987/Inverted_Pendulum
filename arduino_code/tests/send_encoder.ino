void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud rate
}

void loop() {
  float data = random(0, 1000) / 10.0;  // Generate random float data
  Serial.println(data);  // Send data to the serial port
  delay(1000);  // Wait for 1 second before sending the next data
}