#include <LiquidCrystal.h>

const int clockPin = 2;     // Clock pin connected to digital pin 2
const int pressurePin0 = A0; // Analog pin for pressure sensor
const int pressurePin1 = A1; // Analog pin for pressure sensor


const float vcc = 3.3;      // Supply voltage (can be 3.3V or 5V)
const float pMin = 0.0;     // Minimum pressure in kPa
const float pMax = 40.0;    // Maximum pressure in kPa
const float vMin = 0.0;     // Minimum voltage output (assumed)
const float vMax = vcc;     // Maximum voltage output (assumed to be equal to VCC)



// specify the refresh rate of the program
const float refresh_rate_ms = 100;



// LCD Setup
const int rs = 12, en = 11, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


void setup() {

  // begin serial connection
  Serial.begin(9600);

  // connect the clock pin
  pinMode(clockPin, OUTPUT);
  digitalWrite(clockPin, LOW);

  // Initialize a 16x2 LCD
  lcd.begin(16, 2); 

}


void loop() {
  int sensorValue0 = PulseClockReadPressureSensor(pressurePin0);
  int sensorValue1 = PulseClockReadPressureSensor(pressurePin1);
  
  // Convert analog reading to voltage
  float voltage0 = sensorValue0 * (vcc / 1023.0);
  float voltage1 = sensorValue1 * (vcc / 1023.0);

  // Convert voltage to pressure in kPa
  float pressure0 = PressureFromVoltage(voltage0);
  float pressure1 = PressureFromVoltage(voltage1);

  String line0 = "raw val0 : " + formatNumber0000dot(sensorValue0);
  String line1 = "raw val1 : " + formatNumber0000dot(sensorValue1);
  WriteToDisplay(line0, line1);

  PrintToSerial(sensorValue0, pressure0, voltage0);
  PrintToSerial(sensorValue1, pressure1, voltage1);

  delay(refresh_rate_ms);  // Wait for some time before refreshing
}


String formatNumber00dot00(float number) {
  // Check if the number is below the lower threshold
  if (number < 0.01) {
    return "undr";
  } 
  // Check if the number is above the upper threshold
  else if (number > 99.99) {
    return "over";
  } 
  // Format numbers within the valid range
  else {
    // Extract the whole part of the number
    int wholePart = (int)number;
    
    // Calculate the fractional part, multiplied by 100 for two decimal places
    // Add 0.5 for rounding to nearest integer
    int fractionalPart = (int)((number - wholePart) * 100 + 0.5);
    
    // Create a buffer to hold the formatted string
    char buffer[7];  // Size 7 to accommodate "XX.XX\0"
    
    // Format the number with two digits before and after the decimal point
    // %02d ensures two digits with leading zeros if necessary
    sprintf(buffer, "%02d.%02d", wholePart, fractionalPart);
    
    // Convert the char array to a String and return it
    return String(buffer);
  }
}



String formatNumber0000dot(int number) {
  // Check if the number is below the lower threshold
  if (number < 0) {
    return "undr";
  } 
  // Check if the number is above the upper threshold
  else if (number > 1024) {
    return "over";
  } 
  // Format numbers within the valid range (0 to 1024)
  else {
    // Create a buffer to hold the formatted string
    char buffer[5];  // Size 5 to accommodate "XXXX\0"
    
    // Format the number with four digits, adding leading zeros if necessary
    // %04d ensures four digits with leading zeros
    sprintf(buffer, "%04d", number);
    
    // Convert the char array to a String and return it
    return String(buffer);
  }
}


int PulseClockReadPressureSensor(int pressurePin) {
  // Read the analog value from the sensor by pulsing the clock on, then read
  digitalWrite(clockPin, HIGH);
  int sensorValue = analogRead(pressurePin);
  delayMicroseconds(10);  // Adjust this delay as needed
  digitalWrite(clockPin, LOW);

  return sensorValue;
}


float PressureFromVoltage(float voltage){
  float pressure = ((voltage - vMin) * (pMax - pMin) / (vMax - vMin)) + pMin;
  return pressure;
}





