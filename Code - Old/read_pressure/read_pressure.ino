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


float volu_flow_rate_from_delta_p(float pressure1, float pressure2){
  // calculate the volumetric flow rate using the Hagen–Poiseuille equation

  float R = 1;    // pipe radius, meters
  float deltaP = abs(pressure1 - pressure2);  // absolute difference between pressures
  float L = 1;    // pipe length, meters


  // viscosity https://www.engineersedge.com/physics/viscosity_of_air_dynamic_and_kinematic_14483.htm 
  float mu = 1.825*(10**5);    // dynamic viscosity of air at 20 deg C, kg/m-s

  float dotQ_m3_sec = pi()*(R**4)*deltaP / (8*mu*L);

  return dotQ_m3_sec
}



// Function to perform trapezoidal integration of sensor readings from 0 to a specified time
// Parameters:
//   sensorRead: Function pointer to read sensor value
//   interval: Desired sampling interval in milliseconds
//   duration: Total integration time in milliseconds
// Returns:
//   The integrated value over the specified duration
float trapezoidalIntegrate(unsigned long interval, unsigned long duration) {
  // Variables for integration
  float lastValue = 0;     // Last sensor reading
  float integral = 0;      // Accumulated integral
  unsigned long startTime = millis();  // Start time of integration
  unsigned long lastSampleTime = startTime;  // Time of last sample
  
  // Continue integrating until the specified duration has elapsed
  while (millis() - startTime < duration) {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastSampleTime) / 1000.0;  // Time since last sample in seconds
    
    // Check if it's time for a new reading
    if (deltaTime >= interval / 1000.0) {
      // Read current sensor value 
      float pressure1 = read_pressure("sensor1");   // placeholder function
      float pressure2 = read_pressure("sensor2");


      // get the volumetric flow rate
      currentValue = volu_flow_rate_from_delta_p(pressure1, pressure2);
      
      // Calculate area (volume) using trapezoidal rule
      integral += (currentValue + lastValue) * deltaTime * 0.5;
      
      // Update for next iteration
      lastValue = currentValue;
      lastSampleTime = currentTime;
    }
    
    // Small delay to prevent tight looping
    delay(1);
  }
  
  // Return the final integrated value
  return integral;
}



