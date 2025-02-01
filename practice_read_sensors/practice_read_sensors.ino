#include <Wire.h>

const int sensor1Pin = A0;  // Analog pin connected to the first sensor
const int sensor2Pin = A1;  // Analog pin connected to the second sensor
const float maxPressure = 40.0;  // Maximum pressure in kPa
const float voltageRef = 5.0;  // Reference voltage (3.3V or 5V depending on your Arduino)

void setup() {
  Serial.begin(9600);
  pinMode(sensor1Pin, INPUT);
  pinMode(sensor2Pin, INPUT);
}

float readPressure(int sensorPin) {
  int rawValue = digitalRead(sensorPin);
  //float voltage = (rawValue / 1023.0) * voltageRef;
  //return (voltage / voltageRef) * maxPressure;

  return rawValue;
}

void loop() {
  float pressure1 = readPressure(sensor1Pin);
  float pressure2 = readPressure(sensor2Pin);

  Serial.print("Sensor 1 val: ");
  Serial.print(pressure1);
  Serial.print("| Sensor 2 val: ");
  Serial.print(pressure2);
  Serial.println("");

  delay(1000);  // Wait for 1 second before next reading
}
