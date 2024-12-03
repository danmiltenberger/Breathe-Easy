void PrintToSerial(int sensorValue, float pressure, float voltage) {
  Serial.print("Sensor Value: ");
  Serial.print(sensorValue);
  Serial.print("\tVoltage: ");
  Serial.print(voltage, 2);
  Serial.print(" V\tPressure: ");
  Serial.print(pressure, 2);
  Serial.println(" kPa");
}



void clearDisplay() {
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("> ");
}


void WriteToDisplay(String line1, String line2) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(line1);
  lcd.setCursor(0,1);
  lcd.print(line2);
}


