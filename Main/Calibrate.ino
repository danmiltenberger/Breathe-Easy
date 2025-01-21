void Calibrate() {
  Serial.println("calibration executed");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Calibrating...");
  lcd.setCursor(0,1);
  lcd.print("Dan! Code this");
  delay(3000);
}
