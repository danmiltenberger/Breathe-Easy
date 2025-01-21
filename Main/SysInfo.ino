void SysInfo() {
  Serial.println("System info executed");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SysInfo...");
  lcd.setCursor(0,1);
  lcd.print("Dan! Code this");
  delay(3000);
}