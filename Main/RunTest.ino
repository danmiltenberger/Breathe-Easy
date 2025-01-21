void RunTest() {
  Serial.println("Run test executed");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Testing...");
  lcd.setCursor(0,1);
  lcd.print("Dan! Code this");
  delay(3000);
}