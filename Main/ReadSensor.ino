float pretend_sensor(float x) {
  // pretend to be a sensor, outputing a value using an equation
  //Serial.print("ERROR - SENSOR NOT BEING READ, EQUATION SUBSTITUTED ");
  lcd.setCursor(0,0);
  lcd.print("FAKE SENSOR");

  float y;
  y = -sq(0.5*x-1) + 10;
  return y;
}


void TestSensors() {
  long orange_val;
  long yellow_val;
  while (true) {
      yellow_val = readSensor(sensor_yellow_out, sensor_sck);
      orange_val = readSensor(sensor_yellow_out, sensor_sck);

      write_values_LCD(yellow_val, orange_val);
  }
}


long readSensor(byte output_pin, byte sck_pin) {
  
  // wait for the current reading to finish
  while (digitalRead(output_pin)) {}

  // read 24 bits
  long result = 0;
  for (int i = 0; i < 24; i++) {

    // toggle the clock on and off
    digitalWrite(sck_pin, HIGH);
    digitalWrite(sck_pin, LOW);

    // read the result
    result = result << 1;
    if (digitalRead(output_pin)) {
      result++;
    }
  }

  // get the 2s compliment
  result = result ^ 0x800000;

  return result;
}


void write_values_LCD(long yellow, long orange) {
   lcd.clear();

   // value 1
   lcd.setCursor(0,0);
   lcd.print("YLW: ");
   lcd.setCursor(6,0);
   lcd.print(yellow);

   // value 2
   lcd.setCursor(0,1);
   lcd.print("RNG: ");
   lcd.setCursor(6,1);
   lcd.print(orange);
}