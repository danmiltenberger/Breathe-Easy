float pretend_sensor(float x) {
  // pretend to be a sensor, outputing a value using an equation
  //Serial.print("ERROR - SENSOR NOT BEING READ, EQUATION SUBSTITUTED ");
  lcd.setCursor(0,0);
  lcd.print("FAKE SENSOR");

  float y;
  y = -sq(0.5*x-1) + 10;
  return y;
}


void read_sensor() {
  // read raw sensor value, probably a number between 0 - 1600?

  // convert to pascals

  // convert to liters / min

  //TODO!!!
}