void RunTest() {
  Serial.println("Run test executed");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Testing...");
  lcd.setCursor(0,1);
  delay(2000);
  breathe_out();    
}


void breathe_out() {
  unsigned long count_down_duration_ms = 6*1000;
  unsigned long delay_offset_ms = 10;  // built in delay to get the last data point (cause the program doesn't run fast enough)
  float startTime = millis();
  float elapsed_time_ms;
  float elapsed_time_s;
  float timeRemaining;
  float time_since_sensor_read = 0;
  float time_of_last_sensor_read = 0;
  float volume_dot1;
  float volume1;
  float volume_dot1_prev = 0;
  float volume_FEV1 = 0;   // in the first second
  float volume_FVC = 0;    // in the full test
  float dt_ms = 100;   // milliseconds between sensor values
  int num_data_points = 0;  

  Serial.print("Test Parameters---- \n");
  Serial.print("Refresh rate ms:  " + String(dt_ms) +  "\n");
  Serial.print("Test duration ms: " + String(count_down_duration_ms) +  "\n");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Breathe OUT!");

  // begin an infinite loop
  while (true) {

    // keep track of time passing
    elapsed_time_ms = millis() - startTime;
    
    // read sensor values if it is refresh time
    time_since_sensor_read = millis() - time_of_last_sensor_read;
    if (time_since_sensor_read > dt_ms) {

      elapsed_time_s = elapsed_time_ms/1000;
      num_data_points = num_data_points + 1;
      time_of_last_sensor_read = millis();    // now


      // get and read values
      volume_dot1 = pretend_sensor(elapsed_time_s);

      // find the area under the curve by comparing this and previous value
      volume1 = integrate_volume(volume_dot1_prev, volume_dot1, dt_ms/1000);
      volume_dot1_prev = volume_dot1;

      // add to running total
      volume_FVC = volume_FVC + volume1;

      // only add to first second total if, ya know, it's in the first second
      if (elapsed_time_ms < 1000) {
        volume_FEV1 = volume_FEV1 + volume1;
      }

      // Display to serial
      Serial.print(String(num_data_points) + "\t t (sec): " + String(elapsed_time_s) + "\t Vdot (m3/s): " + String(volume_dot1) + "\t FEV1 (m3): " + String(volume_FEV1) +"\t FVC (m3): " + String(volume_FVC) + "\n");
    }

    // update the countdown
    if (elapsed_time_ms < (count_down_duration_ms + delay_offset_ms)) {
      timeRemaining = count_down_duration_ms - elapsed_time_ms;
      displayTime(timeRemaining);
    } 
    
    // exit conditios
    else {
      lcd.setCursor(0, 1);
      lcd.print("Time's up!      ");

      delay(1000);
      display_results(volume_FEV1, volume_FVC, num_data_points);
      break;
    }
  }
}

void displayTime(unsigned long time) {
  unsigned long seconds = time / 1000;
  unsigned long milliseconds = time % 1000;

  lcd.setCursor(0, 1);
  if (seconds < 10) lcd.print("0");
  lcd.print(seconds);
  lcd.print(":");
  if (milliseconds < 100) lcd.print("0");
  if (milliseconds < 10) lcd.print("0");
  lcd.print(milliseconds);
}



float integrate_volume(float volume_dot_prev, float volume_dot, float dt_sec) {
  // trapezoidally integrating using equally spaced panels
  // https://en.wikipedia.org/wiki/Trapezoidal_rule subsection "Numerical Implementation / Uniform Grid"
  
  float this_volume;
  this_volume = (0.5*dt_sec)*(volume_dot + volume_dot_prev);
  return this_volume;
}


void display_results(float FEV1, float FVC, int num_data_points) {

  float ratio = FEV1 / FVC;
  Serial.print("Test Results---- \n");
  Serial.print("FEV1: " + String(FEV1) + "\n");
  Serial.print("FVC: " + String(FVC) + "\n");
  Serial.print("FEV1/FVC:   " + String(ratio) + "\n");
  Serial.print("num data pts: " + String(num_data_points) + "\n");
  
  
  // screen 1
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("FEV1: " + String(FEV1));

  lcd.setCursor(0,1);
  lcd.print("FVC:  " + String(FVC));

  while (digitalRead(SW) == HIGH) {
    // do nothing, wait for button press to go to next screen
  }

  // screen 2
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("FEV1/FVC:   " + String(ratio));

  lcd.setCursor(0,1);
  lcd.print("num data pts: " + String(num_data_points));
  delay(2000);

  while (digitalRead(SW) == HIGH) {
    // do nothing, wait for button press to go to next screen
  }
}

