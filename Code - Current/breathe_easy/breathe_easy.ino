#include <LCD_I2C.h>
#include <Encoder.h>

// Function prototypes
void displayMenu();
void selectMenuItem();
void runTest();
void calibrate();
void sysInfo();


void sensorReadout();
long readSensor(int pin_output, int pin_clock);
void breathe_out(long test_duration_ms);

// from this document: https://docs.google.com/spreadsheets/d/1nrHbGkNhTVyMGgrFzzMd0zXJoE00PVxOz64fbbErKuQ/edit?gid=0#gid=0 
// Curve fitting for kPa
// y = mx+b
// kpa = m*(sensor difference) + b

// old values 2025-03-24 and earlier
float kpa_m = 1.38E-6;
float kpa_b = -0.0349;

// Curve fitting for flow, volume_dot (L/min)
// y = a + bx + cx^2
float flow_a = 0.973;
float flow_b = 64.2;
float flow_c = -37.1;

// Circuit setup (all digital)
// LCD SDA -> SDA, SCL -> SCL
// encoder
const int encoder_CLK = 2;
const int encoder_DT = 3;
const int encoder_SW = 4;

// pressure sensors
const int pressure_sensor1_out = 5;
const int pressure_sensor2_out = 6;
const int pressure_sensor_sck = 7;


// LCD configuration
LCD_I2C lcd(0x27, 16, 4); // I2C address 0x27, 16 columns, 4 rows

// Rotary encoder configuration
Encoder myEncoder(encoder_CLK, encoder_DT);
const int buttonPin = encoder_SW;

// Menu items and associated functions
const int numItems = 4;
String menuItems[numItems] = {
  "Run Test", 
  "Read Sensors", 
  "Calibrate", 
  "Sys Info", 
  };
void (*menuFunctions[numItems])() = {
  runTest, 
  sensorReadout, 
  calibrate, 
  sysInfo, 
};

int menuIndex = 0;
int lastEncoderValue = 0;

const int ledPin = 13; // Built-in LED

void setup() {
  // LCD setup
  lcd.begin();
  lcd.backlight();

  Serial.begin(9600);

  // rotary setup
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  // setup sensors
  pinMode(pressure_sensor1_out, INPUT);
  pinMode(pressure_sensor2_out, INPUT); 
  pinMode(pressure_sensor_sck, OUTPUT); 

  // when setup completed, exit to main menu
  displayMenu();
}


void loop() {
  int encoderValue = myEncoder.read() / 4;  // Divide by 4 for smoother scrolling

  if (encoderValue != lastEncoderValue) {
    menuIndex = constrain(menuIndex + (encoderValue - lastEncoderValue), 0, numItems - 1);
    lastEncoderValue = encoderValue;
    displayMenu();
  }

  if (digitalRead(buttonPin) == LOW) {
    selectMenuItem();
    delay(200);  // Debounce
  }
}


void displayMenu() {
  // show the different options plus the selected option
  lcd.clear();

  // print the options, from 0 to 3 (aka 1 to 4)
  for (int i = 0; i <= 3; i++) {
    lcd.setCursor(0,i);
    lcd.print(menuItems[i]);  
  }

  // update the line with a > to show selection
  lcd.setCursor(0, menuIndex);
  lcd.print("> " + menuItems[menuIndex]);

  //lcd.print("> " + menuItems[menuIndex]);
  //lcd.setCursor(0, 1);
  //lcd.print(menuIndex < numItems - 1 ? menuItems[menuIndex + 1] : "");
}


void selectMenuItem() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Selected:");
  lcd.setCursor(0, 1);
  lcd.print(menuItems[menuIndex]);
  
  // Call the associated function
  menuFunctions[menuIndex]();
  
  delay(2000);
  displayMenu();
}


// Menu Item 2
void sensorReadout() {
  long long five_minutes = 5*60*1000;
  delay(500); // fixes double pressing button

  breathe_out(five_minutes);
}


long read_sensor(int pin_output, int pin_clock) {
  // begin infinite loop
  while (true) {

    // wait for the current reading to finish
    while (digitalRead(pin_output)) {}

    // read 24 bits
    long result = 0;
    for (int i = 0; i < 24; i++) {
      digitalWrite(pin_clock, HIGH);
      digitalWrite(pin_clock, LOW);
      result = result << 1;
      if (digitalRead(pin_output)) {
        result++;
      }
    }

    // get the 2s compliment
    result = result ^ 0x800000;

    // pulse the clock line 3 times to start the next pressure reading
    for (char i = 0; i < 3; i++) {
      digitalWrite(pin_clock, HIGH);
      digitalWrite(pin_clock, LOW);
    }

    return result;
  }
}


float convert_sensor_dif_to_kpa(long dif) {
  // y = mx + b
  // kpa = m*(sensor dif) + b
  // see values at top of the code
  float kpa;
  kpa = kpa_m*dif + kpa_b;
  return kpa;
}


float get_kpa_dif() {
  long sensor1_val = read_sensor(pressure_sensor1_out, pressure_sensor_sck);
  long sensor2_val = read_sensor(pressure_sensor2_out, pressure_sensor_sck);

  long sensor_dif = abs(sensor1_val - sensor2_val);
  float kpa_dif = convert_sensor_dif_to_kpa(sensor_dif);
  return kpa_dif;
}


// Evelyn & Bree code goes here!
// convert the pressure difference to a flow rate
float get_volume_dot_Ls() {
  
  // read sensors
  float delta_pascals = get_kpa_dif();

  //Serial.print("Q, L/min: ");

  // determined through lab testing againast ASL 5000
  float prev_curve_fit = 0.1938; // earlier than 2025-03-24

  // new curve fit
  float m2 = 0.0515206;
  float b2 = 4.63*pow(10,-4);

  float Q_L_min = ((delta_pascals*1000*m2)*0.163)*0.863 + b2;
  float volume_dot_Ls = Q_L_min / 60;

  Serial.println(Q_L_min);

  return volume_dot_Ls;
  //Serial.print(",");
  //Serial.print("Zero: ");
  //Serial.println(0);

  /*
  float D = 0.0064;   // tube diam (meters)
  float mu = 1.8e-5;  // dynamic viscosity, Pascal-seconds
  float rho = 1.2;    // fluid density (kg per m3)
  float L = 0.61;     // tube length m
  float Cd = 0.6;     // Discharge coefficient (asumed for orifice plate)

  // Calculate radius and cross-sectional area
  float R = D/2;
  float A = PI*pow(R,2);


  // Compute flow rate based on flow type
  float Re_threshold = 2000;            // determine laminar threshold

  // Attempt Hagen-Poiseuille equation first (for laminar flow)
  float Q = (PI*pow(R,4)*delta_pascals)/(8*mu*L);   // Hagen-Poiseuille eq calculates flow
  float V = Q/A;                        // resulting velocity
  float Re = (rho*V*D)/mu;              // determine whether it's laminar or not

  if (Re < Re_threshold) {
    // flow is laminar
    float Q_L_per_s = Q*1000;
    return Q_L_per_s;
  }
  else {
    // Use orifice plate equation for turbulent flow
    Q = Cd*A*sqrt(2*delta_pascals/rho);        // orifice plate equation
    V = Q/A;                            // calculate velocity
    Re = (rho*V*D)/mu;                  // calcuate Re to determine flow type
    float Q_L_per_s = Q*1000;
    return Q_L_per_s;
  }
  // TEMP! FIXME
  // EXAMPLE BELOW from a curve fit from in lab testing
  // https://docs.google.com/spreadsheets/d/1nrHbGkNhTVyMGgrFzzMd0zXJoE00PVxOz64fbbErKuQ/edit?gid=0#gid=0 
  // Curve fitting for flow, volume_dot (L/min)
  // y = a + bx + cx^2
  // float volume_dot_L_min;
  // volume_dot_L_min = flow_a + flow_b*kpa_dif + flow_c*pow(kpa_dif, 2);

  // // convert to Liters / sec for simpler math later
  // float volume_dot_Ls = volume_dot_L_min / 60;
  // return volume_dot_Ls;
  */
}


// Menu Item 1
// wrapper for breathe_out, with X seconds
void runTest() {
  unsigned long count_down_duration_ms = 6*1000;
  Serial.println("Run test executed");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Starting Test..");

  delay(500);

  lcd.setCursor(0,1);
  delay(2000);
  breathe_out(count_down_duration_ms);    
}


// primary function, handles getting sensor values over some amount of time, converting, integrating, and displaying
void breathe_out(long count_down_duration_ms) {
  // run a full test, integrating volume flow and display results
  


  unsigned long delay_offset_ms = 10;  // built in delay to get the last data point (cause the program doesn't run fast enough)
  float startTime = millis();

  // handle refresh rate
  float elapsed_time_ms;
  float elapsed_time_s;
  float timeRemaining;
  float time_since_sensor_read = 0;
  float time_of_last_sensor_read = 0;

  // flow values
  float volume_dot_Ls;        // liters / sec
  float volume_dot_Ls_prev = 0;

  // volume values
  float volume_L;             // liters
  float volume_L_FEV1 = 0;    // liters in the first second
  float volume_L_FVC = 0;     // liters in the full test

  // time values
  float dt_ms = 100;          // milliseconds between sensor values
  float dt_s = dt_ms/1000;    // seconds between sensor values (makes math easier later)

  // misc
  int num_data_points = 0;    // count how many data points, just out of curiosity

  // print test start parameters
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
    
    // if it is refresh time, read sensor values 
    time_since_sensor_read = millis() - time_of_last_sensor_read;
    if (time_since_sensor_read > dt_ms) {

      // reset the elapsed time to zero
      elapsed_time_s = elapsed_time_ms/1000;
      num_data_points = num_data_points + 1;
      time_of_last_sensor_read = millis();    // now

      // get and read values
      volume_dot_Ls = get_volume_dot_Ls();

      // find the area under the curve by comparing this and previous value
      volume_L = integrate_volume(volume_dot_Ls_prev, volume_dot_Ls, dt_s);
      volume_dot_Ls_prev = volume_dot_Ls;

      // add to running total
      volume_L_FVC = volume_L_FVC + volume_L;

      // only add to first second total if, ya know, it's in the first second
      if (elapsed_time_ms < 1000) {
        volume_L_FEV1 = volume_L_FEV1 + volume_L;
      }

      // Display to serial
      // Serial.print(String(num_data_points));
      // Serial.print("\t t (sec): " + String(elapsed_time_s));
      // Serial.print("\t Vdot (L/s): " + String(volume_dot_Ls));
      // Serial.print("\t FEV1 (L): " + String(volume_L_FEV1));
      // Serial.print("\t FVC (L): " + String(volume_L_FVC));
      // Serial.print("\n");

      // OR display to serial plotter as Casey requested
      //Serial.println(volume_dot_Ls*1000);
    }

    // EXIT CONDITIONS

    // exit if time is done
    // update the countdown
    if (elapsed_time_ms < (count_down_duration_ms + delay_offset_ms)) {
      timeRemaining = count_down_duration_ms - elapsed_time_ms;
      displayTime(timeRemaining);
    } 
    else {
      lcd.setCursor(0, 1);
      lcd.print("Time's up!      ");

      delay(1000);
      display_results(volume_L_FEV1, volume_L_FVC, num_data_points);
      break;
    }

    // exit if button press
    if (digitalRead(buttonPin) == LOW) {
      delay(200); // Debounce
      lcd.setCursor(0, 1);
      lcd.print("Test Interrupt  ");
      display_results(volume_L_FEV1, volume_L_FVC, num_data_points);
      break;
    }
  }
}


// triangular integration
float integrate_volume(float volume_dot_prev, float volume_dot, float dt_sec) {
  // trapezoidally integrating using equally spaced panels
  // https://en.wikipedia.org/wiki/Trapezoidal_rule subsection "Numerical Implementation / Uniform Grid"
  
  float this_volume;
  this_volume = (0.5*dt_sec)*(volume_dot + volume_dot_prev);
  return this_volume;
}


// final output of test
void display_results(float FEV1, float FVC, int num_data_points) {

  float ratio = FEV1 / FVC;

  // Serial results
  Serial.print("Test Results---- \n");
  Serial.print("FEV1 (L): " + String(FEV1) + "\n");
  Serial.print("FVC  (L): " + String(FVC) + "\n");
  Serial.print("FEV1/FVC:   " + String(ratio) + "\n");
  Serial.print("num data pts: " + String(num_data_points) + "\n");
  
  // LCD results
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("FEV1 (L): " + String(FEV1));
  
  lcd.setCursor(0,1);
  lcd.print("FVC  (L): " + String(FVC));

  lcd.setCursor(0,2);
  lcd.print("FEV1/FVC:   " + String(ratio));

  lcd.setCursor(0,3);
  lcd.print("num data pts: " + String(num_data_points));

  while (digitalRead(encoder_SW) == HIGH) {
    // do nothing, wait for button press to go to next screen
  }
}


// show a countdown to the screen
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


// Menu Item 3 (stretch goal)
void sysInfo() {
  Serial.println("System info executed");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SysInfo...");
  lcd.setCursor(0,1);
  lcd.print("placeholder...");
  delay(3000);
}


// Menu Item 4 (stretch goal)
void calibrate() {
  Serial.println("calibration executed");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Calibrating...");
  lcd.setCursor(0,1);
  lcd.print("placeholder...");
  delay(3000);
}