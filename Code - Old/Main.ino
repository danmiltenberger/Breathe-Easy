#include <LCD_I2C.h>

// Initialize the LCD object with I2C address, columns, and rows
LCD_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 column and 2 rows


// Define pins for the rotary encoder
const int encoder_CLK = 9;  // Clock pin
const int encoder_DT = 10;   // Data pin
const int encoder_SW = 11;   // Switch pin (button)

// Define pins for sensor
const int HX710b_OUT = 2;
const int HX710b_SCK = 3;

// Variables for encoder handling
volatile int encoderValue = 0;  // Current encoder value, volatile for interrupt usage
volatile unsigned long lastInterruptTime = 0;  // Last time the interrupt was triggered
const unsigned long debounceTime = 1; // 1ms debounce time for encoder

// Menu variables
int menuItem = 0;  // Current selected menu item
const int numItems = 4;  // Total number of menu items
String menuItems[numItems] = {"1. Read Sensors", "2. Calibrate", "3. Sys Info", "4. Run Test"};  // Array of menu items


// Pressure Sensor setup
#define sensor_yellow_out 5 
#define sensor_orange_out 6 
#define sensor_sck 7 // blue

void setup() {
  // Initialize the LCD
  lcd.begin();
  lcd.backlight();
  
  // Set up encoder pins as inputs with pull-up resistors
  pinMode(encoder_CLK, INPUT_PULLUP);
  pinMode(encoder_DT, INPUT_PULLUP);
  pinMode(encoder_SW, INPUT_PULLUP);


  // Setup sensors
  pinMode(HX710b_OUT, INPUT);   // Connect HX710 OUT to Arduino pin 2
  pinMode(HX710b_SCK, OUTPUT);  // Connect HX710 SCK to Arduino pin 3
  

  // Attach interrupt to encoder_CLK pin for both rising and falling edges
  attachInterrupt(digitalPinToInterrupt(encoder_CLK), handleEncoder, CHANGE);
  
  // Initialize serial communication for debugging
  Serial.begin(9600);

  lcd.print("Upload complete");
  delay(500);
  Serial.print("Upload complete");
  
  // Initial display update
  updateDisplay();

  // Place any program to run directly below
  // TestSensors(HX710b_OUT, HX710b_SCK);
}

void loop() {
  menu();
}


void menu() {
  static int lastEncoderValue = 0;  // Store the last encoder value
  
  // Check if encoder value has changed
  if (encoderValue != lastEncoderValue) {
    // Calculate the change
    int change = encoderValue - lastEncoderValue;

    Serial.print("Encoder rotated!");
    
    // Update menuItem, but constrain it to valid range
    menuItem = constrain(menuItem + change, 0, numItems - 1);
    
    lastEncoderValue = encoderValue;  // Update last encoder value
    updateDisplay();  // Update the LCD display
  }
  
  // Check if button is pressed (active low)
  if (digitalRead(encoder_SW) == LOW) {
    Serial.print("Encoder pressed!");
    selectFunction();  // Call function to handle selection
    delay(300);  // Debounce delay for button press
  }
}



// Interrupt Service Routine for encoder
void handleEncoder() {
  unsigned long interruptTime = millis();  // Get current time
  // Check if enough time has passed since last interrupt (debounce)
  if (interruptTime - lastInterruptTime > debounceTime) {
    // Determine direction based on the state of encoder_CLK and encoder_DT pins
    if (digitalRead(encoder_DT) != digitalRead(encoder_CLK)) {
      encoderValue++;  // Clockwise rotation
    } else {
      encoderValue--;  // Counter-clockwise rotation
    }
    lastInterruptTime = interruptTime;  // Update last interrupt time
  }
}

// Function to update the LCD display
void updateDisplay() {
  lcd.clear();  // Clear the LCD
  lcd.setCursor(0, 0);  // Set cursor to first row
  lcd.print(menuItems[menuItem]);  // Display current menu item with selector
  lcd.setCursor(13,0);
  lcd.print("<--");
  
  // Display next menu item if it exists
  if (menuItem < numItems - 1) {
    lcd.setCursor(0, 1);  // Set cursor to second row
    lcd.print(menuItems[menuItem + 1]);
  }
}

// Function to handle menu item selection
void selectFunction() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Selected:");
  lcd.setCursor(0, 1);
  lcd.print(menuItems[menuItem]);
  
  // Execute the selected function based on menu item
  switch (menuItem) {
    case 0:
      TestSensors(HX710b_OUT, HX710b_SCK);
      break;
    case 1:
      Calibrate();
      break;
    case 2:
      SysInfo();
      break;
    case 3:
      RunTest();
      break;
  }
  
  delay(2000);  // Display selection for 2 seconds
  updateDisplay();  // Return to menu display
}


// https://swharden.com/blog/2022-11-14-hx710b-arduino/ 


// simply print the output of the sensors, without integrating or anything like that
void TestSensors(int HX710b_OUT, int HX710b_SCK) {

  // begin infinite loop
  while (true) {
    Serial.println("Reading sensor:");

    // wait for the current reading to finish
    while (digitalRead(HX710b_OUT)) {}

    // read 24 bits
    long result = 0;
    for (int i = 0; i < 24; i++) {
      digitalWrite(HX710b_SCK, HIGH);
      digitalWrite(HX710b_SCK, LOW);
      result = result << 1;
      if (digitalRead(HX710b_OUT)) {
        result++;
      }
    }

    // get the 2s compliment
    result = result ^ 0x800000;

    // pulse the clock line 3 times to start the next pressure reading
    for (char i = 0; i < 3; i++) {
      digitalWrite(HX710b_SCK, HIGH);
      digitalWrite(HX710b_SCK, LOW);
    }

    // display pressure
    Serial.println(result);
    lcd.print(result);
  }
}


float pretend_sensor(float elapsed_time)
{
  // placeholder
  return 1;
}


void Calibrate() {
  Serial.println("calibration executed");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Calibrating...");
  lcd.setCursor(0,1);
  lcd.print("placeholder...");
  delay(3000);
}


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

  while (digitalRead(encoder_SW) == HIGH) {
    // do nothing, wait for button press to go to next screen
  }

  // screen 2
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("FEV1/FVC:   " + String(ratio));

  lcd.setCursor(0,1);
  lcd.print("num data pts: " + String(num_data_points));
  delay(2000);

  while (digitalRead(encoder_SW) == HIGH) {
    // do nothing, wait for button press to go to next screen
  }
}


void SysInfo() {
  Serial.println("System info executed");

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SysInfo...");
  lcd.setCursor(0,1);
  lcd.print("placeholder...");
  delay(3000);
}

