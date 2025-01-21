#include <LCD_I2C.h>

// Initialize the LCD object with I2C address, columns, and rows
LCD_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 column and 2 rows


// Define pins for the rotary encoder
const int CLK = 2;  // Clock pin
const int DT = 3;   // Data pin
const int SW = 4;   // Switch pin (button)

// Variables for encoder handling
volatile int encoderValue = 0;  // Current encoder value, volatile for interrupt usage
volatile unsigned long lastInterruptTime = 0;  // Last time the interrupt was triggered
const unsigned long debounceTime = 1; // 1ms debounce time for encoder

// Menu variables
int menuItem = 0;  // Current selected menu item
const int numItems = 3;  // Total number of menu items
String menuItems[numItems] = {"Function 1", "Function 2", "Function 3"};  // Array of menu items

void setup() {
  // Initialize the LCD
  lcd.begin();
  lcd.backlight();
  
  // Set up encoder pins as inputs with pull-up resistors
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  pinMode(SW, INPUT_PULLUP);
  
  // Attach interrupt to CLK pin for both rising and falling edges
  attachInterrupt(digitalPinToInterrupt(CLK), handleEncoder, CHANGE);
  
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Initial display update
  updateDisplay();
}

void loop() {
  static int lastEncoderValue = 0;  // Store the last encoder value
  
  // Check if encoder value has changed
  if (encoderValue != lastEncoderValue) {
    // Calculate the change and update menu item
    int change = encoderValue - lastEncoderValue;
    menuItem = (menuItem + change + numItems) % numItems;  // Wrap around menu items
    lastEncoderValue = encoderValue;  // Update last encoder value
    updateDisplay();  // Update the LCD display
  }
  
  // Check if button is pressed (active low)
  if (digitalRead(SW) == LOW) {
    selectFunction();  // Call function to handle selection
    delay(300);  // Debounce delay for button press
  }
}

// Interrupt Service Routine for encoder
void handleEncoder() {
  unsigned long interruptTime = millis();  // Get current time
  // Check if enough time has passed since last interrupt (debounce)
  if (interruptTime - lastInterruptTime > debounceTime) {
    // Determine direction based on the state of CLK and DT pins
    if (digitalRead(DT) != digitalRead(CLK)) {
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
  lcd.print("> " + menuItems[menuItem]);  // Display current menu item with selector
  lcd.setCursor(0, 1);  // Set cursor to second row
  lcd.print(menuItems[(menuItem + 1) % numItems]);  // Display next menu item
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
      function1();
      break;
    case 1:
      function2();
      break;
    case 2:
      function3();
      break;
  }
  
  delay(2000);  // Display selection for 2 seconds
  updateDisplay();  // Return to menu display
}

// Example functions to be executed when selected
void function1() {
  Serial.println("Function 1 executed");
}

void function2() {
  Serial.println("Function 2 executed");
}

void function3() {
  Serial.println("Function 3 executed");
}
