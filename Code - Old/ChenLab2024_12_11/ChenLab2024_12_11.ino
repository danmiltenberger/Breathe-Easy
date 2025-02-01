#include <LCD_I2C.h>


//double LCD_Result = 0;
float MinPressure = 0;
float MaxPressure = 0;
int LoopCount = 0;


// Create instance of the LCD class
LCD_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
// VCC = power
// GND = ground
// SCA = SCA pin next to digital PWM
// SCK = SKC pin next to digital PWM




void setup() {
  
  lcd.begin();  // initialize the lcd 
  lcd.clear();  
  lcd.backlight();

  // serial connection
  Serial.begin(9600);
  
  
  pinMode(HX710_OUT, INPUT);   // Connect HX710 OUT to Arduino pin 2
  pinMode(HX710_SCK, OUTPUT);  // Connect HX710 SCK to Arduino pin 3

  lcd.setCursor(0,0);
  lcd.print("kPa = ");
  lcd.setCursor(0,1);
  lcd.print("Min=     Max=   ");
}


void loop() {
  long sensor1_val = readSensor(HX710_OUT,HX710_SCK);

  

  // pulse the clock line 3 times to start the next pressure reading
  //min 0 max 16777215
  for (char i = 0; i < 3; i++) {
    digitalWrite(3, HIGH);
    digitalWrite(3, LOW);
  }


  double temp = result*1000;
  double convertedresult = result/16777.215;
  convertedresult = (convertedresult-531)/34.1;

  // display pressure
  Serial.print("Pressure Reading - ");
  Serial.println(convertedresult);

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


void write_values_LCD(float val1, float val2) {
   lcd.clear();

   // value 1
   lcd.setCursor(0,0);
   lcd.WriteToDisplay("Val 1: ")
   lcd.setCursor(6,0);
   lcd.WriteToDisplay(val1);

   // value 2
   lcd.setCursor(0,1);
   lcd.WriteToDisplay("Val 2: ")
   lcd.setCursor(6,1);
   lcd.WriteToDisplay(val2);
}
