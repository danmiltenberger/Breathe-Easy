float area6 = 0;
float area1 = 0;

float area1_exp = 113/12;
float area6_exp = 54;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  float x = 0;
  float y;
  float dx = 0.1;


  while (x < 6) {
    y = pretend_sensor(x);
    area6 = area6 + y*dx;

    if (x < 1) {
      area1 = area1 + y*dx;
    }

    Serial.print("x: " + String(x) + "\t y: " + String(y) + "\t area6: " + String(area6) + "\n");

    x = x + dx;

  }

  Serial.print("area1 expected:" + String(area1_exp) + " \n");
  Serial.print("area1 actual:  " + String(area1) + " \n");
  Serial.print("area1 abs dif: " + String(abs(area1 - area1_exp)) + " \n");

  Serial.print("area6 expected:" + String(area6_exp) + " \n");
  Serial.print("area6 actual:  " + String(area6) + " \n");
  Serial.print("area6 abs dif: " + String(abs(area6 - area6_exp)) + " \n");

}

void loop() {


}


float pretend_sensor(float x) {
  float y;
  y = -sq(0.5*x-1) + 10;
  return y;
}