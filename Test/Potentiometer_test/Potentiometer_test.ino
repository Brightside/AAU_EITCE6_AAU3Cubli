int potPin = 1;                  // potentiometer input pin
int val = 0;                     // store value coming from the sensor
unsigned long currentTime = 0;   // variable to store the current time

void setup() {
  Serial.begin(115200);
}

void loop() {
  int val = analogRead(potPin);
  float voltage = val * (3.3 / 1023); // calculate voltage
  float theta_f = -(90 / 3.3) * voltage + 45; // calculate angle in degrees
  currentTime = millis();  // get the current time

  Serial.print(currentTime);
  Serial.print(";"); // For excel so it can seperate time and angle
  Serial.println(theta_f);
}