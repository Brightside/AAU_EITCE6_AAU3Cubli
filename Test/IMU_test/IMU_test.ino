#include <Adafruit_MPU6050.h>  // https://github.com/adafruit/Adafruit_MPU6050/blob/master/Adafruit_MPU6050.h
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 MPU1;
Adafruit_MPU6050 MPU2;
unsigned long currentTime = 0;  // variable to store the current time
const float Pi = 3.14159;

void setup(void) {
  Serial.begin(115200);
  // Initialize IMUs!
  if (!MPU1.begin() || !MPU2.begin(0x69)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  ////////// IMU1 setup //////////
  // set accelerometer range to +-8g (default is 2g)
  MPU1.setAccelerometerRange(MPU6050_RANGE_8_G);
  // set gyro range to +- 500 deg/s (default is 250 deg/s)
  MPU1.setGyroRange(MPU6050_RANGE_500_DEG);
  // set filter bandwidth to 21 Hz
  MPU1.setFilterBandwidth(MPU6050_BAND_21_HZ);

  ////////// IMU2 setup //////////
  // set accelerometer range to +-8g (default is 2g)
  MPU2.setAccelerometerRange(MPU6050_RANGE_8_G);
  // set gyro range to +- 500 deg/s (default is 250 deg/s)
  MPU2.setGyroRange(MPU6050_RANGE_500_DEG);
  // set filter bandwidth to 21 Hz
  MPU2.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  currentTime = millis();  // get the current time
  // Get IMU1 sensor data
  sensors_event_t a1, g1, temp1;  // Acceleration is in m/s^2 and Rotation is in rad/s
  MPU1.getEvent(&a1, &g1, &temp1);
  // Use atan2 else it cant read the negativ side properly
  float theta_f1 = (-atan2(a1.acceleration.x, a1.acceleration.y) * (180 / PI)) + 45;  // calculate the angle in degrees
  float omega_gyro1 = -g1.gyro.z;
  
  // Get IMU2 sensor data
  sensors_event_t a2, g2, temp2;  // Acceleration is in m/s^2 and Rotation is in rad/s
  MPU2.getEvent(&a2, &g2, &temp2);
  // Use atan2 else it cant read the negativ side properly
  float theta_f2 = (-atan2(a2.acceleration.y, -a2.acceleration.x) * (180 / PI)) + 45;  // calculate the angle in degrees
  float omega_gyro2 = -g2.gyro.z;

  Serial.print(currentTime);
  Serial.print(";");  // For excel so it can seperate the values
  Serial.print(theta_f1);
  Serial.print(";");  // For excel so it can seperate the values
  Serial.print(theta_f2);
  Serial.print(";");  // For excel so it can seperate the values
  Serial.print(omega_gyro1);
  Serial.print(";");  // For excel so it can seperate the values
  Serial.println(omega_gyro2);
}