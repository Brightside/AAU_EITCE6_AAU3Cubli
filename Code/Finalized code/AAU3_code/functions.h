// Required libraries
//#include <Adafruit_MPU6050.h>   // https://github.com/adafruit/Adafruit_MPU6050/blob/master/Adafruit_MPU6050.h
#include <Adafruit_Sensor.h>    // IMU programmer library
#include <Wire.h>               // I2C communicator for IMUs
#include <Servo.h>              // Library for directly controlling servo
#include <SAMD21turboPWM.h>     // PWM generator for high resolution motor
// PROGRAM DEFINITIONS
#define PI 3.141592
// Motor codes
#define HIGH_RES 1
#define LOW_RES 2
// Filtering codes
#define COMPLEMENTARY 1
#define KALMAN 2
// Eror codes
#define IMUS_FAILED 1
#define WRONG_RESOLUTION_MODE 2
#define EXCEEDED_TIME 3

// ------------------------------------------------------------------------------------------------------------------------------------------
// ##############################################################
// # INSTRUCTIONS OF USE                                        #
// ##############################################################
// In functions.h:
// Specify desired parameters under TUNABLE VARIABLES
// In setup():
//  - Initialize the whole system with InitSystem() or use the different InitXXXX() for the required parts
//  - At the end of the funciton use StartTiming() for starting the clock cycle
// In loop():
// - Use updateMeasurements() to obtain current loop's state values in SI units(frame angle (rads), frame speed (rads/s), flywheel speed(rad/s))
// - Use JumpUpIfFallen() to activate the jump up feature. Function refreshes the state variables with measurements when finised if fallen (no need to restart loop).
// - Use angleEstimatorWithTorque() to correct the center of mass of the system dinamically. Its returned value must be added to the frame angle measurement.
// - Use  StateSpaceController() to obtain the torque to apply in Si units (Nm).
// - Use SetMotorTorque() to apply the specified torque.
// - At the end of function use WaitNextRound() to maintain the desired frequency.

// TUNABLE VARIABLES-------------------------------------------------------------------------------------------------------------------------
#define DEBUG false                 // Debug mode: prints information on screen
#define WORKING_ANG 15*PI/180       // º From which the controller can stabilize. If exceed, it jumps
#define RESOLUTION_MODE  HIGH_RES   // Can be 1 (HIGH_RES-7.5A) or 2(LOW_RES-4A). The driver limits must be changed as well (if incorrect mode is chosen, nothing will blow up but it wont work nicely)
#define STOP_IF_TIME_EXCEEDED true  // Stops the system if the time of the loop time exceeds the allowed one (system can't reach desired frequency)
#define FILTERING_MODE KALMAN       // Chooses which mode to filter the angle emasurement with the IMU mode
#define LOOP_FREQ  50               // Frequency of system in Hertzs
#define INIT_ANGLE_GUESS 0/180*PI   // Initial center of mass estimation. Stored in untis
//------------------------------------------------------------------------------------------------------------------

// Timing variables
unsigned long LOOP_MICROS = (unsigned long) (1000000./LOOP_FREQ); // microseconds of a loop with the desired frequency
unsigned long last_time;   // Variable to keep track of elapsed time

// Initialization for IMUs
//Adafruit_MPU6050 MPU1;
//Adafruit_MPU6050 MPU2;

// High resolution motor stuff
struct timerStruct{
  const int enbPin = 1;       // Pin used for enabling motor (HIGH active)
  const int PWMPin = 5;       // Pin used for PWM signal
  const int TIMR = 0;         // Specific timer needed for the desired pin (needed by used library PWM.h)
  const int CLKDiv_1 = 15;    // Dividers needed for frequency specification :
  const int CLKDiv_2 = 15;    // PWM frequency is 3.2kHz
  const int aauMinCyc = 0;    //  (duty cycle: 0) This and next one needed for definition of duty cycle, do not touch
  const int aauMaxCyc = 1000; //  (duty cycle: 1000*20)
  TurboPWM aau;               // Object from PWM.h
} MOTOR_HIGH_RES_DATA;

// Kalman filter variables
struct {
float Q_angle = 0.001*PI/180;
float Q_gyroBias = 0.003*PI/180;
float R_measure = 0.03;
float dt = 1.0/LOOP_FREQ;
float angle = 0;
float ang_speed = 0;
} kalman_filter;
// PINS DEFINITIONS
#define POT_PIN     A1
#define SPEED_PIN   A5
#define EMG_LED     LED_BUILTIN
#define SERVO_PIN   2
#define READ_SWITCH_PIN  4
#define POWER_SWITCH_PIN  3


struct motor_pins{
    int PWM_PIN = 5;        // PWM output pin
    int ENB_PIN = 1;        // ENABLE pin
} MOTOR_LOW_RES_DATA;         // Motor pin layout
#define MAX_MOTOR_CURRENT 4
#define MAX_MOTOR_CURRENT_HIGH_RES 7.5
Servo servo;
#define BRAKE_ANG 104    // º
#define RESET_ANG 135   // º

// ####################################################################################################################################################
// Initialization--------------------------------------------------------------------------------------------

void InitSystem();   
  // Initializes everything needed for the program to start running.
    // Motor+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-  
    void InitMotor();
      // Sets a PWM of 0 and after 2 seconds enables the motor.
    // Alternative motor with high res----------------------
    void InitMotorHighRes();
      // Initalizes the motor, with a total of 800 possible values
    void TurnOffMotor();
      // Deactivatees motor enable pin
    void TurnOnMotor();
      // Activates motor enable pin
    //+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
    void InitServo();
      // Initializes servo.
    void InitIMUsNew();
    //bool initIMUs();
      // Starts IMUs. 
    void ErrorManager(int code);
      // called when something in the system fails. Prints error and blocks into BlinkLed().
      void EmergencyStop(); 
        // Shuts down actuators instantly
      void BlinkLed(int numPulsesOn, int numPulsesOff, int time_of_blink);
        // Makes EMG_LED blink. Locks system in infinte loop. time_of blink on, time of blink off; blinks on numPulsesOn times and then numPulsesOff without blinking
      
void StartTiming(); 
  // Starts the loop timing

// Running---------------------------------------------------------------------------------------------------
void ServoBrake();
  // It also deactivates the motor.
void ServoReset();
  // It activates the motor.


void UpdateMeasurements(float* measured_values);
  // Updates all measured values. Reads Switch value to decide angle computation.
    void updateIMUReadings();
      // Updates the values measured by thw IMUs. Call before computing the new angle. Also needed for computing the frame speed.
    float FrameAngleIMUs();
      // Measure angle with IMUs (to be improved, only uses accelerometers)
        float kalman(float newAngle, float newRate);
          // Computes the kalman filtered angle. Requires a new angle measurements from the accelerometer and the speed of the frame from the gyroscope.
    float getFrameAngle();
      // Reads pot and gives back frame angle in rad
    float getWheelSpeed();
      // Obtains the current relative angular velocity of the motor
    //float getFrameSpeed();
      // Returns a mesaurement of the frame speed. Positive falling right.


float EstimatorPointFallingTorque(float measured_w);
// uses steady state torque estimation to find center of mass.
  float* initTimeArray(int size);
    // Creates a float array of "size" elements and initializes them to 0. The newest value is at [0] up to the latest at [size-1]
  void advanceTimeArray(float* array, int size);
    // Advances the array and initializes new value at 0. [0] into [1], [1] into [2]...
  float* discretizePI(float z, float kp);
    // Discretizes a controller of the type kp*(s+z)/s using tustin's method to the frequency LOOP_FREQ

float StateSpaceController(float* measured_values);
  // Controller using state space equations. Takes in measured values: frame angle, frame speed, wheel speed; and reference(in radians)
    float inner_product(float* vector1, float* vector2, int size);
      // Computes inner product of system


void SetMotorTorque(float torque_to_apply);
  // Sets PWM to equivalent current. Uses RESOLUTION_MODE to decide what metho of motor control to use.
  // I = torque_to_apply/Kt;
  // PWM: 10%~90% (3.3V) 
    int TorqueToPWMDuty(float torque_to_apply);
      // Translates from Torque to apply to the duty of the PWM. Note that all limiters are defined there. Contains limiter.
      //  - Max current = 4
      //  - PWM: 10%~90% (3.3V)
    float TorqueToPWMDutyHighRes(float torque_to_apply);
      // Translates from Torque to apply to the duty of the PWM. Note that all limiters are defined there. Contains limiter.
      //  - Max current = 7.5
      //  - PWM: 10%~90% (3.3V)
    void setDutyHighRes(float duty);
      //Sets the PWM equal to a 0-1 float
        void setAAU(const int pin, int cycle);
          //Setting the duty cycle equal to an input, with interval minmax
        
//--------------------------------------------------------------
void WaitNextRound();   
  // Waits for the specified global time to elapse
    unsigned long TimeDiffMicros(unsigned long newest, unsigned long oldest);  
      // Calculates the time difference accounting for overflow.
bool JumpUpIfFallen(float* measured_values, float stable_point);
  // Takes in the measured values. Jump up is safely performed.
    void JumpUp(int direction);
      // Performs the jump until a satisfactory one happens (cube enters the abs(angle_Frame)<WORKING_ANGLE) region
void printState(float* measured_values , float stable_point, float torque_to_apply);
  //Prints the state of the system in screen. Ignores DEBUG flag.


// ##########################################################################################################################
// System functions


void InitSystem(){  
  // Debugging ---------------------------------
  if(DEBUG){
    Serial.begin(115200);
    while(!Serial);
  }
  if(DEBUG){
    Serial.println("Initializing system:");
  }
  //Servo----------------------------------------
  if(DEBUG){
    Serial.print("Servo.......................");

  }
  InitServo();
  if(DEBUG){
    Serial.println("ok");
  }
  ServoBrake();


  // Switch ------------------------------------
  pinMode(READ_SWITCH_PIN,INPUT);
  pinMode(POWER_SWITCH_PIN,OUTPUT);
  // Emergency led------------------------------
  pinMode(EMG_LED,OUTPUT);
  digitalWrite(EMG_LED,HIGH); //Led is low_active
  
  
  //Motor----------------------------------------
  InitMotor();
  if(DEBUG){
    Serial.print("Motor.......................");
    
  }
  InitMotor();
  if(DEBUG){
    Serial.println("ok");
  }

  
  
  // IMUS----------------------------------------
  if(DEBUG){
    Serial.print("IMUs........................");
  }
  InitIMUsNew();
  /**
  if(initIMUs()){
    ErrorManager(IMUS_FAILED);
  }
  **/
  if(DEBUG){
    Serial.println("ok");
  }
  ServoReset();
}



void InitMotor(){
  //Set enable pin
  pinMode(MOTOR_LOW_RES_DATA.ENB_PIN,OUTPUT);
  TurnOffMotor();
  // Set PWM pins
  pinMode(MOTOR_LOW_RES_DATA.PWM_PIN,OUTPUT);
  pinMode(SPEED_PIN,INPUT);

  
  if(RESOLUTION_MODE == LOW_RES){
    SetMotorTorque(0);
    delay(2000);
  }else{
    if(RESOLUTION_MODE != HIGH_RES){
    ErrorManager(WRONG_RESOLUTION_MODE);
    }
    
    // Set PWM data
    MOTOR_HIGH_RES_DATA.aau.setClockDivider(MOTOR_HIGH_RES_DATA.CLKDiv_1, false);  // clock is divided by 15. Turbo is off/false
    MOTOR_HIGH_RES_DATA.aau.timer(MOTOR_HIGH_RES_DATA.TIMR,MOTOR_HIGH_RES_DATA.CLKDiv_2, 1000, true);  // Timer 0 is set to Generic Clock divided by 15, resolution is 1000, single-slope PWM
    SetMotorTorque(0);
    delay(6000);
  }

  TurnOnMotor();
}


void TurnOffMotor(){
  digitalWrite(MOTOR_HIGH_RES_DATA.enbPin,LOW);
}


void TurnOnMotor(){
  digitalWrite(MOTOR_HIGH_RES_DATA.enbPin,HIGH);
  }


void InitServo(){
  servo.attach(SERVO_PIN);
  servo.write(RESET_ANG);
}
/**
bool initIMUs(){
  bool failed = false;
  if (!MPU1.begin()&& DEBUG){
    Serial.println("\nIMU1 failed.");
    failed = true;
  }
  if (!MPU2.begin(0x69)&& DEBUG){
    Serial.println("\nIMU2 failed.");
    failed = true;
  }
  if (failed) {
    return true;
  }
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
  return false;
}
**/
void ErrorManager(int code){
  EmergencyStop();
  switch(code){
    case IMUS_FAILED:
    Serial.println("ERROR: The IMUs could not be initialized corretly.");
    BlinkLed(4,0,125);
    break;
    case WRONG_RESOLUTION_MODE:
    Serial.println("ERROR: The selected resolution mode is not valid.");
    BlinkLed(3,1,125);
    break;
    case EXCEEDED_TIME:
    BlinkLed(2,1,125);
    break;
  }
  
}

void EmergencyStop(){
      // Deactivate motor
      TurnOffMotor();
      SetMotorTorque(0);
}

void BlinkLed(int numPulsesOn, int numPulsesOff, int time_of_blink){
  while(true){
    for(int i = 0; i < numPulsesOn; i++){
      digitalWrite(EMG_LED,LOW);
      delay(time_of_blink);
      digitalWrite(EMG_LED,HIGH);
      delay(time_of_blink);
    }
    for(int i = 0; i < numPulsesOff; i++){
      digitalWrite(EMG_LED,HIGH);
      delay(2*time_of_blink);
    }
  }
}


void StartTiming(){
  last_time = micros();
}


void ServoBrake(){
  // Security: stop motor before  braking
  TurnOffMotor();
  servo.write(BRAKE_ANG);
}

void ServoReset(){
  // Security: reactivate motor after releasing
  servo.write(RESET_ANG);
  TurnOnMotor();
}

float RateRoll, RatePitch, RateYaw;
float GyroRoll, GyroPitch;
float GyroX_bias = -1.22, GyroY_bias = 1.18, GyroZ_bias = 0.69;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float AccX_bias = 0.01, AccY_bias = 0, AccZ_bias = -0.11;

void UpdateMeasurements(float* measured_values){
  updateIMUReadings();
  float extra = FrameAngleIMUs();
  if(digitalRead(READ_SWITCH_PIN) == HIGH){
    measured_values[0] = extra;
  }else{
    measured_values[0]  = getFrameAngle();              
  }
  
  measured_values[1]  = kalman_filter.ang_speed;
  measured_values[2]  = getWheelSpeed();
  
}

#define IMU1_ADDR 0x68 //0x69 for IMU2
void InitIMUsNew(){
  Wire.setClock(400000); //From MPU6050 product spec sheet
  Wire.begin();
  delay(250);
  //Power up the IMU
  Wire.beginTransmission(IMU1_ADDR); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  //Enable 5 Hz bandwidth low-pass filter for the small vibrations and keep gyro at 1kHz
  Wire.beginTransmission(IMU1_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission(); 
  //Set the gyro sensitivity scale factor at 250 deg/s
  Wire.beginTransmission(IMU1_ADDR);
  Wire.write(0x1B); 
  Wire.write(0x00);
  Wire.endTransmission();
  //Set the accel to +-2g
  Wire.beginTransmission(IMU1_ADDR);
  Wire.write(0x1C); 
  Wire.write(0x00);
  Wire.endTransmission();  
}


void updateIMUReadings() {
  
  //Write to the gyro measurement registers
  Wire.beginTransmission(IMU1_ADDR);
  Wire.write(0x43);
  Wire.endTransmission();
  //Request for 6 bytes
  Wire.requestFrom(IMU1_ADDR,6);
  //Gyro is a 16-bit complement value
  int16_t raw_gyroX=Wire.read()<<8 | Wire.read(); // You have to merge the two gyro registers
  int16_t raw_gyroY=Wire.read()<<8 | Wire.read();
  int16_t raw_gyroZ=Wire.read()<<8 | Wire.read();
  //Convert the gyro values to deg/s
  RateRoll=(float)raw_gyroX/131 - GyroX_bias; // 131 is the LSB sensitivity for 250deg/s
  RatePitch=(float)raw_gyroY/131 - GyroY_bias;
  RateYaw=(float)raw_gyroZ/131 - GyroZ_bias;

 
  //Write to the accel measurement registers
  Wire.beginTransmission(IMU1_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission();
  //Request for 6 bytes
  Wire.requestFrom(IMU1_ADDR,6);
  //Accel is a 16-bit complement value
  int16_t raw_accelX=Wire.read()<<8 | Wire.read(); // You have to merge the two accel registers
  int16_t raw_accelY=Wire.read()<<8 | Wire.read();
  int16_t raw_accelZ=Wire.read()<<8 | Wire.read();
  //Convert the accel values to g
  AccX=(float)raw_accelX/16384 - AccX_bias; // 16384 is the LSB sensitivity for 2g
  AccY=(float)raw_accelY/16384 - AccY_bias;
  AccZ=(float)raw_accelZ/16384 - AccZ_bias;

  kalman_filter.ang_speed = -RateYaw*PI/180;
  kalman_filter.angle = atan(AccY / sqrt(AccX*AccX + AccZ*AccZ)) -44.5/180*PI; // -pi/4 to pi/4 rad
}



float FrameAngleIMUs(){
  static float compAngle = 0;
  static float kalmanAngle = 0;

  if(FILTERING_MODE == KALMAN){
    kalmanAngle = kalman(kalman_filter.angle,kalman_filter.ang_speed);
    return kalmanAngle;
  }
  if(FILTERING_MODE == COMPLEMENTARY){
    compAngle = 0.02 * kalman_filter.angle + 0.98 * (compAngle +kalman_filter.ang_speed * kalman_filter.dt);
    return compAngle;
  }
  Serial.print("Incorrect filtering mode.\n\n\n");
  exit(0);
}

float kalman(float newAngle, float newRate){
  static float P[2][2] = {{0,0},{0,0}};
  static float rate = 0;
  static float bias = 0; // gyro bias calculated by the Kalman filter
  static float angle = 0; //angle calculated by the Kalman filter
  //equation 1:
  rate = newRate - bias;
  angle += kalman_filter.dt * rate;
  //equation 2:
  P[0][0] += kalman_filter.dt * (kalman_filter.dt*P[1][1] - P[0][1] - P[1][0] + kalman_filter.Q_angle);
  P[0][1] -= kalman_filter.dt * P[1][1];
  P[1][0] -= kalman_filter.dt * P[1][1];
  P[1][1] += kalman_filter.Q_gyroBias * kalman_filter.dt;
  //equation 3:
  float y = newAngle - angle;
  //equation 4:
  float S = P[0][0] + kalman_filter.R_measure;
  //equation 5:
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;
  //equation 6:
  angle += K[0] * y;
  bias += K[1] * y;
  //equation 7:
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}

float getFrameAngle(){
  int value = analogRead(POT_PIN);
  float angle = PI/4.0*(1-2.0*value/1024.0);
  //Serial.println(angle*180/PI);
  return angle;
}

float getWheelSpeed(){
  float measure = analogRead(SPEED_PIN);
  return -(measure/1024-0.5)*12000*PI/30;
}
/**
float getFrameSpeed(){
  static sensors_event_t a1, g1, temp1;
  static sensors_event_t a2, g2, temp2;


  MPU1.getEvent(&a1, &g1, &temp1);
  float omega_gyro1 =  (-g1.gyro.z);
  MPU2.getEvent(&a2, &g2, &temp2);
  float omega_gyro2 =  (-g2.gyro.z);



  return (omega_gyro1+omega_gyro2)/2;
}
**/
float* discretizePI(float z, float kp){
  float* arr = new float[2];

  arr[0] = kp*(z/2.0/LOOP_FREQ+1);
  arr[1] = kp*(z/2.0/LOOP_FREQ-1);
  return arr;
}


float EstimatorPointFallingTorque(float measured_w){    // Controller is discretized automatically following tustin's method. It requires a pole (s+z) and a Kp
  static float* w_err = initTimeArray(2);  
  static float* theta_cm = initTimeArray(2);
  static float* num_coeffs = discretizePI(0.01, 9.4406/10000);
  static float den_coeffs[2] = {0,1};


  advanceTimeArray(theta_cm,2);
  advanceTimeArray(w_err,2);

 
  w_err[0] = measured_w;
  
  theta_cm[0] =  inner_product(w_err,num_coeffs,2)+inner_product(theta_cm,den_coeffs,2);
  //theta_cm[0] = w_err[0]*num_coeffs[0] + w_err[1]*num_coeffs[1] + theta_cm[1];

  

  return theta_cm[0];
}

float* initTimeArray(int size){
  float* array = new float[size];
  for(int i = 0; i< size; i++){
    array[i] = 0;
  }
  return array;
}

void advanceTimeArray(float* array,int size){
  for(int i = size-2; i >= 0; i--){
      array[i+1] = array[i];
  }
  array[0] = 0;
}

float StateSpaceController(float *measured_values){
  static float controller_constants[3] =  {-2.1739,-0.2643, -0.00225};// angle err, speed frame, speed wheel
  // {-1.93980094296589,-0.210190099518124,-0.00225545126724467}
  static float max_torque = (MAX_MOTOR_CURRENT_HIGH_RES*(RESOLUTION_MODE == HIGH_RES)+MAX_MOTOR_CURRENT*(RESOLUTION_MODE == LOW_RES))*33.5/1000; // Chooses corrrect max torque. Kt = 0.0335

  float torque_to_apply = -inner_product(measured_values,controller_constants,3);// - to keep consistent with the directions

  if(abs(torque_to_apply)> max_torque){
    torque_to_apply = torque_to_apply/abs(torque_to_apply)*max_torque;    
  }


  return torque_to_apply;
}


float inner_product(float* vector1, float* vector2, int size){
  float product = 0;
  for(int i = 0; i < size; i++){
    product += vector1[i]*vector2[i];
  }
  return product;
}

void SetMotorTorque(float torque_to_apply){
  if(RESOLUTION_MODE == HIGH_RES){
    float duty = TorqueToPWMDutyHighRes(-torque_to_apply); // - to keep consistent with the directions
      setDutyHighRes(duty);
  }else{
    int duty = TorqueToPWMDuty(-torque_to_apply); // - to keep consistent with the directions
    analogWrite(MOTOR_LOW_RES_DATA.PWM_PIN,duty);
  }
}

int TorqueToPWMDuty(float torque_to_apply){
  static float Kt =  33.5/1000; // Nm/A
  static float limits =  MAX_MOTOR_CURRENT; // A

  // Compute curernt to apply
  float current = torque_to_apply/Kt;
  // Limit to allowable values
  if(abs(current) > limits){
    current = current/abs(current)*limits; 
  }

  // map to duty
  float duty = (current + limits) * (255*0.9-255*0.1) / (limits + limits) + 255*0.1;

  return (int)duty;
}



void setDutyHighRes(float duty) {
  setAAU(MOTOR_HIGH_RES_DATA.PWMPin, (int)floor(duty * 1000));
}


void setAAU(const int pin, int cycle) {
  if (cycle < MOTOR_HIGH_RES_DATA.aauMinCyc) {
    cycle = MOTOR_HIGH_RES_DATA.aauMinCyc;
  }
  if (cycle > MOTOR_HIGH_RES_DATA.aauMaxCyc) {
    cycle = MOTOR_HIGH_RES_DATA.aauMaxCyc;
  }
  MOTOR_HIGH_RES_DATA.aau.analogWrite(pin, cycle);
};

float TorqueToPWMDutyHighRes(float torque_to_apply){
  static float Kt =  33.5/1000; // Nm/A
  static float limits =  MAX_MOTOR_CURRENT_HIGH_RES; // A

  // Compute curernt to apply
  float current = torque_to_apply/Kt;
  // Limit to allowable values
  if(abs(current) > limits){
    current = current/abs(current)*limits; 
  }

  // map to duty
  float duty = (current + limits) * (0.9-0.1) / (limits + limits) + 0.1;

  return duty;
}




void WaitNextRound(){
  static unsigned long elapsed_time;
  static unsigned long now_time;
  now_time = micros();

  elapsed_time = TimeDiffMicros(now_time,last_time);


  
    
  if(elapsed_time > LOOP_MICROS && !STOP_IF_TIME_EXCEEDED){               // Exceeded time?
    Serial.println("Exceeded time");
    Serial.print("elapsed_time:");
    Serial.print(elapsed_time);
    Serial.print(" us    Max time: ");
    Serial.print(LOOP_MICROS);
    Serial.println(" us");
    ErrorManager(EXCEEDED_TIME);
  }
    

  //Not exceeded time, wait until its time
  
  while(elapsed_time < LOOP_MICROS){

    now_time = micros();
    elapsed_time = TimeDiffMicros(now_time,last_time);
  };
  last_time = micros();
}

unsigned long TimeDiffMicros(unsigned long newest, unsigned long oldest){         // Returns time diff between two times, corect for overflow
  if(newest > oldest){                                       // Time overflow?
    return newest - oldest;
  }else{
    return  (unsigned long)4294967295 -(oldest - newest);    // max value - inverse difference
  }
}

bool JumpUpIfFallen(float* measured_values, float stable_point){
  bool jumped = false;
  
  // If fallen
  if(abs(measured_values[0])> WORKING_ANG){
    SetMotorTorque(0);
    // COmpute direction, and jump up
    float direction = measured_values[0]/abs(measured_values[0]);
    if(DEBUG){
      Serial.println("---------------------------------------------------------");
      Serial.print("System fell ");
      if(direction < 0){
        Serial.print("left");
      }else{
        Serial.print("right");
      }
      Serial.println(",jumping back up.");
    }
    jumped = true;
    retry: SetMotorTorque(0);
    JumpUp(direction);
    ServoReset();
    // Wait for finishing jump
    float max_angle = 50;
    while(abs(measured_values[0])> WORKING_ANG){
      delay(20);
      UpdateMeasurements(measured_values);
      measured_values[0] = measured_values[0] + stable_point;
      // New maximum
      if(abs(measured_values[0])<max_angle){
        max_angle = abs(measured_values[0]);
      }
      // Jump up failed
      if(abs(measured_values[0]) > 40*PI/180){
        if(DEBUG){
        Serial.print("Jump up failed, maximum angle: ");
        Serial.print(max_angle*direction*180/PI,2);
        Serial.println(".");
        }
        delay(20);
        goto retry;
      }
    }
    if(DEBUG){
      Serial.println("Jump up succeeded.");
    }
    
    StartTiming();
  }
  return jumped;
}

void JumpUp(int direction){
  static float wheel_speed_L = 1300*PI/30, wheel_speed_R  = 1700*PI/30; // L is for negative angle, R for positive
  float wheel_speed = getWheelSpeed();

    delay(2000);
    if(abs(wheel_speed)>10){
      ServoBrake();
      delay(300);
      ServoReset();
    }
    ServoReset();
    SetMotorTorque(-0.05*direction);
    while(abs(wheel_speed) < wheel_speed_L*(direction < 0)+wheel_speed_R*(direction>0)){
      wheel_speed = getWheelSpeed();
      delay(20);
    }
    SetMotorTorque(0);      
    ServoBrake();
    delay(250);
    ServoReset();


  /**
  pot_measurement = getFrameAngle();
  while(abs(pot_measurement)>WORKING_ANG){
    
    float max = 45.0/180*PI;
    pot_measurement = getFrameAngle();
    if(abs(pot_measurement) < max){
      max = abs(pot_measurement);
    }else if(abs(pot_measurement)>40*PI/180){
      if(DEBUG){
        
      }
      goto Retry;
    }
    delay(20);
  }
  if(DEBUG){
    Serial.println("Jump up succeeded.");
  }
  **/
}

void printState(float* measured_values, float stable_point, float torque_to_apply){
    Serial.print("Center mass estimator: ");
    Serial.print(180/PI*stable_point,1);
    Serial.print(" deg | Frame: ");
    Serial.print(180/PI*(measured_values[0]),1);
    Serial.print(" deg , ");
    Serial.print(measured_values[1],1);
    Serial.print(" rad/s \n Flywheel: ");
    Serial.print(measured_values[2],1);
    Serial.print(" rad/s | Torque: ");
    Serial.print(1000*torque_to_apply,1);
    Serial.print(" Nmm | loop time: ");
    Serial.print((micros()-last_time)/1000.0,1);
    Serial.print(" ms : ");
    Serial.print(1000.0/LOOP_FREQ,1);
    Serial.println("ms");
}



































// ###############################################################################################################################
// NON USED FUNCTIONS-----------------------------------------------------------------------------------------------------------------

/**

float ControllerCalculator(float pot_measurement, float reference){
  // Declarations
  static float* angle_err= initTimeArray(2); // rads
  static float* torque = initTimeArray(2); // N*m
  static float max_torque = 77.5/1000;      // N*m

 // Time advance and angle err
  advanceTimeArray(angle_err,2);
  advanceTimeArray(torque,2);

  angle_err[0] = -getFrameAngle()+reference;
  
  //Put your equation here

  torque[0] = 1.882*angle_err[0] -1.88*angle_err[1]+0.998*torque[1]; 


  //Maximum value limiter 
  if(abs(torque[0]) > max_torque){
    torque[0] = torque[0]/abs(torque[0])*0.0775;
  }

  return torque[0];
}




float EstimateReference(float new_measurement){
  static float* speed_rads = initTimeArray(2);
  static float K = 0.01;
  static float max_speed = 2000;
  static float new_ref = 1.7/180.0*PI;
  static float i = 0;
  i++;
  if(i < 200){
    float aux = filterSpeed(new_measurement);
    return new_ref;
  }
  i = 0;
 
  advanceTimeArray(speed_rads);
  speed_rads[0] = filterSpeed(new_measurement);

  
 

  
  

  float accs_rads2 = (speed_rads[0]-speed_rads[1])*1;

  if(abs(speed_rads[0]) > max_speed){               // Exceeded speed?

      EmergencyStop();
      BlinkLed();
    }
  //Serial.println(speed_rads[0]*180/PI);
  new_ref = accs_rads2/200+new_ref;



  //if(abs(new_ref) > 5.0/180*PI){
    //new_ref = new_ref/abs(new_ref)*5.0/180*PI;
  //}

  
  


  return new_ref;
}














float EstimateStablePoint(float* measurements){
  static float stable_point = 0;
  static float angle_aux = 0;
  static float wheel_accel  = 0;
  static float tolerance_angle = 2*PI/180;  // One degree startup
  static float tolerance_accel = 0;      // In rads/s. If it changes faster than 150 rev/min (5*pi rad/s), change reference


  angle_aux = measurements[0]-stable_point;
  wheel_accel = EstimateWheelAccel(measurements[2]);

  if(angle_aux < tolerance_angle){ // If controller finished its job

    if(wheel_accel*angle_aux/abs(angle_aux) >tolerance_accel){   // If wheel is still accelerating
      

      stable_point-=PI/180/LOOP_FREQ/4*wheel_accel/abs(wheel_accel);   // change 1 degree per second, signs are correct.
      // This assumes: if falls to +(right) then wheel accels + (to push left) so ref is to the left (-), so crrect with inverse sign
    
    }
  }
  
  Serial.print("Wheel accel: ");
  Serial.print(wheel_accel);
  Serial.print("rad/s\n");
  Serial.print("Angle diff: ");
  Serial.print(angle_aux/PI*180);
  Serial.print("º\n");
  Serial.print("Angle Correction: ");
  Serial.print(stable_point/PI*180);
  Serial.print("º\n");
  *
  return stable_point;
  
}

float EstimateWheelAccel(float wheel_measurement){  
  static float acc = 0;
  static int wait = 10;
  static float* previous_speed = initTimeArray(wait);
  static float new_speed = 0;

  new_speed = filterSpeed(wheel_measurement);
  //Serial.println(new_speed);
  acc = (new_speed-previous_speed[wait-1])*100/wait;

  advanceTimeArray(previous_speed,10);
  previous_speed[0] = new_speed;

  acc = filterAccel(acc);

  return acc;


}

float filterSpeed(float new_value){       // 1 Hz cutoff 1st order Bw filter, Fd = 100 Hz
  static float* avg_speed_rads = initTimeArray(2);  
  static float* speed_rads = initTimeArray(2);
  static float num_coeffs[] = {0.0609};
  static float den_coeffs[] = {0.9391};
  advanceTimeArray(avg_speed_rads,1);
  advanceTimeArray(speed_rads,1);

  speed_rads[0] = new_value;

  avg_speed_rads[0] =  inner_product(speed_rads,num_coeffs,sizeof(num_coeffs) / sizeof(float))+inner_product(avg_speed_rads,den_coeffs,sizeof(den_coeffs) / sizeof(float));

  return avg_speed_rads[0];


}



float StateSpaceControllerWreference(float *measured_values,float reference){
  static float controller_constants[3] = {-1.93980094296589,-0.210190099518124,-0.00225545126724467}; // angle err, speed frame, speed wheel  
  static float max_torque = MAX_MOTOR_CURRENT*33.3/1000;
  static float prev_u = 0;
  float torque_to_apply = inner_product(measured_values,controller_constants,3);



  if(abs(torque_to_apply)> max_torque){
    torque_to_apply = torque_to_apply/abs(torque_to_apply)*max_torque;    
  }

  return torque_to_apply;
}


float controller1(float new_value){
  static float* avg_speed_rads = initTimeArray(2);  
  static float* out = initTimeArray(2);  
  static float* in = initTimeArray(2);
  static float num_coeffs[] = {-1.57, 1.428};
  static float den_coeffs[] = {0,0.8187};
  advanceTimeArray(out,2);
  advanceTimeArray(in,2);

  in[0] = new_value;

  out[0] =  inner_product(in,num_coeffs,sizeof(num_coeffs) / sizeof(float))+inner_product(out,den_coeffs,sizeof(den_coeffs) / sizeof(float));

  return out[0];
}
float controller2(float new_value){
  static float* avg_speed_rads = initTimeArray(2);  
  static float* out = initTimeArray(2);  
  static float* in = initTimeArray(2);
  static float num_coeffs[] = {0,0.000953};
  static float den_coeffs[] = {0,1};
  advanceTimeArray(out,2);
  advanceTimeArray(in,2);

  in[0] = new_value;

  out[0] =  inner_product(in,num_coeffs,sizeof(num_coeffs) / sizeof(float))+inner_product(out,den_coeffs,sizeof(den_coeffs) / sizeof(float));

  return out[0];
}

float angleEstimatorWithTorque(float* measured_values){
  static int count = 0;
  static float CM_angle = 0;
  static float acc = 0;
  if(abs(measured_values[0])<2*PI/180){
    count++;
  }else{
    count  = 0;
  }
  if(count < 50){
    return CM_angle;
  }
  // If angle is smaller than 2º for more than half a second, activate.
  acc = EstimateWheelAccel(measured_values[2]);
  return 0.001011*acc;
}


float filterAccel(float new_value){       // 1 Hz cutoff 1st order Bw filter, Fd = 100 Hz
  static float* avg_accel_rads = initTimeArray(2);  
  static float* accel_rads = initTimeArray(2);
  static float num_coeffs[] = {0.0609};
  static float den_coeffs[] = {0.9391};
  advanceTimeArray(avg_accel_rads,2);
  advanceTimeArray(accel_rads,2);

  accel_rads[0] = new_value;

  avg_accel_rads[0] =  inner_product(accel_rads,num_coeffs,sizeof(num_coeffs) / sizeof(float))+inner_product(avg_accel_rads,den_coeffs,sizeof(den_coeffs) / sizeof(float));

  return avg_accel_rads[0];


}


float complementaryFilter(float measured_values) {
  static float tau = 0.5;
  static float samplingperiod_s = LOOP_FREQ / 1000.0;
  static float c1 = samplingperiod_s / (samplingperiod_s + 2 * tau);
  static float ang_filt_old = 0;
  
  float speedFrame = measured_values;
  float acc_data = FrameAngleIMUs();

  float ang_filt = c1 * ( tau * speedFrame + acc_data) + ang_filt_old;
  ang_filt_old = c1 * ( tau * speedFrame + acc_data) - (samplingperiod_s - 2 * tau) / (samplingperiod_s + 2 * tau) * ang_filt;

  return ang_filt;
}

**/
