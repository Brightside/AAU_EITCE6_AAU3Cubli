#include <Adafruit_MPU6050.h>  // https://github.com/adafruit/Adafruit_MPU6050/blob/master/Adafruit_MPU6050.h
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <SAMD21turboPWM.h>


// PROGRAM DEFINITIONS
#define PI 3.141592
int LOOP_FREQ = 50; // Frequency of system in Hertzs. If changed, correct Cm estimation: filter and accel code
unsigned long LOOP_MICROS = (unsigned long) (1000000./LOOP_FREQ); // microseconds of a loop

bool DEBUG = true; // Enter debug mode (not used rn)


float INIT_ANGLE_GUESS = 0/180*PI;  // Initial center of mass estimation

unsigned long last_time,now_time;   // Variables to keep track of elapsed time

// Library for IMUs
Adafruit_MPU6050 MPU1;
Adafruit_MPU6050 MPU2;

// High res motor stuff
struct timerStruct{
  const int enbPin = 1;
  const int PWMPin = 5;
  const int TIMR = 0;
  const int CLKDiv_1 = 15;
  const int CLKDiv_2 = 15;
  const int aauMinCyc = 0;  //  (duty cycle: 5*20)
  const int aauMaxCyc = 1000; // (duty cycle: 1000*20)
  TurboPWM aau;
} motor_timer;



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
} PINS_MOTOR;         // Motor pin layout
#define MAX_MOTOR_CURRENT 4
#define MAX_MOTOR_CURRENT_HIGH_RES 7.5
Servo servo;
#define BRAKE_ANG 105    // º
#define RESET_ANG 135   // º
#define WORKING_ANG 10*PI/180  // º
// ####################################################################################################################################################
// Initialization--------------------------------------------------------------------------------------------

void InitSystem();   
  // Initializes everything needed for the program to start running.  
    void InitMotor();
      // Sets a PWM of 0 and after 2 seconds enables the motor.
    // Alternative motor with high res----------------------
    void InitMotorHighRes();
      // Initalizes the motor, with a total of 800 possible values
    //------------------------------------------------------
    void InitServo();
      // Initializes servo.
    bool initIMUs();
      // Starts IMUs. False if worked, true if failed.
    void EmergencyStop(); 
      // Shuts down actuators instantly
    void BlinkLed();
      // Makes EMG_LED blink. Locks system in infinte loop
      
void StartTiming(); 
  // Starts the loop timing

// Running---------------------------------------------------------------------------------------------------
void ServoBrake();
  // It also deactivates the motor.
void ServoReset();
  // It activates the motor.


void UpdateMeasurements(float* measured_values);
  // Updates all measured values. Reads Switch value to decide angle computation.
    float FrameAngleIMUs();
      // Measure angle with IMUs (to be improved, only uses accelerometers)
    float getFrameAngle();
      // Reads pot and gives back frame angle in rad
    float getWheelSpeed();
      // Obtains the current relative angular velocity of the motor
    float getFrameSpeed();
      // Returns a mesaurement of the frame speed. Positive falling right.


float EstimatorPointFallingTorque(float measured_w);
// uses steady state torque estimation to find center of mass.
  float* initTimeArray(int size);
    // Creates a float array of "size" elements and initializes them to 0. The newest value is at [0] up to the latest at [size-1]
  void advanceTimeArray(float* array, int size);
    // Advances the array and initializes new value at 0. [0] into [1], [1] into [2]...

float StateSpaceController(float* measured_values);
  // Controller using state space equations. Takes in measured values: frame angle, frame speed, wheel speed; and reference(in radians)
    float inner_product(float* vector1, float* vector2, int size);
      // Computes inner product of system


void setMotorTorque(float torque_to_apply);
  // Sets PWM to equivalent current.
  // I = torque_to_apply/Kt;
  // PWM: 10%~90% (3.3V) 
    int TorqueToPWMDuty(float torque_to_apply);
      // Translates from Torque to apply to the duty of the PWM. Note that all limiters are defined there. Contains limiter.
      //  - Max current = 4
      //  - PWM: 10%~90% (3.3V)
// Alternative motor -------------------------------------------
void setMotorTorqueHighRes(float torque_to_apply);
  // Allows a total of 800 possible values
    void setDutyHighRes(float duty);
      //Sets the PWM equal to a 0-1 float
        void setAAU(const int pin, int cycle);
          //Setting the duty cycle equal to an input, with interval minmax
        float TorqueToPWMDutyHighRes(float torque_to_apply);
          // Translates from Torque to apply to the duty of the PWM. Note that all limiters are defined there. Contains limiter.
          //  - Max current = 7.5
          //  - PWM: 10%~90% (3.3V)
//--------------------------------------------------------------
unsigned long WaitNextRound(unsigned long prev_time, bool no_stop);   
  // Waits for the specified global time to elapse
    unsigned long TimeDiffMicros(unsigned long now_time, unsigned long prev_time);  
      // Calculates the time difference accounting for overflow.
void JumpUp();
  // Use when system falls, it can jump up by itself.
// Non-Used functions-----------------------------------------------





float ControllerCalculator(float pot_measurement, float reference);
  // Calculates next torque to be applied by the motor. Requires the measured angle and the reference. All inputs and outputs in float






float StateSpaceControllerWreference(float *measured_values,float reference);
  // Same as state space controller, but can follow a w reference.

float integrate(float new_value);
  // integrates the value. Autocorrects with varying FREQ;



float EstimateStablePoint(float* measurements);
  // Estimates the Stable angle. Needed as the zero of the pot does not coincide with the stable point.

float EstimateWheelAccel(float wheel_measurement);
  // Estimates wheel acceleration

float filterSpeed(float new_value);
  // filters the speed at 20Hz, first order Bw filter



float controller1(float new_value);
float controller2(float new_value);
// PID + extra controller

float angleEstimatorWithTorque(float* measured_wheel);
// Estimates center of mass angle w.r.t the steady state force.

float filterAccel(float new_value);
// filters the speed at 20Hz, first order Bw filter

float complementaryFilter(float measured_values);
// From previous years



// ##########################################################################################################################
// System functions


void InitSystem(){  // Initializes everything needed for the program to start running.
  if(DEBUG){
    Serial.begin(115200);
    while(!Serial);
  }
  if(DEBUG){
    Serial.println("Initializing system:");
  }
  // Switch ------------------------------------
  //pinMode(READ_SWITCH_PIN,INPUT);
  //pinMode(POWER_SWITCH_PIN,OUTPUT);
  // Emergency led------------------------------
  pinMode(EMG_LED,OUTPUT);
  digitalWrite(EMG_LED,HIGH); //Led is low_active
  //Servo----------------------------------------
  if(DEBUG){
    Serial.print("Servo.........................");

  }
  InitServo();
  if(DEBUG){
    Serial.println("ok");
  }
  
  //Motor----------------------------------------
  //InitMotor();
  if(DEBUG){
    Serial.print("Motor.......................");
    
  }
  InitMotorHighRes();
  if(DEBUG){
    Serial.println("ok");
  }
  // Value screening-----------------------------
  if(DEBUG){
    Serial.print("IMUs........................");
  }
  
  // IMUS----------------------------------------
  if(initIMUs()){
    Serial.println("Initialization failed");
    EmergencyStop();
    BlinkLed();
    
  }
  if(DEBUG){
    Serial.println("ok");
  }

}

void InitMotor(){
  pinMode(PINS_MOTOR.ENB_PIN,OUTPUT);
  pinMode(PINS_MOTOR.PWM_PIN,OUTPUT);
  pinMode(SPEED_PIN,INPUT);
  analogWrite(PINS_MOTOR.PWM_PIN,floor(255/2));

  delay(2000); // Need to wait for a while before starting

  digitalWrite(PINS_MOTOR.ENB_PIN,HIGH);
}

void InitMotorHighRes(){
  pinMode(motor_timer.PWMPin,OUTPUT);
  pinMode(SPEED_PIN,INPUT);
  //Serial.println("Waiting...");
  pinMode(motor_timer.enbPin,OUTPUT);
  digitalWrite(motor_timer.enbPin,LOW);
  motor_timer.aau.setClockDivider(motor_timer.CLKDiv_1, false);  // clock is divided by 15. Turbo is off/false
  motor_timer.aau.timer(motor_timer.TIMR,motor_timer.CLKDiv_2, 1000, true);  // Timer 0 is set to Generic Clock divided by 15, resolution is 1000, single-slope PWM
  setMotorTorqueHighRes(0);
  delay(6000);
  //Serial.println("Done, starting up");
  pinMode(motor_timer.enbPin,OUTPUT);
  digitalWrite(motor_timer.enbPin,HIGH);
}



void InitServo(){
  servo.attach(SERVO_PIN);
  servo.write(RESET_ANG);
}

bool initIMUs(){
  if (!MPU1.begin() || !MPU2.begin(0x69)) {
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

void EmergencyStop(){
      // Deactivate motor
      digitalWrite(PINS_MOTOR.ENB_PIN,LOW);
      digitalWrite(PINS_MOTOR.PWM_PIN,LOW);
}

void BlinkLed(){
  while(true){
    digitalWrite(EMG_LED,LOW);
    delay(500);
    digitalWrite(EMG_LED,HIGH);
    delay(500);
  }
}


void StartTiming(){
  last_time = micros();
}


void ServoBrake(){
  // Security: stop motor before  braking
  digitalWrite(PINS_MOTOR.ENB_PIN,LOW);
  servo.write(BRAKE_ANG);
}

void ServoReset(){
  // Security: reactivate motor after releasing
  servo.write(RESET_ANG);
  digitalWrite(PINS_MOTOR.ENB_PIN,HIGH);
}

void UpdateMeasurements(float* measured_values){
  //if(digitalRead(READ_SWITCH_PIN) == HIGH){
  //  measured_values[0] = FrameAngleIMUs();
  //}else{
    measured_values[0]  = getFrameAngle();              
  //}
    
  measured_values[1]  = getFrameSpeed();
  measured_values[2]  = getWheelSpeed();
  
}




float FrameAngleIMUs(){
  // Get measurements
  sensors_event_t a1, g1, temp1; 
  MPU1.getEvent(&a1, &g1, &temp1);
  sensors_event_t a2, g2, temp2;  
  MPU2.getEvent(&a2, &g2, &temp2);    
  // Compute angle
  float theta_f1 = -atan2(a1.acceleration.x, a1.acceleration.y)+PI/4;
  float theta_f2 = -atan2(a2.acceleration.y, -a2.acceleration.x)+PI/4;
  return (theta_f1+theta_f2)/2-0.035;
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

float getFrameSpeed(){
  static sensors_event_t a1, g1, temp1;
  static sensors_event_t a2, g2, temp2;


  MPU1.getEvent(&a1, &g1, &temp1);
  float omega_gyro1 =  (-g1.gyro.z);
  MPU2.getEvent(&a2, &g2, &temp2);
  float omega_gyro2 =  (-g2.gyro.z);



  return (omega_gyro1+omega_gyro2)/2;
}

float EstimatorPointFallingTorque(float measured_w){
  static float* w_err = initTimeArray(2);  
  static float* theta_cm = initTimeArray(2);
  static float num_coeffs[] = { 0.0009441,-0.0009422};
  static float den_coeffs[] = {0,1};
 

  advanceTimeArray(theta_cm,2);
  advanceTimeArray(w_err,2);

 
  w_err[0] = measured_w;
  
  theta_cm[0] =  inner_product(w_err,num_coeffs,sizeof(num_coeffs) / sizeof(float))+inner_product(theta_cm,den_coeffs,sizeof(den_coeffs) / sizeof(float));
  

  

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
  static float max_torque = MAX_MOTOR_CURRENT_HIGH_RES*33.5/1000;

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

void setMotorTorque(float torque_to_apply){

  int duty = TorqueToPWMDuty(-torque_to_apply); // - to keep consistent with the directions
  //Serial.print("4A: ");
  //Serial.print(duty/255.0);
  analogWrite(PINS_MOTOR.PWM_PIN,duty);
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

void setMotorTorqueHighRes(float torque_to_apply){

  float duty = TorqueToPWMDutyHighRes(-torque_to_apply); // - to keep consistent with the directions
  //Serial.print("   |   7.5A: ");
  //Serial.println(duty);
  setDutyHighRes(duty);
}

void setDutyHighRes(float duty) {
  setAAU(motor_timer.PWMPin, (int)floor(duty * 1000));
}


void setAAU(const int pin, int cycle) {
  if (cycle < motor_timer.aauMinCyc) {
    cycle = motor_timer.aauMinCyc;
  }
  if (cycle > motor_timer.aauMaxCyc) {
    cycle = motor_timer.aauMaxCyc;
  }
  motor_timer.aau.analogWrite(pin, cycle);
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




unsigned long WaitNextRound(unsigned long prev_time, bool no_stop){
  static unsigned long elapsed_time;
  
  now_time = micros();

  elapsed_time = TimeDiffMicros(now_time,prev_time);
  if(DEBUG && false){
    Serial.print("Time of loop:");
    Serial.print(elapsed_time);
    Serial.print("us.\n");
  }
  
    
  if(elapsed_time > LOOP_MICROS && !no_stop){               // Exceeded time?
    Serial.println("Exceeded time");
    Serial.print("elapsed_time:");
    Serial.print(elapsed_time);
    Serial.print(" us    Max time: ");
    Serial.print(LOOP_MICROS);
    Serial.println(" us");
    EmergencyStop();
    BlinkLed();
  }
    

  //Not exceeded time, wait until its time
  
  while(elapsed_time < LOOP_MICROS){

    now_time = micros();
    elapsed_time = TimeDiffMicros(now_time,prev_time);
  };
  return now_time;
}

unsigned long TimeDiffMicros(unsigned long now_time, unsigned long prev_time){         // Returns time diff between two times, corect for overflow
  if(now_time > prev_time){                                       // Time overflow?
    return now_time - prev_time;
  }else{
    return  (unsigned long)4294967295 -(prev_time - now_time);    // max value - inverse difference
  }
}



void JumpUp(){
  static float wheel_speed_L = 1280*PI/30, wheel_speed_R  = 1600*PI/30; // L is for negative angle, R for positive
  
  Retry:
  float pot_measurement = getFrameAngle();
  float wheel_speed = getWheelSpeed();
  float direction = pot_measurement/abs(pot_measurement);

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
    delay(2000);
    if(abs(wheel_speed)>10){
      ServoBrake();
      delay(300);
      ServoReset();
    }
    
    setMotorTorque(-0.05*direction);
    while(abs(wheel_speed) < wheel_speed_L*(direction < 0)+wheel_speed_R*(direction>0)){
      wheel_speed = getWheelSpeed();
      delay(20);
    }
    setMotorTorque(0);      
    ServoBrake();
    delay(300);
    ServoReset();



  pot_measurement = getFrameAngle();
  while(abs(pot_measurement)>WORKING_ANG){
    float max = 45/180*PI;
    pot_measurement = getFrameAngle();
    if(pot_measurement > max){
      max = pot_measurement;
    }
    delay(20);
    if(abs(pot_measurement)>40*PI/180){
      if(DEBUG){
        Serial.print("Jump up failed, maximum angle: ");
        Serial.print(max*180/PI,2);
        Serial.println(".");
      }
      goto Retry;
    }
  }
  if(DEBUG){
    Serial.println("Jump up succeeded.");
  }
}




































// ###############################################################################################################################
// NON USED FUNCTIONS-----------------------------------------------------------------------------------------------------------------



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



/**
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



**/










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
  /**
  Serial.print("Wheel accel: ");
  Serial.print(wheel_accel);
  Serial.print("rad/s\n");
  Serial.print("Angle diff: ");
  Serial.print(angle_aux/PI*180);
  Serial.print("º\n");
  Serial.print("Angle Correction: ");
  Serial.print(stable_point/PI*180);
  Serial.print("º\n");
  **/
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


