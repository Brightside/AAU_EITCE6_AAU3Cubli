#include "functions.h"

/** 

########################################################################################################################
#
#       AAU3 Stabilization and control program
#
#   This program cointains:
#       - State machine to differenciate between the different control algorithms implemented and the different states of the AAU (balancing, fallen, falling,...)(TBD)
#       - Frequency stabilization to ensure correct periodic reading.  (TBD - Basic while(time<x) - timer interrupts - FreeRTOS )
#       - Functions for every single action returning void for future simplicity implementation and adapt
#
#  TBD
#     Create main loop running at fixed time
#
#      Control variables
#       LIST_SENSORS = enum with all possible sensors
#       LIST_ACTUATORS = enum with all possible actuators
#       ACTIVE_SENSORS = array with LIST_SENSORS sensors in use.
#       ACTIVE_ACTUATORS = array with LIST_ACTUATORS in use.
#
#       LIST_ STATES = enum with different states(balancing,fallen,falling...)
#       LIST_CONTROLS = enum with different control possibilities
#
#     Create funcitons:
#         - ReadSensors();            Checks which sensors to call and obtains their values (in read variable units: degrees, meters, seconds,...SI units unless otherwise)
#         - ReadingCorrection();      Operates with obtained sensor values to correlate/corerct/average/study/denoise their result
#         - ControlCalculator();      Calls the currently active control method
#         - WriteActuators():         Check in use actuators and sets their values
# 
#         - InitSystem();
#               
#             
#
#
#
#
#
########################################################################################################################


**/

// Test measurement declarations - Start
#define STABLE_POINT -2.6*PI/180


// Test measurement declarations - End
void setup() {
  InitSystem();
  // Initialize the neded stuff
  pinMode(POWER_SWITCH_PIN,OUTPUT);
  pinMode(READ_SWITCH_PIN,INPUT_PULLUP);
  digitalWrite(POWER_SWITCH_PIN,OUTPUT);
  digitalWrite(EMG_LED,LOW);
  Serial.begin(115200);
  while(!Serial);       //ensure serial is connected 
  digitalWrite(EMG_LED,HIGH);
  //Once ready, start counting time.

  }

void loop() {
  static float measured_values[3];

  if(digitalRead(READ_SWITCH_PIN)==HIGH) // Frame Angle, Frame Speed, Wheel Speed
  {
    setMotorTorqueHighRes(0);
    EnableMotor();
    
    UpdateMeasurements(measured_values);
    if(measured_values[0]<0){
      jump2otherside();
    }
    Serial.println("Doing tries from right side");
    for(int i = 0; i < 30 ; i++){
      JumpingTest();
    }
    jump2otherside();
    Serial.println("Doing tries from left side");
    for(int i = 0; i < 30 ; i++){
      JumpingTest();
    }
    DisableMotor();
    while(digitalRead(READ_SWITCH_PIN)==HIGH);
  }
  
}

void JumpingTest(){
  static float wheel_speed_L = 1280*PI/30, wheel_speed_R  = 1600*PI/30; // L is for negative angle, R for positive
  static float measured_values[3] = {0,0,0}; // Frame Angle, Frame Speed, Wheel Speed

  UpdateMeasurements(measured_values);
  measured_values[0]+=STABLE_POINT;
  float direction = measured_values[0]/abs(measured_values[0]);

  delay(2000);
  if(abs(measured_values[2])>10){
    ServoBrake();
    delay(300);
    ServoReset();
  }
    
  setMotorTorque(-0.05*direction);
  while(abs(measured_values[2]) < wheel_speed_L*(direction < 0)+wheel_speed_R*(direction>0)){
    UpdateMeasurements(measured_values);
    delay(20);
  }
    setMotorTorque(0);      
    ServoBrake();

    
    // Measure values
    UpdateMeasurements(measured_values);
    measured_values[0]+=STABLE_POINT;
  delay(300);
  StartTiming();

  float passed_angle = 100*direction;
  while(abs(measured_values[0])<abs(passed_angle)){
    
    if((direction>0 && measured_values[0]< passed_angle)||(direction<0 && measured_values[0]> passed_angle)){
      passed_angle = measured_values[0];
    }

    
    last_time = WaitNextRound(last_time,false);
    // Measure values
    UpdateMeasurements(measured_values);
    measured_values[0]+=STABLE_POINT;

    // Track the smallest angle attained
    //Serial.print("Angle:");
    //Serial.print(measured_values[0]*180/PI);
    //Serial.print("   |   Passed angle:");
    //Serial.println(passed_angle*180/PI);
  }

  if(direction*measured_values[0]<=0){
      Serial.print("0.0000");
      Serial.print(",");
      Serial.println(measured_values[1],4); 
    }else{
      Serial.print(passed_angle/PI*180,4);
      Serial.println(",0.0000");
    }
 



  while(abs(measured_values[0]) < 40*PI/180){
    UpdateMeasurements(measured_values);
    measured_values[0]+=STABLE_POINT;
  }
  
  ServoReset();

  if(measured_values[0]*direction<0){
    jump2otherside();
  }
}
void DisableMotor(){
  digitalWrite(PINS_MOTOR.ENB_PIN,LOW);
}

void EnableMotor(){
  digitalWrite(PINS_MOTOR.ENB_PIN,HIGH);
}

void jump2otherside(){
  static float wheel_speed_L = 1480*PI/30, wheel_speed_R  = 1700*PI/30; // L is for negative angle, R for positive
  static float measured_values[3] = {0,0,0}; // Frame Angle, Frame Speed, Wheel Speed

  retry: UpdateMeasurements(measured_values);
  measured_values[0]+=STABLE_POINT;
  float direction = measured_values[0]/abs(measured_values[0]);

  delay(2000);
  if(abs(measured_values[2])>10){
    ServoBrake();
    delay(300);
    ServoReset();
  }
    
  setMotorTorque(-0.05*direction);
  while(abs(measured_values[2]) < wheel_speed_L*(direction < 0)+wheel_speed_R*(direction>0)){
    UpdateMeasurements(measured_values);
    delay(20);
  }
    setMotorTorque(0);      
    ServoBrake();

    
    // Measure values
    UpdateMeasurements(measured_values);
    measured_values[0]+=STABLE_POINT;
  delay(400);

  ending: 
  while(abs(measured_values[0]) < 40*PI/180){
    UpdateMeasurements(measured_values);
    measured_values[0]+=STABLE_POINT;
}
  if(measured_values[0]*direction<0){
    goto retry;
  }
}
//---------------------------------------------------------
