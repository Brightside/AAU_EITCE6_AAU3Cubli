#include "functions.h"



void setup() {
  // Initialize the neded stuff
  InitSystem();

  //Serial.begin(9600);
  //while(!Serial);

  //Once ready, start counting time.
  

  StartTiming();
  }

void loop() {
  // define the variables to contain the important information
  static float measured_values[3] = {0,0,0}; // Frame Angle, Frame Speed, Wheel Speed
  static float torque_to_apply = 0;
  static float stable_point = INIT_ANGLE_GUESS;

  // Update the measurements
  UpdateMeasurements(measured_values);
  
  if(JumpUpIfFallen(measured_values,stable_point)){
    stable_point = INIT_ANGLE_GUESS;
  }
  
  if(abs(measured_values[0]) < 5*PI/180){
    stable_point = EstimatorPointFallingTorque(measured_values[2]);
    measured_values[0]+=stable_point;
  }

  
  torque_to_apply = StateSpaceController(measured_values);

  

  SetMotorTorque(torque_to_apply);
  

  if(DEBUG){
    printState(measured_values,stable_point,torque_to_apply);
  }
  WaitNextRound();
  
}



