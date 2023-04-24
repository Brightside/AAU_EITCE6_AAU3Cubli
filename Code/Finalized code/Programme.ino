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






void setup() {
  // Initialize the neded stuff
  InitSystem();

  //Serial.begin(9600);
  //while(!Serial);
  //Once ready, start counting time.
  StartTiming();
  }

void loop() {
  
  static float measured_values[3] = {0,0,0}; // Frame Angle, Frame Speed, Wheel Speed
  static float torque_to_apply = 0;
  static float stable_point = INIT_ANGLE_GUESS;


  start: UpdateMeasurements(measured_values);

  //Shut down controller if angle goes over 20 degrees
  
  if(abs(measured_values[0])> WORKING_ANG){
    setMotorTorqueHighRes(0);
    JumpUp();
    stable_point = INIT_ANGLE_GUESS;
    StartTiming();
    goto start;
  }
  
  
  stable_point = EstimatorPointFallingTorque(measured_values[2]);
    measured_values[0]+=stable_point;

  
  torque_to_apply = StateSpaceController(measured_values);

  

  //setMotorTorque(torque_to_apply);
  setMotorTorqueHighRes(torque_to_apply);
  


  
  //Serial.print("CM angle: ");
  //Serial.print(180/PI*stable_point);
  //Serial.print("º\n");
  
  // Check if loop exceeded time:
  
  //Serial.print("Frame Angle: ");
  //Serial.print(180/PI*(measured_values[0]));
  //Serial.print(" º\n");
  
  //Serial.print()
  /**
  Serial.print("Frame speed: ");
  Serial.print(measured_values[1]);
  Serial.print(" rad/s\n");
  **/
  
  //Serial.print("Wheel speed: ");
  //Serial.print(measured_values[2]*30/PI);
  //Serial.print(" rpm\n");
  //Serial.print("Torque: ");
  
  //Serial.print("    | tau:  ");
  //Serial.print(1000*torque_to_apply);
  //Serial.println(" Nmm");

  
  last_time = WaitNextRound(last_time,false);
  
}





