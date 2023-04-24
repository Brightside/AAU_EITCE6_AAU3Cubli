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

#define TIME_EQ 20*50
//float TestMeasurementsFrameAngle[1000];
//float TestMeasurementsFrameSpeed[1000];
//float TestMeasurementsWheelSpeed[1000];
//float TestMeasurementsTorque[1000];

int MeasureCounter = 0;


// Test measurement declarations - End

void setup() {
  // Initialize the neded stuff
  InitSystem();

  pinMode(POWER_SWITCH_PIN,OUTPUT);
  pinMode(READ_SWITCH_PIN,INPUT_PULLUP);
  digitalWrite(POWER_SWITCH_PIN,OUTPUT);
  digitalWrite(EMG_LED,LOW);
  Serial.begin(115200);
  while(!Serial);       //ensure serial is connected 
  digitalWrite(EMG_LED,HIGH);
  
}

void loop() 
{
  static float measured_values[3] = {0,0,0};
  while(digitalRead(READ_SWITCH_PIN)==LOW) // Frame Angle, Frame Speed, Wheel Speed
  {
    
    
    setMotorTorqueHighRes(0);
 
    UpdateMeasurements(measured_values);
    Serial.print("0,");
    Serial.println(measured_values[0]*180/PI);

    MeasureCounter = 0;
    delay(200);
  }
  if(MeasureCounter == 0){
    Serial.println("Time(ms),FrameAng(deg),FrameSpeed(rad/s),WheelSpeed(rpm),AngEstim(deg)");
    //Once ready, start counting time.
    StartTiming();
  }
  


  static float torque_to_apply = 0;
  static float stable_point = INIT_ANGLE_GUESS;

  // perform measurement of system states
  UpdateMeasurements(measured_values);

// Measure angle till wanted angle is found, then continues



  //Shut down controller if angle goes over 20 degrees
  // if(abs(measured_values[0])> 20.0/180*PI)
  // {
  //   EmergencyStop();
  //   BlinkLed();
  // }
  
  // Correct measurement to be in reference to CM
  stable_point = EstimatorPointFallingTorque(measured_values[2]);
  measured_values[0]+=stable_point;
  
  //Calculate needed Torque
  torque_to_apply = StateSpaceController(measured_values);

  // Sets motor torques
  //setMotorTorque(torque_to_apply);
  setMotorTorqueHighRes(torque_to_apply);
  
  // Print first 20 seconds of measurements to serial
  if(MeasureCounter < TIME_EQ)
  { 
    // save values 
    Serial.print(((float)MeasureCounter)*1.0/LOOP_FREQ,6);    
    Serial.print(",");
    Serial.print((measured_values[0]-stable_point)*180/PI,6);
    Serial.print(",");
    Serial.print(measured_values[1],6);
    Serial.print(",");
    Serial.print(measured_values[2]*30/PI,6);
    Serial.print(",");
    Serial.print(torque_to_apply,6);
    Serial.print(",");
    Serial.println(stable_point*180/PI,6);
      
    MeasureCounter++;     
    
  } 


  // waits til a total of 20 ms has elapsed for loop
  last_time = WaitNextRound(last_time,false);
  
}


