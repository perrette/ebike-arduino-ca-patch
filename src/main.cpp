#include <Arduino.h>
#include <avr/wdt.h>
#include "helpers.h"
#include <All_Define.h>
#include <BikerRPM.h>

void setup()
{
  // I/O init
  pinMode(DirectionPin_In, INPUT_PULLUP);
  pinMode(SpeedPin_In, INPUT_PULLUP);
  pinMode(ThrottlePin_Out, OUTPUT);
  pinMode(TorquePin_In, INPUT);
  Serial.begin(115200);

  // Interrupt init
  // attachInterrupt(digitalPinToInterrupt(DirectionPin_In), StopBikeEvent, FALLING); // Call StopBikeEvent each falling edge on DirectionPin
  attachInterrupt(digitalPinToInterrupt(SpeedPin_In), SpeedPulseEvent, RISING); // Call SpeedPulseEvent each rising edge on SpeedPin

  // Security
  wdt_enable(WDTO_500MS); // Set 0.5 Sec watchdog
  ResetBike();            // Reset the bike to be sure to start with a known state
}

void loop()
{
  wdt_reset(); // Must be called at least every 500ms otherwise Arduino Reset is triggered

  UpdateBikerRpmFiltered(); // Update Biker Rpm filtered with auto Stop detector

  // TODO: treat cases where the loop is slower than the poles counts (an
  // interruption situation occurs in between two loops passes)
  // In practice, my guess is this is not critical, since that would mean the
  // biker restarted / continued to pedal in the meanwhile  =====>>>> Should work well now

  if (StopMotorPower) // If StopMotorPower is set to true in the interrupt routine, stop the bike
  {
    event_type = EVENT_TYPE_BACK_PEDALING;
    // BikerRpmFiltered = 0; // Set filtered RPM to zero to avoid a jump when stopping the bike
    StopMotorPower = false; // Reset StopMotorPower flag to avoid a long stop if the interrupt is triggered again by mistake
// StopBike() ; // Stop the bike by resetting the state and applying the neutral throttle value
#ifdef DEBUG
    Serial.println("Bike stopped since StopMotorPower flag is set (deactivated)"); // Remove @ Release
#endif
    // return; // Skip the rest of the loop
  }

  // if (DEBUG_MODE)
  //   BikerRpmFiltered = 30.0f; // Remove @ Release

  TorqueValue = analogRead(TorquePin_In) * ANALOG_TO_VOLT_IN;
  TorqueValueFiltered = TorqueValueFiltered * TorqueValueFilteredAlphaGain + TorqueValue * (1 - TorqueValueFilteredAlphaGain);
  TorqueValueFiltered = constrain(TorqueValueFiltered, TorqueValueNeutral, TorqueValueMax);    // Constrain filtered torque value to be between 0 and reference voltage
  float TorqueValueFilteredNm = (TorqueValueFiltered - TorqueValueNeutral) * TorqueSensorGain; // Convert voltage to torque in Nm, and remove the neutral value

  HumanPowerWatt = TorqueValueFilteredNm * BikerRpmFiltered * 2.0 * 3.14159265f / 60.0;

  HumanPowerWattFiltered = HumanPowerWattFiltered * HumanPowerWattAlphaGain + HumanPowerWatt * (1 - HumanPowerWattAlphaGain);
  HumanPowerWattFiltered = constrain(HumanPowerWattFiltered, 0, HumanPowerWattMax);

  float MotorPower = HumanPowerWattFiltered * MotorPowerBoostFactor;

  Throttle_Value_Volt = mapFloat(MotorPower, 0, MotorPowerAtMaxThrottle, MinThrottleValue,
                                 MaxThrottleValue); // Map RPM to throttle voltage

  Throttle_Value = Throttle_Value_Volt * VOLT_TO_ANALOG_OUT; //
  analogWrite(ThrottlePin_Out, Throttle_Value);              // Throttle_Value has to be set as you want

#ifdef DEBUG

  Serial.print(">");
  // Serial.print("Raw_RPM:");
  // Serial.print(BikerRpmRaw);
  Serial.print(",Filtered_RPM:");
  Serial.print(BikerRpmFiltered);

  //   Serial.print(",Torque_Value:");
  //   Serial.print(TorqueValue);
  //   Serial.print(",Filtered_Torque_V:");
  //   Serial.print(TorqueValueFiltered);
  Serial.print(",Torque_Nm:");
  Serial.print(TorqueValueFilteredNm);
  //   Serial.print(",Human_Power_Watt:");
  //   Serial.print(HumanPowerWatt);
  Serial.print(",Filtered_Human_Power_Watt:");
  Serial.print(HumanPowerWattFiltered);
  //   Serial.print(",Motor_Power_Watt:");
  //   Serial.print(MotorPower);
  //   Serial.print(",Throttle_Value:");
  //   Serial.print(Throttle_Value_Volt);
  //   // Serial.print(",Throttle_Check:");
  //   // Serial.print(analogRead(ThrottleCheckPin) * ANALOG_TO_VOLT_IN);
  //   Serial.print(",Event_Type:");
  //   Serial.print(event_type);
  Serial.println();
  //   Serial.println("Time left to wait (ms): " + String((NextLoopTime - micros()) / 1000.0f) + " ms (total " + String(LoopTimeUs / 1000.0f) + " ms)"); // Remove @ Release
#endif

  while (micros() < NextLoopTime) // wait before starting next loop
    ;
  NextLoopTime += LoopTimeUs; // Init next loop time
}

void StopBikeEvent() // Interrupt called on falling edge of DirectionPin
{
  StopMotorPower = true; // Set flag to stop the bike
#ifdef DEBUG
  Serial.println("StopBikeEvent called"); // Remove @ Release
#endif
}

void ResetBike() // Function to reset the bike, can be called in case of emergency or to restart the bike
{
  StopMotorPower = false;                   // Reset StopMotorPower flag
  BikerRpmFiltered = 0;                     // Set filtered RPM to zero
  TorqueValueFiltered = TorqueValueNeutral; // Set torque to neutral value to avoid a jump when restarting
  HumanPowerWattFiltered = 0;               // Set human power to zero to avoid a jump when restarting
  Throttle_Value = NeutralThrottleValue;    // Reset throttle to neutral value
  // TimeBetween2edge = 10e10; // must be larger than deboune time
  // LastTimeOfLastEdge = micros(); // must not be larger than current (micros()) + TimeStopLimit to avoid a long wait before stopping the bike in case of no pulse detected
  // RpmStopDetector = 0;
}

void ResetTime()
{
  NextLoopTime = micros() + LoopTimeUs; // Reset loop time to avoid a long wait before the next loop
}

void StopBike() // Function to stop the bike by resetting the state and applying the neutral throttle value
{
  analogWrite(ThrottlePin_Out, NeutralThrottleValue); // Apply the throttle value to stop the bike
  ResetBike();
  ResetTime();
}