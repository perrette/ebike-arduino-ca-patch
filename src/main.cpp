#include <Arduino.h>
#include <avr/wdt.h>
#include "helpers.h"
#include <All_Define.h>
#include <BikerRPM.h>

// Timing
uint32_t NextLoopTime;     // For loop time calibration

float TargetMotorPower;
float Throttle_Value;

void setup()
{
  // I/O init
  pinMode(DirectionPin_In, INPUT_PULLUP);
  pinMode(SpeedPin_In, INPUT_PULLUP);
  // pinMode(DigitalBrakePin_In, INPUT_PULLDOWN);
  pinMode(Brake_In, INPUT);
  pinMode(ThrottleUserControl_In, INPUT);
  pinMode(PotentiometerPin_In, INPUT);
  pinMode(ThrottlePin_Out, OUTPUT);
  pinMode(TorquePin_In, INPUT);
  Serial.begin(115200);

  // Interrupt init
  attachInterrupt(digitalPinToInterrupt(DirectionPin_In), BackPedalEvent, FALLING); // Call StopBikeEvent each falling edge on DirectionPin
  attachInterrupt(digitalPinToInterrupt(SpeedPin_In), SpeedPulseEvent, RISING);     // Call SpeedPulseEvent each rising edge on SpeedPin

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

  // if (DEBUG_MODE)
  //   BikerRpmFiltered = 30.0f; // Remove @ Release

  TorqueValue = analogRead(TorquePin_In) * ANALOG_TO_VOLT_IN;
  TorqueValueFiltered = TorqueValueFiltered * TorqueValueFilteredAlphaGain + TorqueValue * (1 - TorqueValueFilteredAlphaGain);
  TorqueValueFiltered = constrain(TorqueValueFiltered, TorqueValueNeutral, TorqueValueMax);    // Constrain filtered torque value to be between 0 and reference voltage
  float TorqueValueFilteredNm = (TorqueValueFiltered - TorqueValueNeutral) * TorqueSensorGain; // Convert voltage to torque in Nm, and remove the neutral value

  HumanPowerWatt = TorqueValueFilteredNm * BikerRpmFiltered * 2.0 * 3.14159265f / 60.0;

  HumanPowerWattFiltered = HumanPowerWattFiltered * HumanPowerWattAlphaGain + HumanPowerWatt * (1 - HumanPowerWattAlphaGain);
  HumanPowerWattFiltered = constrain(HumanPowerWattFiltered, 0, HumanPowerWattMax);

  float Brake_Volt = analogRead(Brake_In) * ANALOG_TO_VOLT_IN;
  float ThrottleUserControl_Volt = analogRead(ThrottleUserControl_In) * ANALOG_TO_VOLT_IN;
  float Potentiometer_Volt = analogRead(PotentiometerPin_In) * ANALOG_TO_VOLT_IN;
  float PotentiometerFraction = mapFloat(Potentiometer_Volt, PotentiometerMin, PotentiometerMax, 0, 1);
  float MotorPowerAtMaxThrottle = MotorPowerMax;

  if (Brake_Volt > Brake_Volt_Threshold) // If the brake is pressed, stop the bike
  {
    event_type = EVENT_TYPE_BRAKE;
    StopMotor(); // Stop the bike by resetting the state and applying the neutral throttle value
  }

  if (StopMotorPower) // If StopMotorPower is set to true in the interrupt routine, stop the bike
  {
    if (Brake_Volt > Brake_Volt_Min) // If the brake is pressed, apply a stronger stop by setting the throttle to zero, otherwise just set it to neutral
    {
      // regen
      float TargetBrakePercent = (Brake_Volt - Brake_Volt_Min) / (Brake_Volt_Max - Brake_Volt_Min) * 100.0f;
      TargetMotorPower = -TargetBrakePercent;  // at negative values, TargetMotorPower is interpreted as a brake percent, to be mapped to the brake throttle voltage for regen
    } else {
      // stp motor
      TargetMotorPower = 0;
    }
    StopMotorPower = false; // Reset StopMotorPower flag to avoid a long stop if the interrupt is triggered again by mistake
#ifdef DEBUG
    Serial.println("Bike stopped since StopMotorPower flag is set");
#endif

  } else {

    if (ThrottleUserControl_Volt > ThrottleInputThreshold) // If the user is controlling the throttle, use that value instead of the automatic one
    {
      TargetMotorPower = mapFloat(ThrottleUserControl_Volt, ThrottleInputMin, ThrottleInputMax,
          0, MotorPowerAtMaxThrottle * PotentiometerFraction); // Map throttle voltage to motor power, to be calibrated
    } else {
      // Calculate the throttle value to apply to the Phase Runner
      float HumanPowerBoostFactor = HumanPowerWattFiltered * HumanBoostFactorMax * PotentiometerFraction ;
      TargetMotorPower = HumanPowerWattFiltered * HumanPowerBoostFactor;
    }

  }

  if (TargetMotorPower < 0) {
    Throttle_Value = mapFloat(-TargetMotorPower, 0, 100,
      MinThrottleBrakeValue, MaxThrottleBrakeValue); // Map brake percent to throttle voltage for regen, to be calibrated
    } else {
    Throttle_Value = mapFloat(TargetMotorPower, 0, MotorPowerAtMaxThrottle, MinThrottleValue,
                                    MaxThrottleValue);
    if (DualMotorMode) Throttle_Value *= 0.5f;
  }

  Throttle_Value = constrain(Throttle_Value, 0.0f, MaxThrottleValue); // Constrain throttle voltage to be between the max brake value and the max throttle value
  analogWrite(ThrottlePin_Out, Throttle_Value * VOLT_TO_ANALOG_OUT);

#ifdef DEBUG

  Serial.print(">");
  // Serial.print("Raw_RPM:");
  // Serial.print(BikerRpmRaw);
  // Serial.print(",Filtered_RPM:");
  Serial.print(BikerRpmFiltered);
  //   Serial.print(",Torque_Value:");
  //   Serial.print(TorqueValue);
  //   Serial.print(",Filtered_Torque_V:");
  Serial.print(TorqueValueFiltered);
  // Serial.print(",Torque_Nm:");
  // Serial.print(TorqueValueFilteredNm);
  //   Serial.print(",Human_Power_Watt:");
  //   Serial.print(HumanPowerWatt);
  // Serial.print(",Filtered_Human_Power_Watt:");
  Serial.print(HumanPowerWattFiltered);
  // Serial.print(",DigitalBrake:");
  // Serial.print(digitalRead(DigitalBrakePin_In));
  Serial.print(",AnalogBrake:");
  Serial.print(Brake_Volt);
  Serial.print(",ThrottleInput:");
  Serial.print(analogRead(ThrottleUserControl_In) * ANALOG_TO_VOLT_IN);
  Serial.print(",Potentiometer:");
  Serial.print(analogRead(PotentiometerPin_In) * ANALOG_TO_VOLT_IN);
  Serial.print(",Motor_Power_Watt:");
  Serial.print(TargetMotorPower);
  Serial.print(",Throttle_Value:");
  Serial.print(Throttle_Value);
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