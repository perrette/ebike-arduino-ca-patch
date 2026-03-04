#include <Arduino.h>
#include "helpers.h"

void SpeedPulseEvent(); // Interrupt routine called each rising edge on Speed
                        // Signal

// I/O configuration
#define SpeedPin 4          // Speed Pin on D4
#define ThrottlePin 5       // Throttle Pin on D5
#define ThrottleCheckPin A1 // Throttle Pin on A1
#define TorquePin A0        // Torque Pin on A0

#define REF_VOLT 5.0f // Reference voltage for the throttle output
#define ANALOG_TO_VOLT_IN REF_VOLT / 1023.0f
#define VOLT_TO_ANALOG_OUT 255.0f / REF_VOLT

// Speed Sensor :
#define PasPole 48.0 // Pole nbr
#define RpmAlphaGain \
  0.95 // To average RPM, higher is smoother [0-1] ! The filtering also depends
       // on the loop duration !
#define TimeStopLimit \
  200UL // A stop is considered if the time between two edges is greater than
        // this value (mSec)
#define DebouneTime \
  2UL                        // RPM is not calculated if is lower than this value (mSec)
#define BikerMaxRpm 200      // Max biker rpm to map the throttle
float BikerRpm;              // Raw Biker RPM value
float BikerRpmFiltered;      // Filtered Biker RPM value
uint32_t TimeBetween2edge;   // Time between 2 rising edges on Speed Signal (uSec)
uint32_t LastTimeOfLastEdge; // Time of last edge on speed signal (uSec)

// from sensor data sheet
#define TorqueValueNeutral 1.5f
#define TorqueValueMax 3.0f
// #define TorqueValueSlope 0.025f  // 0.25V / 10Kg.f
// calibration for specific cranks etc (made to work in practice)
#define TorqueSensorGain 100 // Nm/V, to be calibrated with a torque bench
// #define TorqueValueMax 500.0f // Maximum torque value in Nm, to be calibrated
#define TorqueValueFilteredAlphaGain 0.9f
float TorqueValue;
float TorqueValueFiltered = TorqueValueNeutral; // initialize filtered torque value to neutral value to avoid a jump at the beginning

#define HumanPowerWattMax 200.0f // Max throttle at that power
#define HumanPowerWattAlphaGain 0.0f
float HumanPowerWatt;
float HumanPowerWattFiltered = 0.0f;

// Throttle :
float Throttle_Value_Volt = 0.8;
uint16_t Throttle_Value =
    Throttle_Value_Volt *
    VOLT_TO_ANALOG_OUT; // Throttle Value : [0-1023] = Duty Cycle [0-100]%
// 128;     // Throttle Value : [0-1023] = Duty Cycle [0-100]%
#define MinThrottleValue 1.1f
#define MaxThrottleValue 3.5f

// Timin
#define LoopTimeUs 50000UL // Main loop duration (uSec)
uint32_t NextLoopTime;     // For loop time calibration

void setup()
{
  // I/O init
  pinMode(SpeedPin, INPUT_PULLUP);
  pinMode(ThrottlePin, OUTPUT);
  pinMode(ThrottleCheckPin, INPUT);
  pinMode(TorquePin, INPUT);

  Serial.begin(115200);

  // Interrupt init
  attachInterrupt(digitalPinToInterrupt(SpeedPin), SpeedPulseEvent,
                  RISING); // Call SpeedPulseEvent each rising edge on SpeedPin
}

void loop()
{

  if (TimeBetween2edge >
      (DebouneTime * 1000)) // Rpm is not calculated if time is too low between
                            // two edges on speed signal
  {
    BikerRpm = 60000000.0 / (TimeBetween2edge * PasPole); // RPM calculation
    BikerRpmFiltered = BikerRpmFiltered * RpmAlphaGain +
                       BikerRpm * (1 - RpmAlphaGain); // Add Filter to average
  }

  // TODO: treat cases where the loop is slower than the poles counts (an
  // interruption situation occurs in between two loops passes)
  // In practice, my guess is this is not critical, since that would mean the
  // biker restarted / continued to pedal in the meanwhile
  if ((micros() - LastTimeOfLastEdge) >
      TimeStopLimit * 1000UL) // Stop dectector
    BikerRpmFiltered = 0;

  BikerRpmFiltered = constrain(BikerRpmFiltered, 0, BikerMaxRpm);

  TorqueValue = analogRead(TorquePin) * ANALOG_TO_VOLT_IN;
  TorqueValueFiltered = TorqueValueFiltered * TorqueValueFilteredAlphaGain + TorqueValue * (1 - TorqueValueFilteredAlphaGain);
  constrain(TorqueValueFiltered, TorqueValueNeutral, TorqueValueMax);                          // Constrain filtered torque value to be between 0 and reference voltage
  float TorqueValueFilteredNm = (TorqueValueFiltered - TorqueValueNeutral) * TorqueSensorGain; // Convert voltage to torque in Nm, and remove the neutral value

  HumanPowerWatt =
      TorqueValueFilteredNm * BikerRpmFiltered * 2 * 3.14159265f / 60;

  HumanPowerWattFiltered = HumanPowerWattFiltered * HumanPowerWattAlphaGain + HumanPowerWatt * (1 - HumanPowerWattAlphaGain);
  HumanPowerWattFiltered = constrain(HumanPowerWattFiltered, 0, HumanPowerWattMax);

  Throttle_Value_Volt =
      mapFloat(HumanPowerWattFiltered, 0, HumanPowerWattMax, MinThrottleValue,
               MaxThrottleValue);                            // Map RPM to throttle voltage
  Throttle_Value = Throttle_Value_Volt * VOLT_TO_ANALOG_OUT; //
  analogWrite(ThrottlePin,
              Throttle_Value); // Throttle_Value has to be set as you want

  bool DEBUG_MODE = true;
  if (DEBUG_MODE)
  {
    Serial.print(">"); // Remove @ Release
    Serial.print("Raw_time_between_2_edge_uS:");
    Serial.print(BikerRpm);
    Serial.print(",Filtered_RPM:");
    Serial.print(BikerRpmFiltered);
    Serial.print(",Torque_Value:");
    Serial.print(TorqueValue);
    Serial.print(",Filtered_Torque_V:");
    Serial.print(TorqueValueFiltered);
    Serial.print(",Filtered_Torque_Nm:");
    Serial.print(TorqueValueFilteredNm);
    Serial.print(",Human_Power_Watt:");
    Serial.print(HumanPowerWatt);
    Serial.print(",Filtered_Human_Power_Watt:");
    Serial.print(HumanPowerWattFiltered);
    Serial.print(",Throttle_Value:");
    Serial.print(Throttle_Value_Volt);
    Serial.print(",Throttle_Check:");
    Serial.print(analogRead(ThrottleCheckPin) * ANALOG_TO_VOLT_IN);
    Serial.println();
  }

  while (micros() < NextLoopTime) // wait before starting next loop
    ;
  NextLoopTime = micros() + LoopTimeUs; // Init next loop time
}

void SpeedPulseEvent() // Interrupt called each rising edge and refresh time
                       // between 2 edges if it's valid
{
  uint32_t CurrentTime = micros();      // save current time (uSec)
  if (CurrentTime > LastTimeOfLastEdge) // Don't refresh time since last edge if
                                        // micro() overflowed
    TimeBetween2edge =
        CurrentTime - LastTimeOfLastEdge; // Get time since last edge
  LastTimeOfLastEdge = CurrentTime;       // Save current time for next calculation
}