#include <Arduino.h>
#include <avr/wdt.h>
#include "helpers.h"

void SpeedPulseEvent(); // Interrupt routine called each rising edge on Speed
                        // Signal

void StopBikeEvent(); // Interrupt routine called on falling edge of DirectionPin
void ResetBike(); // Function to reset the bike, can be called in case of emergency or to restart the bike
void ResetTime(); // Function to reset the loop time, to be called when stopping the bike to avoid a long wait before the next loop
void StopBike();  // Function to stop the bike by resetting the state and applying the neutral

// I/O configuration
#define DirectionPin 3      // Direction Pin on D4
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
  200UL                  // A stop is considered if the time between two edges is greater than
                         // this value (mSec)
uint8_t RpmStopDetector 200UL ; // +1 each loop, and cleared each speed pulse. If this > TimeStopLimit : Rpm = zero
                    // A stop is considered if the time between two edges is greater than
                         // this value (mSec)
uint8_t RpmStopDetector; // +1 each loop, and cleared each speed pulse. If this > TimeStopLimit : Rpm = zero
#define DebouneTime \
  2UL                        // RPM is not calculated if is lower than this value (mSec)
#define BikerMaxRpm 200      // Max biker rpm to map the throttle
float BikerRpm;              // Raw Biker RPM value
float BikerRpmFiltered;      // Filtered Biker RPM value
volatile uint32_t TimeBetween2edge;   // Time between 2 rising edges on Speed Signal (uSec)
volatile uint32_t LastTimeOfLastEdge; // Time of last edge on speed signal (uSec)

// from sensor data sheet
#define TorqueValueNeutral 1.5f
#define TorqueValueMax 3.0f
// calibration for specific cranks etc (made to work in practice)
// sensor : 0.25 kg / 10V
// pedal krank: 15 cm
// => (10 kg / 0.25 V) * (9.81 m/s2) * (0.15 m) = 58.86 Nm/V
#define TorqueSensorGain 60 // Nm/V
// #define TorqueValueMax 500.0f // Maximum torque value in Nm, to be calibrated
#define TorqueValueFilteredAlphaGain 0.9f
float TorqueValue;
float TorqueValueFiltered ; // Set in ResetBike() to neutral value to avoid a jump at the beginning

#define HumanPowerWattMax 1000.0f // Constrain the calculation to that value
#define HumanPowerWattAlphaGain 0.0f
#define MotorPowerBoostFactor 1.0f // Motor Watt per Human Watt : Gain to convert the human power to motor power
float HumanPowerWatt;
float HumanPowerWattFiltered; // set in ResetBike() to zero to avoid a jump at the beginning

// Throttle :
#define MinThrottleValue 1.1f
#define MaxThrottleValue 3.8f
#define NeutralThrottleValue 1.0f
#define MotorPowerAtMaxThrottle 1500.0f // Motor power in Watt at max throttle

float Throttle_Value_Volt; // set in ResetBike() to zero to avoid a jump at the beginning
uint16_t Throttle_Value =
    Throttle_Value_Volt *
    VOLT_TO_ANALOG_OUT; // Throttle Value : [0-1023] = Duty Cycle [0-100]%
// 128;     // Throttle Value : [0-1023] = Duty Cycle [0-100]%

// State machine
volatile bool StopMotorPower = false; // Set to true in the interrupt routine to stop the bike

// Timin
#define LoopTimeUs 50000UL // Main loop duration (uSec)
uint32_t NextLoopTime;     // For loop time calibration

void setup()
{
  // I/O init
  pinMode(DirectionPin, INPUT_PULLUP);
  pinMode(SpeedPin, INPUT_PULLUP);
  pinMode(ThrottlePin, OUTPUT);
  // pinMode(ThrottleCheckPin, INPUT);
  pinMode(TorquePin, INPUT);

  Serial.begin(115200);

  // Interrupt init
  attachInterrupt(digitalPinToInterrupt(DirectionPin), StopBikeEvent,
                  FALLING); // Call StopBikeEvent each falling edge on DirectionPin
  attachInterrupt(digitalPinToInterrupt(SpeedPin), SpeedPulseEvent,
                  RISING); // Call SpeedPulseEvent each rising edge on SpeedPin

  wdt_enable(WDTO_500MS); // Set 0.5 Sec watchdog

  ResetBike(); // Reset the bike to be sure to start with a known state
}

#define DEBUG_MODE true
// #define DEBUG_MODE false

uint16_t event_type;
#define EVENT_TYPE_DEFAULT 0
#define EVENT_TYPE_RPM_RESET 1
#define EVENT_TYPE_BACK_PEDALING 2

void loop()
{

  event_type = EVENT_TYPE_DEFAULT;
  wdt_reset(); // Must be called at least every 500ms otherwise Arduino Reset is triggered  

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
  // biker restarted / continued to pedal in the meanwhile  =====>>>> Should work well now
    if (RpmStopDetector++ > (1000 * TimeStopLimit / LoopTimeUs))   // Reset Biker Rpm if the time since the last pulse exceeds TimeStopLimit
    //if (micros() - LastTimeOfLastEdge > TimeStopLimit * 1000) // Reset Biker Rpm if the time since the last pulse exceeds TimeStopLimit
    // if (false) // Reset Biker Rpm if the time since the last pulse exceeds TimeStopLimit
    {
      event_type = EVENT_TYPE_RPM_RESET;
      if (DEBUG_MODE)
        Serial.println("Rpm reset to zero since no pulse detected for a while (deactivated)"); // Remove @ Release
      BikerRpmFiltered = 0;
      // StopBike(); // Stop the bike if no pulse is detected for a while, to avoid a long wait before stopping the bike
      // return;
    }

  if (StopMotorPower) // If StopMotorPower is set to true in the interrupt routine, stop the bike
  {
    event_type = EVENT_TYPE_BACK_PEDALING;
    // BikerRpmFiltered = 0; // Set filtered RPM to zero to avoid a jump when stopping the bike
    StopMotorPower = false; // Reset StopMotorPower flag to avoid a long stop if the interrupt is triggered again by mistake
    // StopBike() ; // Stop the bike by resetting the state and applying the neutral throttle value
    if (DEBUG_MODE)
      Serial.println("Bike stopped since StopMotorPower flag is set (deactivated)"); // Remove @ Release
    // return; // Skip the rest of the loop
  }

  BikerRpmFiltered = constrain(BikerRpmFiltered, 0, BikerMaxRpm);

  // if (DEBUG_MODE)
  //   BikerRpmFiltered = 30.0f; // Remove @ Release

  TorqueValue = analogRead(TorquePin) * ANALOG_TO_VOLT_IN;
  TorqueValueFiltered = TorqueValueFiltered * TorqueValueFilteredAlphaGain + TorqueValue * (1 - TorqueValueFilteredAlphaGain);
  TorqueValueFiltered = constrain(TorqueValueFiltered, TorqueValueNeutral, TorqueValueMax);       // Constrain filtered torque value to be between 0 and reference voltage
  float TorqueValueFilteredNm = (TorqueValueFiltered - TorqueValueNeutral) * TorqueSensorGain; // Convert voltage to torque in Nm, and remove the neutral value

  HumanPowerWatt =
      TorqueValueFilteredNm * BikerRpmFiltered * 2.0 * 3.14159265f / 60.0;

  HumanPowerWattFiltered = HumanPowerWattFiltered * HumanPowerWattAlphaGain + HumanPowerWatt * (1 - HumanPowerWattAlphaGain);
  HumanPowerWattFiltered = constrain(HumanPowerWattFiltered, 0, HumanPowerWattMax);

  float MotorPower = HumanPowerWattFiltered * MotorPowerBoostFactor;

  Throttle_Value_Volt =
      mapFloat(MotorPower, 0, MotorPowerAtMaxThrottle, MinThrottleValue,
               MaxThrottleValue);                            // Map RPM to throttle voltage

  Throttle_Value = Throttle_Value_Volt * VOLT_TO_ANALOG_OUT; //
  analogWrite(ThrottlePin,
              Throttle_Value); // Throttle_Value has to be set as you want

  if (DEBUG_MODE)
  {
    Serial.print(">"); // Remove @ Release
    Serial.print("RPM:");
    Serial.print(BikerRpm);
    Serial.print(",Filtered_RPM:");
    Serial.print(BikerRpmFiltered);
    Serial.print(",Torque_Value:");
    Serial.print(TorqueValue);
    Serial.print(",Filtered_Torque_V:");
    Serial.print(TorqueValueFiltered);
    Serial.print(",Torque_Nm:");
    Serial.print(TorqueValueFilteredNm);
    Serial.print(",Human_Power_Watt:");
    Serial.print(HumanPowerWatt);
    Serial.print(",Filtered_Human_Power_Watt:");
    Serial.print(HumanPowerWattFiltered);
    Serial.print(",Motor_Power_Watt:");
    Serial.print(MotorPower);
    Serial.print(",Throttle_Value:");
    Serial.print(Throttle_Value_Volt);
    // Serial.print(",Throttle_Check:");
    // Serial.print(analogRead(ThrottleCheckPin) * ANALOG_TO_VOLT_IN);
    Serial.print(",Event_Type:");
    Serial.print(event_type);
    Serial.println();
    Serial.println("Time left to wait (ms): " + String((NextLoopTime - micros()) / 1000.0f) + " ms (total " + String(LoopTimeUs / 1000.0f) + " ms)"); // Remove @ Release
  }

  while (micros() < NextLoopTime) // wait before starting next loop
    ;
  NextLoopTime += LoopTimeUs; // Init next loop time
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

void StopBikeEvent() // Interrupt called on falling edge of DirectionPin
{
  StopMotorPower = true; // Set flag to stop the bike
  if (DEBUG_MODE)
    Serial.println("StopBikeEvent called"); // Remove @ Release
}

void ResetBike() // Function to reset the bike, can be called in case of emergency or to restart the bike
{
  StopMotorPower = false; // Reset StopMotorPower flag
  BikerRpmFiltered = 0;   // Set filtered RPM to zero
  TorqueValueFiltered = TorqueValueNeutral; // Set torque to neutral value to avoid a jump when restarting
  HumanPowerWattFiltered = 0; // Set human power to zero to avoid a jump when restarting
  Throttle_Value = NeutralThrottleValue;    // Reset throttle to neutral value
  //TimeBetween2edge = 10e10; // must be larger than deboune time
  //LastTimeOfLastEdge = micros(); // must not be larger than current (micros()) + TimeStopLimit to avoid a long wait before stopping the bike in case of no pulse detected
  //RpmStopDetector = 0;
}

void ResetTime() {
  NextLoopTime = micros() + LoopTimeUs;     // Reset loop time to avoid a long wait before the next loop
}

void StopBike() // Function to stop the bike by resetting the state and applying the neutral throttle value
{
  analogWrite(ThrottlePin, NeutralThrottleValue); // Apply the throttle value to stop the bike
  ResetBike();
  ResetTime();
}