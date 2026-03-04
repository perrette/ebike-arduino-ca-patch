#include <Arduino.h>
void SpeedPulseEvent(); // Interrupt routine called each rising edge on Speed Signal

// I/O configuration
#define SpeedPin 4    // Speed Pin on D4
#define ThrottlePin 5 // Throttle Pin on D5

// Speed Sensor :
#define PasPole 48.0         // Pole nbr
#define RpmAlphaGain 0.95    // To average RPM, higher is smoother [0-1] ! The filtering also depends on the loop duration !
#define DebouneTime 1UL      // RPM is not calculated if is lower than this value (mSec)
#define BikerMaxRpm 20000.0  // Max biker rpm to map the throttle
#define TimeStopLimit 200UL  // A stop is considered if the time between two edges is greater than this value (mSec)
uint8_t RpmStopDetector;     // +1 each loop, and cleared each speed pulse. If this > TimeStopLimit : Rpm = zero
float BikerRpm;              // Raw Biker RPM value
float BikerRpmFiltered;      // Filtered Biker RPM value
uint32_t TimeBetween2edge;   // Time between 2 rising edges on Speed Signal (uSec)
uint32_t LastTimeOfLastEdge; // Time of last edge on speed signal (uSec)

// Throttle :
uint16_t Throttle_Value = 128; // Throttle Value : [0-1023] = Duty Cycle [0-100]%
#define MinThrottleValue 40    // = 0.8v
#define MaxThrottleValue 240   // = 4v

// Timing
#define LoopTimeUs 20000UL // Main loop duration (uSec)
uint32_t NextLoopTime;     // For loop time calibration

void setup()
{
  // I/O init
  pinMode(SpeedPin, INPUT_PULLUP);
  pinMode(ThrottlePin, OUTPUT);
  Serial.begin(115200);

  // Interrupt init
  attachInterrupt(digitalPinToInterrupt(SpeedPin), SpeedPulseEvent, RISING); // Call SpeedPulseEvent each rising edge on SpeedPin
}

void loop()
{

  if (TimeBetween2edge > (DebouneTime * 1000)) // Rpm is not calculated if time is too low between two edges on speed signal
  {
    BikerRpm = 60000000.0 / (TimeBetween2edge * PasPole);                               // RPM calculation
    BikerRpmFiltered = BikerRpmFiltered * RpmAlphaGain + BikerRpm * (1 - RpmAlphaGain); // Add Filter to average
  }

  if (RpmStopDetector++ > (1000 * TimeStopLimit / LoopTimeUs)) // Reset Biker Rpm if the time since the last pulse exceeds TimeStopLimit
    BikerRpmFiltered = 0;

  BikerRpmFiltered = constrain(BikerRpmFiltered, 0.0, BikerMaxRpm);
  Throttle_Value = map(BikerRpmFiltered, 0, BikerMaxRpm, MinThrottleValue, MaxThrottleValue);
  analogWrite(ThrottlePin, Throttle_Value); // Throttle_Value has to be set as you want

  Serial.print(">"); // Remove @ Release
  Serial.print("Raw_RPM:");
  Serial.print(BikerRpm);
  Serial.print(",Filtered_RPM:");
  Serial.print(BikerRpmFiltered);
  Serial.println();

  while (micros() < NextLoopTime) // wait before starting next loop
    ;
  NextLoopTime += LoopTimeUs; // Init next loop time
}

void SpeedPulseEvent() // Interrupt called each rising edge and refresh time between 2 edges if it's valid
{
  uint32_t CurrentTime = micros();                       // save current time (uSec)
  if (CurrentTime > LastTimeOfLastEdge)                  // Don't refresh time since last edge if micro() overflowed
    TimeBetween2edge = CurrentTime - LastTimeOfLastEdge; // Get time since last edge
  LastTimeOfLastEdge = CurrentTime;                      // Save current time for next calculation
  RpmStopDetector = 0;                                   // Clear Rpm Detector each interrupt
}