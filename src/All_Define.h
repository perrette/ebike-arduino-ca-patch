#define DEBUG // Comment this line to switch to normal mode

//===================
// I/O configuration
//===================
// Input
#define DigitalBrakePin_In 3 // Digital brake (Interrupt)
#define SpeedPin_In 4        // Speed from torque sensor (Interrupt)
#define DirectionPin_In 6    // Direction from torque sensor (Interrupt)

// Output
#define ThrottlePin_Out 5 // Throttle to Phase Runner

// Analog to digital
#define TorquePin_In A0           // Torque signal from torque sensor
#define ThrottleUserControl_In A1 // Throttle user control
#define Brake_In A2               // Analog brake signal
#define PotentiometerPin_In A3    // General purpose potentiometer
#define REF_VOLT 5.0f             // Reference voltage for the throttle output
#define ANALOG_TO_VOLT_IN REF_VOLT / 1023.0f
#define VOLT_TO_ANALOG_OUT 255.0f / REF_VOLT

//===================================
// Torque/Speed Sensor & Human power
//===================================
// Speed Sensor & Biker RPM :
#define PasPole 48.0                  // Pole nbr
#define RpmAlphaGain 0.95             // To average RPM, higher is smoother [0-1] ! The filtering also depends on the loop duration !
#define TimeStopLimit 200UL           // A stop is considered if the time between two edges is greater than this value (mSec)
uint8_t RpmStopDetector;              // +1 each loop, and cleared each speed pulse. If this > TimeStopLimit : Rpm = zero
#define DebouneTime 2UL               // RPM is not calculated if is lower than this value (mSec)
#define BikerMaxRpm 200               // Max biker rpm to map the throttle
float BikerRpmRaw;                    // Raw Biker RPM value
float BikerRpmFiltered;               // Filtered Biker RPM value
volatile uint32_t TimeBetween2edge;   // Time between 2 rising edges on Speed Signal (uSec)
volatile uint32_t LastTimeOfLastEdge; // Time of last edge on speed signal (uSec)

// Torque Sensor :
// calibration for specific cranks etc (made to work in practice)
// sensor : 0.25 kg / 10V
// pedal krank: 15 cm
// => (10 kg / 0.25 V) * (9.81 m/s2) * (0.15 m) = 58.86 Nm/V
#define TorqueValueNeutral 1.5f // from sensor data sheet
#define TorqueValueMax 3.0f     // from sensor data sheet
#define TorqueSensorGain 60     // Nm/V
// #define TorqueValueMax 500.0f // Maximum torque value in Nm, to be calibrated
#define TorqueValueFilteredAlphaGain 0.9f
float TorqueValue;
float TorqueValueFiltered; // Set in ResetBike() to neutral value to avoid a jump at the beginning

// Human Power
#define HumanPowerWattMax 1000.0f // Constrain the calculation to that value
#define HumanPowerWattAlphaGain 0.0f
#define MotorPowerBoostFactor 1.0f // Motor Watt per Human Watt : Gain to convert the human power to motor power
float HumanPowerWatt;
float HumanPowerWattFiltered; // set in ResetBike() to zero to avoid a jump at the beginning

//=========================
// Throttle to PhaseRunner
//=========================
// Throttle :
#define MinThrottleValue 1.1f
#define MaxThrottleValue 3.8f
#define NeutralThrottleValue 1.0f
#define MotorPowerAtMaxThrottle 1500.0f // Motor power in Watt at max throttle
float Throttle_Value_Volt;              // set in ResetBike() to zero to avoid a jump at the beginning
uint16_t Throttle_Value;                // Throttle Value : [0-255] = Duty Cycle [0-100]%

//==================
// General & Timing
//==================
// State machine
volatile bool StopMotorPower = false; // Set to true in the interrupt routine to stop the bike
uint16_t event_type;
#define EVENT_TYPE_DEFAULT 0
#define EVENT_TYPE_RPM_RESET 1
#define EVENT_TYPE_BACK_PEDALING 2

// Timing
#define LoopTimeUs 50000UL // Main loop duration (uSec)
uint32_t NextLoopTime;     // For loop time calibration

//=========================
// Subroutine declaration
//=========================
void SpeedPulseEvent();        // Interrupt routine called each rising edge on Speed  Signal
void StopBikeEvent();          // Interrupt routine called on falling edge of DirectionPin
void ResetBike();              // Function to reset the bike, can be called in case of emergency or to restart the bike
void ResetTime();              // Function to reset the loop time, to be called when stopping the bike to avoid a long wait before the next loop
void StopBike();               // Function to stop the bike by resetting the state and applying the neutral
void UpdateBikerRpmFiltered(); // Update Biker Rpm filtered with auto Stop detector
