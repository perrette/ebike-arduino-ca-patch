#ifndef ALL_DEFINE_H
#define ALL_DEFINE_H

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

//============================
// Torque Sensor & Human power
//============================

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
extern float TorqueValue;
extern float TorqueValueFiltered; // Set in ResetBike() to neutral value to avoid a jump at the beginning

// Human Power
#define HumanPowerWattMax 1000.0f // Constrain the calculation to that value
#define HumanPowerWattAlphaGain 0.0f
#define MotorPowerBoostFactor 1.0f // Motor Watt per Human Watt : Gain to convert the human power to motor power
extern float HumanPowerWatt;
extern float HumanPowerWattFiltered; // set in ResetBike() to zero to avoid a jump at the beginning

// Brake
#define Brake_Volt_Threshold 1.2f // Threshold voltage to consider the brake is pressed, to be calibrated
#define Brake_Volt_Min 1.25f      // Threshold voltage to consider the brake is pressed, to be calibrated
#define Brake_Volt_Max 4.5f       // Maximum voltage for the brake sensor, to be calibrated

//=========================
// Throttle to PhaseRunner
//=========================
// Throttle :
#define MinThrottleValue 1.1f
#define MaxThrottleValue 3.8f
#define MinThrottleBrakeValue 0.85f
#define MaxThrottleBrakeValue 0.0f
#define NeutralThrottleValue 1.0f
#define MotorPowerAtMaxThrottle 1500.0f // Motor power in Watt at max throttle

//===========
// Bike state
//===========
// volatile bool MotorIsPowered = false;       // Is true if the motor is on (throttle > neutral)

//=================
// Bike transitions
//=================
extern volatile bool StopMotorPower; // Set to true in the interrupt routine to stop the bike

//==================
// General & Timing
//==================
// State machine
extern volatile uint16_t event_type;
#define EVENT_TYPE_DEFAULT 0
#define EVENT_TYPE_RPM_RESET 1
#define EVENT_TYPE_BACK_PEDALING 2
#define EVENT_TYPE_BRAKE 3
// Timing
#define LoopTimeUs 50000UL // Main loop duration (uSec)

//=========================
// Subroutine declaration
//=========================
void ResetBike();               // Used in Setup
void BackPedalEvent();          // Interrupt routine called on falling edge of DirectionPin
void BrakeEvent();              // Interrupt routine called on rising edge of DigitalBrakePin
void StopMotor();               // Function to stop the bike by resetting the state and applying the neutral throttle value

#endif // ALL_DEFINE_H