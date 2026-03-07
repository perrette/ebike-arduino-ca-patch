#ifndef ALL_DEFINE_H
#define ALL_DEFINE_H

#define DEBUG // Comment this line to switch to normal mode
#define DEBUG_ALL 0
#define DEBUG_PHYSICS 1
#define DEBUG_VOLTAGE 2
#define DEBUG_EBRAKE 3

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
// Sensor specs
// SEMPU T2: 0.25 V / 10 kg.f
// however it is tricky to calculate in practice (would require knowledge of the exact geometry of the sensor, the crank, and the pedal...)

// Torque Sensor calibration
// -------------------------
// pedal crank length (m) x weight (kg) x 9.81 (m/s^2) = measured torque in Nm when standing on the pedal with crank in horizontal position
// Example: man of 70 kg standing on a 15 cm pedal in horizontal position
// with a 15 cm crank, should result in about 105 Nm
// For a typical 15 cm crank, you should adjust the gain to obtain a measurement of 1.5 times the weight in kg.
// A lower weight, more representative of pedalling strength, is likely better for calibration. Below a crank of 15 cm is assumed:
// - recreational cyclist : 10 kg => 15 Nm
// - sportive cyclist : 20-30 kg => 30-45 Nm
// - sprint or standing start: 70-100 kg (up to three times the body weight) => 105-150 Nm
#define TorqueValueNeutral 1.5f // from sensor data sheet
#define TorqueValueMax 3.0f     // from sensor data sheet
#define TorqueSensorGain 80     // Nm/V (range 60-100 in practice, to be calibrated as explained above)
#define TorqueValueFilteredAlphaGain 0.9f
extern float TorqueValue;
extern float TorqueValueFiltered; // Set in ResetBike() to neutral value to avoid a jump at the beginning

// Human Power
#define HumanPowerWattMax 1000.0f // Constrain the calculation to that value
#define HumanPowerWattAlphaGain 0.0f
#define HumanBoostFactorMax 4.0f
extern float HumanPowerWatt;
extern float HumanPowerWattFiltered; // set in ResetBike() to zero to avoid a jump at the beginning

//=========================
// Analog E-Brake
//=========================
#define Brake_Volt_Threshold 1.2f // Threshold voltage to consider the brake is pressed
#define Brake_Volt_Min 1.25f      // Minimum voltage to consider the brake is pressed
#define Brake_Volt_Max 4.5f       // Maximum voltage for the brake sensor

//=========================
// Input throttle
//=========================
#define ThrottleInputThreshold 1.0f // Voltage corresponding to zero throttle
#define ThrottleInputMin 1.1f // Voltage corresponding to zero throttle (measured 0.9V)
#define ThrottleInputMax 4.2f // Voltage corresponding to max throttle

//=========================
// Input Potentiometer
//=========================
#define PotentiometerMin 0.9f // Voltage corresponding to minimum potentiometer value
#define PotentiometerMax 5.0f // Voltage corresponding to maximum potentiometer value

//=========================
// Throttle to PhaseRunner
//=========================
#define MinThrottleValue 1.1f
#define MaxThrottleValue 3.8f
#define MinThrottleBrakeValue 0.85f
#define MaxThrottleBrakeValue 0.0f
#define NeutralThrottleValue 1.0f
#define DualMotorMode false // If true, the throttle will be applied to both motors, to the target motor power is split in two (halved)
#define MotorPowerMax 1500.0f // Motor power in Watt at max throttle, used to map the throttle voltage to motor power, to be modulated


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
#define EVENT_TYPE_BRAKE_DIGITAL 3
#define EVENT_TYPE_BRAKE_ANALOG 4
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