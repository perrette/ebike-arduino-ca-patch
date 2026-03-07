#include <stdbool.h>
#include <Arduino.h>
#include "All_Define.h"
#include "BikerRPM.h"

volatile uint16_t event_type;
volatile bool StopMotorPower;
float HumanPowerWatt;
float HumanPowerWattFiltered;
float TorqueValue;
float TorqueValueFiltered;

void ResetBike() // Function to reset the bike to be sure to start with a known state
{
#ifdef DEBUG
    Serial.println("Bike reset to default state");
#endif
    StopMotorPower = false;                      // Reset StopMotorPower flag to avoid a long stop if the interrupt is triggered by mistake
    BikerRpmFiltered = 0;                        // Set filtered RPM to zero to avoid a jump when starting the bike
    TorqueValueFiltered = TorqueValueNeutral;   // Set filtered torque value to neutral value to avoid a jump when starting the bike
    HumanPowerWattFiltered = 0;                 // Set filtered human power to zero to avoid a jump when starting the bike
    event_type = EVENT_TYPE_DEFAULT;                      // Set event type to default
}

void StopMotor() // Function to stop the bike by resetting the state and applying the neutral throttle value
{
#ifdef DEBUG
    Serial.println("StopMotor: Set StopMotorPower to true");
#endif
    StopMotorPower = true; // Set flag to stop the bike
}

void BackPedalEvent() // Interrupt called on falling edge of DirectionPin
{
#ifdef DEBUG
    Serial.println("BackPedalEvent called");
#endif
    event_type = EVENT_TYPE_BACK_PEDALING;
    // StopMotor(); // Stop the bike by resetting the state and applying the neutral throttle value
}

void BrakeEvent() // Interrupt called on rising edge of BrakePin
{
#ifdef DEBUG
    Serial.println("BrakeEvent called");
#endif
    event_type = EVENT_TYPE_BRAKE_DIGITAL;
    StopMotor(); // Stop the bike by resetting the state and applying the neutral throttle value
}