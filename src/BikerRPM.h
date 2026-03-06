#ifndef BIKERPM_H
#define BIKERPM_H

//=============
// Speed Sensor
//=============
#define PasPole 48.0                  // Pole nbr
#define RpmAlphaGain 0.95             // To average RPM, higher is smoother [0-1] ! The filtering also depends on the loop duration !
#define TimeStopLimit 200UL           // A stop is considered if the time between two edges is greater than this value (mSec)
#define DebouneTime 2UL               // RPM is not calculated if is lower than this value (mSec)
#define BikerMaxRpm 200               // Max biker rpm to map the throttle

extern float BikerRpmRaw;               // Raw Biker RPM value
extern float BikerRpmFiltered;               // Filtered Biker RPM value

void UpdateBikerRpmFiltered(); // Update Biker Rpm filtered with auto Stop detector
void SpeedPulseEvent();        // Interrupt routine called each rising edge on SpeedPin to refresh time between

#endif // BIKERPM_H