//=================================================================
// Here, all we need to get and filter Biker Rpm and detect a stop
//=================================================================

void UpdateBikerRpmFiltered()
{
    if (TimeBetween2edge > (DebouneTime * 1000)) // Rpm is not calculated if time is too low between two edges on speed signal
    {
        BikerRpmRaw = 60000000.0 / (TimeBetween2edge * PasPole);                               // RPM calculation
        BikerRpmFiltered = BikerRpmFiltered * RpmAlphaGain + BikerRpmRaw * (1 - RpmAlphaGain); // Add Filter to average
    }

    if (RpmStopDetector++ > (1000 * TimeStopLimit / LoopTimeUs)) // Reset Biker Rpm if the time since the last pulse exceeds TimeStopLimit
    {
        event_type = EVENT_TYPE_RPM_RESET;
#ifdef DEBUG
        Serial.println("Rpm reset to zero since no pulse detected for a while (deactivated)"); // Remove @ Release
#endif
        BikerRpmFiltered = 0;
    }

    BikerRpmFiltered = constrain(BikerRpmFiltered, 0, BikerMaxRpm);
}

void SpeedPulseEvent() // Interrupt called each rising edge and refresh time between 2 edges if it's valid
{
    uint32_t CurrentTime = micros();                         // save current time (uSec)
    if (CurrentTime > LastTimeOfLastEdge)                    // Don't refresh time since last edge if micro() overflowed
        TimeBetween2edge = CurrentTime - LastTimeOfLastEdge; // Get time since last edge
    LastTimeOfLastEdge = CurrentTime;                        // Save current time for next calculation
    RpmStopDetector = 0;                                     // Reset Detector each interrupt on Speed signal
}
