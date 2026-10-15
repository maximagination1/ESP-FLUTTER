#include "flight_mode.h"
#include <Arduino.h>

static FlightMode currentMode = FLIGHT_MODE_DISARMED;

void setFlightMode(FlightMode m)
{
    if (m != currentMode)
    {
        currentMode = m;
        Serial.printf("Flight mode changed to %s\n", flightModeToString(m));
    }
}

FlightMode getFlightMode()
{
    return currentMode;
}

const char* flightModeToString(FlightMode m)
{
    switch (m)
    {
        case FLIGHT_MODE_DISARMED: return "DISARMED";
        case FLIGHT_MODE_ARMED: return "ARMED";
        case FLIGHT_MODE_PERCH: return "PERCH";
        default: return "UNKNOWN";
    }
}

void setFlightModeFromInt(int v)
{
    if (v < 0) v = 0;
    if (v > 2) v = 2;
    setFlightMode((FlightMode)v);
}

void initFlightMode()
{
    currentMode = FLIGHT_MODE_DISARMED;
    Serial.printf("Flight mode initialized: %s\n", flightModeToString(currentMode));
}
