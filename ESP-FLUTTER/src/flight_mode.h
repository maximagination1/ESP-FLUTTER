#pragma once

typedef enum {
    FLIGHT_MODE_DISARMED = 0,
    FLIGHT_MODE_ARMED = 1,
    FLIGHT_MODE_PERCH = 2,
} FlightMode;

// Set/get current flight mode
void setFlightMode(FlightMode m);
FlightMode getFlightMode();

// Convert mode to human-readable string
const char* flightModeToString(FlightMode m);

// Helper to set mode from integer (web UI)
void setFlightModeFromInt(int v);

// Initialize flight mode state (loads persisted selection)
void initFlightMode();

