#pragma once

void initWiFiControl();
void updateWiFiControl();
void setAmplitude(float amplitude);

// Return timestamp (millis) of last received control packet
unsigned long getLastControlPacket();

// Broadcast a small trim-update JSON over WebSocket to notify clients
void broadcastTrimUpdate(int left, int right, float steeringTrim);