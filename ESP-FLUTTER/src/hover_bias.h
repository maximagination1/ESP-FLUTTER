#pragma once

void initHoverBias();

void setHoverBiasEnabled(bool enabled);
bool isHoverBiasEnabled();

void setHoverBiasConfig(float forwardNeutralDeg, float hoverNeutralDeg);
void updateHoverBiasDevicePitch(float devicePitchDeg);

float getHoverBias();
float getHoverBiasNeutralAngle(float fallbackNeutralDeg);
float getHoverBiasDevicePitch();
float getHoverBiasForwardNeutralAngle();
float getHoverBiasHoverNeutralAngle();
