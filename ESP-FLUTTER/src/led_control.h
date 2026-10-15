#pragma once

void initLedControl();
void updateLedControl();
void reattachLedOutput();

void setLedMode(int mode);
void cycleLedMode();
int getLedMode();

void setLedBrightnessLevel(int level);
int getLedBrightnessLevel();

void setLedEffectSpeedLevel(int level);
int getLedEffectSpeedLevel();

const char *getLedModeName();
