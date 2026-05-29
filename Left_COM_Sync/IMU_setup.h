#pragma once
#include "Globals.h"

/*
*************************** IMU_setup.h ************************************************

Filename:       IMU_setup.h
Author:         Jestin

Description:    Declares the helpers responsible for bringing up one finger's pair of
                MPU6050 IMUs (mid and proximal) including DMP initialisation and
                calibration, as well as the thin wrapper around the TCA9548A used
                to switch the I2C bus to a given channel.

Dependencies:   Globals.h

*****************************************************************************************
*/


void initFingerChannel(FingerChannel& fc, bool (&status)[4]);

void tcaSelectChannel(uint8_t ch);
