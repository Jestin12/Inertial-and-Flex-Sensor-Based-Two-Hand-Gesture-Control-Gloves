#pragma once
#include "MPU6050_6Axis_MotionApps612.h"
#include <TCA9548A.h>
#include <ArduinoJson.h>

/*
*************************** Globals.h **************************************************

Filename:       Globals.h
Author:         Jestin

Description:    Shared declarations for the left-hand glove firmware. This header
                defines the FingerChannel struct, which binds a TCA9548A channel and
                a pair of flex-sensor ADC pins to a finger label, and exposes the
                global IMU handles, the per-finger HandChannels array, and the DMP
                state variables that are written by IMU_setup.cpp and read by the
                main streaming loop.

Dependencies:   MPU6050_6Axis_MotionApps612     TCA9548A        ArduinoJson

*****************************************************************************************
*/


struct FingerChannel {
  /*
  Configuration record for a single finger (or the palm) on the glove.

  Members:
      tca_channel (int):      The TCA9548A bus channel that the two finger MPU6050s
                              and any local I2C peripherals sit behind.

      label (String):         Human-readable name of the finger, used as the JSON
                              key for that finger's data block.

      adc_channel (int[2]):   ESP32 ADC pin numbers for the MCP and PIP flex sensors
                              on this finger. Either value can be -1 to indicate
                              that no flex sensor is wired on that joint.

      IMU_MID_EN (bool):      Set true by initFingerChannel() once the mid-segment
                              MPU6050 on this channel has been detected and its
                              DMP has been brought up successfully.

      IMU_PROX_EN (bool):     Set true by initFingerChannel() once the proximal
                              MPU6050 on this channel has been detected and its
                              DMP has been brought up successfully.
  */
  const int tca_channel;
  String    label;
  const int adc_channel[2];
  bool      IMU_MID_EN;
  bool      IMU_PROX_EN;
};


// Shared hardware handles, defined in the main .ino
extern MPU6050   IMU_MID;
extern MPU6050   IMU_PROX;
extern TCA9548A  TCA;
extern FingerChannel HandChannels[6];

// DMP state for the mid-segment MPU6050, written by initFingerChannel() and
// consumed by the main loop's DMP read helpers
extern bool     dmpReady1;
extern uint8_t  devStatus1;
extern uint16_t packetSize1;

// DMP state for the proximal-segment MPU6050, written by initFingerChannel() and
// consumed by the main loop's DMP read helpers
extern bool     dmpReady2;
extern uint8_t  devStatus2;
extern uint16_t packetSize2;
