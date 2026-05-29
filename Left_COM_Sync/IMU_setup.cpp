#include <Arduino.h>
#include "Globals.h"
#include "IMU_setup.h"

/*
*************************** IMU_setup.cpp **********************************************

Filename:       IMU_setup.cpp
Author:         Jestin

Description:    Implementation of the I2C multiplexer and per-finger IMU bring-up
                helpers. tcaSelectChannel() switches the TCA9548A so subsequent
                I2C transactions reach the IMUs on a given finger. initFingerChannel()
                drives the full initialisation sequence for that finger's pair of
                MPU6050s, including connection probing, DMP setup, accel/gyro
                calibration, and reporting per-stage timings over Serial. The four
                flag positions in the status[] array record which of the four
                bring-up stages succeeded so the caller can print a per-channel
                summary.

Dependencies:   Arduino.h       Globals.h       IMU_setup.h

*****************************************************************************************
*/


void tcaSelectChannel(uint8_t ch)
{
  /*
  Closes every TCA9548A channel and reopens the requested one, so subsequent
  I2C transactions on the shared bus reach only the devices behind channel ch.

  Input:
      ch (uint8_t):   TCA9548A channel index to activate (0 to 7).

  Output:
      No return value, the multiplexer state is updated in place.
  */
  TCA.closeAll();
  TCA.openChannel(ch);
}


void initFingerChannel(FingerChannel& fc, bool (&status)[4])
{
  /*
  Brings up the pair of MPU6050s sitting behind one TCA channel. The function
  selects the channel, probes both IMUs, runs dmpInitialize() and calibration
  on each one that responds, enables the DMP, and updates fc.IMU_MID_EN /
  fc.IMU_PROX_EN to reflect which IMUs are usable. Each major stage is timed
  via millis() and the durations are printed to Serial for debugging slow
  initialisation paths.

  Input:
      fc (FingerChannel&):    The configuration record for the finger being
                              initialised. The IMU_MID_EN and IMU_PROX_EN
                              flags are updated in place to reflect which
                              IMUs were brought up successfully.

      status (bool (&)[4]):   Output array used by the caller to print a
                              per-stage summary. The four slots are written
                              as follows:
                                  status[0]: IMU_MID detected
                                  status[1]: IMU_MID DMP enabled
                                  status[2]: IMU_PROX detected
                                  status[3]: IMU_PROX DMP enabled

  Output:
      No return value, fc and status are updated in place.
  */
  unsigned long t_total = millis();

  tcaSelectChannel(fc.tca_channel);

  Serial.println("==== Initialising " + fc.label + " ====");

  // Probe both IMUs and time how long the base initialize() call takes
  unsigned long t = millis();
  IMU_MID.initialize();
  IMU_PROX.initialize();
  Serial.print("  initialize() took: ");
  Serial.print(millis() - t);
  Serial.println("ms");

  Serial.println(IMU_PROX.testConnection()
    ? "MPU6050 " + fc.label + " Proximal OK"
    : "MPU6050 " + fc.label + " Proximal FAIL");
  Serial.println(IMU_MID.testConnection()
    ? "MPU6050 " + fc.label + " Mid OK"
    : "MPU6050 " + fc.label + " Mid FAIL");

  fc.IMU_PROX_EN = IMU_PROX.testConnection();
  fc.IMU_MID_EN  = IMU_MID.testConnection();

  status[0] = fc.IMU_MID_EN;
  status[2] = fc.IMU_PROX_EN;

  // Run dmpInitialize() on both IMUs and time each call independently
  Serial.println(F("Initializing DMP..."));

  t = millis();
  devStatus1 = IMU_MID.dmpInitialize();
  Serial.print("  MID  dmpInitialize() took: ");
  Serial.print(millis() - t);
  Serial.print("ms  (status=");
  Serial.print(devStatus1);
  Serial.println(")");

  t = millis();
  devStatus2 = IMU_PROX.dmpInitialize();
  Serial.print("  PROX dmpInitialize() took: ");
  Serial.print(millis() - t);
  Serial.print("ms  (status=");
  Serial.print(devStatus2);
  Serial.println(")");

  // Calibrate the mid-segment IMU and enable its DMP. On any failure the
  // IMU is marked disabled so the main loop will skip it.
  if (fc.IMU_MID_EN) {
    if (devStatus1 == 0) {
      t = millis();
      IMU_MID.CalibrateAccel(6);
      IMU_MID.CalibrateGyro(6);
      Serial.print("  MID  calibration took: ");
      Serial.print(millis() - t);
      Serial.println("ms");

      IMU_MID.PrintActiveOffsets();
      Serial.println(F("Enabling DMP for MID IMU..."));
      IMU_MID.setDMPEnabled(true);
      packetSize1 = IMU_MID.dmpGetFIFOPacketSize();
      dmpReady1 = true;
      status[1] = true;
    } else {
      Serial.print(F("MID DMP init failed (code "));
      Serial.print(devStatus1);
      Serial.println(F(")"));
      dmpReady1 = false;
      fc.IMU_MID_EN = false;
    }
  }

  // Calibrate the proximal IMU and enable its DMP. On any failure the
  // IMU is marked disabled so the main loop will skip it.
  if (fc.IMU_PROX_EN) {
    if (devStatus2 == 0) {
      t = millis();
      IMU_PROX.CalibrateAccel(6);
      IMU_PROX.CalibrateGyro(6);
      Serial.print("  PROX calibration took: ");
      Serial.print(millis() - t);
      Serial.println("ms");

      IMU_PROX.PrintActiveOffsets();
      Serial.println(F("Enabling DMP for PROX IMU..."));
      IMU_PROX.setDMPEnabled(true);
      packetSize2 = IMU_PROX.dmpGetFIFOPacketSize();
      dmpReady2 = true;
      status[3] = true;
    } else {
      Serial.print(F("PROX DMP init failed (code "));
      Serial.print(devStatus2);
      Serial.println(F(")"));
      dmpReady2 = false;
      fc.IMU_PROX_EN = false;
    }
  }

  Serial.print("==== " + fc.label + " total init took: ");
  Serial.print(millis() - t_total);
  Serial.println("ms ====\n");
}
