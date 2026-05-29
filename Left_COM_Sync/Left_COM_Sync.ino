#include <Wire.h>
#include "MPU6050_6Axis_MotionApps612.h"
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "Globals.h"
#include "IMU_setup.h"
#include "CreateJson.h"
#include "Net.h"


/*
*************************** Left_COM_Sync.ino ******************************************

Filename:       Left_COM_Sync.ino
Author:         Jestin

Description:    ESP32-S3 firmware for the left half of the sensorised bimanual glove
                system, request/response ("sync") variant. On boot the firmware
                brings up the I2C bus, the TCA9548A multiplexer, the BNO055 wrist
                IMU, and a pair of MPU6050 IMUs (mid and proximal) on every finger
                channel that has them populated. While that bring-up is in progress
                a background FreeRTOS task blinks the onboard LED to signal "still
                initialising". Once the bring-up is complete the LED switches to
                solid on, the Wi-Fi/TCP link to the PC is opened, and the main
                loop simply polls the TCP socket for inbound commands.

                In this variant the firmware does not stream data autonomously,
                instead the PC drives the cadence. When the PC sends a
                REQUEST_DATA command, handleRequestData() reads every IMU and
                every flex sensor, builds one JSON packet that echoes back the
                PC-supplied request_id and request_ts, and transmits the packet
                over TCP. The packet includes raw MPU6050 quaternions and the
                BNO055 quaternion in addition to yaw/pitch/roll, so the PC-side
                pose reconstruction can choose the representation that best suits
                it. The right-hand glove runs a sibling firmware that listens on
                a different TCP port, so the PC can dispatch matched request_ids
                to both gloves and align the responses without any
                synchronisation between the two ESP32-S3 boards themselves.

Dependencies:   Wire                    MPU6050_6Axis_MotionApps612     ArduinoJson
                Adafruit_Sensor         Adafruit_BNO055                 imumaths
                Globals.h               IMU_setup.h                     CreateJson.h
                Net.h

*****************************************************************************************
*/


// I2C bus and TCA9548A multiplexer configuration
#define I2C_BUS_SDA  8
#define I2C_BUS_SCL  9
#define TCA_FREQ     400000
#define TCA_ADDR     0x71

// TCA9548A channel assignments for each finger and the wrist sensor
#define TCA_CH_PALM   7
#define TCA_CH_THUMB  6
#define TCA_CH_INDEX  5
#define TCA_CH_MIDDLE 4
#define TCA_CH_RING   3
#define TCA_CH_PINKY  2
#define TCA_CH_WRIST  1

// Onboard LED used to indicate boot status: blinking during init, solid once ready
#define LED_PIN 1

// Number of finger channels in the HandChannels array, evaluated at compile time
#define NUM_FINGERS (int)(sizeof(HandChannels) / sizeof(HandChannels[0]))


// Definitions of globals declared in Globals.h
TCA9548A         TCA(TCA_ADDR);
MPU6050          IMU_MID(0x69);
MPU6050          IMU_PROX(0x68);
Adafruit_BNO055  bno = Adafruit_BNO055(55, 0x29, &Wire);

// Per-finger configuration table. Each entry binds a TCA channel and a pair
// of ADC pins (-1 if absent) to a finger label, along with two boolean flags
// that are set by initFingerChannel() to indicate whether each IMU was
// detected on that channel.
FingerChannel HandChannels[6] = {
  {TCA_CH_PALM,   "Palm",   {-1, -1}, false, false},
  {TCA_CH_THUMB,  "Thumb",  {2,  3},  false, false},
  {TCA_CH_INDEX,  "Index",  {4,  5},  false, false},
  {TCA_CH_MIDDLE, "Middle", {6,  13}, false, false},
  {TCA_CH_RING,   "Ring",   {12, 11}, false, false},
  {TCA_CH_PINKY,  "Pinky",  {10, 7},  false, false}
};

// DMP state for the mid-segment MPU6050
bool     dmpReady1   = false;
uint8_t  devStatus1  = 1;
uint16_t packetSize1 = 0;
uint8_t  fifoBuffer1[64];

// DMP state for the proximal-segment MPU6050
bool     dmpReady2   = false;
uint8_t  devStatus2  = 1;
uint16_t packetSize2 = 0;
uint8_t  fifoBuffer2[64];

// Decoded DMP outputs for both IMUs, populated by readDmpMid() and readDmpProx()
Quaternion  q1, q2;
VectorFloat gravity1, gravity2;
float       ypr1[3] = {0, 0, 0};
float       ypr2[3] = {0, 0, 0};
VectorInt16 aa1, aaReal1, aaWorld1;
VectorInt16 aa2, aaReal2, aaWorld2;
float ax1 = 0, ay1 = 0, az1 = 0;
float ax2 = 0, ay2 = 0, az2 = 0;

// Set to true at the end of setup() once all hardware and Wi-Fi are up.
// handleRequestData() ignores commands until this is set so that no garbage
// packets are transmitted during initialisation.
volatile bool gloveInitialised = false;

// Identifies which physical glove this firmware is running on. The Python
// receiver uses this field to route packets into the correct CSV column
// prefix. The sibling firmware on the right glove sets this to "RightGlove".
const char* HAND_NAME = "LeftGlove";

// Handle of the LED blink task, used to stop the task once init is finished
TaskHandle_t ledTaskHandle = nullptr;

// Forward declarations for helpers defined further down
void ledBlinkTask(void *parameter);
void stopLedBlink();
void handleInit();
void handleRequestData(uint32_t requestId, const char* requestTs);


static bool readDmpMid()
{
  /*
  Reads exactly one DMP packet from the mid-segment MPU6050 (IMU_MID) and
  decodes it into the module-level quaternion, gravity vector, yaw/pitch/roll
  (in degrees) and world-frame linear acceleration (in m/s^2) variables
  (q1, gravity1, ypr1, ax1, ay1, az1).

  Input:
      No direct inputs. Reads dmpReady1, packetSize1 and the FIFO of IMU_MID.

  Output:
      success (bool): true if a valid DMP packet was read and decoded, false
                      if the DMP was not initialised, if there was no complete
                      packet available, or if the FIFO had overflowed and was
                      reset. On success the module-level decoded values are
                      updated in place.
  */
  if (!dmpReady1 || packetSize1 == 0) {
    return false;
  }

  uint16_t fc = IMU_MID.getFIFOCount();

  // FIFO has overflowed, reset it and bail so the next call gets fresh data
  if (fc >= 1024) {
    IMU_MID.resetFIFO();
    return false;
  }

  // Not enough bytes for a full packet yet
  if (fc < packetSize1) {
    return false;
  }

  // Read the oldest complete packet currently queued
  IMU_MID.getFIFOBytes(fifoBuffer1, packetSize1);

  // Decode DMP outputs from the packet
  IMU_MID.dmpGetQuaternion(&q1, fifoBuffer1);
  IMU_MID.dmpGetGravity(&gravity1, &q1);
  IMU_MID.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);
  IMU_MID.dmpGetAccel(&aa1, fifoBuffer1);
  IMU_MID.dmpGetLinearAccel(&aaReal1, &aa1, &gravity1);
  IMU_MID.dmpGetLinearAccelInWorld(&aaWorld1, &aaReal1, &q1);

  // Convert yaw/pitch/roll from radians to degrees
  constexpr float RAD_TO_DEG_F = 180.0f / PI;
  ypr1[0] *= RAD_TO_DEG_F;
  ypr1[1] *= RAD_TO_DEG_F;
  ypr1[2] *= RAD_TO_DEG_F;

  // Convert raw int16 accel counts (LSB/g at +/-2g full scale) to m/s^2
  constexpr float accelScale_g = 1.0f / 16384.0f;
  constexpr float g_to_ms2     = 9.80665f;
  ax1 = aaWorld1.x * accelScale_g * g_to_ms2;
  ay1 = aaWorld1.y * accelScale_g * g_to_ms2;
  az1 = aaWorld1.z * accelScale_g * g_to_ms2;

  // Flush any remaining backlog so the next call stays fast
  if (fc > packetSize1) {
    IMU_MID.resetFIFO();
  }

  return true;
}


static bool readDmpProx()
{
  /*
  Reads exactly one DMP packet from the proximal-segment MPU6050 (IMU_PROX)
  and decodes it into the module-level quaternion, gravity vector,
  yaw/pitch/roll (in degrees) and world-frame linear acceleration (in m/s^2)
  variables (q2, gravity2, ypr2, ax2, ay2, az2).

  Input:
      No direct inputs. Reads dmpReady2, packetSize2 and the FIFO of IMU_PROX.

  Output:
      success (bool): true if a valid DMP packet was read and decoded, false
                      if the DMP was not initialised, if there was no complete
                      packet available, or if the FIFO had overflowed and was
                      reset. On success the module-level decoded values are
                      updated in place.
  */
  if (!dmpReady2 || packetSize2 == 0) {
    return false;
  }

  uint16_t fc = IMU_PROX.getFIFOCount();

  // FIFO has overflowed, reset it and bail so the next call gets fresh data
  if (fc >= 1024) {
    IMU_PROX.resetFIFO();
    return false;
  }

  // Not enough bytes for a full packet yet
  if (fc < packetSize2) {
    return false;
  }

  // Read the oldest complete packet currently queued
  IMU_PROX.getFIFOBytes(fifoBuffer2, packetSize2);

  // Decode DMP outputs from the packet
  IMU_PROX.dmpGetQuaternion(&q2, fifoBuffer2);
  IMU_PROX.dmpGetGravity(&gravity2, &q2);
  IMU_PROX.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);
  IMU_PROX.dmpGetAccel(&aa2, fifoBuffer2);
  IMU_PROX.dmpGetLinearAccel(&aaReal2, &aa2, &gravity2);
  IMU_PROX.dmpGetLinearAccelInWorld(&aaWorld2, &aaReal2, &q2);

  // Convert yaw/pitch/roll from radians to degrees
  constexpr float RAD_TO_DEG_F = 180.0f / PI;
  ypr2[0] *= RAD_TO_DEG_F;
  ypr2[1] *= RAD_TO_DEG_F;
  ypr2[2] *= RAD_TO_DEG_F;

  // Convert raw int16 accel counts (LSB/g at +/-2g full scale) to m/s^2
  constexpr float accelScale_g = 1.0f / 16384.0f;
  constexpr float g_to_ms2     = 9.80665f;
  ax2 = aaWorld2.x * accelScale_g * g_to_ms2;
  ay2 = aaWorld2.y * accelScale_g * g_to_ms2;
  az2 = aaWorld2.z * accelScale_g * g_to_ms2;

  // Flush any remaining backlog so the next call stays fast
  if (fc > packetSize2) {
    IMU_PROX.resetFIFO();
  }

  return true;
}


void ledBlinkTask(void *parameter)
{
  /*
  FreeRTOS task that toggles the onboard LED every 200 ms while the firmware
  is still initialising, giving a visible "still booting" indication. The
  task runs forever until stopLedBlink() deletes it from setup() once init
  is finished.

  Input:
      parameter (void*):  Standard FreeRTOS task parameter, unused here.

  Output:
      No return value, the task never returns under normal operation.
  */
  pinMode(LED_PIN, OUTPUT);

  while (true) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}


void stopLedBlink()
{
  /*
  Stops the LED blink task started in setup() and drives the LED low so a
  subsequent digitalWrite(LED_PIN, HIGH) can put it in a known solid-on
  state. Safe to call even if the task has already been stopped.

  Input:
      No direct inputs, operates on the module-level ledTaskHandle.

  Output:
      No return value, the task handle is cleared on success.
  */
  if (ledTaskHandle != nullptr) {
    vTaskDelete(ledTaskHandle);
    ledTaskHandle = nullptr;
    digitalWrite(LED_PIN, LOW);
  }
}


void setup()
{
  /*
  Standard Arduino setup hook. Brings up Serial, spawns the LED blink task
  on core 1 so the board visibly indicates "still initialising", then sets
  up the I2C bus, the TCA9548A multiplexer, the BNO055 wrist IMU, every
  populated finger channel (each with its pair of MPU6050s and flex sensor
  inputs), and the Wi-Fi/TCP link to the PC. Finally the blink task is
  stopped, the LED is driven solid on, and the main loop is armed by
  setting gloveInitialised to true so handleRequestData() will respond to
  inbound commands.

  Input:
      No direct inputs.

  Output:
      No return value. On success the glove is fully initialised and ready
      to respond to PC commands. On BNO055 failure the firmware enters an
      infinite loop after printing a diagnostic message.
  */
  Serial.begin(115200);
  delay(1200);
  Serial.println("\nBooting...");

  // Spawn the LED blink task on core 1 so it runs independently of setup()
  xTaskCreatePinnedToCore(
    ledBlinkTask,      // task function
    "LED Blink Task",  // task name
    2048,              // stack size in words
    nullptr,           // task parameter
    1,                 // task priority
    &ledTaskHandle,    // out: task handle
    1                  // core to pin the task to
  );

  // Bring up the I2C bus and the TCA9548A multiplexer
  Wire.begin(I2C_BUS_SDA, I2C_BUS_SCL);
  Wire.setClock(TCA_FREQ);
  Wire.setTimeOut(10);              // 10 ms max per transaction so a stuck IMU cannot hang the bus
  TCA.begin(Wire);
  analogReadResolution(12);

  // Bring up the BNO055 on the wrist channel
  tcaSelectChannel(TCA_CH_WRIST);
  delay(10);

  if (!bno.begin()) {
    Serial.println("BNO055 not detected! Check wiring or TCA channel.");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 ready on TCA Channel 1.");

  // Initialise every finger channel and report the status of each IMU.
  // The status array is filled by initFingerChannel():
  //   status[0]: IMU_MID detected
  //   status[1]: IMU_MID DMP enabled
  //   status[2]: IMU_PROX detected
  //   status[3]: IMU_PROX DMP enabled
  bool status[4] = {false, false, false, false};
  for (int i = 0; i < NUM_FINGERS; i++) {
    initFingerChannel(HandChannels[i], status);
    Serial.println("---");
    Serial.println(HandChannels[i].label);
    Serial.println(String("IMU_MID      : ") + (status[0] ? "OK" : "FAIL"));
    Serial.println(String("IMU_MID_DMP  : ") + (status[1] ? "OK" : "FAIL"));
    Serial.println(String("IMU_PROX     : ") + (status[2] ? "OK" : "FAIL"));
    Serial.println(String("IMU_PROX_DMP : ") + (status[3] ? "OK" : "FAIL"));
    status[0] = status[1] = status[2] = status[3] = false;
  }

  // Stop the blink task, bring up Wi-Fi, then drive the LED solid on and
  // arm the request handler
  stopLedBlink();
  initWifi();
  gloveInitialised = true;
  digitalWrite(LED_PIN, HIGH);
}


void loop()
{
  /*
  Standard Arduino loop hook. The firmware is purely reactive in this
  variant, every iteration just drains any inbound TCP commands and
  dispatches them. Building and transmitting glove packets is the
  responsibility of handleRequestData(), which is invoked from
  pollTcpCommands() when the PC sends a REQUEST_DATA message.

  Input:
      No direct inputs.

  Output:
      No return value.
  */
  pollTcpCommands(handleInit, handleRequestData);
  delay(1);
}


void handleInit()
{
  /*
  Callback invoked by pollTcpCommands() when the PC sends an INIT command.
  In the current firmware the heavy bring-up has already run in setup(),
  so this hook only drives the status LED solid on to confirm the glove
  is responsive.

  Input:
      No direct inputs.

  Output:
      No return value.
  */
  digitalWrite(LED_PIN, HIGH);
}


void handleRequestData(uint32_t requestId, const char* requestTs)
{
  /*
  Callback invoked by pollTcpCommands() when the PC sends a REQUEST_DATA
  command. Builds one JSON packet containing the current flex sensor
  readings, DMP-decoded quaternion / yaw-pitch-roll / world-frame
  acceleration for every populated MPU6050, and the orientation,
  acceleration and quaternion of the BNO055 wrist sensor. The PC-supplied
  request_id and request_ts are echoed back so the receiver can match the
  response to its outstanding request. The packet is printed to Serial for
  debugging and transmitted to the PC over TCP.

  Input:
      requestId (uint32_t):       Sequence number from the PC's request,
                                  echoed back in the response.

      requestTs (const char*):    Timestamp string from the PC's request,
                                  echoed back in the response.

  Output:
      No return value. One JSON packet is written to Serial and one is sent
      to the PC over TCP. If the glove has not finished initialising the
      function returns immediately without sending anything.
  */
  if (!gloveInitialised) return;

  // 6 kB document, large enough for raw quaternions on every IMU plus the
  // BNO055 wrist quaternion
  DynamicJsonDocument doc(6144);
  doc["Hand"]          = HAND_NAME;
  doc["request_id"]    = requestId;
  doc["request_ts"]    = requestTs;
  doc["glove_time_ms"] = millis();

  JsonObject fingerData = doc.createNestedObject("Data");

  // Walk every finger channel, switch the TCA to its bus, then read both
  // IMUs and both flex sensors. Missing sensors are reported as 0.0f for
  // IMU fields or -1 for flex fields so the receiver can identify them.
  for (int i = 0; i < NUM_FINGERS; i++) {
    FingerChannel &fc = HandChannels[i];
    tcaSelectChannel(fc.tca_channel);

    bool gotMid  = fc.IMU_MID_EN  ? readDmpMid()  : false;
    bool gotProx = fc.IMU_PROX_EN ? readDmpProx() : false;

    int MCP_flex = (fc.adc_channel[0] != -1) ? analogRead(fc.adc_channel[0]) : -1;
    int PIP_flex = (fc.adc_channel[1] != -1) ? analogRead(fc.adc_channel[1]) : -1;

    JsonObject finger = fingerData.createNestedObject(fc.label);

    finger["flex_mcp"] = MCP_flex;
    finger["flex_pip"] = PIP_flex;

    // Mid-segment IMU readings. Quaternions are rounded to 4 decimal places
    // and angles / accelerations to 2 decimal places to keep packets small.
    finger["quat_w_mid"] = gotMid ? roundf(q1.w * 10000.0f) / 10000.0f : 0.0f;
    finger["quat_x_mid"] = gotMid ? roundf(q1.x * 10000.0f) / 10000.0f : 0.0f;
    finger["quat_y_mid"] = gotMid ? roundf(q1.y * 10000.0f) / 10000.0f : 0.0f;
    finger["quat_z_mid"] = gotMid ? roundf(q1.z * 10000.0f) / 10000.0f : 0.0f;

    finger["yaw_mid"]    = gotMid ? roundf(ypr1[0] * 100.0f) / 100.0f : 0.0f;
    finger["pitch_mid"]  = gotMid ? roundf(ypr1[1] * 100.0f) / 100.0f : 0.0f;
    finger["roll_mid"]   = gotMid ? roundf(ypr1[2] * 100.0f) / 100.0f : 0.0f;
    finger["ax_mid"]     = gotMid ? roundf(ax1     * 100.0f) / 100.0f : 0.0f;
    finger["ay_mid"]     = gotMid ? roundf(ay1     * 100.0f) / 100.0f : 0.0f;
    finger["az_mid"]     = gotMid ? roundf(az1     * 100.0f) / 100.0f : 0.0f;

    // Proximal-segment IMU readings, same rounding as above
    finger["quat_w_prox"] = gotProx ? roundf(q2.w * 10000.0f) / 10000.0f : 0.0f;
    finger["quat_x_prox"] = gotProx ? roundf(q2.x * 10000.0f) / 10000.0f : 0.0f;
    finger["quat_y_prox"] = gotProx ? roundf(q2.y * 10000.0f) / 10000.0f : 0.0f;
    finger["quat_z_prox"] = gotProx ? roundf(q2.z * 10000.0f) / 10000.0f : 0.0f;

    finger["yaw_prox"]    = gotProx ? roundf(ypr2[0] * 100.0f) / 100.0f : 0.0f;
    finger["pitch_prox"]  = gotProx ? roundf(ypr2[1] * 100.0f) / 100.0f : 0.0f;
    finger["roll_prox"]   = gotProx ? roundf(ypr2[2] * 100.0f) / 100.0f : 0.0f;
    finger["ax_prox"]     = gotProx ? roundf(ax2    * 100.0f) / 100.0f : 0.0f;
    finger["ay_prox"]     = gotProx ? roundf(ay2    * 100.0f) / 100.0f : 0.0f;
    finger["az_prox"]     = gotProx ? roundf(az2    * 100.0f) / 100.0f : 0.0f;
  }

  // Switch the TCA to the wrist channel and append the BNO055 readings
  tcaSelectChannel(TCA_CH_WRIST);

  JsonObject wrist = fingerData.createNestedObject("Wrist");

  sensors_event_t orientEvent;
  bno.getEvent(&orientEvent);

  sensors_event_t accelEvent;
  bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // Native quaternion straight off the BNO055 fusion engine
  imu::Quaternion wristQuat = bno.getQuat();

  wrist["ax"]      = roundf(accelEvent.acceleration.x * 100.0f) / 100.0f;
  wrist["ay"]      = roundf(accelEvent.acceleration.y * 100.0f) / 100.0f;
  wrist["az"]      = roundf(accelEvent.acceleration.z * 100.0f) / 100.0f;

  wrist["heading"] = roundf(orientEvent.orientation.x * 100.0f) / 100.0f;
  wrist["pitch"]   = roundf(orientEvent.orientation.y * 100.0f) / 100.0f;
  wrist["roll"]    = roundf(orientEvent.orientation.z * 100.0f) / 100.0f;

  wrist["quat_w"]  = roundf(wristQuat.w() * 10000.0f) / 10000.0f;
  wrist["quat_x"]  = roundf(wristQuat.x() * 10000.0f) / 10000.0f;
  wrist["quat_y"]  = roundf(wristQuat.y() * 10000.0f) / 10000.0f;
  wrist["quat_z"]  = roundf(wristQuat.z() * 10000.0f) / 10000.0f;

  // Timestamp is set once per packet, after every sensor has been polled,
  // so it reflects the moment the packet leaves the glove
  doc["Time"] = millis();

  serializeJson(doc, Serial);
  Serial.println();

  sendJsonOverTcp(doc);
}
