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

// ── I2C & TCA config ────────────────────────────────────────
#define I2C_BUS_SDA  8
#define I2C_BUS_SCL  9
#define TCA_FREQ     400000
#define TCA_ADDR     0x71

#define TCA_CH_PALM   1
#define TCA_CH_THUMB  6
#define TCA_CH_INDEX  5
#define TCA_CH_MIDDLE 4
#define TCA_CH_RING   3
#define TCA_CH_PINKY  2
#define TCA_CH_WRIST  7

#define LED_PIN 1

// ── Definitions of globals declared in Globals.h ────────────
TCA9548A  TCA(TCA_ADDR);
MPU6050   IMU_MID(0x69);
MPU6050   IMU_PROX(0x68);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);

FingerChannel HandChannels[6] = {
  {TCA_CH_PALM,   "Palm",   {-1, -1}, false, false},
  {TCA_CH_THUMB,  "Thumb",  {2,  3},  false, false},
  {TCA_CH_INDEX,  "Index",  {4,  5},  false, false},
  {TCA_CH_MIDDLE, "Middle", {6,  13}, false, false},
  {TCA_CH_RING,   "Ring",   {12, 11}, false, false},
  {TCA_CH_PINKY,  "Pinky",  {10, 7},  false, false}
};

bool     dmpReady1   = false;
uint8_t  devStatus1  = 1;
uint16_t packetSize1 = 0;
uint8_t  fifoBuffer1[64];

bool     dmpReady2   = false;
uint8_t  devStatus2  = 1;
uint16_t packetSize2 = 0;
uint8_t  fifoBuffer2[64];

Quaternion  q1, q2;
VectorFloat gravity1, gravity2;
float       ypr1[3] = {0, 0, 0};
float       ypr2[3] = {0, 0, 0};
VectorInt16 aa1, aaReal1, aaWorld1;
VectorInt16 aa2, aaReal2, aaWorld2;
float ax1 = 0, ay1 = 0, az1 = 0;
float ax2 = 0, ay2 = 0, az2 = 0;

DynamicJsonDocument DataPacket(4096);

volatile bool gloveInitialised = false;
const char* HAND_NAME = "RightGlove";   // or "LeftGlove"

TaskHandle_t ledTaskHandle = nullptr;

void ledBlinkTask(void *parameter);
void stopLedBlink();


// ── FIFO helpers ─────────────────────────────────────────────

// static bool readDmpMid() {
//   if (!dmpReady1 || packetSize1 == 0) return false;
//   uint16_t fc = IMU_MID.getFIFOCount();
//   if (fc >= 1024) { IMU_MID.resetFIFO(); return false; }
//   if (fc < packetSize1) return false;
//   while (fc >= packetSize1) {
//     IMU_MID.getFIFOBytes(fifoBuffer1, packetSize1);
//     fc -= packetSize1;
//   }
//   IMU_MID.dmpGetQuaternion(&q1, fifoBuffer1);
//   IMU_MID.dmpGetGravity(&gravity1, &q1);
//   IMU_MID.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);
//   IMU_MID.dmpGetAccel(&aa1, fifoBuffer1);
//   IMU_MID.dmpGetLinearAccel(&aaReal1, &aa1, &gravity1);
//   IMU_MID.dmpGetLinearAccelInWorld(&aaWorld1, &aaReal1, &q1);
//   ax1 = aaWorld1.x / 16384.0f;
//   ay1 = aaWorld1.y / 16384.0f;
//   az1 = aaWorld1.z / 16384.0f;
//   return true;
// }

// static bool readDmpProx() {
//   if (!dmpReady2 || packetSize2 == 0) return false;
//   uint16_t fc = IMU_PROX.getFIFOCount();
//   if (fc >= 1024) { IMU_PROX.resetFIFO(); return false; }
//   if (fc < packetSize2) return false;
//   while (fc >= packetSize2) {
//     IMU_PROX.getFIFOBytes(fifoBuffer2, packetSize2);
//     fc -= packetSize2;
//   }
//   IMU_PROX.dmpGetQuaternion(&q2, fifoBuffer2);
//   IMU_PROX.dmpGetGravity(&gravity2, &q2);
//   IMU_PROX.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);
//   IMU_PROX.dmpGetAccel(&aa2, fifoBuffer2);
//   IMU_PROX.dmpGetLinearAccel(&aaReal2, &aa2, &gravity2);
//   IMU_PROX.dmpGetLinearAccelInWorld(&aaWorld2, &aaReal2, &q2);
//   ax2 = aaWorld2.x / 16384.0f;
//   ay2 = aaWorld2.y / 16384.0f;
//   az2 = aaWorld2.z / 16384.0f;
//   return true;
// }
static bool readDmpMid() {
  if (!dmpReady1 || packetSize1 == 0) {
    return false;
  }

  uint16_t fc = IMU_MID.getFIFOCount();

  if (fc >= 1024) {
    IMU_MID.resetFIFO();
    return false;
  }

  if (fc < packetSize1) {
    return false;
  }

  IMU_MID.getFIFOBytes(fifoBuffer1, packetSize1);

  IMU_MID.dmpGetQuaternion(&q1, fifoBuffer1);
  IMU_MID.dmpGetGravity(&gravity1, &q1);
  IMU_MID.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);
  IMU_MID.dmpGetAccel(&aa1, fifoBuffer1);
  IMU_MID.dmpGetLinearAccel(&aaReal1, &aa1, &gravity1);
  IMU_MID.dmpGetLinearAccelInWorld(&aaWorld1, &aaReal1, &q1);

  constexpr float RAD_TO_DEG_F = 180.0f / PI;
  ypr1[0] *= RAD_TO_DEG_F;
  ypr1[1] *= RAD_TO_DEG_F;
  ypr1[2] *= RAD_TO_DEG_F;

  constexpr float accelScale_g = 1.0f / 16384.0f;
  constexpr float g_to_ms2 = 9.80665f;
  ax1 = aaWorld1.x * accelScale_g * g_to_ms2;
  ay1 = aaWorld1.y * accelScale_g * g_to_ms2;
  az1 = aaWorld1.z * accelScale_g * g_to_ms2;

  if (fc > packetSize1) {
    IMU_MID.resetFIFO();
  }

  return true;
}

static bool readDmpProx() {
  if (!dmpReady2 || packetSize2 == 0) {
    return false;
  }

  uint16_t fc = IMU_PROX.getFIFOCount();

  if (fc >= 1024) {
    IMU_PROX.resetFIFO();
    return false;
  }

  if (fc < packetSize2) {
    return false;
  }

  IMU_PROX.getFIFOBytes(fifoBuffer2, packetSize2);

  IMU_PROX.dmpGetQuaternion(&q2, fifoBuffer2);
  IMU_PROX.dmpGetGravity(&gravity2, &q2);
  IMU_PROX.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);
  IMU_PROX.dmpGetAccel(&aa2, fifoBuffer2);
  IMU_PROX.dmpGetLinearAccel(&aaReal2, &aa2, &gravity2);
  IMU_PROX.dmpGetLinearAccelInWorld(&aaWorld2, &aaReal2, &q2);

  constexpr float RAD_TO_DEG_F = 180.0f / PI;
  ypr2[0] *= RAD_TO_DEG_F;
  ypr2[1] *= RAD_TO_DEG_F;
  ypr2[2] *= RAD_TO_DEG_F;

  constexpr float accelScale_g = 1.0f / 16384.0f;
  constexpr float g_to_ms2 = 9.80665f;
  ax2 = aaWorld2.x * accelScale_g * g_to_ms2;
  ay2 = aaWorld2.y * accelScale_g * g_to_ms2;
  az2 = aaWorld2.z * accelScale_g * g_to_ms2;

  if (fc > packetSize2) {
    IMU_PROX.resetFIFO();
  }

  return true;
}


void handleRequestData(uint32_t requestId, const char* requestTs);

// ── setup ────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println("\nBooting...");

  xTaskCreatePinnedToCore(
    ledBlinkTask,      // task function
    "LED Blink Task",  // name
    2048,              // stack size in words
    nullptr,           // parameter
    1,                 // priority
    &ledTaskHandle,    // handle
    1                  // core (0 or 1)
  );


  Wire.begin(I2C_BUS_SDA, I2C_BUS_SCL);
  Wire.setClock(TCA_FREQ);
  Wire.setTimeOut(10); // 10ms max per transaction — fail fast
  TCA.begin(Wire);
  analogReadResolution(12);

  tcaSelectChannel(TCA_CH_WRIST);

  delay(10);

  if (!bno.begin()) {
    Serial.println("BNO055 not detected! Check wiring or TCA channel.");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 ready on TCA Channel 1.");

  bool status[4] = {false, false, false, false};
  for (int i = 0; i < int(sizeof(HandChannels) / sizeof(HandChannels[0])); i++) {
    initFingerChannel(HandChannels[i], status);
    Serial.println("---");
    Serial.println(HandChannels[i].label);
    Serial.println(String("IMU_MID      : ") + (status[0] ? "OK" : "FAIL"));
    Serial.println(String("IMU_MID_DMP  : ") + (status[1] ? "OK" : "FAIL"));
    Serial.println(String("IMU_PROX     : ") + (status[2] ? "OK" : "FAIL"));
    Serial.println(String("IMU_PROX_DMP : ") + (status[3] ? "OK" : "FAIL"));
    status[0] = status[1] = status[2] = status[3] = false;
  }

  stopLedBlink();


  initWifi();
  gloveInitialised = true;
  digitalWrite(1, HIGH);  // for LEDsendReadyMessage(HAND_NAME);

}



// ── loop ─────────────────────────────────────────────────────

void loop() {

  // if (!gloveInitialised) return;

  // DynamicJsonDocument doc(4096);
  // doc["Hand"]         = HAND_NAME;
  // doc["glove_time_ms"] = millis();

  // JsonObject fingerData = doc.createNestedObject("Data");

  // for (int i = 0; i < int(sizeof(HandChannels) / sizeof(HandChannels[0])); i++) {
  //   FingerChannel &fc = HandChannels[i];
  //   tcaSelectChannel(fc.tca_channel);

  //   bool gotMid  = fc.IMU_MID_EN  ? readDmpMid()  : false;
  //   bool gotProx = fc.IMU_PROX_EN ? readDmpProx() : false;

  //   int MCP_flex = (fc.adc_channel[0] != -1) ? analogRead(fc.adc_channel[0]) : -1;
  //   int PIP_flex = (fc.adc_channel[1] != -1) ? analogRead(fc.adc_channel[1]) : -1;

  //   // ↓ createNestedObject on fingerData, not a lookup on doc["Data"]
  //   JsonObject finger = fingerData.createNestedObject(fc.label);

  //   finger["flex_mcp"] = MCP_flex;
  //   finger["flex_pip"] = PIP_flex;

  //   finger["yaw_mid"]   = gotMid ? roundf(ypr1[0] * 100.0f) / 100.0f : 0.0f;
  //   finger["pitch_mid"] = gotMid ? roundf(ypr1[1] * 100.0f) / 100.0f : 0.0f;
  //   finger["roll_mid"]  = gotMid ? roundf(ypr1[2] * 100.0f) / 100.0f : 0.0f;
  //   finger["ax_mid"]    = gotMid ? roundf(ax1 * 100.0f) / 100.0f     : 0.0f;
  //   finger["ay_mid"]    = gotMid ? roundf(ay1 * 100.0f) / 100.0f     : 0.0f;
  //   finger["az_mid"]    = gotMid ? roundf(az1 * 100.0f) / 100.0f     : 0.0f;

  //   finger["yaw_prox"]   = gotProx ? roundf(ypr2[0] * 100.0f) / 100.0f : 0.0f;
  //   finger["pitch_prox"] = gotProx ? roundf(ypr2[1] * 100.0f) / 100.0f : 0.0f;
  //   finger["roll_prox"]  = gotProx ? roundf(ypr2[2] * 100.0f) / 100.0f : 0.0f;
  //   finger["ax_prox"]    = gotProx ? roundf(ax2 * 100.0f) / 100.0f     : 0.0f;
  //   finger["ay_prox"]    = gotProx ? roundf(ay2 * 100.0f) / 100.0f     : 0.0f;
  //   finger["az_prox"]    = gotProx ? roundf(az2 * 100.0f) / 100.0f     : 0.0f;
  // }

  //  tcaSelectChannel(TCA_CH_WRIST);   // <-- add this

  // JsonObject finger = fingerData.createNestedObject("Wrist");

  // // --- Euler Angles ---
  // sensors_event_t orientEvent;
  // bno.getEvent(&orientEvent);

  // // --- Acceleration ---
  // sensors_event_t accelEvent;
  // bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);


  // finger["ax"]       = roundf(accelEvent.acceleration.x * 100.0f) / 100.0f;
  // finger["ay"]       = roundf(accelEvent.acceleration.y * 100.0f) / 100.0f;
  // finger["az"]       = roundf(accelEvent.acceleration.z * 100.0f) / 100.0f;

  // finger["heading"]  = roundf(orientEvent.orientation.x * 100.0f) / 100.0f;
  // finger["pitch"]    = roundf(orientEvent.orientation.y * 100.0f) / 100.0f;
  // finger["roll"]     = roundf(orientEvent.orientation.z * 100.0f) / 100.0f;


  // doc["Time"] = millis();   // moved outside the loop — only needs to be set once

  // serializeJson(doc, Serial);
  // Serial.println();

  // sendJsonOverTcp(doc);
  pollTcpCommands(handleInit, handleRequestData);

  delay(1);
}


// void handleRequestData(uint32_t requestId, const char* requestTs)
// {
//     if (!gloveInitialised) return;

//     int start = millis();

//     DynamicJsonDocument doc(4096);
//     doc["Hand"] = HAND_NAME;
//     doc["request_id"] = requestId;
//     doc["request_ts"] = requestTs;
//     doc["glove_time_ms"] = millis();

//     JsonObject fingerData = doc.createNestedObject("Data");

//     int start_json = millis() - start;
//     Serial.println("To create json: " + String(start_json));

//     for (int i = 0; i < int(sizeof(HandChannels) / sizeof(HandChannels[0])); i++) {
//         FingerChannel &fc = HandChannels[i];
//         tcaSelectChannel(fc.tca_channel);

//         bool gotMid = fc.IMU_MID_EN ? readDmpMid() : false;
//         bool gotProx = fc.IMU_PROX_EN ? readDmpProx() : false;

//         int MCP_flex = (fc.adc_channel[0] != -1) ? analogRead(fc.adc_channel[0]) : -1;
//         int PIP_flex = (fc.adc_channel[1] != -1) ? analogRead(fc.adc_channel[1]) : -1;

//         JsonObject finger = fingerData.createNestedObject(fc.label);

//         finger["flex_mcp"] = MCP_flex;
//         finger["flex_pip"] = PIP_flex;

//         finger["yaw_mid"] = gotMid ? roundf(ypr1[0] * 100.0f) / 100.0f : 0.0f;
//         finger["pitch_mid"] = gotMid ? roundf(ypr1[1] * 100.0f) / 100.0f : 0.0f;
//         finger["roll_mid"] = gotMid ? roundf(ypr1[2] * 100.0f) / 100.0f : 0.0f;
//         finger["ax_mid"] = gotMid ? roundf(ax1 * 100.0f) / 100.0f : 0.0f;
//         finger["ay_mid"] = gotMid ? roundf(ay1 * 100.0f) / 100.0f : 0.0f;
//         finger["az_mid"] = gotMid ? roundf(az1 * 100.0f) / 100.0f : 0.0f;

//         finger["yaw_prox"] = gotProx ? roundf(ypr2[0] * 100.0f) / 100.0f : 0.0f;
//         finger["pitch_prox"] = gotProx ? roundf(ypr2[1] * 100.0f) / 100.0f : 0.0f;
//         finger["roll_prox"] = gotProx ? roundf(ypr2[2] * 100.0f) / 100.0f : 0.0f;
//         finger["ax_prox"] = gotProx ? roundf(ax2 * 100.0f) / 100.0f : 0.0f;
//         finger["ay_prox"] = gotProx ? roundf(ay2 * 100.0f) / 100.0f : 0.0f;
//         finger["az_prox"] = gotProx ? roundf(az2 * 100.0f) / 100.0f : 0.0f;

//         finger["yaw_mid"] = gotMid ? roundf(ypr1[0] * 100.0f) / 100.0f : 0.0f;
//         finger["pitch_mid"] = gotMid ? roundf(ypr1[1] * 100.0f) / 100.0f : 0.0f;
//         finger["roll_mid"] = gotMid ? roundf(ypr1[2] * 100.0f) / 100.0f : 0.0f;
//         finger["ax_mid"] = gotMid ? roundf(ax1 * 100.0f) / 100.0f : 0.0f;
//         finger["ay_mid"] = gotMid ? roundf(ay1 * 100.0f) / 100.0f : 0.0f;
//         finger["az_mid"] = gotMid ? roundf(az1 * 100.0f) / 100.0f : 0.0f;

//         finger["yaw_prox"] = gotProx ? roundf(ypr2[0] * 100.0f) / 100.0f : 0.0f;
//         finger["pitch_prox"] = gotProx ? roundf(ypr2[1] * 100.0f) / 100.0f : 0.0f;
//         finger["roll_prox"] = gotProx ? roundf(ypr2[2] * 100.0f) / 100.0f : 0.0f;
//         finger["ax_prox"] = gotProx ? roundf(ax2 * 100.0f) / 100.0f : 0.0f;
//         finger["ay_prox"] = gotProx ? roundf(ay2 * 100.0f) / 100.0f : 0.0f;
//         finger["az_prox"] = gotProx ? roundf(az2 * 100.0f) / 100.0f : 0.0f;
//     }

//     int sample_mpu = millis() - start_json;

//     Serial.println("To sample mpu6050: " + String(sample_mpu));

//     tcaSelectChannel(TCA_CH_WRIST);

//     JsonObject wrist = fingerData.createNestedObject("Wrist");

//     sensors_event_t orientEvent;
//     bno.getEvent(&orientEvent);

//     sensors_event_t accelEvent;
//     bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);

//     wrist["ax"] = roundf(accelEvent.acceleration.x * 100.0f) / 100.0f;
//     wrist["ay"] = roundf(accelEvent.acceleration.y * 100.0f) / 100.0f;
//     wrist["az"] = roundf(accelEvent.acceleration.z * 100.0f) / 100.0f;

//     wrist["heading"] = roundf(orientEvent.orientation.x * 100.0f) / 100.0f;
//     wrist["pitch"] = roundf(orientEvent.orientation.y * 100.0f) / 100.0f;
//     wrist["roll"] = roundf(orientEvent.orientation.z * 100.0f) / 100.0f;

//     int sample_bno = millis() - sample_mpu;

//     Serial.println("to sample bno: " + String(sample_bno));

//     doc["Time"] = millis();

//     serializeJson(doc, Serial);
//     Serial.println();

//     sendJsonOverTcp(doc);
// }


void handleRequestData(uint32_t requestId, const char* requestTs)
{
    if (!gloveInitialised) return;

    DynamicJsonDocument doc(6144);
    doc["Hand"] = HAND_NAME;
    doc["request_id"] = requestId;
    doc["request_ts"] = requestTs;
    doc["glove_time_ms"] = millis();

    JsonObject fingerData = doc.createNestedObject("Data");

    for (int i = 0; i < int(sizeof(HandChannels) / sizeof(HandChannels[0])); i++) {
        FingerChannel &fc = HandChannels[i];
        tcaSelectChannel(fc.tca_channel);

        bool gotMid = fc.IMU_MID_EN ? readDmpMid() : false;
        bool gotProx = fc.IMU_PROX_EN ? readDmpProx() : false;

        int MCP_flex = (fc.adc_channel[0] != -1) ? analogRead(fc.adc_channel[0]) : -1;
        int PIP_flex = (fc.adc_channel[1] != -1) ? analogRead(fc.adc_channel[1]) : -1;

        JsonObject finger = fingerData.createNestedObject(fc.label);

        finger["flex_mcp"] = MCP_flex;
        finger["flex_pip"] = PIP_flex;

        finger["quat_w_mid"] = gotMid ? roundf(q1.w * 10000.0f) / 10000.0f : 0.0f;
        finger["quat_x_mid"] = gotMid ? roundf(q1.x * 10000.0f) / 10000.0f : 0.0f;
        finger["quat_y_mid"] = gotMid ? roundf(q1.y * 10000.0f) / 10000.0f : 0.0f;
        finger["quat_z_mid"] = gotMid ? roundf(q1.z * 10000.0f) / 10000.0f : 0.0f;

        finger["yaw_mid"] = gotMid ? roundf(ypr1[0] * 100.0f) / 100.0f : 0.0f;
        finger["pitch_mid"] = gotMid ? roundf(ypr1[1] * 100.0f) / 100.0f : 0.0f;
        finger["roll_mid"] = gotMid ? roundf(ypr1[2] * 100.0f) / 100.0f : 0.0f;
        finger["ax_mid"] = gotMid ? roundf(ax1 * 100.0f) / 100.0f : 0.0f;
        finger["ay_mid"] = gotMid ? roundf(ay1 * 100.0f) / 100.0f : 0.0f;
        finger["az_mid"] = gotMid ? roundf(az1 * 100.0f) / 100.0f : 0.0f;

        finger["quat_w_prox"] = gotProx ? roundf(q2.w * 10000.0f) / 10000.0f : 0.0f;
        finger["quat_x_prox"] = gotProx ? roundf(q2.x * 10000.0f) / 10000.0f : 0.0f;
        finger["quat_y_prox"] = gotProx ? roundf(q2.y * 10000.0f) / 10000.0f : 0.0f;
        finger["quat_z_prox"] = gotProx ? roundf(q2.z * 10000.0f) / 10000.0f : 0.0f;

        finger["yaw_prox"] = gotProx ? roundf(ypr2[0] * 100.0f) / 100.0f : 0.0f;
        finger["pitch_prox"] = gotProx ? roundf(ypr2[1] * 100.0f) / 100.0f : 0.0f;
        finger["roll_prox"] = gotProx ? roundf(ypr2[2] * 100.0f) / 100.0f : 0.0f;
        finger["ax_prox"] = gotProx ? roundf(ax2 * 100.0f) / 100.0f : 0.0f;
        finger["ay_prox"] = gotProx ? roundf(ay2 * 100.0f) / 100.0f : 0.0f;
        finger["az_prox"] = gotProx ? roundf(az2 * 100.0f) / 100.0f : 0.0f;
    }

    tcaSelectChannel(TCA_CH_WRIST);

    JsonObject wrist = fingerData.createNestedObject("Wrist");

    sensors_event_t orientEvent;
    bno.getEvent(&orientEvent);

    sensors_event_t accelEvent;
    bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);

    imu::Quaternion wristQuat = bno.getQuat();

    wrist["ax"] = roundf(accelEvent.acceleration.x * 100.0f) / 100.0f;
    wrist["ay"] = roundf(accelEvent.acceleration.y * 100.0f) / 100.0f;
    wrist["az"] = roundf(accelEvent.acceleration.z * 100.0f) / 100.0f;

    wrist["heading"] = roundf(orientEvent.orientation.x * 100.0f) / 100.0f;
    wrist["pitch"] = roundf(orientEvent.orientation.y * 100.0f) / 100.0f;
    wrist["roll"] = roundf(orientEvent.orientation.z * 100.0f) / 100.0f;

    wrist["quat_w"] = roundf(wristQuat.w() * 10000.0f) / 10000.0f;
    wrist["quat_x"] = roundf(wristQuat.x() * 10000.0f) / 10000.0f;
    wrist["quat_y"] = roundf(wristQuat.y() * 10000.0f) / 10000.0f;
    wrist["quat_z"] = roundf(wristQuat.z() * 10000.0f) / 10000.0f;

    doc["Time"] = millis();

    serializeJson(doc, Serial);
    Serial.println();

    sendJsonOverTcp(doc);
}

void ledBlinkTask(void *parameter) {
  pinMode(LED_PIN, OUTPUT);

  while (true) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    vTaskDelay(200 / portTICK_PERIOD_MS);  // blink every 200 ms
  }
}

// Call this when you want the LED to stop blinking
void stopLedBlink() {
  if (ledTaskHandle != nullptr) {
    vTaskDelete(ledTaskHandle);
    ledTaskHandle = nullptr;
    digitalWrite(LED_PIN, LOW);  // ensure LED off
  }
}

void handleInit() {

  // bool status[4] = {false, false, false, false};
  // for (int i = 0; i < int(sizeof(HandChannels) / sizeof(HandChannels[0])); i++) {
  //   initFingerChannel(HandChannels[i], status);
  //   Serial.println("---");
  //   Serial.println(HandChannels[i].label);
  //   Serial.println(String("IMU_MID      : ") + (status[0] ? "OK" : "FAIL"));
  //   Serial.println(String("IMU_MID_DMP  : ") + (status[1] ? "OK" : "FAIL"));
  //   Serial.println(String("IMU_PROX     : ") + (status[2] ? "OK" : "FAIL"));
  //   Serial.println(String("IMU_PROX_DMP : ") + (status[3] ? "OK" : "FAIL"));
  //   status[0] = status[1] = status[2] = status[3] = false;
  // }

  // gloveInitialised = true;
  // sendReadyMessage(HAND_NAME);
  digitalWrite(1, HIGH);  // for LEDsendReadyMessage(HAND_NAME);
}

// void handleRequestData(uint32_t requestId, const char* requestTs)
// {
//   if (!gloveInitialised) return;

//   DynamicJsonDocument doc(4096);
//   doc["Hand"]         = HAND_NAME;
//   doc["request_id"]   = requestId;
//   doc["request_ts"]   = requestTs;
//   doc["glove_time_ms"] = millis();

//   JsonObject fingerData = doc.createNestedObject("Data");

//   for (int i = 0; i < int(sizeof(HandChannels) / sizeof(HandChannels[0])); i++) {
//     FingerChannel &fc = HandChannels[i];
//     tcaSelectChannel(fc.tca_channel);

//     bool gotMid  = fc.IMU_MID_EN  ? readDmpMid()  : false;
//     bool gotProx = fc.IMU_PROX_EN ? readDmpProx() : false;

//     int MCP_flex = (fc.adc_channel[0] != -1) ? analogRead(fc.adc_channel[0]) : -1;
//     int PIP_flex = (fc.adc_channel[1] != -1) ? analogRead(fc.adc_channel[1]) : -1;

//     // ↓ createNestedObject on fingerData, not a lookup on doc["Data"]
//     JsonObject finger = fingerData.createNestedObject(fc.label);

//     finger["flex_mcp"] = MCP_flex;
//     finger["flex_pip"] = PIP_flex;

//     finger["yaw_mid"]   = gotMid ? roundf(ypr1[0] * 100.0f) / 100.0f : 0.0f;
//     finger["pitch_mid"] = gotMid ? roundf(ypr1[1] * 100.0f) / 100.0f : 0.0f;
//     finger["roll_mid"]  = gotMid ? roundf(ypr1[2] * 100.0f) / 100.0f : 0.0f;
//     finger["ax_mid"]    = gotMid ? roundf(ax1 * 100.0f) / 100.0f     : 0.0f;
//     finger["ay_mid"]    = gotMid ? roundf(ay1 * 100.0f) / 100.0f     : 0.0f;
//     finger["az_mid"]    = gotMid ? roundf(az1 * 100.0f) / 100.0f     : 0.0f;

//     finger["yaw_prox"]   = gotProx ? roundf(ypr2[0] * 100.0f) / 100.0f : 0.0f;
//     finger["pitch_prox"] = gotProx ? roundf(ypr2[1] * 100.0f) / 100.0f : 0.0f;
//     finger["roll_prox"]  = gotProx ? roundf(ypr2[2] * 100.0f) / 100.0f : 0.0f;
//     finger["ax_prox"]    = gotProx ? roundf(ax2 * 100.0f) / 100.0f     : 0.0f;
//     finger["ay_prox"]    = gotProx ? roundf(ay2 * 100.0f) / 100.0f     : 0.0f;
//     finger["az_prox"]    = gotProx ? roundf(az2 * 100.0f) / 100.0f     : 0.0f;
//   }

//   doc["Time"] = millis();   // moved outside the loop — only needs to be set once

//   serializeJson(doc, Serial);
//   Serial.println();

//   sendJsonOverTcp(doc);
// }