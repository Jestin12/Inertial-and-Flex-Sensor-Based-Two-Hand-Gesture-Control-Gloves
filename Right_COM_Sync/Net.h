#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>



// Call once from setup()
void initWifi();

void handleRequestData(uint32_t requestId, const char* requestTs);

// Call periodically from loop() to send the current DataPacket frame
void sendJsonOverTcp(const DynamicJsonDocument& doc);

void sendReadyMessage(const char* handName);

void pollTcpCommands(void (*onInit)(), void (*onRequestData)(uint32_t, const char*));