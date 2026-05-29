#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>

/*
*************************** Net.h ******************************************************

Filename:       Net.h
Author:         Jestin

Description:    Declares the networking helpers used by the request/response
                ("sync") firmware variant. initWifi() brings up the Wi-Fi
                station and opens the first TCP connection to the PC receiver,
                sendJsonOverTcp() writes one packet of glove data over that
                connection, and pollTcpCommands() consumes inbound JSON lines
                from the PC and dispatches them to the supplied INIT and
                REQUEST_DATA callbacks. handleRequestData() is the firmware-side
                callback that responds to a REQUEST_DATA command by building
                and transmitting one fresh glove packet.

Dependencies:   Arduino.h       WiFi.h          ArduinoJson

*****************************************************************************************
*/


void initWifi();

void sendJsonOverTcp(const DynamicJsonDocument& doc);

void pollTcpCommands(void (*onInit)(), void (*onRequestData)(uint32_t, const char*));

void handleRequestData(uint32_t requestId, const char* requestTs);
