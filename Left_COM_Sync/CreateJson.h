#pragma once

#include <ArduinoJson.h>
#include "Globals.h"

/*
*************************** CreateJson.h ***********************************************

Filename:       CreateJson.h
Author:         Jestin

Description:    Declares helpers for prepopulating a JsonObject with the per-finger
                key schema expected by the PC receiver. These helpers are useful
                when an empty packet skeleton needs to be constructed up front
                (e.g. for handshake messages or for guaranteeing every finger key
                exists even when sensors are absent).

Dependencies:   ArduinoJson     Globals.h

*****************************************************************************************
*/


void addFinger(JsonObject parent, const char *name);

void buildFingerData(JsonObject fingerData);
