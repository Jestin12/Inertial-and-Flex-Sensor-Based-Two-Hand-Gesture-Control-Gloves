#pragma once

#include <ArduinoJson.h>
#include "Globals.h"

void addFinger(JsonObject parent, const char *name);

void buildFingerData(JsonObject fingerData);
