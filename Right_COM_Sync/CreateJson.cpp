#include "CreateJson.h"
#include "Globals.h"

void addFinger(JsonObject parent, const char *name) {
  JsonObject f = parent.createNestedObject(name);
  f["flex_mcp"]   = 0;
  f["flex_pip"]   = 0;
  f["yaw_mid"]    = 0.0f;
  f["pitch_mid"]  = 0.0f;
  f["roll_mid"]   = 0.0f;
  f["ax_mid"]     = 0.0f;
  f["ay_mid"]     = 0.0f;
  f["az_mid"]     = 0.0f;
  f["yaw_prox"]   = 0.0f;
  f["pitch_prox"] = 0.0f;
  f["roll_prox"]  = 0.0f;
  f["ax_prox"]    = 0.0f;
  f["ay_prox"]    = 0.0f;
  f["az_prox"]    = 0.0f;
}

void buildFingerData(JsonObject fingerData) {
  for (int i = 0; i < int(sizeof(HandChannels) / sizeof(HandChannels[0])); i++) {
    addFinger(fingerData, HandChannels[i].label.c_str());
  }
}
