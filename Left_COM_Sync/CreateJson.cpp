#include "CreateJson.h"
#include "Globals.h"

/*
*************************** CreateJson.cpp *********************************************

Filename:       CreateJson.cpp
Author:         Jestin

Description:    Implementation of the JSON-skeleton helpers declared in CreateJson.h.
                The keys added here mirror the schema produced by the streaming loop
                in the main .ino, so a packet built via buildFingerData() will be
                accepted by the same PC receiver code that ingests live data.

Dependencies:   CreateJson.h    Globals.h

*****************************************************************************************
*/


void addFinger(JsonObject parent, const char *name)
{
  /*
  Adds a single empty finger block as a nested object under parent, with every
  flex sensor and IMU field zero-initialised.

  Input:
      parent (JsonObject):    The JSON object the new finger block is attached to,
                              typically the "Data" object of the outgoing packet.

      name (const char*):     The label for the new finger block, used as the
                              key under parent.

  Output:
      No return value, parent is updated in place with the new nested object.
  */
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


void buildFingerData(JsonObject fingerData)
{
  /*
  Populates fingerData with one empty finger block per entry in HandChannels,
  producing a complete skeleton packet that mirrors the schema used by the
  live streaming loop.

  Input:
      fingerData (JsonObject):    The JSON object that the per-finger blocks
                                  are attached to, typically the "Data" object
                                  of the outgoing packet.

  Output:
      No return value, fingerData is updated in place.
  */
  const int numFingers = (int)(sizeof(HandChannels) / sizeof(HandChannels[0]));
  for (int i = 0; i < numFingers; i++) {
    addFinger(fingerData, HandChannels[i].label.c_str());
  }
}
