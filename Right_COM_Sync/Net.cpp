#include "Net.h"

// ===== USER SETTINGS =====
// static const char* WIFI_SSID     = "Belong96E660";
// static const char* WIFI_PASSWORD = "u7255uutshe3gzaq";

// // PC server config (change to your PC's LAN IP)
// static const char* TCP_HOST      = "192.168.1.57";  // your computer IP
// static const uint16_t TCP_PORT   = 5001;


static const char* WIFI_SSID     = "jestin-OMEN-Gaming-Laptop-16-am0";
static const char* WIFI_PASSWORD = "87654321";

// PC server config (change to your PC's LAN IP)
static const char* TCP_HOST      = "10.42.0.1";  // your computer IP
static const uint16_t TCP_PORT   = 5001;

// static const char* WIFI_SSID     = "Jestin's S22 Ultra";
// static const char* WIFI_PASSWORD = "12345678";

// // PC server config (change to your PC's LAN IP)
// static const char* TCP_HOST      = "10.91.215.94";  // your computer IP
// static const uint16_t TCP_PORT   = 5001;

// static const char* WIFI_SSID     = "TelstraCACE44";
// static const char* WIFI_PASSWORD = "ht9cwmhxzf";

// // PC server config (change to your PC's LAN IP)
// static const char* TCP_HOST      = "192.168.0.140";  // your computer IP
// static const uint16_t TCP_PORT   = 5001;


// Amelines's
// static const char* WIFI_SSID     = "A iPhone";
// static const char* WIFI_PASSWORD = "ilyssa123";



// // PC server config (change to your PC's LAN IP)
// static const char* TCP_HOST      = "172.20.10.9";  // your computer IP
// static const uint16_t TCP_PORT   = 5001;



// static const char* WIFI_SSID     = "Telstra39AF91";
// static const char* WIFI_PASSWORD = "ns25garzcm";

// // PC server config (change to your PC's LAN IP)
// static const char* TCP_HOST      = "192.168.0.58";  // your computer IP
// static const uint16_t TCP_PORT   = 5001;


// ==========================

static WiFiClient tcpClient;
static unsigned long lastConnectAttempt = 0;
static String rxBuffer;

static void ensureTcpConnected() {
  if (tcpClient.connected()) return;

  unsigned long now = millis();
  if (now - lastConnectAttempt < 2000) return;  // retry every 2s

  lastConnectAttempt = now;
  Serial.print("Connecting TCP to ");
  Serial.print(TCP_HOST);
  Serial.print(":");
  Serial.println(TCP_PORT);

  if (tcpClient.connect(TCP_HOST, TCP_PORT)) {
    Serial.println("TCP connected");
  } else {
    Serial.println("TCP connect failed");
  }
}

void initWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to " + String(WIFI_SSID) + " Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Wi-Fi connected, IP: ");
  Serial.println(WiFi.localIP());

  // First TCP attempt (subsequent handled in ensureTcpConnected)
  ensureTcpConnected();
}

void sendJsonOverTcp(const DynamicJsonDocument& doc) {
  ensureTcpConnected();
  if (!tcpClient.connected()) return;

  char jsonBuf[4096];

  // leave room for '\n' + '\0'
  size_t len = serializeJson(doc, jsonBuf, sizeof(jsonBuf) - 2);
  if (len == 0 || len >= sizeof(jsonBuf) - 2) {
    Serial.println("TCP: JSON too large or error serializing");
    return;
  }

  jsonBuf[len] = '\n';
  jsonBuf[len + 1] = '\0';
  len += 1;

  size_t written = tcpClient.write((uint8_t*)jsonBuf, len);
  if (written != len) {
    Serial.println("TCP: short write");
  }
}

void sendReadyMessage(const char* handName) {
  DynamicJsonDocument doc(128);
  doc["type"] = "READY";
  doc["hand"] = handName;
  sendJsonOverTcp(doc);
}

void pollTcpCommands(void (*onInit)(), void (*onRequestData)(uint32_t, const char*)) {
  ensureTcpConnected();
  if (!tcpClient.connected()) {
    // Optional: uncomment if you want to see disconnections
    // Serial.println("pollTcpCommands: not connected");
    return;
  }

  // Comment this down once things are stable; it's very spammy
  // Serial.println("pollTcpCommands running");

  while (tcpClient.available()) {
    char c = (char)tcpClient.read();
    // Serial.print("RX char: ");
    // Serial.println((int)c);

    if (c == '\n') {
      Serial.print("Full RX line: ");
      Serial.println(rxBuffer);

      DynamicJsonDocument doc(256);
      DeserializationError err = deserializeJson(doc, rxBuffer);
      rxBuffer = "";

      if (err) {
        Serial.print("JSON parse failed: ");
        Serial.println(err.c_str());
        continue;
      }

      const char* type = doc["type"] | "";
      Serial.print("Received type: ");
      Serial.println(type);

      if (strcmp(type, "INIT") == 0) {
        Serial.println("Calling onInit()");
        onInit();
      } else if (strcmp(type, "REQUEST_DATA") == 0) {
        uint32_t requestId = doc["request_id"] | 0;
        const char* requestTs = doc["request_ts"] | "";
        Serial.print("Calling onRequestData(), id=");
        Serial.println(requestId);
        onRequestData(requestId, requestTs);
      }
      else if (strcmp(type, "RESTART") == 0) {
        Serial.println("Received RESTART");
        digitalWrite(1, LOW);  // for LED
        delay(100);
        ESP.restart();
      }
    } else {
      rxBuffer += c;
    }
  }
}



// void handleRequestData(uint32_t requestId, const char* requestTs) {
//   if (!gloveInitialised) return;

//   DataPacket["Hand"] = HAND_NAME;
//   DataPacket["request_id"] = requestId;
//   DataPacket["request_ts"] = requestTs;
//   DataPacket["glove_time_ms"] = millis();

//   sendJsonOverTcp(DataPacket);
// }


