#include "Net.h"

/*
*************************** Net.cpp ****************************************************

Filename:       Net.cpp
Author:         Jestin

Description:    Implementation of the Wi-Fi and TCP transport for the
                request/response ("sync") firmware. initWifi() joins the
                configured access point and opens the first TCP connection
                to the PC receiver. sendJsonOverTcp() serialises one
                DynamicJsonDocument into a stack buffer, appends a newline
                delimiter (so the PC side can split on '\n'), and writes the
                result over TCP. pollTcpCommands() reads inbound JSON lines
                from the PC, parses each one, and dispatches INIT,
                REQUEST_DATA and RESTART commands to the supplied callbacks
                or to the firmware restart path.

                The active access-point and PC IP are selected from the user
                settings block below. The alternate configs (home router,
                phone hotspots, etc.) are kept as documented presets, comment
                out the active block and uncomment the desired one when
                moving between locations.

Dependencies:   Net.h

*****************************************************************************************
*/


// ===== USER SETTINGS =====
// Active config: laptop hotspot
static const char*    WIFI_SSID     = "jestin-OMEN-Gaming-Laptop-16-am0";
static const char*    WIFI_PASSWORD = "87654321";
static const char*    TCP_HOST      = "10.42.0.1";
static const uint16_t TCP_PORT      = 5000;

// Preset: home router
// static const char*    WIFI_SSID     = "Belong96E660";
// static const char*    WIFI_PASSWORD = "u7255uutshe3gzaq";
// static const char*    TCP_HOST      = "192.168.1.57";
// static const uint16_t TCP_PORT      = 5000;

// Preset: phone hotspot (S22 Ultra)
// static const char*    WIFI_SSID     = "Jestin's S22 Ultra";
// static const char*    WIFI_PASSWORD = "12345678";
// static const char*    TCP_HOST      = "10.91.215.94";
// static const uint16_t TCP_PORT      = 5000;

// Preset: Telstra router (CACE44)
// static const char*    WIFI_SSID     = "TelstraCACE44";
// static const char*    WIFI_PASSWORD = "ht9cwmhxzf";
// static const char*    TCP_HOST      = "192.168.0.140";
// static const uint16_t TCP_PORT      = 5000;

// Preset: Jason's Telstra router (39AF91)
// static const char*    WIFI_SSID     = "Telstra39AF91";
// static const char*    WIFI_PASSWORD = "ns25garzcm";
// static const char*    TCP_HOST      = "192.168.0.58";
// static const uint16_t TCP_PORT      = 5000;

// Preset: Ameline iPhone hotspot
// static const char*    WIFI_SSID     = "A iPhone";
// static const char*    WIFI_PASSWORD = "ilyssa123";
// static const char*    TCP_HOST      = "172.20.10.9";
// static const uint16_t TCP_PORT      = 5000;
// ==========================


// Persistent TCP client, reconnect cooldown timestamp, and the rolling
// line buffer used to assemble inbound JSON commands a character at a time
static WiFiClient    tcpClient;
static unsigned long lastConnectAttempt = 0;
static String        rxBuffer;


static void ensureTcpConnected()
{
  /*
  Reopens the TCP connection to the PC receiver if it has dropped. To avoid
  hammering the network when the PC is unreachable, this function only
  attempts a new connection every 2 seconds.

  Input:
      No direct inputs, reads the module-level tcpClient and the static
      TCP_HOST / TCP_PORT settings.

  Output:
      No return value. On success tcpClient is left in a connected state.
  */
  if (tcpClient.connected()) return;

  unsigned long now = millis();
  if (now - lastConnectAttempt < 2000) return;

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


void initWifi()
{
  /*
  Brings up the Wi-Fi station, joins the configured access point, and opens
  the first TCP connection to the PC receiver. The function blocks until
  the Wi-Fi association completes, printing a dot every 500 ms while
  waiting. Once the association is up, the assigned IP is printed and
  ensureTcpConnected() is called for the initial TCP handshake.

  Input:
      No direct inputs, reads the static WIFI_SSID / WIFI_PASSWORD settings.

  Output:
      No return value. The Wi-Fi radio and tcpClient are left in a connected
      state on success.
  */
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Wi-Fi connected, IP: ");
  Serial.println(WiFi.localIP());

  ensureTcpConnected();
}


void sendJsonOverTcp(const DynamicJsonDocument& doc)
{
  /*
  Serialises one DynamicJsonDocument into a stack buffer, appends a newline
  delimiter so the PC receiver can split on '\n', and writes the result
  over the TCP connection. The transport is reconnected first if it has
  dropped, and silently skipped if the reconnect failed, so a missing PC
  does not stall the request handler.

  Input:
      doc (const DynamicJsonDocument&):   The fully populated glove data
                                          packet to be transmitted.

  Output:
      No return value. On success one newline-terminated JSON line is
      written to the PC, on failure a diagnostic message is printed to
      Serial and the function returns without sending.
  */
  ensureTcpConnected();
  if (!tcpClient.connected()) return;

  char jsonBuf[4096];

  // Reserve two bytes at the end of the buffer for '\n' and the trailing '\0'
  size_t len = serializeJson(doc, jsonBuf, sizeof(jsonBuf) - 2);
  if (len == 0 || len >= sizeof(jsonBuf) - 2) {
    Serial.println("TCP: JSON too large or error serializing");
    return;
  }

  jsonBuf[len]     = '\n';
  jsonBuf[len + 1] = '\0';
  len += 1;

  size_t written = tcpClient.write((uint8_t*)jsonBuf, len);
  if (written != len) {
    Serial.println("TCP: short write");
  }
}


void pollTcpCommands(void (*onInit)(), void (*onRequestData)(uint32_t, const char*))
{
  /*
  Drains any bytes currently waiting on the TCP socket and assembles them
  into newline-terminated JSON lines in rxBuffer. Each complete line is
  parsed into a temporary JsonDocument and dispatched by its "type" field:
  INIT triggers the onInit callback, REQUEST_DATA triggers the
  onRequestData callback with the request_id / request_ts echoed back to
  the PC, and RESTART reboots the ESP32-S3. Malformed lines are logged to
  Serial and skipped.

  Input:
      onInit (void (*)()):                            Callback invoked when
                                                      an INIT command is
                                                      received.

      onRequestData (void (*)(uint32_t, const char*)): Callback invoked when
                                                      a REQUEST_DATA command
                                                      is received, with the
                                                      request_id and
                                                      request_ts arguments
                                                      forwarded from the PC.

  Output:
      No return value. The two callbacks may transmit a response packet
      via sendJsonOverTcp().
  */
  ensureTcpConnected();
  if (!tcpClient.connected()) return;

  while (tcpClient.available()) {
    char c = (char)tcpClient.read();

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
        uint32_t requestId    = doc["request_id"] | 0;
        const char* requestTs = doc["request_ts"] | "";
        Serial.print("Calling onRequestData(), id=");
        Serial.println(requestId);
        onRequestData(requestId, requestTs);
      } else if (strcmp(type, "RESTART") == 0) {
        Serial.println("Received RESTART");
        digitalWrite(1, LOW);  // drop the status LED before rebooting
        delay(100);
        ESP.restart();
      }
    } else {
      rxBuffer += c;
    }
  }
}
