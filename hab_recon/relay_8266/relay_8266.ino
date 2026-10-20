/*
  =====================================================================
   HAB-RECON  -  RELAY NODE ("second balloon")     board: ESP8266 (ESP-12 / NodeMCU)
  =====================================================================
   OPTIONAL. Re-broadcasts every packet it hears with hops = 1, so traffic
   can go payload -> relay -> ground. Shows the balloon-to-balloon mesh idea.

   For the demo set ACCEPT_DIRECT 0 in ground_c6.ino: the ground station
   then only listens to the relay. Unplug the relay -> link lost;
   plug it back -> link restored. The dashboard shows the path.

   Power it from any USB charger / power bank. Nothing to wire.

   ARDUINO IDE
     Board: "Generic ESP8266 Module" or "NodeMCU 1.0 (ESP-12E Module)"
     (esp8266 by ESP8266 Community, version 3.x)
  =====================================================================
*/
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "protocol.h"

static uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

volatile bool havePkt = false;
uint8_t buf[250];
uint8_t bufLen = 0;
uint32_t relayed = 0;

void onRecv(uint8_t *mac, uint8_t *data, uint8_t len) {
  (void)mac;
  if (havePkt || len > sizeof(buf)) return;           // still busy with the last one
  bool tlm = len == sizeof(Telemetry) && data[0] == TLM_MAGIC && data[1] == 0;
  bool cmd = len == sizeof(CmdPacket) && data[0] == CMD_MAGIC && data[1] == 0;
  bool hlt = len == sizeof(HealthPacket) && data[0] == HEALTH_MAGIC && data[1] == 0;
  if (!tlm && !cmd && !hlt) return;                           // only relay first-hop packets (no loops)
  memcpy(buf, data, len);
  bufLen = len;
  havePkt = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // off (active low)

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_set_channel(ESPNOW_CHANNEL);
  if (esp_now_init() != 0) {
    Serial.println("ESP-NOW init FAILED");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(onRecv);
  esp_now_add_peer(BCAST, ESP_NOW_ROLE_COMBO, ESPNOW_CHANNEL, NULL, 0);
  Serial.println("\nRelay ready");
}

void loop() {
  static uint32_t ledOff = 0, lastDbg = 0;
  if (havePkt) {
    buf[1] = 1;  // hops: byte 1 in both packet types
    esp_now_send(BCAST, buf, bufLen);
    havePkt = false;
    relayed++;
    digitalWrite(LED_BUILTIN, LOW);
    ledOff = millis() + 10;
  }
  if (ledOff && millis() >= ledOff) { digitalWrite(LED_BUILTIN, HIGH); ledOff = 0; }
  if (millis() - lastDbg > 2000) {
    lastDbg = millis();
    Serial.printf("relayed %lu packets\n", (unsigned long)relayed);
  }
}
