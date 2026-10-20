/*
  =====================================================================
   HAB-RECON  -  GROUND STATION RECEIVER           board: Seeed XIAO ESP32-C6
  =====================================================================
   Bridge between the radio and the laptop:
     radio -> USB :  "T,<packet hex>,<rssi dBm>"   for every telemetry packet
                     "H,<packet hex>,<rssi dBm>"   for every sensor-health packet (1/s)
     USB -> radio :  "C,<cmd>,<seq>,<lat_e7>,<lon_e7>"  from the dashboard,
                     broadcast 3x so a lost packet doesn't lose the command
   Plug it into the laptop with a USB-C DATA cable and open dashboard.html.

   ARDUINO IDE
     Board: "XIAO_ESP32C6"  (esp32 by Espressif, version 3.x)
     Tools -> USB CDC On Boot: Enabled
  =====================================================================
*/
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "protocol.h"

// Set to 0 to accept ONLY packets that came through the relay (ESP8266).
// Use it to demo the "balloon-to-balloon mesh": payload -> relay -> ground.
#define ACCEPT_DIRECT 1

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct RxItem {
  uint8_t len;
  int8_t rssi;
  uint8_t data[250];
};
QueueHandle_t rxQ;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  int8_t rssi = (info && info->rx_ctrl) ? info->rx_ctrl->rssi : 0;
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  int8_t rssi = 0;
#endif
  bool tlm = len == sizeof(Telemetry) && data[0] == TLM_MAGIC;
  bool hlt = len == sizeof(HealthPacket) && data[0] == HEALTH_MAGIC;
  if (!tlm && !hlt) return;
  RxItem it;
  it.len = len;
  it.rssi = rssi;
  memcpy(it.data, data, len);
  xQueueSend(rxQ, &it, 0);
}

void sendCommand(uint8_t cmd, uint16_t seq, int32_t lat, int32_t lon) {
  CmdPacket c = {};
  c.magic = CMD_MAGIC;
  c.hops = 0;
  c.cmd_seq = seq;
  c.cmd = cmd;
  c.lat_e7 = lat;
  c.lon_e7 = lon;
  for (int i = 0; i < 3; i++) {
    esp_now_send(BCAST, (const uint8_t *)&c, sizeof c);
    delay(25);
  }
  Serial.printf("I,command %u sent (seq %u)\n", cmd, seq);
}

void handleSerialLine(char *line) {
  if (line[0] != 'C' || line[1] != ',') return;
  long v[4] = {0, 0, 0, 0};
  char *p = line + 2;
  for (int i = 0; i < 4 && *p; i++) {
    v[i] = strtol(p, &p, 10);
    if (*p == ',') p++;
  }
  sendCommand((uint8_t)v[0], (uint16_t)v[1], (int32_t)v[2], (int32_t)v[3]);
}

void setup() {
  Serial.begin(115200);
  rxQ = xQueueCreate(16, sizeof(RxItem));
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // off (active low)

  // XIAO ESP32-C6 antenna switch: GPIO3 low turns it on, GPIO14 low = built-in ceramic antenna
  // (set it explicitly so the radio works whichever board option was used for the upload)
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);
  delay(50);
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  if (esp_now_init() != ESP_OK) {
    Serial.println("I,ESP-NOW init FAILED");
    return;
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  Serial.printf("I,ground station ready on channel %d, MAC %s\n", ESPNOW_CHANNEL, WiFi.macAddress().c_str());
}

void loop() {
  static char line[96];
  static uint8_t len = 0;
  static uint32_t ledOff = 0, lastHello = 0, lastRx = 0;
  static bool hearing = false, warned = false;

  // radio -> USB
  RxItem it;
  while (xQueueReceive(rxQ, &it, 0) == pdTRUE) {
    if (!ACCEPT_DIRECT && it.data[1] == 0) continue;  // hops == 0 -> came straight from payload
    static char out[2 * 250 + 24];
    int n = snprintf(out, sizeof out, "%c,", it.data[0] == HEALTH_MAGIC ? 'H' : 'T');
    for (int i = 0; i < it.len; i++) n += snprintf(out + n, sizeof out - n, "%02X", it.data[i]);
    snprintf(out + n, sizeof out - n, ",%d", it.rssi);
    Serial.println(out);
    digitalWrite(LED_BUILTIN, LOW);
    ledOff = millis() + 10;
    lastRx = millis();
    if (!hearing) { hearing = true; warned = false; Serial.println("I,receiving the payload"); }
  }
  // say so when the payload goes quiet (once, not every second)
  if (hearing && millis() - lastRx > 5000) {
    hearing = false;
    Serial.println("I,no packets from the payload for 5 s - is it powered, antenna on, same firmware version?");
  } else if (!hearing && !warned && millis() > 10000 && !lastRx) {
    warned = true;
    Serial.println("I,no packets from the payload yet - is it powered, antenna on, same firmware version?");
  }
  if (ledOff && millis() >= ledOff) { digitalWrite(LED_BUILTIN, HIGH); ledOff = 0; }

  // USB -> radio
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len) { line[len] = 0; handleSerialLine(line); len = 0; }
    } else if (len < sizeof(line) - 1) {
      line[len++] = ch;
    }
  }

  if (millis() - lastHello > 5000) {  // heartbeat so the dashboard knows we're alive
    lastHello = millis();
    Serial.println("I,alive");
  }
}
