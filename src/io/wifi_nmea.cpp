#include "io/wifi_nmea.h"

#if ENABLE_WIFI_NMEA
#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

static constexpr char WIFI_AP_SSID[] = "Tab5-GPS";
static constexpr char WIFI_AP_PASS[] = "gpstest123"; // WPA2 requires >=8 chars
static constexpr uint16_t NMEA_TCP_PORT = 10110; // conventional NMEA-over-TCP port

struct NmeaQueueMsg { char text[96]; };
static QueueHandle_t nmeaQueue;
static WiFiServer nmeaServer(NMEA_TCP_PORT);
static WiFiClient nmeaClients[4];
volatile int nmeaClientCount = 0;
volatile bool wifiEnabled = true;

void initWifiQueue() {
  nmeaQueue = xQueueCreate(64, sizeof(NmeaQueueMsg));
}

void enqueueNmea(const char *line) {
  if (!nmeaQueue) return;
  NmeaQueueMsg msg;
  strlcpy(msg.text, line, sizeof(msg.text));
  xQueueSend(nmeaQueue, &msg, 0); // non-blocking: drop rather than ever stall gpsTask
}

static void wifiTaskFn(void *) {
  bool apUp = false;

  for (;;) {
    if (wifiEnabled && !apUp) {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
      nmeaServer.begin();
      apUp = true;
      Serial.printf("WiFi AP \'%s\' up -- connect to %s:%u for raw NMEA (TCP)\n", WIFI_AP_SSID,
                    WiFi.softAPIP().toString().c_str(), NMEA_TCP_PORT);
    } else if (!wifiEnabled && apUp) {
      for (auto &c : nmeaClients) {
        if (c.connected()) c.stop();
      }
      nmeaServer.end();
      WiFi.softAPdisconnect(true); // true also powers the radio down
      WiFi.mode(WIFI_OFF);
      nmeaClientCount = 0;
      apUp = false;
      Serial.println("WiFi AP down (device asleep)");
    }

    if (apUp) {
      if (nmeaServer.hasClient()) {
        WiFiClient newClient = nmeaServer.accept();
        bool placed = false;
        for (auto &c : nmeaClients) {
          if (!c.connected()) {
            c = newClient;
            placed = true;
            break;
          }
        }
        if (!placed) newClient.stop(); // pool full
      }

      NmeaQueueMsg msg;
      while (xQueueReceive(nmeaQueue, &msg, 0) == pdTRUE) {
        for (auto &c : nmeaClients) {
          if (c.connected()) {
            c.print(msg.text);
            c.print("\r\n");
          }
        }
      }

      int cnt = 0;
      for (auto &c : nmeaClients) if (c.connected()) cnt++;
      nmeaClientCount = cnt;
    } else {
      // Keep draining while the radio is down, otherwise the queue sits full
      // and gpsTask's non-blocking sends all fail until wake.
      NmeaQueueMsg msg;
      while (xQueueReceive(nmeaQueue, &msg, 0) == pdTRUE) {
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void startWifiTask() {
  xTaskCreatePinnedToCore(wifiTaskFn, "wifi_task", 4096, nullptr, 1, nullptr, tskNO_AFFINITY);
}

#endif // ENABLE_WIFI_NMEA
