/**
 * ============================================================
 *  ESP32 — МАСТЕР / ШЛЮЗ (MASTER / GATEWAY)
 *  Протокол: ESP-NOW (без Wi-Fi роутера)
 *  Принимает данные от любого количества узлов (Node)
 * ============================================================
 *
 *  ★ ВАЖНО: перед прошивкой Node'ов получите MAC-адрес этой
 *    платы (Master выведет его в Serial Monitor при старте)
 *    и вставьте в masterMacAddr[] каждого Node.
 *
 *  Возможности:
 *    - Приём данных от N узлов одновременно
 *    - Вывод в Serial Monitor: "Датчик №[ID] зафиксировал [Значение]"
 *    - Хранение последних данных от каждого узла (до MAX_NODES)
 *    - Автоматическая регистрация новых узлов (broadcast-режим)
 *    - Мониторинг тайм-аута: предупреждение, если узел молчит > TIMEOUT_MS
 *    - Heartbeat: вывод сводки каждые 60 с
 *
 *  Зависимости:
 *    - esp_now.h  (встроенная в ESP32 Arduino)
 *    - WiFi.h     (встроенная)
 * ============================================================
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ──────────────────────────────────────────────────────────
//  Конфигурация Master
// ──────────────────────────────────────────────────────────
#define MAX_NODES           10      // Максимум узлов
#define NODE_TIMEOUT_MS     90000   // 90 с — узел считается недоступным
#define HEARTBEAT_INTERVAL  60000   // Сводка в Serial каждые 60 с

// ──────────────────────────────────────────────────────────
//  Структура данных (ДОЛЖНА СОВПАДАТЬ с Node!)
// ──────────────────────────────────────────────────────────
struct __attribute__((packed)) sensorData {
  uint8_t nodeId;
  float   pm25;
  float   so2;
  float   nh3;
};

// ──────────────────────────────────────────────────────────
//  Запись о каждом узле
// ──────────────────────────────────────────────────────────
struct NodeRecord {
  bool    active;
  uint8_t mac[6];
  uint8_t nodeId;
  float   pm25;
  float   so2;
  float   nh3;
  uint32_t lastSeen;   // millis() последнего пакета
  uint32_t rxCount;    // счётчик принятых пакетов
};

static NodeRecord nodes[MAX_NODES];
static uint8_t    nodeCount = 0;

// Мьютекс-флаг для безопасного доступа из callback (однопоточный Arduino)
static volatile bool newDataReady = false;
static sensorData    pendingData;
static uint8_t       pendingMac[6];

// ──────────────────────────────────────────────────────────
//  Вспомогательные функции
// ──────────────────────────────────────────────────────────
static void printMac(const uint8_t* mac) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static bool macEqual(const uint8_t* a, const uint8_t* b) {
  return memcmp(a, b, 6) == 0;
}

// ──────────────────────────────────────────────────────────
//  Поиск / регистрация узла по MAC
//  Возвращает указатель на запись или nullptr если нет места
// ──────────────────────────────────────────────────────────
static NodeRecord* findOrCreateNode(const uint8_t* mac, uint8_t nodeId) {
  // Поиск существующего
  for (uint8_t i = 0; i < nodeCount; i++) {
    if (macEqual(nodes[i].mac, mac)) return &nodes[i];
  }
  // Регистрация нового
  if (nodeCount >= MAX_NODES) {
    Serial.println(F("[MASTER] Превышен лимит узлов!"));
    return nullptr;
  }
  NodeRecord& r = nodes[nodeCount++];
  r.active   = true;
  r.nodeId   = nodeId;
  r.rxCount  = 0;
  r.lastSeen = 0;
  memcpy(r.mac, mac, 6);

  Serial.printf("[MASTER] Новый узел #%u (", nodeId);
  printMac(mac);
  Serial.println(')');

  // Зарегистрировать как ESP-NOW peer, если ещё не зарегистрирован
  if (!esp_now_is_peer_exist(mac)) {
    esp_now_peer_info_t pi;
    memset(&pi, 0, sizeof(pi));
    memcpy(pi.peer_addr, mac, 6);
    pi.channel = 0;
    pi.encrypt = false;
    esp_now_add_peer(&pi);
  }

  return &r;
}

// ──────────────────────────────────────────────────────────
//  Callback: приём данных (вызывается из Wi-Fi ISR!)
//  Только копируем данные — обработка в loop()
// ──────────────────────────────────────────────────────────
void IRAM_ATTR onDataReceived(const esp_now_recv_info_t* info,
                               const uint8_t* data, int len) {
  if (len != sizeof(sensorData)) return;
  memcpy(&pendingData, data, sizeof(sensorData));
  memcpy(pendingMac,   info->src_addr, 6);
  newDataReady = true;
}

// ──────────────────────────────────────────────────────────
//  Обработка принятых данных (в контексте loop)
// ──────────────────────────────────────────────────────────
static void processNewData() {
  NodeRecord* rec = findOrCreateNode(pendingMac, pendingData.nodeId);
  if (!rec) return;

  rec->pm25     = pendingData.pm25;
  rec->so2      = pendingData.so2;
  rec->nh3      = pendingData.nh3;
  rec->lastSeen = millis();
  rec->rxCount++;

  // ──── Вывод в Serial Monitor ────
  Serial.printf("\n┌─────────────────────────────────────────\n");
  Serial.printf("│ Датчик №%u зафиксировал:\n", rec->nodeId);
  Serial.printf("│   PM2.5 = %.1f мкг/м³\n",    rec->pm25);
  Serial.printf("│   SO2   = %.2f ppm\n",         rec->so2);
  Serial.printf("│   NH3   = %.2f ppm\n",         rec->nh3);
  Serial.printf("│   Пакетов получено: %lu\n",    rec->rxCount);
  Serial.printf("└─────────────────────────────────────────\n");

  // Проверка пороговых значений (пример)
  if (rec->pm25 > 35.0f)
    Serial.printf("[!] ВНИМАНИЕ: PM2.5 превышает 35 мкг/м³ на узле #%u!\n", rec->nodeId);
  if (rec->so2 > 5.0f)
    Serial.printf("[!] ВНИМАНИЕ: SO2 превышает 5 ppm на узле #%u!\n", rec->nodeId);
  if (rec->nh3 > 25.0f)
    Serial.printf("[!] ВНИМАНИЕ: NH3 превышает 25 ppm на узле #%u!\n", rec->nodeId);
}

// ──────────────────────────────────────────────────────────
//  Heartbeat — периодическая сводка
// ──────────────────────────────────────────────────────────
static void printHeartbeat() {
  uint32_t now = millis();
  Serial.println(F("\n╔══════════ СВОДКА СИСТЕМЫ ════════════╗"));
  Serial.printf( "║  Активных узлов: %u/%u\n", nodeCount, MAX_NODES);
  Serial.printf( "║  Uptime: %lu с\n", now / 1000);

  for (uint8_t i = 0; i < nodeCount; i++) {
    NodeRecord& r = nodes[i];
    uint32_t age = (now - r.lastSeen) / 1000;
    bool online  = (now - r.lastSeen) < NODE_TIMEOUT_MS;
    Serial.printf("║  Узел #%u [%s] PM2.5=%.1f SO2=%.2f NH3=%.2f (%.1f с назад)\n",
                  r.nodeId,
                  online ? "ONLINE " : "OFFLINE",
                  r.pm25, r.so2, r.nh3,
                  (float)age);
  }
  Serial.println(F("╚══════════════════════════════════════╝"));
}

// ──────────────────────────────────────────────────────────
//  Инициализация ESP-NOW
// ──────────────────────────────────────────────────────────
static bool initEspNow() {
  esp_now_deinit();
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("[MASTER] ESP-NOW init FAILED"));
    return false;
  }

  esp_now_register_recv_cb(onDataReceived);
  Serial.println(F("[MASTER] ESP-NOW init OK"));
  return true;
}

// ──────────────────────────────────────────────────────────
//  SETUP
// ──────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("\n╔══════════════════════════════════════╗"));
  Serial.println(F("║     ESP32 MASTER / ШЛЮЗ               ║"));
  Serial.println(F("╚══════════════════════════════════════╝"));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print(F("[MASTER] Мой MAC: "));
  Serial.println(WiFi.macAddress());
  Serial.println(F("[MASTER] ^^^ Скопируйте этот MAC в masterMacAddr[] на каждом Node!\n"));

  memset(nodes, 0, sizeof(nodes));

  if (!initEspNow()) {
    delay(5000);
    ESP.restart();
  }

  Serial.println(F("[MASTER] Ожидание данных от узлов..."));
}

// ──────────────────────────────────────────────────────────
//  LOOP
// ──────────────────────────────────────────────────────────
void loop() {
  static uint32_t lastHeartbeat = 0;
  static uint32_t lastTimeoutCheck = 0;

  // ── 1. Обработка новых данных (потокобезопасно) ──
  if (newDataReady) {
    newDataReady = false;  // сбрасываем ДО обработки
    processNewData();
  }

  uint32_t now = millis();

  // ── 2. Периодическая проверка тайм-аутов узлов ──
  if (now - lastTimeoutCheck >= 10000) { // каждые 10 с
    lastTimeoutCheck = now;
    for (uint8_t i = 0; i < nodeCount; i++) {
      if (nodes[i].lastSeen > 0 &&
          (now - nodes[i].lastSeen) >= NODE_TIMEOUT_MS) {
        Serial.printf("[MASTER] ⚠ Узел #%u не выходил на связь %lu сек!\n",
                      nodes[i].nodeId,
                      (now - nodes[i].lastSeen) / 1000);
      }
    }
  }

  // ── 3. Heartbeat сводка ──
  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    lastHeartbeat = now;
    printHeartbeat();
  }

  // ── 4. Автовосстановление ESP-NOW (раз в 5 минут как превентивная мера) ──
  static uint32_t lastReinit = 0;
  if (now - lastReinit >= 300000UL) { // 5 мин
    lastReinit = now;
    if (!initEspNow()) {
      Serial.println(F("[MASTER] Реинициализация не удалась, перезагрузка..."));
      delay(2000);
      ESP.restart();
    }
  }

  delay(5);
}
