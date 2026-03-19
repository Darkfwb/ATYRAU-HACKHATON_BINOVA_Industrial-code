/**
 * ============================================================
 *  ESP32 — УЗЕЛ (NODE) системы промышленного мониторинга
 *  Протокол: ESP-NOW (без Wi-Fi роутера)
 *  Отправка данных на Master каждые 30 секунд
 * ============================================================
 *
 *  Подключение датчиков (пример):
 *    SPS30  → I2C (SDA=GPIO21, SCL=GPIO22)
 *    SO2    → ADC GPIO34 (или отдельный I2C/UART модуль)
 *    NH3    → ADC GPIO35
 *
 *  Перед прошивкой установите:
 *    - nodeId       : уникальный ID этого узла
 *    - masterMacAddr: MAC-адрес платы Master
 *
 *  Зависимости (PlatformIO / Arduino IDE):
 *    - esp_now.h  (встроенная в ESP32 Arduino)
 *    - WiFi.h     (встроенная)
 *    - Sensirion I2C SPS30 (для реальных данных PM2.5)
 * ============================================================
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ──────────────────────────────────────────────────────────
//  ★ НАСТРОЙТЕ ЭТИ ПАРАМЕТРЫ ДЛЯ КАЖДОГО УЗЛА
// ──────────────────────────────────────────────────────────
static const uint8_t NODE_ID = 1;  // Уникальный ID узла (1, 2, 3 ...)

// MAC-адрес платы Master (узнать командой: Serial.println(WiFi.macAddress()))
static uint8_t masterMacAddr[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

// Интервал отправки данных (мс)
#define SEND_INTERVAL_MS  30000

// ──────────────────────────────────────────────────────────
//  Структура данных (должна совпадать на Node и Master!)
// ──────────────────────────────────────────────────────────
struct __attribute__((packed)) sensorData {
  uint8_t nodeId;   // ID узла
  float   pm25;     // PM2.5  мкг/м³
  float   so2;      // SO2    ppm
  float   nh3;      // NH3    ppm
};

// ──────────────────────────────────────────────────────────
//  Глобальные переменные
// ──────────────────────────────────────────────────────────
static esp_now_peer_info_t peerInfo;
static volatile bool       lastSendOK     = false;
static volatile bool       sendResultReady= false;
static uint8_t             failCount      = 0;
#define MAX_FAIL_BEFORE_REINIT  5

// ──────────────────────────────────────────────────────────
//  Callback: результат отправки
// ──────────────────────────────────────────────────────────
void IRAM_ATTR onDataSent(const uint8_t* macAddr, esp_now_send_status_t status) {
  lastSendOK      = (status == ESP_NOW_SEND_SUCCESS);
  sendResultReady = true;
}

// ──────────────────────────────────────────────────────────
//  Чтение реальных данных с датчиков
//  Замените заглушки реальными драйверами
// ──────────────────────────────────────────────────────────
static float readPM25() {
  // TODO: подключить Sensirion SPS30 по I2C
  // sps30.readMeasuredValues(...);
  return 12.5f + (float)(random(0, 100)) / 10.0f; // заглушка
}

static float readSO2() {
  // TODO: АЦП или I2C-модуль датчика SO2
  int raw = analogRead(34);
  return raw * (20.0f / 4095.0f); // 0..20 ppm (линейная модель)
}

static float readNH3() {
  // TODO: АЦП или I2C-модуль датчика NH3
  int raw = analogRead(35);
  return raw * (50.0f / 4095.0f); // 0..50 ppm
}

// ──────────────────────────────────────────────────────────
//  Инициализация ESP-NOW с регистрацией peer
// ──────────────────────────────────────────────────────────
static bool initEspNow() {
  // Остановить предыдущий сеанс, если он был
  esp_now_deinit();
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("[ESP-NOW] Init FAILED"));
    return false;
  }

  esp_now_register_send_cb(onDataSent);

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, masterMacAddr, 6);
  peerInfo.channel = 0;   // автовыбор канала
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println(F("[ESP-NOW] Add peer FAILED"));
    return false;
  }

  Serial.println(F("[ESP-NOW] Init OK"));
  return true;
}

// ──────────────────────────────────────────────────────────
//  SETUP
// ──────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\n[NODE-%u] Загрузка...\n", NODE_ID);

  // ESP-NOW требует Wi-Fi в режиме STA, но БЕЗ подключения к роутеру
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Печатаем свой MAC (нужен для регистрации в Master)
  Serial.print(F("[NODE] MAC: "));
  Serial.println(WiFi.macAddress());

  if (!initEspNow()) {
    Serial.println(F("[NODE] Перезагрузка через 5 сек..."));
    delay(5000);
    ESP.restart();
  }

  // Настройка АЦП
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // 0..3.3 В

  Serial.printf("[NODE-%u] Готов. Интервал отправки: %u с\n",
                NODE_ID, SEND_INTERVAL_MS / 1000);
}

// ──────────────────────────────────────────────────────────
//  LOOP
// ──────────────────────────────────────────────────────────
void loop() {
  static uint32_t lastSendTime = 0;
  uint32_t now = millis();

  // ── Периодическая отправка ──
  if (now - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime    = now;
    sendResultReady = false;

    // Собираем данные
    sensorData payload;
    payload.nodeId = NODE_ID;
    payload.pm25   = readPM25();
    payload.so2    = readSO2();
    payload.nh3    = readNH3();

    Serial.printf("[NODE-%u] Отправка: PM2.5=%.1f SO2=%.2f NH3=%.2f\n",
                  payload.nodeId, payload.pm25, payload.so2, payload.nh3);

    esp_err_t result = esp_now_send(masterMacAddr,
                                    (uint8_t*)&payload,
                                    sizeof(payload));

    if (result != ESP_OK) {
      Serial.printf("[NODE-%u] esp_now_send error: %d\n", NODE_ID, result);
      failCount++;
    }
  }

  // ── Проверяем результат доставки (из callback) ──
  if (sendResultReady) {
    sendResultReady = false;
    if (lastSendOK) {
      Serial.printf("[NODE-%u] ✓ Доставлено успешно\n", NODE_ID);
      failCount = 0;
    } else {
      Serial.printf("[NODE-%u] ✗ Ошибка доставки (сбоев подряд: %u)\n",
                    NODE_ID, ++failCount);
    }
  }

  // ── Автовосстановление при накоплении ошибок ──
  if (failCount >= MAX_FAIL_BEFORE_REINIT) {
    Serial.printf("[NODE-%u] %u сбоев подряд — реинициализация ESP-NOW\n",
                  NODE_ID, failCount);
    failCount = 0;
    if (!initEspNow()) {
      Serial.println(F("[NODE] Реинициализация не удалась, перезагрузка..."));
      delay(2000);
      ESP.restart();
    }
  }

  delay(10); // Освобождаем CPU для системных задач ESP-IDF
}
