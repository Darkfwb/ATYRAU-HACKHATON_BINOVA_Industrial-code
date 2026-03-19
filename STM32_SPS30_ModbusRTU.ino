/**
 * ============================================================
 *  STM32 + Sensirion SPS30 (I2C) → Modbus RTU (Hardware Serial)
 *  Arduino Framework (STM32duino / PlatformIO)
 *  Оптимизировано для непрерывной работы 24/7
 * ============================================================
 *
 *  Подключение:
 *    SPS30 SDA  → PB7 (I2C1 SDA)
 *    SPS30 SCL  → PB6 (I2C1 SCL)
 *    SPS30 SEL  → GND (режим I2C)
 *    SPS30 VDD  → 5V (или 3.3V, зависит от модели)
 *    SPS30 GND  → GND
 *
 *    RS-485 модуль → USART2 (PA2=TX, PA3=RX)
 *    RS-485 DE/RE  → PA1 (контроль направления)
 *
 *  Карта Modbus Holding Registers (Function Code 03/04):
 *    40001 (0x0000) - PM1.0  × 10  [uint16]
 *    40002 (0x0001) - PM2.5  × 10  [uint16]
 *    40003 (0x0002) - PM4.0  × 10  [uint16]
 *    40004 (0x0003) - PM10   × 10  [uint16]
 *    40005 (0x0004) - NC0.5  × 10  [uint16] (# концентрация)
 *    40006 (0x0005) - NC1.0  × 10  [uint16]
 *    40007 (0x0006) - NC2.5  × 10  [uint16]
 *    40008 (0x0007) - NC4.0  × 10  [uint16]
 *    40009 (0x0008) - NC10   × 10  [uint16]
 *    40010 (0x0009) - TPS    × 10  [uint16] (типичный размер частиц)
 *    40011 (0x000A) - Статус: 0=OK, 1=ошибка датчика, 2=не инициализирован
 *    40012 (0x000B) - Счётчик успешных измерений (uint16, wraps at 65535)
 *
 *  Зависимости (PlatformIO / Arduino IDE Library Manager):
 *    - Sensirion I2C SPS30  (by Sensirion AG)  v1.x
 *    - Wire (встроенная)
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2cSps30.h>

// ──────────────────────────────────────────────────────────
//  Конфигурация
// ──────────────────────────────────────────────────────────
#define MODBUS_SLAVE_ID       0x01    // Адрес этого устройства на шине Modbus
#define MODBUS_BAUD           9600    // Скорость RS-485
#define MODBUS_SERIAL         Serial2 // USART2: PA2(TX), PA3(RX) на STM32F103
#define RS485_DE_RE_PIN       PA1     // Управление направлением TX/RX

#define SPS30_MEASURE_INTERVAL_MS  1000  // Интервал опроса датчика (мс)
#define SPS30_WARMUP_MS            30000 // Прогрев вентилятора SPS30

#define MODBUS_RESPONSE_TIMEOUT_US 1750  // t3.5 для 9600 бод ≈ 1.6 мс
#define MODBUS_MAX_ADU_SIZE        256

// ──────────────────────────────────────────────────────────
//  Глобальные объекты
// ──────────────────────────────────────────────────────────
SensirionI2cSps30 sps30;

// Holding Registers (индекс 0 = адрес 40001)
#define REG_COUNT 12
static volatile uint16_t holdingRegs[REG_COUNT] = {0};
// Индексы регистров
enum RegIdx : uint8_t {
  REG_PM1   = 0,
  REG_PM25  = 1,
  REG_PM4   = 2,
  REG_PM10  = 3,
  REG_NC05  = 4,
  REG_NC1   = 5,
  REG_NC25  = 6,
  REG_NC4   = 7,
  REG_NC10  = 8,
  REG_TPS   = 9,
  REG_STATUS= 10,
  REG_COUNT_MEAS = 11
};

// Статус датчика
enum SensorStatus : uint16_t {
  STAT_OK     = 0,
  STAT_ERROR  = 1,
  STAT_INIT   = 2
};

// ──────────────────────────────────────────────────────────
//  CRC-16/ANSI (Modbus)
// ──────────────────────────────────────────────────────────
static uint16_t crc16(const uint8_t* buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= (uint16_t)buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else              crc >>= 1;
    }
  }
  return crc;
}

// ──────────────────────────────────────────────────────────
//  RS-485: отправка байт
// ──────────────────────────────────────────────────────────
static void rs485Send(const uint8_t* data, uint8_t len) {
  digitalWrite(RS485_DE_RE_PIN, HIGH); // Передача
  MODBUS_SERIAL.write(data, len);
  MODBUS_SERIAL.flush();               // Ждём окончания отправки
  digitalWrite(RS485_DE_RE_PIN, LOW);  // Приём
}

// ──────────────────────────────────────────────────────────
//  Modbus: отправка исключения
// ──────────────────────────────────────────────────────────
static void modbusException(uint8_t fc, uint8_t code) {
  uint8_t pdu[5];
  pdu[0] = MODBUS_SLAVE_ID;
  pdu[1] = fc | 0x80;
  pdu[2] = code;
  uint16_t crc = crc16(pdu, 3);
  pdu[3] = (uint8_t)(crc & 0xFF);
  pdu[4] = (uint8_t)(crc >> 8);
  rs485Send(pdu, 5);
}

// ──────────────────────────────────────────────────────────
//  Modbus: обработка входящего запроса (FC03 / FC04)
//  Возвращает true, если ответ отправлен
// ──────────────────────────────────────────────────────────
static bool modbusProcess(const uint8_t* req, uint8_t reqLen) {
  if (reqLen < 8) return false;

  // Проверка CRC
  uint16_t crcCalc = crc16(req, reqLen - 2);
  uint16_t crcRecv = (uint16_t)req[reqLen - 1] << 8 | req[reqLen - 2];
  if (crcCalc != crcRecv) return false;

  // Проверка адреса
  if (req[0] != MODBUS_SLAVE_ID) return false;

  uint8_t fc        = req[1];
  uint16_t startAddr = (uint16_t)req[2] << 8 | req[3];
  uint16_t quantity  = (uint16_t)req[4] << 8 | req[5];

  // Поддерживаем FC03 и FC04
  if (fc != 0x03 && fc != 0x04) {
    modbusException(fc, 0x01); // Illegal Function
    return true;
  }

  // Проверка диапазона регистров
  if (quantity == 0 || quantity > 125 ||
      startAddr + quantity > REG_COUNT) {
    modbusException(fc, 0x02); // Illegal Data Address
    return true;
  }

  // Формируем ответ
  uint8_t byteCount = quantity * 2;
  uint8_t respLen   = 5 + byteCount;
  uint8_t resp[MODBUS_MAX_ADU_SIZE];

  resp[0] = MODBUS_SLAVE_ID;
  resp[1] = fc;
  resp[2] = byteCount;

  for (uint16_t i = 0; i < quantity; i++) {
    uint16_t val = holdingRegs[startAddr + i];
    resp[3 + i * 2]     = (uint8_t)(val >> 8);
    resp[3 + i * 2 + 1] = (uint8_t)(val & 0xFF);
  }

  uint16_t crc = crc16(resp, respLen - 2);
  resp[respLen - 2] = (uint8_t)(crc & 0xFF);
  resp[respLen - 1] = (uint8_t)(crc >> 8);

  rs485Send(resp, respLen);
  return true;
}

// ──────────────────────────────────────────────────────────
//  Инициализация SPS30 с повторными попытками
// ──────────────────────────────────────────────────────────
static bool initSPS30() {
  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    int16_t err = sps30.startContinuousMeasurement(0x0300);
    if (err == 0) {
      holdingRegs[REG_STATUS] = STAT_INIT; // прогрев
      return true;
    }
    delay(500);
  }
  holdingRegs[REG_STATUS] = STAT_ERROR;
  return false;
}

// ──────────────────────────────────────────────────────────
//  Чтение данных SPS30 и обновление регистров
// ──────────────────────────────────────────────────────────
static void updateSensorData() {
  float pm1, pm25, pm4, pm10;
  float nc05, nc1, nc25, nc4, nc10, tps;

  int16_t err = sps30.readMeasuredValues(
    pm1, pm25, pm4, pm10,
    nc05, nc1, nc25, nc4, nc10, tps
  );

  if (err != 0) {
    holdingRegs[REG_STATUS] = STAT_ERROR;
    // Попытка реинициализации
    sps30.stopMeasurement();
    delay(100);
    initSPS30();
    return;
  }

  // Масштабируем float × 10 → uint16 (одна десятичная)
  holdingRegs[REG_PM1]  = (uint16_t)(pm1  * 10.0f + 0.5f);
  holdingRegs[REG_PM25] = (uint16_t)(pm25 * 10.0f + 0.5f);
  holdingRegs[REG_PM4]  = (uint16_t)(pm4  * 10.0f + 0.5f);
  holdingRegs[REG_PM10] = (uint16_t)(pm10 * 10.0f + 0.5f);
  holdingRegs[REG_NC05] = (uint16_t)(nc05 * 10.0f + 0.5f);
  holdingRegs[REG_NC1]  = (uint16_t)(nc1  * 10.0f + 0.5f);
  holdingRegs[REG_NC25] = (uint16_t)(nc25 * 10.0f + 0.5f);
  holdingRegs[REG_NC4]  = (uint16_t)(nc4  * 10.0f + 0.5f);
  holdingRegs[REG_NC10] = (uint16_t)(nc10 * 10.0f + 0.5f);
  holdingRegs[REG_TPS]  = (uint16_t)(tps  * 10.0f + 0.5f);
  holdingRegs[REG_STATUS] = STAT_OK;

  uint16_t cnt = holdingRegs[REG_COUNT_MEAS];
  holdingRegs[REG_COUNT_MEAS] = (cnt < 65535) ? cnt + 1 : 0;
}

// ──────────────────────────────────────────────────────────
//  Watchdog через аппаратный IWDG STM32
//  PlatformIO: добавить в platformio.ini:
//    build_flags = -DUSE_IWDG
// ──────────────────────────────────────────────────────────
#ifdef USE_IWDG
  #include <IWatchdog.h>
  #define WDG_TIMEOUT_MS  8000   // 8 секунд
  static void wdgInit() { IWatchdog.begin(WDG_TIMEOUT_MS * 1000); }
  static void wdgReset() { IWatchdog.reload(); }
#else
  static void wdgInit()  {}
  static void wdgReset() {}
#endif

// ──────────────────────────────────────────────────────────
//  SETUP
// ──────────────────────────────────────────────────────────
void setup() {
  // Отладочный UART (опционально)
  Serial.begin(115200);

  // RS-485
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  MODBUS_SERIAL.begin(MODBUS_BAUD, SERIAL_8N1);

  // I2C + SPS30
  Wire.begin();
  Wire.setClock(100000); // 100 кГц (SPS30 не поддерживает 400кГц стабильно)
  sps30.begin(Wire);

  holdingRegs[REG_STATUS] = STAT_INIT;

  if (!initSPS30()) {
    Serial.println(F("[SPS30] Init FAILED"));
  } else {
    Serial.println(F("[SPS30] Init OK, warming up..."));
  }

  wdgInit();
}

// ──────────────────────────────────────────────────────────
//  LOOP
// ──────────────────────────────────────────────────────────
void loop() {
  static uint32_t lastMeasure = 0;
  static uint32_t startTime   = millis();
  static bool     warmupDone  = false;
  static uint8_t  rxBuf[MODBUS_MAX_ADU_SIZE];
  static uint8_t  rxLen = 0;
  static uint32_t lastByteTime = 0;

  uint32_t now = millis();

  // ── 1. Прогрев SPS30 (30 с после старта) ──
  if (!warmupDone && (now - startTime >= SPS30_WARMUP_MS)) {
    warmupDone = true;
    Serial.println(F("[SPS30] Warmup complete"));
  }

  // ── 2. Периодическое чтение датчика ──
  if (warmupDone && (now - lastMeasure >= SPS30_MEASURE_INTERVAL_MS)) {
    lastMeasure = now;
    updateSensorData();
    wdgReset();
  }

  // ── 3. Приём Modbus запроса ──
  //    Используем тайм-аут межсимвольного интервала (t3.5)
  while (MODBUS_SERIAL.available()) {
    if (rxLen < MODBUS_MAX_ADU_SIZE) {
      rxBuf[rxLen++] = (uint8_t)MODBUS_SERIAL.read();
    }
    lastByteTime = micros();
  }

  // Если прошло t3.5 после последнего байта — фрейм завершён
  if (rxLen > 0 && (micros() - lastByteTime) > MODBUS_RESPONSE_TIMEOUT_US) {
    modbusProcess(rxBuf, rxLen);
    rxLen = 0;
  }

  // ── 4. Авто-очистка буфера при переполнении ──
  if (rxLen >= MODBUS_MAX_ADU_SIZE) {
    rxLen = 0;
  }

  // Небольшая задержка, чтобы не жечь CPU впустую
  delayMicroseconds(50);
}
