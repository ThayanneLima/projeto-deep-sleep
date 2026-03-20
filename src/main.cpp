// importação de biblioteca - Versão OTAA
#include <Arduino.h> //caso não utilizar Arduino IDE
#include <lmic.h>
#include <Wire.h>
#include <WiFi.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Adafruit_INA219.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include <esp_sleep.h>

#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define OLED_ADDR 0x3C

// Tempo de deep sleep em microssegundos (120 segundos)
#define SLEEP_TIME 120000000
// Tempo de deep sleep quando bateria está crítica (600 segundos)
#define SLEEP_TIME_LOW_BATTERY 600000000
// Tensão mínima da bateria antes de aumentar o intervalo
#define BATTERY_MIN_VOLTAGE 3.6

Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);
Adafruit_INA219 ina219(0x40);

// payload para envio
uint8_t payload[9];

// PROTÓTIPOS DE FUNÇÕES
void sensors_build_payload();
void sensors_setup();

// configuração de padrão de segurança
// descomentar a configuração desejada
#define USE_OTAA
// #define USE_ABP

#ifdef USE_ABP                                                                                                                            // dados atualizados
static const PROGMEM u1_t NWKSKEY[16] = {0xD2, 0xAF, 0x51, 0xA7, 0xE5, 0x1B, 0x8E, 0xC8, 0xA6, 0x0D, 0x43, 0x84, 0xCE, 0x48, 0xFD, 0x3A}; // msb format
static const u1_t PROGMEM APPSKEY[16] = {0xF5, 0x9D, 0x4E, 0x58, 0x41, 0x51, 0x1C, 0x36, 0xAE, 0x04, 0xF9, 0xCB, 0x77, 0x6B, 0x54, 0x82}; // msb format
static const u4_t DEVADDR = 0x260DB618;
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}
#endif

#ifdef USE_OTAA
static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // lsb format
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
static const u1_t PROGMEM DEVEUI[8] = {0xAA, 0x4D, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70}; // lsb format
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
static const u1_t PROGMEM APPKEY[16] = {0xFD, 0xE1, 0x58, 0x5F, 0x53, 0x5E, 0x1D, 0xDC, 0x79, 0x47, 0x7E, 0xA5, 0xA2, 0x02, 0xBD, 0x0C}; // msb format
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }
#endif

float batteryVoltage = 0.0;
float batteryCurrent = 0.0;

// === Sensor BMP280 ===
Adafruit_BMP280 bmp;

unsigned long lastMsg = 0;
float temperatura = 0;
float pressao = 0;
float p = 0.0; // potência em W
RTC_DATA_ATTR float energy_Wh = 0.0;
RTC_DATA_ATTR bool bateria_critica = false;

// objeto que será mandado para a função do_send()
static osjob_t sendjob;

// intervalo de envio
const unsigned TX_INTERVAL = 120;

// Mapa de pinos Heltec ESP32Lora v2
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 35, 34},
};

void do_send(osjob_t *j);

// Callback de evento: todo evento do LoRaAN irá chamar essa
// callback, de forma que seja possível saber o status da
// comunicação com o gateway LoRaWAN.

void entrarDeepSleep()
{
  if (bateria_critica)
  {
    Serial.println("Bateria critica! Sleep estendido (600s)...");
    esp_sleep_enable_timer_wakeup(SLEEP_TIME_LOW_BATTERY);
  }
  else
  {
    Serial.println("Entrando em Deep Sleep...");
    esp_sleep_enable_timer_wakeup(SLEEP_TIME);
  }
  esp_deep_sleep_start();
}

void onEvent(ev_t ev)
{
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev)
  {
  case EV_SCAN_TIMEOUT:
    Serial.println(F("EV_SCAN_TIMEOUT"));
    break;
  case EV_BEACON_FOUND:
    Serial.println(F("EV_BEACON_FOUND"));
    break;
  case EV_BEACON_MISSED:
    Serial.println(F("EV_BEACON_MISSED"));
    break;
  case EV_BEACON_TRACKED:
    Serial.println(F("EV_BEACON_TRACKED"));
    break;
  case EV_JOINING:
    Serial.println(F("EV_JOINING"));
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));
    break;
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    LMIC_setLinkCheckMode(0);
    break;
  case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK)
      Serial.println(F("Received ack"));
    if (LMIC.dataLen)
    {
      Serial.print(F("Received "));
      Serial.print(LMIC.dataLen);
      Serial.println(F(" bytes of payload"));
    }
    if (LMIC.dataLen == 1)
    {
      uint8_t dados_recebidos = LMIC.frame[LMIC.dataBeg + 0];
      Serial.print(F("Dados recebidos: "));
      Serial.write(dados_recebidos);
    }
    entrarDeepSleep();
    break;
  case EV_LOST_TSYNC:
    Serial.println(F("EV_LOST_TSYNC"));
    break;
  case EV_RESET:
    Serial.println(F("EV_RESET"));
    break;
  case EV_RXCOMPLETE:
    Serial.println(F("EV_RXCOMPLETE"));
    break;
  case EV_LINK_DEAD:
    Serial.println(F("EV_LINK_DEAD"));
    break;
  case EV_LINK_ALIVE:
    Serial.println(F("EV_LINK_ALIVE"));
    break;
  case EV_TXSTART:
    Serial.println(F("EV_TXSTART"));
    break;
  case EV_TXCANCELED:
    Serial.println(F("EV_TXCANCELED"));
    break;
  case EV_RXSTART:
    break;
  case EV_JOIN_TXCOMPLETE:
    Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
    break;
  default:
    Serial.print(F("Unknown event: "));
    Serial.println((unsigned)ev);
    break;
  }
}

void sensors_setup()
{
  batteryCurrent = ina219.getCurrent_mA();
  batteryVoltage = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000.0);
  // Temperatura e pressão representativas do bloco de código BMP280
  temperatura = bmp.readTemperature();   // °C
  pressao = bmp.readPressure() / 100.0F; // hPa
  p = ina219.getPower_mW() / 1000.0;

  energy_Wh += p * (TX_INTERVAL / 3600.0); // Wh
  Serial.printf("Energia acumulada: %.5f Wh\n", energy_Wh);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("Bateria:");
  display.setCursor(0, 10);
  display.printf("%.1fmA", batteryCurrent);
  display.setCursor(0, 20);
  display.printf("%.1fV", batteryVoltage);
  display.display();

  Serial.printf("Tensão da bateria: %.2f V\n", batteryVoltage);
  Serial.printf("Corrente da bateria: %.2f mA\n", batteryCurrent);
  Serial.printf("Temperatura: %.2f C\n", temperatura);
  Serial.printf("Pressao: %.2f hPa\n", pressao);
}

void sensors_build_payload()
{
  sensors_setup();
  int16_t voltage_mv = static_cast<int16_t>(batteryVoltage * 1000);
  int16_t current_ma = static_cast<int16_t>(batteryCurrent);
  // int16_t temperature_c = static_cast<int16_t>(temperatura * 100);
  // int32_t pressure_pa = static_cast<int32_t>(pressao * 100);
  uint32_t energy_mWh = (uint32_t)(energy_Wh * 1000000.0); // mWh

  payload[0] = (voltage_mv >> 8) & 0xFF;
  payload[1] = voltage_mv & 0xFF;
  payload[2] = (current_ma >> 8) & 0xFF;
  payload[3] = current_ma & 0xFF;
  // payload[4] = (temperature_c >> 8) & 0xFF;
  // payload[5] = temperature_c & 0xFF;
  // payload[6] = (pressure_pa >> 16) & 0xFF;
  // payload[7] = (pressure_pa >> 8) & 0xFF;
  // payload[8] = pressure_pa & 0xFF;
  payload[4] = (energy_mWh >> 24) & 0xFF;
  payload[5] = (energy_mWh >> 16) & 0xFF;
  payload[6] = (energy_mWh >> 8) & 0xFF;
  payload[7] = energy_mWh & 0xFF;
  payload[8] = bateria_critica ? 0x01 : 0x00;
}

void do_send(osjob_t *j)
{
  sensors_build_payload();

  if (batteryVoltage > 0 && batteryVoltage <= BATTERY_MIN_VOLTAGE)
  {
    bateria_critica = true;
    Serial.printf("ALERTA: Bateria baixa! %.2fV - intervalo de sleep aumentado para 600s\n", batteryVoltage);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.printf("BATERIA BAIXA!");
    display.setCursor(0, 10);
    display.printf("%.2fV", batteryVoltage);
    display.display();
  }
  else
  {
    bateria_critica = false;
  }

  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    LMIC_setTxData2(1, payload, sizeof(payload), 0);
    Serial.println(F("Sended"));
  }
}

void setup()
{
#ifdef USE_ABP
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);
#endif

  Serial.begin(115200);
  Serial.println(F("Starting"));

  // Display
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(10);
  digitalWrite(OLED_RST, HIGH);
  delay(10);
  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
  {
    Serial.println("Falha ao iniciar o display OLED!");
    while (true)
      ;
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Iniciando...");
  display.display();

  // INA219
  if (!ina219.begin())
  {
    Serial.println("Falha no INA219");
  }
  ina219.setCalibration_32V_1A();

  // ina219.configure(
  // INA219_CONFIG_BADCRES_12BIT,
  // INA219_CONFIG_SADCRES_12BIT_128S
  //);

  // Inicia BMP280
  if (!bmp.begin(0x76))
  { // ou 0x77 dependendo do seu sensor
    Serial.println("Sensor BMP280 nao encontrado!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Erro BMP280!");
    display.display();
    while (true)
      ;
  }

  Serial.println("Sensor BMP280 iniciado.");

  sensors_setup();

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  do_send(&sendjob); // Start
}

void loop()
{
  os_runloop_once();
}