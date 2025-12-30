#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_INA219.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP280.h>
#include <PubSubClient.h>
#include <esp_sleep.h> // Biblioteca necessária para usar o deep sleep
#include <LittleFS.h>
#include <HTTPClient.h>
#include <lmic.h>
#include <hal/hal.h>

#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define OLED_ADDR 0x3C

// Tempo de deep sleep em microssegundos (15 segundos)
#define SLEEP_TIME 15000000

Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);

Adafruit_INA219 ina219(0x40);

#define LMIC_DEBUG_LEVEL 1
#define CFG_au915
#define LORA_GAIN 20

void buildPacket(uint8_t txBuffer[24]);
void do_send(osjob_t *j);

#ifdef COMPILE_REGRESSION_TEST
#define FILLMEIN 0
#else
#warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
#define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// Configurações da API
// const char *host = "api.thingspeak.com";       // Este não altera
// const int httpPort = 80;                       // Este não altera
// const String channelID = "3165224";            // COLOCAR O SEU CHANNELID
// const String writeApiKey = "LJ9592VTW4FD06P3"; // COLOCAR A SUA KEY
// void readResponse(WiFiClient *client);

// ======== CONFIG URL IOT AGENT =========
// Troque o IP abaixo pelo IP DO SEU DOCKER/HOST
// String url = "http://192.168.0.105:4041/iot/json";

// config for  DHT Lorawan
static const PROGMEM u1_t NWKSKEY[16] = {0x49, 0x67, 0x69, 0x68, 0x9C, 0xB7, 0xE9, 0xDC, 0xBD, 0x80, 0x66, 0xE9, 0xBA, 0x5A, 0x68, 0xDD};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = {0x84, 0xBE, 0xC9, 0xC9, 0x81, 0x3D, 0x08, 0x0A, 0x80, 0x80, 0x14, 0xD8, 0xC0, 0x40, 0x24, 0x62};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260D45C0;

int n_packet = 0;
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

// static uint8_t mydata[4];
uint8_t txBuffer[100];

const uint8_t NUM_SAMPLES = 15;

static osjob_t sendjob;

float batteryVoltage = 0.0;
float batteryCurrent = 0.0;

// === Sensor BMP280 ===
Adafruit_BMP280 bmp;

// === Dados do Wifi ===
// const char *ssid = "Instructiva_Group";
// const char *password = "Familia10k*#";
// const char *mqtt_server = "test.mosquitto.org";

unsigned long lastMsg = 0;
float temperatura = 0;
float pressao = 0;

unsigned long displayTime = 0;

struct SampleEntry
{
  uint16_t vb;  // mV
  uint16_t ib;  // mA * 10
  uint8_t mode; // OperationMode
};

SampleEntry samples[NUM_SAMPLES];

uint8_t sampleIndex = 0;
bool bufferFull = false;
uint32_t t0_samples_ms = 0;

float lastTemp = 0.0;
float lastPress = 0.0;
unsigned long lastSampleTime = 0;

enum OperationMode
{
  MODE_ACTIVE,      // more than 4.0V
  MODE_MODEM_SLEEP, // 3.8V to 4.0V
  MODE_LIGHT_SLEEP, // 3.7V to 3.8V
  MODE_DEEP_SLEEP,  // 3.5V to 3.7V
  MODE_HIBERNATION  // less than 3.5V
};

OperationMode operationMode;

unsigned long lastSentData = 0;

const unsigned TX_INTERVAL = 15; // 300=5 minutos

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,          // For TTGO 14, T-Beam 23
    .dio = {26, 33, 32} // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

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
  /*
    || This event is defined but not used in the code. No
    || point in wasting codespace on it.
    ||
    || case EV_RFU1:
    ||     Serial.println(F("EV_RFU1"));
    ||     break;
  */
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
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
      Serial.println(F("Received "));
      Serial.println(LMIC.dataLen);
      Serial.println(F(" bytes of payload"));
    }
    // Schedule next transmission
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send); // o LMIC agenda o próximo envio após cada transmissão:
    break;
  case EV_LOST_TSYNC:
    Serial.println(F("EV_LOST_TSYNC"));
    break;
  case EV_RESET:
    Serial.println(F("EV_RESET"));
    break;
  case EV_RXCOMPLETE:
    // data received in ping slot
    Serial.println(F("EV_RXCOMPLETE"));
    break;
  case EV_LINK_DEAD:
    Serial.println(F("EV_LINK_DEAD"));
    break;
  case EV_LINK_ALIVE:
    Serial.println(F("EV_LINK_ALIVE"));
    break;
  /*
    || This event is defined but not used in the code. No
    || point in wasting codespace on it.
    ||
    || case EV_SCAN_FOUND:
    ||    Serial.println(F("EV_SCAN_FOUND"));
    ||    break;
  */
  case EV_TXSTART:
    Serial.println(F("EV_TXSTART"));
    break;
  case EV_TXCANCELED:
    Serial.println(F("EV_TXCANCELED"));
    break;
  case EV_RXSTART:
    /* do not print anything -- it wrecks timing */
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
int count = 5;
void do_send(osjob_t *j)
{
  // Check if there is not a current TX/RX job running

  // displayValues();

  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
    // LoraStatus = "OP_TXRXPEND, not sending";
  }
  else
  {
    // led.clear();
    // led.drawString(0,0,"fake packet");
    buildPacket(txBuffer);

    n_packet++;
    LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0); // é responsável por enviar os dados no uplink para o servidor LoRaWAN
    Serial.println(F("Packet queued"));
    // LoraStatus = "Packet queued";
  }

  // Next TX is scheduled after TX_COMPLETE event.
}

void setupLoRaWAN()
{

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession(0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set. The LMIC doesn't let you change
  // the three basic settings, but we show them here.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band
                                                                               // LMIC_setupChannel(8, 994000000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  2);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915) || defined(CFG_au915)
  // NA-US and AU channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);

#elif defined(CFG_as923)
  // Set up the channels used in your country. Only two are defined by default,
  // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
  // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

  // ... extra definitions for channels 2..n here
#elif defined(CFG_kr920)
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#elif defined(CFG_in866)
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#else
#error Region not supported
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF10;

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF10, LORA_GAIN);

  // Start job
  do_send(&sendjob);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Iniciando");

  // Inicializa o barramento I2C nos pinos GPIO 21 (SDA) e GPIO 22 (SCL)
  Wire.begin(4, 15);

  setupLoRaWAN();

  // WiFi.begin(ssid, password);
  // Serial.println("Aguardando conexão...");
  /*while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi Conectado...");
*/
  // Display
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(10);
  digitalWrite(OLED_RST, HIGH);
  delay(10);
  //Wire.begin(OLED_SDA, OLED_SCL);
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

  operationMode = MODE_ACTIVE;

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
}

void loop()
{
  unsigned long start = millis();

  batteryCurrent = ina219.getCurrent_mA();
  batteryVoltage = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000.0);

  // coleta uma amostra a cada 1 segundo e guarda no vetor
  if (millis() - lastSampleTime >= 1000)
  {
    float vbat = ina219.getBusVoltage_V() +
                 (ina219.getShuntVoltage_mV() / 1000.0);
    float ibat = ina219.getCurrent_mA();

    // Temperatura e pressão representativas do bloco
    lastTemp = bmp.readTemperature();        // °C
    lastPress = bmp.readPressure() / 100.0F; // hPa

    // converte para inteiros compactos
    uint16_t vb_i = (uint16_t)(vbat * 1000.0); // mV
    uint16_t ib_i = (uint16_t)(ibat * 10.0);   // mA * 10

    if (!bufferFull && sampleIndex == 0)
    {
      t0_samples_ms = millis(); // marca tempo da 1ª amostra do pacote
    }

    samples[sampleIndex].vb = vb_i;
    samples[sampleIndex].ib = ib_i;
    samples[sampleIndex].mode = (uint8_t)operationMode;

    sampleIndex++;
    if (sampleIndex >= NUM_SAMPLES)
    {
      sampleIndex = 0;
      bufferFull = true; // bloco pronto para envio no do_send
    }

    lastSampleTime = millis();
  }

  /* Operation mode
  if (batteryVoltage >= 4.0)
  {
    operationMode = MODE_ACTIVE;
  }
  else if (batteryVoltage >= 3.8)
  {
    operationMode = MODE_MODEM_SLEEP;
  }
  else if (batteryVoltage >= 3.7)
  {
    operationMode = MODE_LIGHT_SLEEP;
  }
  else if (batteryVoltage >= 3.5)
  {
    operationMode = MODE_DEEP_SLEEP;
  }
  else
  {
    operationMode = MODE_HIBERNATION;
  }
*/
  operationMode = MODE_ACTIVE;
  display.clearDisplay();

  switch (operationMode)
  {
  case MODE_ACTIVE:
    // Serial.println("Modo Ativo");
    display.setCursor(0, 30);
    break;

  case MODE_MODEM_SLEEP:
    // Serial.println("Modo Modem Sleep");
    display.setCursor(0, 30);
    display.printf("Modo Modem Sleep");
    Serial.println("Modo Modem Sleep");
    WiFi.setSleep(true);

    break;

  case MODE_LIGHT_SLEEP:
    // Serial.println("Modo Light Sleep");
    display.setCursor(0, 30);
    display.printf("Modo Light Sleep");
    Serial.println("Modo Light Sleep");
    esp_sleep_enable_timer_wakeup(SLEEP_TIME);
    esp_light_sleep_start();
    break;

  case MODE_DEEP_SLEEP:
    display.setCursor(0, 30);
    display.printf("Modo Deep Sleep");
    Serial.println("Modo Deep Sleep");

    esp_sleep_enable_timer_wakeup(SLEEP_TIME);
    esp_deep_sleep_start();
    break;

  case MODE_HIBERNATION: // O ESP32 não tem hibernação no Arduino.Só no ESP-IDF (e funciona igual ao deep sleep, mas com mais domínios desligados).
    display.setCursor(0, 30);
    display.printf("Modo Hibernacao");
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF); // Mas isso não é “hibernação real”, é só deep sleep mais agressivo.
    esp_deep_sleep_start();
    break;
  }

  display.setCursor(0, 0);
  display.printf("Bateria:");
  display.setCursor(0, 10);
  display.printf("%.1fmA", batteryCurrent);
  display.setCursor(0, 20);
  display.printf("%.1fV", batteryVoltage);
  display.display();

  float temperatura = bmp.readTemperature();   // °C
  float pressao = bmp.readPressure() / 100.0F; // hPa

  Serial.printf("Temperatura: %.2f C\n", temperatura);
  Serial.printf("Pressao: %.2f hPa\n", pressao);

  // Necessário para LMIC processar eventos e disparar do_send
  os_runloop_once();
}

void buildPacket(uint8_t txBuffer[100])
{
  // tempo da primeira amostra em segundos desde o boot
  uint32_t t0_s = t0_samples_ms / 1000;

  // compacta temperatura e pressao do bloco
  int16_t temp_i = (int16_t)(lastTemp * 100.0);   // centésimos de °C
  uint16_t pres_i = (uint16_t)(lastPress * 10.0); // décimos de hPa

  // numero da mensagem (contador de uplinks)
  uint16_t msg_i = (uint16_t)n_packet;

  // CABEÇALHO
  // bytes 0–1: numero da mensagem
  txBuffer[0] = highByte(msg_i);
  txBuffer[1] = lowByte(msg_i);

  // bytes 2–5: t0 (segundos desde o boot)
  txBuffer[2] = (t0_s >> 24) & 0xFF;
  txBuffer[3] = (t0_s >> 16) & 0xFF;
  txBuffer[4] = (t0_s >> 8) & 0xFF;
  txBuffer[5] = t0_s & 0xFF;

  // bytes 6–7: temperatura do bloco
  txBuffer[6] = highByte(temp_i);
  txBuffer[7] = lowByte(temp_i);

  // bytes 8–9: pressao do bloco
  txBuffer[8] = highByte(pres_i);
  txBuffer[9] = lowByte(pres_i);

  // byte 10: numero de amostras no vetor
  txBuffer[10] = NUM_SAMPLES;

  // A partir do byte 11: vetor de amostras
  // cada amostra = 5 bytes: vb (2) + ib (2) + mode (1)
  uint8_t idx = 11;

  for (uint8_t i = 0; i < NUM_SAMPLES; i++)
  {
    uint16_t vb_i = samples[i].vb; // mV
    uint16_t ib_i = samples[i].ib; // mA*10
    uint8_t md = samples[i].mode;  // OperationMode

    txBuffer[idx++] = highByte(vb_i);
    txBuffer[idx++] = lowByte(vb_i);

    txBuffer[idx++] = highByte(ib_i);
    txBuffer[idx++] = lowByte(ib_i);

    txBuffer[idx++] = md;
  }

  Serial.print("Pacote ");
  Serial.print(n_packet);
  Serial.print(" com ");
  Serial.print(NUM_SAMPLES);
  Serial.println(" amostras pronto.");

  Serial.print("Temperatura: ");
  Serial.print(temperatura);
  Serial.println(" °C");
  Serial.print("Pressão: ");
  Serial.print(pressao);
  Serial.println(" hPa");
}
