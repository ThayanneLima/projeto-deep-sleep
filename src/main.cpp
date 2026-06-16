#include <Arduino.h>
#include <lmic.h>
#include <Wire.h>
#include <WiFi.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Adafruit_INA219.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <Adafruit_BMP280.h>

#define OLED_SDA  4
#define OLED_SCL  15
#define OLED_RST  16
#define OLED_ADDR 0x3C

#define LORA_GAIN 20
#define CFG_au915

// ============================================================
// Credenciais LoRaWAN OTAA
// ============================================================
static const u1_t PROGMEM APPEUI[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // lsb
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

static const u1_t PROGMEM DEVEUI[8]  = { 0xAA, 0x4D, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }; // lsb
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

static const u1_t PROGMEM APPKEY[16] = {
    0xFD, 0xE1, 0x58, 0x5F, 0x53, 0x5E, 0x1D, 0xDC, 
    0x79, 0x47, 0x7E, 0xA5, 0xA2, 0x02, 0xBD, 0x0C
}; // msb
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// ============================================================
// Periféricos
// ============================================================
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);
Adafruit_INA219  ina219(0x40);
//Adafruit_BMP280  bmp;

// ============================================================
// Dados dos sensores
// ============================================================
float batteryVoltage = 0.0;
float batteryCurrent = 0.0;
//float temperatura    = 0.0;
//float pressao        = 0.0;
float energy_Wh      = 0.0;

// ============================================================
// Payload LoRaWAN — 8 bytes
//   [0-1] tensão    × 1000 → int16  (mV)
//   [2-3] corrente  × 100  → int16  (0.01 mA)
//   [4-7] energia   × 1e6  → uint32 (mWh)
// ============================================================
#define PAYLOAD_SIZE 8
uint8_t payload[PAYLOAD_SIZE];

// ============================================================
// LMIC
// ============================================================
static osjob_t sendjob;
const unsigned TX_INTERVAL = 20;

const lmic_pinmap lmic_pins = {
    .nss  = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst  = 14,
    .dio  = { 26, 35, 34 },
};

void do_send(osjob_t* j);

// ============================================================
// Leitura dos sensores
// ============================================================
static void sensors_read() {
    delay(10);
    batteryVoltage = ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV() / 1000.0f);
    batteryCurrent = ina219.getCurrent_mA();
    //temperatura    = bmp.readTemperature();
    //pressao        = bmp.readPressure() / 100.0f; // Pa → hPa

    // Energia acumulada
    float p_W = batteryVoltage * (batteryCurrent / 1000.0f);
    energy_Wh += p_W * (TX_INTERVAL / 3600.0f);

    Serial.printf("Tensão:    %.3f V\n",   batteryVoltage);
    Serial.printf("Corrente:  %.2f mA\n",  batteryCurrent);
    Serial.printf("Potência:  %.2f mW\n",  p_W * 1000.0f);
    Serial.printf("Energia:   %.5f Wh\n",  energy_Wh);
    //Serial.printf("Temp:      %.2f °C\n",  temperatura);
    //Serial.printf("Pressão:   %.2f hPa\n", pressao);
}

// ============================================================
// Montagem do payload
// ============================================================
static void build_payload() {
    int16_t  voltage_mv  = (int16_t) (batteryVoltage * 1000.0f);
    int16_t  current_x100 = (int16_t)(batteryCurrent * 100.0f);
    uint32_t energy_mWh  = (uint32_t)(energy_Wh * 1000000.0f);

    payload[0] = (voltage_mv   >> 8) & 0xFF;
    payload[1] =  voltage_mv         & 0xFF;
    payload[2] = (current_x100 >> 8) & 0xFF;
    payload[3] =  current_x100       & 0xFF;
    payload[4] = (energy_mWh   >> 24) & 0xFF;
    payload[5] = (energy_mWh   >> 16) & 0xFF;
    payload[6] = (energy_mWh   >>  8) & 0xFF;
    payload[7] =  energy_mWh          & 0xFF;

    Serial.print("TX bytes: ");
    for (int k = 0; k < PAYLOAD_SIZE; k++) Serial.printf("%02X ", payload[k]);
    Serial.println();
}

// ============================================================
// Atualiza display
// ============================================================
/*static void update_display() {
    display.clearDisplay();
    display.setCursor(0,  0); display.printf("V:  %.3f V",   batteryVoltage);
    display.setCursor(0, 12); display.printf("I:  %.1f mA",  batteryCurrent);
    //display.setCursor(0, 24); display.printf("T:  %.1f C",   temperatura);
    //display.setCursor(0, 36); display.printf("P:  %.1f hPa", pressao);
    display.setCursor(0, 48); display.printf("E:  %.4f Wh",  energy_Wh);
    display.display();
}*/

// ============================================================
// Callback LMIC
// ============================================================
void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
        case EV_JOINING:          Serial.println(F("EV_JOINING"));          break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:      Serial.println(F("EV_JOIN_FAILED"));      break;
        case EV_REJOIN_FAILED:    Serial.println(F("EV_REJOIN_FAILED"));    break;
        case EV_TXSTART:          Serial.println(F("EV_TXSTART"));          break;
        case EV_TXCANCELED:       Serial.println(F("EV_TXCANCELED"));       break;
        case EV_RXSTART:          break;
        case EV_LINK_DEAD:        Serial.println(F("EV_LINK_DEAD"));        break;
        case EV_LINK_ALIVE:       Serial.println(F("EV_LINK_ALIVE"));       break;
        case EV_JOIN_TXCOMPLETE:  Serial.println(F("EV_JOIN_TXCOMPLETE"));  break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE"));
            if (LMIC.txrxFlags & TXRX_ACK)
                Serial.println(F("ACK recebido."));
            if (LMIC.dataLen)
                Serial.printf("Downlink: %d bytes.\n", LMIC.dataLen);
            // Agenda o próximo envio
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            break;
        default:
            Serial.printf("Evento desconhecido: %u\n", (unsigned)ev);
            break;
    }
}

// ============================================================
// Envio
// ============================================================
void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND — aguardando TX anterior."));
        return;
    }

    sensors_read();
    build_payload();
    //update_display();

    LMIC_setTxData2(1, payload, PAYLOAD_SIZE, 0);
    Serial.println(F("Packet queued."));
}

// ============================================================
// setup()
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.println(F("Iniciando..."));

    // I2C
    Wire.begin(OLED_SDA, OLED_SCL);

    // Display OLED
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);  delay(10);
    digitalWrite(OLED_RST, HIGH); delay(10);

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println(F("Falha ao iniciar o display OLED!"));
        while (true);
    }
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Iniciando...");
    display.display();

    // INA219
    if (!ina219.begin()) {
        Serial.println(F("Falha no INA219!"));
        while (true);
    }
    ina219.setCalibration_16V_400mA();
    Serial.println(F("INA219 inicializado."));

    /* BMP280
    if (!bmp.begin(0x76)) {
        Serial.println(F("BMP280 não encontrado!"));
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Erro BMP280!");
        display.display();
        while (true);
    }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X4,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println(F("BMP280 inicializado."));*/

    // LMIC
    os_init();
    LMIC_reset();
    LMIC_selectSubBand(1);       // AU915 — sub-banda 2 (recomendada pelo TTN)
    LMIC_setLinkCheckMode(0);
    LMIC.dn2Dr = DR_SF12CR;      // AU915: RX2 usa SF12/500 kHz (DR8)
    LMIC_setDrTxpow(DR_SF10, LORA_GAIN);

    do_send(&sendjob);
}

// ============================================================
// loop()
// ============================================================
void loop() {
    os_runloop_once();
}
