#include "Arduino.h"
#include "LoRaWan_APP.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "esp_sleep.h"

//=========== CLAVES OTAA (REEMPLAZA POR LAS REALES) ===========
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x53, 0xC9 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x91, 0x3F, 0xA7, 0x2B, 0x54, 0xCC, 0x89, 0x10,
                     0xEF, 0xD2, 0x73, 0x41, 0x65, 0x9B, 0x20, 0x8D };

//=========== DEFINICIONES QUE EXIGE LA LIBRERÍA (aunque uses OTAA) ===========
uint8_t nwkSKey[] = { 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
                      0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F };
uint8_t appSKey[] = { 0x0F,0x0E,0x0D,0x0C,0x0B,0x0A,0x09,0x08,
                      0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00 };
uint32_t devAddr   = 0x26011111;

// Más símbolos requeridos por LoRaWan_APP.cpp
// Canalización por máscara: ajusta a tu subbanda si hace falta.
// US915 común: 0x00FF = canales 0..7; 0xFF00 = 8..15.
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

// Número de reintentos para confirmados (aunque isTxConfirmed=false, la lib lo referencia)
uint8_t confirmedNbTrials = 4;

//=========== PARÁMETROS LoRaWAN ===========
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;   // Define en Tools → LoRaWAN Region
DeviceClass_t   loraWanClass  = CLASS_A;
bool overTheAirActivation     = true;            // OTAA
bool loraWanAdr               = true;
bool isTxConfirmed            = false;           // sin ACK
uint8_t appPort               = 2;
uint32_t appTxDutyCycle       = 60000;           // 60 s entre uplinks

//=========== HARDWARE ===========
#define SLEEP_SECONDS  60

// BME280 en I2C1
#define BME_SDA   41
#define BME_SCL   42
#define I2C_FREQ  400000

// Alimentación sensor y batería
#define VEXT_CTRL  36              // LOW=ON, HIGH=OFF
#define VBAT_PIN    1              // ADC1_CH0
#define VBAT_RATIO  2.00f          // calibra si hace falta

TwoWire I2CBME(1);
Adafruit_BME280 bme;

// appData y appDataSize están declarados por la librería (extern)
// deviceState, txDutyCycleTime también (¡no los declares ni con extern!)

// Empaqueta: T*100, H*100, P[hPa], Vbat[mV]  => 8 bytes
static void prepareTxFrame(uint8_t port)
{
  // Enciende Vext para el BME
  pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, LOW);
  delay(20);

  I2CBME.begin(BME_SDA, BME_SCL, I2C_FREQ);
  bool ok = bme.begin(0x76, &I2CBME);
  if (!ok) ok = bme.begin(0x77, &I2CBME);

  float T=0, H=0, P_hPa=0;
  if (ok) {
    bme.setSampling(
      Adafruit_BME280::MODE_FORCED,
      Adafruit_BME280::SAMPLING_X16,
      Adafruit_BME280::SAMPLING_X16,
      Adafruit_BME280::SAMPLING_X16,
      Adafruit_BME280::FILTER_X16
    );
    bme.takeForcedMeasurement();
    T     = bme.readTemperature();
    H     = bme.readHumidity();
    P_hPa = bme.readPressure() / 100.0f;
  }

  // Batería
  analogReadResolution(12);
  analogSetPinAttenuation(VBAT_PIN, ADC_11db);
  uint16_t vbat_mV = (uint16_t)(analogReadMilliVolts(VBAT_PIN) * VBAT_RATIO + 0.5f);

  // Empaque binario
  int16_t t = (int16_t)(T * 100);
  int16_t h = (int16_t)(H * 100);
  int16_t p = (int16_t)(P_hPa);
  uint16_t v = vbat_mV;

  appDataSize = 8;
  appData[0] = (uint8_t)(t >> 8); appData[1] = (uint8_t)(t & 0xFF);
  appData[2] = (uint8_t)(h >> 8); appData[3] = (uint8_t)(h & 0xFF);
  appData[4] = (uint8_t)(p >> 8); appData[5] = (uint8_t)(p & 0xFF);
  appData[6] = (uint8_t)(v >> 8); appData[7] = (uint8_t)(v & 0xFF);

  // Apaga Vext
  digitalWrite(VEXT_CTRL, HIGH);
}

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
}

void loop()
{
  switch (deviceState)
  {
    case DEVICE_STATE_INIT:
    {
#if (LORAWAN_DEVEUI_AUTO)
      LoRaWAN.generateDeveuiByChipID();
#endif
      LoRaWAN.init(loraWanClass, loraWanRegion);
      // DR por defecto (ajusta si hace falta según región)
      LoRaWAN.setDefaultDR(3);
      break;
    }

    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();   // OTAA
      break;
    }

    case DEVICE_STATE_SEND:
    {
      prepareTxFrame(appPort);
      LoRaWAN.send();   // usa appData/appDataSize
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }

    case DEVICE_STATE_CYCLE:
    {
      txDutyCycleTime = appTxDutyCycle; // variable global de la lib
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }

    case DEVICE_STATE_SLEEP:
    {
      LoRaWAN.sleep(loraWanClass);
      break;
    }

    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}
