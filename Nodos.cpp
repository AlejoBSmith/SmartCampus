#include "Arduino.h"
#include "LoRaWan_APP.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "esp_sleep.h"

//=========== CLAVES OTAA (REEMPLAZA POR LAS REALES) ===========
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x53, 0xC0 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x91, 0x3F, 0xA7, 0x2B, 0x54, 0xCC, 0x89, 0x10,
                     0xEF, 0xD2, 0x73, 0x41, 0x65, 0x9B, 0x20, 0x8E };

//=========== DEFINICIONES QUE EXIGE LA LIBRERÍA ===========
uint8_t nwkSKey[] = { 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
                      0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F };
uint8_t appSKey[] = { 0x0F,0x0E,0x0D,0x0C,0x0B,0x0A,0x09,0x08,
                      0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00 };
uint32_t devAddr   = 0x26011111;

uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };
uint8_t confirmedNbTrials = 4;

//=========== PARÁMETROS LoRaWAN ===========
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;   
DeviceClass_t   loraWanClass  = CLASS_A;
bool overTheAirActivation     = true;            
bool loraWanAdr               = true;
bool isTxConfirmed            = false;           
uint8_t appPort               = 2;
uint32_t appTxDutyCycle       = 60000;           

//=========== HARDWARE ===========
#define SLEEP_SECONDS  60

// BME280 en I2C1 (Heltec V2 usa GPIO4 y GPIO15)
#define BME_SDA   4
#define BME_SCL   15
#define I2C_FREQ  400000

// Control de alimentación externa
#define VEXT_CTRL  21   // LOW=ON, HIGH=OFF en V2

TwoWire I2CBME(1);
Adafruit_BME280 bme;

//=========== FRAME A ENVIAR ===========
// Empaqueta: T*100, H*100, P[hPa]  => 6 bytes (sin batería)
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

  // Empaque binario
  int16_t t = (int16_t)(T * 100);
  int16_t h = (int16_t)(H * 100);
  int16_t p = (int16_t)(P_hPa);

  appDataSize = 6;
  appData[0] = (uint8_t)(t >> 8); appData[1] = (uint8_t)(t & 0xFF);
  appData[2] = (uint8_t)(h >> 8); appData[3] = (uint8_t)(h & 0xFF);
  appData[4] = (uint8_t)(p >> 8); appData[5] = (uint8_t)(p & 0xFF);

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
      LoRaWAN.setDefaultDR(3);
      break;
    }

    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();   
      break;
    }

    case DEVICE_STATE_SEND:
    {
      prepareTxFrame(appPort);
      LoRaWAN.send();   
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }

    case DEVICE_STATE_CYCLE:
    {
      txDutyCycleTime = appTxDutyCycle; 
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
