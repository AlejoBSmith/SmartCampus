#include "Arduino.h"
#include "LoRaWan_APP_DeepSleep_HeltecV4.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "esp_sleep.h"

//=========== CLAVES OTAA ===========
uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06 };

//=========== DEFINICIONES QUE EXIGE LA LIBRERÍA (aunque uses OTAA) ===========
// ABP (solo para satisfacer el linker; no se usan con OTAA)
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
#define VEXT_PIN   Vext   // pin que controla el MOSFET de Vext
#define BME_CSB_PIN 45    // tu pin CSB del BME
#define VBAT_PIN    1
#define VBAT_RATIO  2.00f

TwoWire I2CBME(1);
Adafruit_BME280 bme;
volatile bool loraTxDone = false;

extern "C" void uplinkFinished(McpsConfirm_t *mcpsConfirm)
{
  // Aquí ya terminó TX + ventanas RX del uplink
  loraTxDone = true;
  // Si quieres, puedes imprimir algo rápido:
  Serial.println("Callback uplinkFinished(): TX completo.");
}

// appData y appDataSize están declarados por la librería (extern)
// deviceState, txDutyCycleTime también (¡no los declares ni con extern!)

// Empaqueta: T*100, H*100, P[hPa], Vbat[mV]  => 8 bytes
static void prepareTxFrame(uint8_t port)
{
  // Enciende Vext y quita el back-power por CSB
  pinMode(VEXT_PIN, OUTPUT);
  pinMode(BME_CSB_PIN, OUTPUT);

  digitalWrite(VEXT_PIN, HIGH);   // Vext ON (como en tu prueba)
  digitalWrite(BME_CSB_PIN, HIGH); // CSB alto -> modo I2C sin back-power raro
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

  // Apaga Vext y baja CSB para evitar back-power cuando no hay Vcc
  digitalWrite(BME_CSB_PIN, LOW);
  digitalWrite(VEXT_PIN, LOW);
}


void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  pinMode(VEXT_PIN, OUTPUT);
  pinMode(BME_CSB_PIN, OUTPUT);
  digitalWrite(BME_CSB_PIN, LOW);
  digitalWrite(VEXT_PIN, LOW);   // BME off al iniciar
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
        // Mientras el uplink sigue en proceso, dejamos que la librería haga su sueño ligero
        if (!loraTxDone) {
          LoRaWAN.sleep(loraWanClass);
          break;
        }

        // Aquí sabemos, por McpsConfirm, que el uplink terminó
        Serial.println("TX confirmado por McpsConfirm, entrando a DEEP SLEEP...");
        Serial.flush();

        // Apaga Vext y CSB para que el BME no quede back-powered
        pinMode(VEXT_PIN, OUTPUT);
        pinMode(BME_CSB_PIN, OUTPUT);
        digitalWrite(BME_CSB_PIN, LOW);
        digitalWrite(VEXT_PIN, LOW);

        // 2) Desmontar I2C y dejar SDA/SCL en alta impedancia
        I2CBME.end();                     // suelta el bus I2C
        pinMode(BME_SDA, INPUT);
        pinMode(BME_SCL, INPUT);
        // Asegúrate que no haya pull-ups internos
        digitalWrite(BME_SDA, LOW);       // en modo INPUT, esto quita el pull-up interno
        digitalWrite(BME_SCL, LOW);

        esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_SECONDS * 1000000ULL);
        esp_deep_sleep_start();
        break;  // no debería volver
      }

    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}
