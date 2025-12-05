# Heltec WiFi LoRa 32 V4 – BME280 + LoRaWAN + Deep Sleep

This project uses a Heltec WiFi LoRa 32 V4 and a BME280 to send temperature, humidity, pressure and battery voltage over LoRaWAN (OTAA).  
The node enters **ESP32 deep sleep** only after the LoRaMAC stack has finished the uplink, and powers the BME280 down between measurements.

---

## Hardware

- Heltec WiFi LoRa 32 V4 (ESP32 + SX1262)
- BME280 breakout (I²C)
- External battery / USB power

---

## Main idea

- On wake-up:
  - Join via OTAA (if needed).
  - Power up the BME280, take a single forced measurement.
  - Read battery voltage through the ADC.
  - Send the packed payload via LoRaWAN.
- Only after the MAC confirms that the uplink finished, the ESP32:
  - Powers down the BME280.
  - Releases I²C pins.
  - Enters deep sleep for a fixed interval.

---

## Issues found

### 1. Deep sleep happening at the wrong moment

**Problem**

- Calling `LoRaWAN.sleep()` only triggers the internal low power mode of the Heltec LoRaWAN library, not real ESP32 deep sleep.
- Calling `esp_deep_sleep_start()` directly from the sketch could happen **before** the LoRaMAC stack had fully completed the uplink (TX + RX windows).
- The message `unconfirmed uplink sending ...` is printed in the library *before* the actual `LoRaMacMcpsRequest()` call, so it does not mean “TX complete”.

**What was changed**

- A new callback was added inside the Heltec `LoRaWan_APP` library:
  - A weak function called when `McpsConfirm` runs, i.e. when the MAC layer has finished an uplink.
- The sketch overrides this callback and sets a flag indicating “uplink finished”.
- The main state machine checks this flag in `DEVICE_STATE_SLEEP`:
  - While the uplink is still in progress, it calls the library’s sleep (`LoRaWAN.sleep(...)`).
  - Once the callback has fired, it performs cleanup and then calls `esp_deep_sleep_start()`.

**Effect**

- Deep sleep is entered only after the LoRaMAC stack signals that the uplink is complete.
- No more “falling asleep” mid-transmission or mid-RX window.

---

### 2. BME280 never really turning off

**Problems**

- At first, the BME280 was powered from the 3.3 V rail, so it was always on.
- Later attempts to power it from a GPIO or Vext led to:
  - The sensor still seeing voltage even when Vext/GPIO was “off”.
  - VCC sitting around ~2.2 V with the sensor connected, instead of going to 0 V.
- This happened because:
  - `CSB` was tied to 3.3 V to force I²C mode, which back-powered the chip when VCC was cut.
  - I²C pull-ups and internal diodes also provided a path in previous tests.

**What was changed**

- The BME280 is now powered from the rail controlled by the Heltec Vext MOSFET.
- Two lines are explicitly controlled from the sketch:
  - Vext control pin (through the symbol provided by the Heltec core).
  - The BME280 `CSB` pin.
- Power sequence:
  - Before reading:
    - Vext is turned **on**.
    - `CSB` is driven **high** to select I²C mode without back-power issues.
    - The I²C bus is started and a forced measurement is taken.
  - After reading:
    - `CSB` is driven **low** to avoid any back-power path from that pin.
    - Vext is turned **off** to cut power to the module.
- Just before entering deep sleep:
  - The I²C instance used for the BME is closed.
  - SDA and SCL pins are set as inputs with no internal pull-ups.

**Effect**

- The BME280 is no longer held at a mid-level voltage (2.2 V) by CSB or I²C lines.
- A small residual reading around ~0.7 V at the sensor VCC is expected due to leakage and meter impedance, but there is no solid power path.
- Sensor current between measurements is effectively minimized, and the ESP32 can stay in deep sleep without the BME staying “half alive”.

---

## Library modifications

All changes are inside the Heltec `LoRaWan_APP` library used by the Heltec board package.

- New weak callback declaration and definition for “uplink finished”.
- Single call to this callback at the end of `McpsConfirm`.

The sketch:

- Implements its own version of this callback and sets a flag.

