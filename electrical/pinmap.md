# Pin map: ESP32 DevKit V1 (classic ESP32, WROOM-32)

Status: draft v0.1. All 22 signals fit, with **zero spare pins**. Check the silkscreen on your exact board before wiring, since DevKit V1 clones differ slightly.

## Motor outputs (12), DRV8833 inputs

Each DRV8833 serves one module: channel A = steer, channel B = drive.

| Module | Driver pin | Function | GPIO |
|---|---|---|---|
| 0 | AIN1 / AIN2 | steer | 13 / 16 |
| 0 | BIN1 / BIN2 | drive | 17 / 18 |
| 1 | AIN1 / AIN2 | steer | 19 / 23 |
| 1 | BIN1 / BIN2 | drive | 25 / 26 |
| 2 | AIN1 / AIN2 | steer | 27 / 32 |
| 2 | BIN1 / BIN2 | drive | 33 / 2 |

- Drive each pair with LEDC at about 20 kHz (12 of the 16 channels). Forward: IN1 = PWM, IN2 = 0. Reverse: swapped.
- DRV8833 inputs have internal pulldowns, so motors stay off while the ESP32 boots and its pins float.
- GPIO2 is a strapping pin, but it is only a problem if something drives it high during flashing. The DRV8833 input only pulls it low, so it is safe as an output here.
- Tie each driver's nSLEEP high (most breakouts already do) and ISEN pins to GND if you don't use current sensing.

## Drive encoder inputs (6), PCNT hardware counters

| Module | A | B |
|---|---|---|
| 0 | 34 | 35 |
| 1 | 36 | 14 |
| 2 | 5 | 4 |

- 34, 35, 36 are input-only pins with no internal pull-ups. That is fine for push-pull Hall encoders.
- **GPIO14 and GPIO5 output a short PWM burst at boot**, so put a **1 kΩ series resistor** on each of those encoder lines.

## Everything else

| Signal | GPIO | Notes |
|---|---|---|
| I²C SDA | 21 | 400 kHz, 4.7 kΩ pull-ups to 3.3 V (check whether the mux board already has them) |
| I²C SCL | 22 | |
| Lidar RX (UART2) | 15 | Receive only. Strapping pin that defaults high, so a lidar that idles high is fine. 1 kΩ series resistor. |
| Battery sense (ADC1) | 39 | Divider 100 kΩ / 47 kΩ (8.4 V -> ~2.7 V), 0.1 µF at the pin. Use ADC1 only, because ADC2 doesn't work while WiFi is on. Oversample and calibrate. |

## Pins you must not use

- **6–11:** connected to the module's flash.
- **12:** boot voltage strapping pin. If it is high at reset, the board may fail to boot.
- **0:** BOOT button. It must be high at reset for a normal start.
- **1, 3:** UART0, used for USB programming and logging. Keep them free.

## I²C bus layout

```
ESP32 SDA/SCL --+-- navX2 (0x32, verify)
                +-- TCA9548A (0x70)
                       ch0 -> AS5600 module 0 (0x36)
                       ch1 -> AS5600 module 1 (0x36)
                       ch2 -> AS5600 module 2 (0x36)
```

Estimated I²C time for three AS5600 reads through the mux at 400 kHz is about 0.8 ms per cycle. That fits a 500 Hz steering loop but not by a wide margin. Read the navX at a slower rate (50–100 Hz). Both figures are estimates to confirm on the bench.

## Power wiring

- The 5 V BEC feeds the DevKit's VIN pin, and also the lidar and navX.
- **Flash with the battery off.** If USB and the BEC both feed the 5 V rail, the BEC can back-feed your computer's USB port. A Schottky diode in series with the BEC output prevents this.
- The DevKit's onboard 3.3 V regulator (an AMS1117, ~800 mA) feeds the AS5600s, the mux, the encoders and the pull-ups. Expected total is about 0.4–0.6 A with WiFi, so it will run warm.
