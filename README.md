# Sensor Fusion Warning System

A wearable obstacle-detection system built on **STM32F103C8** (Blue Pill). The device fuses data from an ultrasonic sensor (**HC-SR04**) for wide-area scanning and a Time-of-Flight sensor (**VL53L0X**) for precise close-range measurement, then drives a buzzer to warn the user proportionally to the detected distance.

---

## Table of Contents

- [Introduction](#introduction)
- [Hardware Components](#hardware-components)
- [System Architecture](#system-architecture)
- [Installation](#installation)
- [Code Structure](#code-structure)
- [Future Improvements](#future-improvements)

---

## Introduction

Visually impaired individuals and workers in hazardous environments require a reliable, hands-free method to detect nearby obstacles. This project addresses that need by combining two complementary sensing technologies:

| Feature | HC-SR04 (Ultrasonic) | VL53L0X (ToF Laser) |
|---|---|---|
| Detection range | 2 – 400 cm | 3 – 200 cm |
| Field of view | ~30° cone | ~25° narrow beam |
| Strength | Wide coverage, robust in sunlight | High precision, works on sound-absorbing surfaces |
| Weakness | Unreliable on soft/angled surfaces | Limited range, affected by strong ambient light |

By **fusing** the outputs of both sensors, the system overcomes the individual weaknesses of each, delivering a more trustworthy distance estimate under varied real-world conditions.

---

## Hardware Components

| # | Component | Quantity | Role |
|---|---|---|---|
| 1 | STM32F103C8T6 (Blue Pill) | 1 | Main MCU — ARM Cortex-M3 @ 72 MHz |
| 2 | HC-SR04 | 1 | Ultrasonic distance sensor |
| 3 | VL53L0X | 1 | Time-of-Flight laser ranging module |
| 4 | Passive Buzzer | 1 | Audible warning output (PWM-driven) |
| 5 | ST-Link V2 | 1 | Programming / debugging probe |

### Pin Mapping

| Peripheral | STM32 Pin | Function |
|---|---|---|
| HC-SR04 TRIG | PC14 | GPIO Output — 10 µs trigger pulse |
| HC-SR04 ECHO | PA0 | TIM2_CH1 — Input Capture (rising/falling edge) |
| VL53L0X SDA | PB7 | I2C1 Data |
| VL53L0X SCL | PB6 | I2C1 Clock |
| Buzzer | PA6 | TIM3_CH1 — PWM output (1 kHz) |
| UART TX (Debug) | PA9 | USART1 TX — 115200 baud |
| UART RX (Debug) | PA10 | USART1 RX |

---

## System Architecture

The firmware follows a **bare-metal super-loop** architecture with interrupt-driven sensor acquisition.

```
┌─────────────────────────────────────────────────────────────┐
│                      HARDWARE LAYER                         │
│  HC-SR04 (Echo→PA0)    VL53L0X (I2C1)    Buzzer (PA6 PWM)   │
└──────────┬──────────────────┬───────────────────┬───────────┘
           │                  │                   │
           ▼                  ▼                   ▼
┌──────────────────┐ ┌────────────────┐ ┌─────────────────────┐
│ TIM2 Input       │ │ I2C1 Polling   │ │ TIM3 PWM            │
│ Capture ISR      │ │ (Single-shot)  │ │ (Start / Stop)      │
│ (Edge detection) │ │                │ │                     │
└────────┬─────────┘ └───────┬────────┘ └──────────┬──────────┘
         │ data_ready flag   │ tof_raw_distance     │
         ▼                   ▼                      │
┌─────────────────────────────────────┐             │
│         MAIN LOOP (100 ms tick)     │             │
│  ┌─────────────────────────────┐    │             │
│  │  sonic_Sensor_Filter()      │    │             │
│  │  (Median-of-3 from 5 samples)   │             │
│  └──────────┬──────────────────┘    │             │
│             ▼                       │             │
│  ┌─────────────────────────────┐    │             │
│  │  sensor_Fusion()            │    │             │
│  │  (Weighted decision logic)  │    │             │
│  └──────────┬──────────────────┘    │             │
│             │ system_distance_cm    │             │
│             ▼                       │             │
│  ┌─────────────────────────────┐    │             │
│  │  buzzer_Beep()              │────┼─────────────┘
│  │  (Non-blocking PWM control) │    │
│  └─────────────────────────────┘    │
└─────────────────────────────────────┘
```

### Interrupt Flow

1. **`HAL_TIM_IC_CaptureCallback()`** — Triggered by TIM2 on every rising/falling edge captured from HC-SR04 ECHO pin.
   - **Rising edge**: records `start_time`, switches polarity to falling.
   - **Falling edge**: records `end_time`, computes `distance_raw = (end_time - start_time) / 58` (cm), sets `data_ready = 1`, switches polarity back to rising.
2. **`SysTick_Handler()`** — Increments `HAL_GetTick()` every 1 ms, providing the time base for the software-scheduled tasks.
3. **`TIM2_IRQHandler()` / `TIM3_IRQHandler()`** — Vector entries that delegate to the HAL driver's generic IRQ handler.

---

## Installation

### Prerequisites

| Tool | Version | Purpose |
|---|---|---|
| [PlatformIO](https://platformio.org/) | ≥ 6.x | Build system & dependency manager |
| [VS Code](https://code.visualstudio.com/) | Latest | IDE with PlatformIO extension |
| ST-Link V2 Driver | Latest | USB driver for the programming probe |

### Build & Flash

```bash
# 1. Clone the repository
git clone https://github.com/emsinooM/Sensor-Fusion-Warning-System.git
cd Sensor-Fusion-Warning-System

# 2. Build the firmware
pio run -e genericSTM32F103C8

# 3. Flash to target via ST-Link
pio run -e genericSTM32F103C8 --target upload

# 4. Open serial monitor (115200 baud)
pio device monitor -b 115200
```

### PlatformIO Configuration Summary

```ini
[platformio]
src_dir   = Sensor_Fusion\Core\Src
include_dir = Sensor_Fusion\Core\Inc

[env:genericSTM32F103C8]
platform        = ststm32
board           = genericSTM32F103C8
framework       = stm32cube
upload_protocol = stlink
debug_tool      = stlink
monitor_speed   = 115200
```

---

## Code Structure

### Directory Layout

```
small_proj_PIO/
├── platformio.ini                  # PlatformIO build configuration
├── lib/
│   └── VL53L0X_HAL/
│       ├── VL53L0X.c              # VL53L0X driver (I2C HAL-based)
│       └── VL53L0X.h
├── Sensor_Fusion/
│   └── Core/
│       ├── Inc/
│       │   ├── main.h             # Global defines & function prototypes
│       │   ├── tim.h              # Timer handle declarations
│       │   ├── i2c.h              # I2C handle declarations
│       │   ├── usart.h            # UART handle declarations
│       │   ├── gpio.h             # GPIO init prototype
│       │   └── stm32f1xx_it.h     # ISR prototypes
│       └── Src/
│           ├── main.c             # ★ Application entry & sensor fusion logic
│           ├── tim.c              # TIM2 (Input Capture) & TIM3 (PWM) init
│           ├── i2c.c              # I2C1 peripheral init
│           ├── usart.c            # USART1 peripheral init
│           ├── gpio.c             # GPIO init (Trigger pin PC14)
│           └── stm32f1xx_it.c     # Interrupt vector handlers
└── README.md
```

### Key Functions (`main.c`)

#### 1. `HAL_TIM_IC_CaptureCallback()` — Ultrasonic Echo ISR

Interrupt callback invoked by TIM2 on each edge transition. Measures the pulse width of the HC-SR04 ECHO signal and converts it to distance in centimeters.

```c
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        if (is_first_captured == 0) {
            // Rising edge: record start timestamp
            start_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            is_first_captured = 1;
            // Switch to detect falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
                                          TIM_INPUTCHANNELPOLARITY_FALLING);
        } else {
            // Falling edge: compute distance
            end_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            distance_raw = (end_time > start_time)
                ? (end_time - start_time) / 58
                : ((65535 - start_time) + end_time) / 58;

            data_ready = 1;          // Signal main loop
            is_first_captured = 0;
            // Switch back to detect rising edge
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
                                          TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}
```

#### 2. `sonic_Sensor_Filter()` — Outlier Removal Filter

Collects 5 consecutive ultrasonic samples into a circular buffer, discards the highest and lowest values, and averages the remaining 3 to produce a stable reading.

```c
void sonic_Sensor_Filter(void) {
    if (sample_count < 5) sample_count++;

    if (sample_count == 5) {
        uint32_t sum = 0, top = 0, bot = distance_list[0];
        for (int i = 0; i < 5; i++) {
            sum += distance_list[i];
            if (distance_list[i] > top) top = distance_list[i];
            if (distance_list[i] < bot) bot = distance_list[i];
        }
        sum -= (top + bot);           // Remove outliers
        sonic_final_cm = sum / 3;     // Average remaining 3
    }
}
```

#### 3. `sensor_Fusion()` — Multi-Sensor Decision Logic

Combines the validated ultrasonic and ToF readings using a priority-based strategy:

```c
uint32_t sensor_Fusion(uint32_t sonic_cm, uint32_t tof_mm) {
    uint32_t tof_cm   = tof_mm / 10;
    uint8_t  sonic_valid = (sonic_cm > 0 && sonic_cm < 400);
    uint8_t  tof_valid   = (tof_mm  > 0 && tof_mm  < 2000);

    // Case 1: Both valid
    //   → Large discrepancy (>20 cm): pick the shorter (safety-first)
    //   → Otherwise: trust ToF (higher precision)
    // Case 2: Only ultrasonic valid → use ultrasonic
    // Case 3: Only ToF valid       → use ToF
    // Case 4: Neither valid         → return OUT_OF_RANGE (999)
}
```

#### 4. `buzzer_Beep()` — Non-Blocking Buzzer Driver

Controls the PWM buzzer without blocking the main loop. The beep pattern adapts to distance:

| Distance | Behavior |
|---|---|
| > 100 cm | Buzzer OFF |
| 31 – 100 cm | Intermittent beeping (interval proportional to distance) |
| ≤ 30 cm | Continuous beep (danger zone) |

```c
void buzzer_Beep(uint32_t now_Time) {
    if (system_distance_cm > 100) {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);   // Safe — silence
    }
    else if (system_distance_cm <= 30) {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // Danger — continuous
    }
    else {
        // Intermittent: ON for 50 ms, OFF for (distance×5 + 50) ms
        // Uses non-blocking timestamp comparison
    }
}
```

### Main Loop Summary

```c
while (1) {
    now = HAL_GetTick();

    // ── TASK 1: Read & Filter Sensors (every 100 ms) ──────────
    if (now - last_change >= 100) {
        // 1a. Process ultrasonic (if ISR set data_ready)
        // 1b. Read VL53L0X via I2C (single-shot)
        // 1c. Fuse both readings → system_distance_cm
        // 1d. Re-trigger HC-SR04 (10 µs pulse on PC14)
    }

    // ── TASK 2: Drive Buzzer (runs every iteration) ───────────
    buzzer_Beep(now);
}
```

---

## Future Improvements

- [ ] **Multi-sensor array** — Add 2–3 additional HC-SR04 sensors at different angles to increase the detection field of view.
- [ ] **Haptic feedback** — Replace or supplement the buzzer with a vibration motor for silent/discreet operation.
- [ ] **Kalman filter** — Implement a Kalman filter for sensor fusion instead of the current rule-based approach, enabling smoother and more predictive distance tracking.
- [ ] **Low-power mode** — Utilize STM32 sleep modes and duty-cycle the sensors to extend battery life for all-day wearable use.
- [ ] **Bluetooth / BLE** — Stream real-time distance data to a smartphone app for logging, visualization, or remote alerts.
- [ ] **FreeRTOS migration** — Refactor the super-loop into FreeRTOS tasks to improve scheduling determinism and allow easier addition of new features.
- [ ] **PCB design** — Design a compact custom PCB to replace the breadboard prototype, making the device truly wearable.

---

## License

This project is developed for educational purposes.
