# SmartKnob — Haptic Embedded HMI

A reproduction of [Scott Bez's SmartKnob](https://github.com/scottbez1/smartknob) — a brushless motor-driven haptic input device that simulates configurable detents and torque feedback as a human-machine interface.

> **Goal of this reproduction:** Deeply understand Field-Oriented Control (FOC), PID tuning, and embedded IoT integration by building the project from scratch rather than just flashing a binary.

---

## Features

- Configurable haptic detents via **Field-Oriented Control (FOC)**
- **PID control loop** for precise torque and position regulation
- **Kalman filter** on magnetic angle encoder — suppresses noise while preserving positional accuracy through careful gain tuning
- **Enable-pin duty-cycle control** to prevent coil overheating during idle
- **MQTT integration** implemented at the firmware level; full end-to-end testing pending network environment constraints (see notes below)
- Built and debugged entirely on ESP32 using VSCode + PlatformIO

---

## Hardware

| Component | Description |
|-----------|-------------|
| MCU | ESP32 |
| Motor | Brushless DC (BLDC) |
| Encoder | Magnetic angle encoder (AS5600 or equivalent) |
| Driver | BLDC motor driver |
| Interface | MQTT over Wi-Fi (partial — see notes) |

---

## Software Stack

| Layer | Technology |
|-------|-----------|
| Language | C |
| IDE | VSCode + PlatformIO |
| Control | Field-Oriented Control (FOC), PID |
| Filtering | Kalman Filter |
| Communication | MQTT (firmware implemented) |

---

## Key Engineering Challenges

### 1. Magnetic encoder noise
The AS5600 magnetic angle encoder introduced angular noise that caused jitter in the haptic feedback. A **Kalman filter** was applied to smooth the signal. The filter gain was carefully tuned — too aggressive and positional accuracy degrades; too light and noise passes through.

### 2. Coil overheating
Keeping the motor coils continuously energized caused significant heat buildup during idle states. This was resolved by implementing **enable-pin duty-cycle control**, cutting power to the coils when no active torque is needed, substantially reducing thermal load.

### 3. FOC tuning
Field-Oriented Control requires accurate real-time knowledge of the rotor's magnetic field angle. Any lag or error in the angle estimate directly degrades torque quality. Tuning the PID parameters alongside the Kalman filter gain required iterative testing across different detent configurations.

---

## MQTT — Implementation Notes

MQTT publish/subscribe was implemented at the firmware level on the ESP32 for wireless real-time parameter tuning. However, **end-to-end testing was limited** due to network environment constraints at the development location (shared rental housing with restricted local network access).

The MQTT client code is included in the repository. To enable full functionality:

```c
// src/mqtt.c — configure before building
#define MQTT_BROKER   "YOUR_BROKER_IP"   // e.g. a local Mosquitto instance
#define MQTT_TOPIC    "smartknob/params"
```

A local MQTT broker (e.g. [Mosquitto](https://mosquitto.org/)) on the same subnet is required. Full integration testing is planned once a stable local network environment is available.

---

## Project Structure

```
smartknob/
├── src/
│   ├── main.c          # Entry point, MQTT init, main loop
│   ├── foc.c / foc.h   # Field-Oriented Control implementation
│   ├── pid.c / pid.h   # PID controller
│   ├── kalman.c        # Kalman filter for encoder angle
│   └── mqtt.c          # MQTT client (parameter sync)
├── include/
├── platformio.ini
└── README.md
```

> *(Adjust paths to match your actual file structure)*

---

## Setup & Build

### Prerequisites
- [VSCode](https://code.visualstudio.com/) + [PlatformIO extension](https://platformio.org/)
- ESP32 board support installed via PlatformIO

### Build & Flash

```bash
# Clone the repo
git clone https://github.com/blubana/smartknob-haptic-hmi.git
cd smartknob-haptic-hmi

# Open in VSCode, then build and upload via PlatformIO toolbar
# Or via CLI:
pio run --target upload
```

---

## What I Learned

- How **Field-Oriented Control** decomposes motor current into torque-producing and flux-producing components
- Trade-offs in **Kalman filter gain tuning** — noise rejection vs. latency vs. accuracy
- Practical **thermal management** in embedded motor systems
- **MQTT** publish/subscribe architecture for real-time embedded parameter control; encountered real-world network constraints that highlighted the importance of testbed environment planning

---

## Reference

- Original project: [scottbez1/smartknob](https://github.com/scottbez1/smartknob)
- SimpleFOC library (reference): [simplefoc.com](https://simplefoc.com)

---

## Author

**Ting-Hsu Chen (陳廷緒)**
B.S. Electrical Engineering, NKUST
[github.com/blubana](https://github.com/blubana) · [cruisedottw@gmail.com](mailto:cruisedottw@gmail.com)
