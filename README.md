# 💧 Water Level Monitoring System

An IoT-based Water Level Monitoring System built using an ESP32 that automatically monitors water levels, controls a water pump, and provides real-time monitoring through MQTT, Node-RED, Blynk, and Telegram notifications.

---

## Overview

This project automates household or industrial water tank management by continuously measuring the water level using an ultrasonic sensor.

The ESP32 processes sensor readings, calculates the water level percentage, automatically controls a water pump using a relay module, publishes live data via MQTT, updates a Blynk mobile dashboard, displays the current status on an LCD, and sends Telegram alerts when the tank becomes empty or full.

---

## Features

- Real-time water level monitoring
- Automatic pump control
- Manual and Automatic operating modes
- MQTT communication
- Node-RED dashboard
- Blynk mobile application support
- Telegram notifications
- Local Web API
- LCD status display
- Sensor smoothing and filtering
- Pump protection using hysteresis
- WiFi auto reconnect

---

## Hardware

- ESP32
- HC-SR04 Ultrasonic Sensor
- 16x2 I2C LCD
- Relay Module
- Mini DC Water Pump
- External Power Supply

---

## Software Stack

- MicroPython
- MQTT
- Node-RED
- Blynk Cloud
- Telegram Bot API

---

## System Architecture

```
                 Ultrasonic Sensor
                         │
                         ▼
                     ESP32 Controller
                         │
     ┌───────────────┬───────────────┬───────────────┐
     ▼               ▼               ▼               ▼
  LCD Display      MQTT          Relay Module     Web Server
                     │                 │
                     ▼                 ▼
              Node-RED Dashboard    Water Pump
                     │
                     ▼
              Telegram & Blynk
```

---

## Automatic Pump Logic

| Water Level | Pump |
|-------------|------|
| ≤ 15% | ON |
| ≥ 75% | OFF |

Additional filtering and hysteresis are implemented to prevent rapid relay switching.

---

## Repository Structure

```
.
├── water_level.py
├── water_level_monitoring_system (Node-red).json
└── README.md
```

---

## Node-RED

Import

```
water_level_monitoring_system (Node-red).json
```

into Node-RED to visualize:

- Water Level (cm)
- Water Percentage
- Pump Status

---

## MQTT Topics

```
water/level_cm
water/percentage
pump/status
pump/control
```

---

## Dashboard

The system supports:

- Node-RED Dashboard
- Blynk Mobile Dashboard
- LCD Display

---

## Notifications

Telegram notifications are sent when:

- Water level is LOW
- Water level is FULL
- Device starts successfully

---

## Future Improvements

- Waterproof ultrasonic sensor
- Temperature monitoring
- Power saving mode
- Better calibration
- Cloud database integration

---

## Project Report

The complete project report is included in this repository.

📄 **WATER LEVEL MONITORING SYSTEM REPORT.pdf**

If viewing on GitHub, you can open it directly from the repository:

[Project Report]([./WATER%20LEVEL%20MONITORING%20SYSTEM%20REPORT.pdf](https://docs.google.com/document/d/1xT2ycIWod0kWHgZJWN_VjIGu8yS-_d0iD5NoQ3psgIY/export?format=pdf))

---

## Authors

- Keo Phlyna
- Seng Sereysolida

American University of Phnom Penh

---

## License

This project was developed for academic purposes.
