# C3mini Jammer Basic - ESP32-C3 Universal 2.4GHz Jammer

<div align="center">
  
  ![Project Version](https://img.shields.io/badge/Version-1.0.0-blue)
  ![License](https://img.shields.io/badge/License-MIT-green)
  ![ESP32-C3](https://img.shields.io/badge/ESP32--C3-SuperMini-red)
  ![NRF24L01](https://img.shields.io/badge/2x-NRF24L01-orange)
  
  **Universal 2.4GHz Spectrum Jammer for ESP32-C3 with dual NRF24L01 modules**
  
  [Features](#-features) • [Hardware](#-hardware) • [Installation](#-installation) • [Usage](#-usage) • [Modes](#-modes) • [Logging](#-logging)

</div>

## 📋 Project Overview

**C3mini Jammer Basic** is a powerful 2.4GHz spectrum jammer designed for the ESP32-C3 SuperMini microcontroller with dual NRF24L01 radio modules. This project can jam Bluetooth, WiFi, and the entire 2.4GHz ISM band with configurable modes and timing.

**Author:** NhanMinhz 🇻🇳  
**Discord:** nhan2k91221  
**License:** MIT  
**Version:** 1.0.0

## ✨ Features

- **4 Operating Modes:**
  - 📴 Standby (Power saving)
  - 📱 Bluetooth (80 channels, 2402-2480MHz)
  - 📶 WiFi (14 channels, 2412-2484MHz)
  - 🌐 Full Spectrum (126 channels, 2400-2525MHz)

- **Smart Control:**
  - 120-second auto timeout per mode
  - BOOT button control (short press = next mode, long press = reset)
  - Auto module detection with retry mechanism
  - Status LED with mode indication

- **Performance Optimized:**
  - 250µs delay between packets for maximum throughput
  - Dual module load balancing
  - Channel caching to reduce SPI overhead
  - Error recovery system

## 🛠️ Hardware Requirements

### Required Components:
1. **ESP32-C3 SuperMini**
2. **2x NRF24L01+ modules** (E01-2G4M27D or similar)
3. **Power supply**: 3.3v stable source (≥500mA for both modules)
4. **Breadboard and jumper wires**
5. **USB cable** for programming

### Pin Connections:
| ESP32-C3 Pin | NRF24 Module 1 | NRF24 Module 2 | Function |
|-------------|----------------|----------------|----------|
| GPIO20 | CE | - | Chip Enable 1 |
| GPIO21 | CSN | - | Chip Select 1 |
| GPIO0 | - | CE | Chip Enable 2 |
| GPIO1 | - | CSN | Chip Select 2 |
| GPIO6 | SCK | SCK | SPI Clock |
| GPIO5 | MISO | MISO | SPI MISO |
| GPIO7 | MOSI | MOSI | SPI MOSI |
| 3.3V | VCC | VCC | Power (3.3V) |
| GND | GND | GND | Ground |

**Additional connections:**
- GPIO8 → LED (with 220Ω resistor to GND)
- GPIO9 → BOOT button (to GND when pressed)

## 🔧 Installation

### 1. Software Requirements
- **Arduino IDE** (≥2.0) or **PlatformIO**
- **ESP32 Arduino Core** (≥3.0.0)
- **RF24 Library** by TMRh20 (`TMRh20/RF24`)

### 2. Uploading the Code
```cpp
1. Connect ESP32-C3 via USB
2. Select board: "ESP32-C3 Dev Module"
3. Set Flash Mode: "QIO"
4. Set Flash Size: "4MB"
5. Set CPU Frequency: "160MHz"
6. Set Upload Speed: "921600"
7. Upload the code
