# NeuroSole – Embedded Firmware for Closed-Loop TENS System

**Author:** Jonas Schröder  
**Affiliations:** ETH Zurich – Neural Engineering Lab · TUM  
**Master Thesis Project, 2024**

---

## Overview

**NeuroSole** is a closed-loop **wearable neuromodulation system** for the non-invasive treatment of **diabetic peripheral neuropathy (DPN)**. It delivers synchronized **transcutaneous electrical nerve stimulation (TENS)** based on **gait phase detection** and real-time pressure feedback.

This repository contains all firmware components, configuration files, and scripts used in the development of the NeuroSole system, implemented and tested as part of my Master's thesis using **Simplicity Studio 5** and the **Gecko SDK 4.4.0**.

---

## 🧠 System Architecture

- **Target MCU:** BGM240PB32VNA (ARM Cortex-M33 with BLE 5.4)
- **SDK:** Gecko SDK Suite 4.4.0
- **Operating System:** Micrium OS Kernel (optional)
- **Development Platform:** Simplicity Studio 5

The firmware integrates:

- **Bluetooth stack with GATT services**
- **Bootloader OTA support** (`sl_bt_in_place_ota_dfu`)
- **IADC-based multi-channel voltage acquisition** for sensor signals
- **Custom stimulation triggering** based on gait phase
- **Real-time BLE communication and GATT updates**

---

## 🧪 Hardware Integration

**Connected sensors and modules:**

- **Tekscan A301 pressure sensors** (x6) via analog frontend
- **LSM6DSL IMU** for gait phase classification
- **SmartStim stimulator** controlled via GPIO
- **3.7 V LiPo** with custom power management
- **PCB Version:** NeuroSole 2.0 – 4-layer EMI-optimized board

**Peripheral Drivers:**

- GPIO, TIMERx, LETIMER, IADC0, USART0
- BLE (GATT server & system ID via `sl_gatt_service_device_information.c`)
- ADC scan via `app.c`, interrupt-driven data processing

---

## 🔧 Build & Flash Instructions

### 1. **Open Project**
- Import the `.slcp` project in **Simplicity Studio 5**
- Ensure Gecko SDK v4.4.0 is installed

### 2. **Compile**
- Use standard "Build Project" button
- Target board: `BGM240PB32VNA`

### 3. **Flash Firmware**
- Use onboard debugger or J-Link
- To generate DFU/GBL upgrade files:
  ```bash
  ./scripts/create_bl_files.sh
