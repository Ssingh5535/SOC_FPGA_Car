# SOC_FPGA_Car
self-driving RC car SoC leverages the PYNQ Z2’s integration of a dual-core ARM Cortex-A9 running a Linux-based vision and control stack with tightly coupled PL fabric to implement real-time image preprocessing, AXI-driven PWM servo control, and deterministic I²C and Pmod sensor interfaces. By streaming line-sensor data via UART and AXI4.

# FPGA-SoC Lane-Following RC Car

## Table of Contents
1. [Project Overview](#project-overview)  
2. [Hardware Architecture](#hardware-architecture)  
   - [PYNQ Z2 Platform](#pynq-z2-platform)  
   - [Processing System (PS)](#processing-system-ps)  
   - [Programmable Logic (PL)](#programmable-logic-pl)  
3. [Block Diagram](#block-diagram)  
4. [Design Modules](#design-modules)  
   - [OV7670_GRAYSCALE_TO_AXIS](#ov7670_grayscale_to_axis)  
   - [AXI VDMA](#axi-vdma)  
   - [AXI IIC](#axi-iic)  
   - [AXI GPIO](#axi-gpio)  
   - [Convolution Filter](#convolution-filter)  
   - [AXI2DVI](#axi2dvi)  
   - [Clock & Reset IP](#clock--reset-ip)  
5. [Software Stack](#software-stack)  
   - [Jupyter Notebook Interface](#jupyter-notebook-interface)  
   - [Camera Initialization & SCCB](#camera-initialization--sccb)  
   - [Filter Configuration & Update](#filter-configuration--update)  
   - [VDMA & Framebuffer Management](#vdma--framebuffer-management)  
   - [Lane Detection Algorithm](#lane-detection-algorithm)  
   - [Motor Control via PWM](#motor-control-via-pwm)  
6. [Power Analysis](#power-analysis)  
   - [On-Chip Power Summary](#on-chip-power-summary)  
7. [Device Layout & Resource Utilization](#device-layout--resource-utilization)  
   - [Floorplan](#floorplan)  
   - [Clocking Summary](#clocking-summary)  
8. [Getting Started](#getting-started)  
   - [Prerequisites](#prerequisites)  
   - [Building the FPGA Bitstream](#building-the-fpga-bitstream)  
   - [Deploying to PYNQ Z2](#deploying-to-pynq-z2)  
   - [Running the Application](#running-the-application)  
9. [Future Work](#future-work)  
10. [License](#license)  

---

## 1. Project Overview
This repository implements a lane-following RC car SoC on the PYNQ Z2 platform. A dual-core ARM Cortex-A9 PS runs high-level vision and control software under Linux, while the PL fabric accelerates image preprocessing (grayscale conversion, convolution), deterministic I²C/Pmod sensor interfacing, and hardware PWM for servo control. The system captures frames from an OV7670 camera, detects lane markings in real time, and issues precise steering/throttle commands to keep the car centered between lines.

## 2. Hardware Architecture

### PYNQ Z2 Platform
- **Zynq-7000 SoC** combining  
  - Dual-core ARM Cortex-A9 running PYNQ/OpenCV under Linux  
  - FPGA fabric with AXI4/AXI-Lite interconnects  

### Processing System (PS)
- Runs Python/Jupyter notebooks  
- Configures PL via device-tree overlays  
- Manages SCCB/I²C camera setup, VDMA control, and MMIO for PWM  

### Programmable Logic (PL)
- **OV7670_GRAYSCALE_TO_AXIS**: converts raw camera data to AXI4-Stream  
- **Convolution_Filter**: 7×7 hardware accelerator for user-loaded kernels  
- **AXI VDMA**: frame buffering for PS ↔ PL data transfer  
- **AXI IIC**: SCCB interface for OV7670 register control  
- **AXI GPIO**: Pmod and servo I/O  
- **AXI2DVI**: optional video output over HDMI  
- **Clock & Reset IP**: generates 48 MHz, 100 MHz domains and PS resets  

## 3. Block Diagram
![Block Diagram](/Images/block_diagram.png)

## 4. Design Modules

### OV7670_GRAYSCALE_TO_AXIS
- Interfaces camera D0–D7, VSYNC, HREF, PCLK, XCLK  
- Converts YUV/RGB input into grayscale stream on AXI4-Stream bus  

### AXI VDMA
- **Read Channel**: captures processed frames from PL to PS memory  
- **Write Channel**: returns data for display or further processing  

### AXI IIC
- SCCB master for camera register read/write  
- Implements UP/down counters, FIFOs, and dynamic-master FSM  

### AXI GPIO
- Exposes PS-controlled GPIO for Pmod sensors and status LEDs  
- Drives servo enable lines and reads limit switches  

### Convolution Filter
- 7×7 streaming convolution engine  
- MMIO registers at `0x43C1xxxx` hold 51 bytes of kernel data  
- Supports “on-fly” kernel updates via Python API  

### AXI2DVI
- Converts AXI4-Stream RGB data to TMDS signals for HDMI output  

### Clock & Reset IP
- **clk_wiz**: generates 100 MHz, 48 MHz from onboard oscillator  
- **proc_sys_reset**: handles resets for PL and PS domains  

## 5. Software Stack

### Jupyter Notebook Interface
All control and tuning code resides in `0V7670_pynq.ipynb`, providing interactive cells to:

1. Load `design_1_wrapper.xsa` overlay  
2. Instantiate MMIO, VDMA, IIC, GPIO drivers  
3. Read/write PL registers and kernels  

### Camera Initialization & SCCB
```python
from pynq import Overlay, MMIO
overlay = Overlay("design_1_wrapper.xsa")
iic = overlay.axi_iic
ov7670 = OV7670(iic)
ov7670.default_setup()
