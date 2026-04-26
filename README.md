# FW_SAT_PAYLOAD_2024  
Embedded Payload Flight Software for TÜRKSAT Model Satellite Competition

## Overview
FW_SAT_PAYLOAD_2024 is the embedded flight software developed for the payload subsystem of our TÜRKSAT Model Satellite Competition project. The software was designed to manage mission-specific payload operations, ensure reliable subsystem communication, and execute autonomous tasks during different mission phases.

This project was developed with a strong focus on modularity, reliability, real-time behavior, and clean embedded software architecture.

## Project Objectives
- Develop reliable payload-side embedded software for model satellite missions  
- Execute autonomous mission tasks during flight phases  
- Handle sensor data acquisition and subsystem monitoring  
- Communicate with carrier satellite and ground station systems  
- Ensure deterministic and fault-tolerant software behavior  

## Key Features
- Modular embedded software architecture  
- Real-time payload task execution  
- Sensor reading and health monitoring  
- UART/I2C/SPI based peripheral communication  
- Telemetry packet generation and transmission  
- State-machine based mission logic  
- Fail-safe operational structure  
- Easy integration with external subsystems  

## Software Architecture
The firmware was developed using layered and modular design principles:

- **Application Layer**  
  Mission logic, payload task sequencing, autonomous decisions  

- **Service Layer**  
  Telemetry handling, timing services, communication management  

- **Driver Layer**  
  GPIO, UART, I2C, SPI, ADC, timers, interrupts  

- **Hardware Layer**  
  STM32 microcontroller peripherals and external devices  

## Mission Capabilities
- Autonomous payload activation after separation  
- Sensor-triggered operations  
- Timed mission events  
- Data logging and transmission  
- Health/status reporting to carrier and ground station  

## Technologies Used
- Embedded C / C++  
- STM32 Microcontrollers  
- STM32CubeIDE / HAL Libraries  
- UART / I2C / SPI Communication  
- State Machine Design  
- Real-Time Embedded Programming  
- Git Version Control  

## Engineering Practices
- Clean code principles  
- Modular reusable components  
- Version-controlled development workflow  
- Incremental testing and debugging  
- Hardware-in-the-loop validation  

## Repository Structure
```text
/Core : Drivers are kept on the files
/Drivers
/Application
/Services
/Utilities
/Docs

::contentReference[oaicite:0]{index=0}
