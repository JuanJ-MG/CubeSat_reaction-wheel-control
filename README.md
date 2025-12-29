# CubeSat Single-Axis Reaction Wheel Control (ESP32)

This project implements a **single-axis attitude control system for a CubeSat prototype** using a **reaction wheel**, built with **commercial off-the-shelf (COTS) components**.

An **ESP32** serves as the onboard computer, reading inertial data from an **MPU6050 IMU** and controlling a DC motor–based reaction wheel through a feedback controller. A **web-based ground interface** allows real-time monitoring and command input over Wi-Fi, including a live 3D visualization of the CubeSat’s attitude.

### Features
- Single-axis attitude control using a reaction wheel  
- Real-time telemetry via Wi-Fi  
- Web-based ground station (HTML / CSS / JavaScript)  
- Low-cost CubeSat ADCS prototyping with 3D-printed structure  

### Context
This project was developed and experimentally validated as an **academic CubeSat ADCS prototype**, demonstrating that reliable attitude control and ground interaction can be achieved using accessible hardware and open-source tools.
