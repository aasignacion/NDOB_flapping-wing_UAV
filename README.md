# NDOB_flapping-wing_UAV
NDOB-Based Robust Control for Flapping-Wing Aerial Manipulators

A reference implementation of the parallel nonlinear disturbance observer (NDOB)–based robust controller for Flapper Nimble+ flapping-wing aerial manipulators (FW-AEROMs). This repository accompanies the paper:

> **“A Parallel Nonlinear Disturbance Observer–Based Robust Controller for Flapping-Wing Aerial Manipulators”**  
> *[Your Name], [Coauthors], ICRA 2025*

## Overview

Flapping-wing aerial manipulators offer high efficiency and compliance but suffer from severe modeling uncertainties, low-Reynolds aerodynamics, and strong dynamic coupling. This project implements and validates an NDOB-augmented controller that:

- Independently compensates disturbances in altitude and pitch channels  
- Leverages the vendor’s default PID loops for simplicity  
- Guarantees global exponential convergence of disturbance estimates  
- Demonstrates significant RMSE and STD reductions under wind gusts, payload changes, and manipulation tasks  

## Usage

1. **Clone Crazyflie Firmware**  
   ```bash
   git clone https://github.com/bitcraze/crazyflie-firmware.git
   cd crazyflie-firmware

2. **Copy the NDOB-PID Controller**
Copy your modified controller_pid.c (which wraps the stock PID loops with the NDOB logic) into the firmware’s controller module:

  cp path/to/ndob-controller/controller_pid.c src/modules/src/controller/

 3. Build and flash to the  Flapper Nimble+: https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/

