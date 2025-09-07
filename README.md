# PIDlab 7

A sophisticated PID control system simulator for motor control applications, featuring real-time visualization and comprehensive parameter tuning capabilities.

## Overview

PIDlab 7 is a Python-based graphical application designed for engineers, students, and robotics enthusiasts to simulate and analyze PID (Proportional-Integral-Derivative) control systems. The simulator provides accurate motor modeling with support for various motor types commonly used in robotics applications.

## Features

### Motor Support

- **Pre-configured Motors**: NEO, Kraken, and tiny brush motors with accurate parameters
- **Custom Motor Configuration**: Define your own motor parameters including inertia, resistance, torque constants, and current limits
- **Gearbox Integration**: Configurable gear ratios for mechanical advantage calculations

### Control Modes

- **Speed Control**: PID control for angular velocity regulation
- **Position Control**: PID control for precise angular positioning
- **Feedforward Control**: Velocity feedforward (kF) for improved tracking performance

### System Modes

- **Free PID Mode**: Basic PID control without external loads
- **Vertical Load-Bearing PIDF Mode**: Advanced mode with gravity compensation and load inertia modeling for applications like elevators or arm mechanisms

### Measurement Systems

- **SI Units**: Standard international units (rad/s, rad, kg, m, Nm)
- **Imperial Units**: US customary units (deg/s, deg, lb, ft, lb⋅ft)

### Advanced Features

- **Current Limiting**: Realistic motor current limiting based on back-EMF and resistance
- **Motor Inversion**: Support for inverted motor configurations
- **Multiple Target Sequences**: Define complex motion profiles with multiple setpoints over time
- **Acceptable Error Bands**: Visual indication of when the system reaches acceptable performance
- **Real-time Visualization**: Three synchronized plots showing output response, control voltage, and motor current

## Installation

### Requirements

- Python 3.7+
- tkinter (usually included with Python)
- matplotlib
- numpy

### Setup

1. Clone or download the repository
2. Install required packages:
   ```bash
   pip install matplotlib numpy
   ```
3. Run the application:
   ```bash
   python pidlab7.py
   ```

## Usage

### Basic Operation

1. **Select Motor Type**: Choose from preset motors or configure custom parameters
2. **Set PID Parameters**: Tune kp, ki, and kd values for desired performance
3. **Configure Targets**: Set target values and their activation times
4. **Choose Control Mode**: Select speed or position control
5. **Run Simulation**: The system automatically updates as you modify parameters

### Advanced Configuration

- **Gearbox**: Enter gear ratio as motor rotations per output rotation
- **Load Modeling**: In PIDF mode, specify mass and radius for gravity compensation
- **Current Limits**: Set maximum motor current for realistic simulation
- **Acceptable Error**: Define tolerance bands for performance evaluation

### Understanding the Plots

- **Top Plot**: System response (speed or position) vs. time with target lines and error bands
- **Middle Plot**: Control voltage output showing PID effort
- **Bottom Plot**: Motor current consumption for power analysis

## Motor Parameters

### Preset Motors

| Motor      | Inertia (kg⋅m²) | Resistance (Ω) | Kt (Nm/A) | Ke (V⋅s/rad) |
| ---------- | --------------- | -------------- | --------- | ------------ |
| NEO        | 1.5×10⁻⁶        | 0.02           | 0.027     | 0.027        |
| Kraken     | 6×10⁻⁶          | 0.015          | 0.035     | 0.035        |
| Tiny Brush | 5×10⁻⁷          | 2.0            | 0.03      | 0.03         |

## Applications

- **Robotics Education**: Learn PID control principles with realistic motor models
- **System Design**: Prototype and validate control systems before hardware implementation
- **Parameter Tuning**: Optimize PID gains for specific motor and load combinations
- **Performance Analysis**: Evaluate system response, power consumption, and stability

## License

MIT License - see LICENSE.txt for details.

Copyright (c) 2025 Team Resistance

## Technical Notes

The simulator uses accurate motor physics including:

- Electrical time constants based on L/R ratios
- Mechanical damping effects
- Back-EMF compensation
- Current limiting with voltage saturation
- Gravity compensation for vertical load applications
- Semi-implicit Euler integration for numerical stability

For questions or contributions, please refer to the project repository.
