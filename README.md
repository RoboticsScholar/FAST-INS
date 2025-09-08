# Computationally Efficient Integrated Navigation Using FAST-INS and STEKF

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

## Overview

**FAST-INS** combined with **STEKF** is a computationally efficient integrated navigation system designed for **maritime, aerial, and terrestrial platforms**. By leveraging **precise IMU preintegration** and the **State Transformation Extended Kalman Filter (STEKF)**, the system decouples the navigation output rate from high-frequency IMU sampling. This significantly **reduces computational load** while maintaining high-dynamics accuracy, making it well-suited for **real-time implementation on low-power embedded systems** such as autonomous marine, aerial, and ground vehicles.

## Key Features

- **FAST-INS Algorithm:**  
  - Decouples INS output rate from IMU sampling frequency  
  - Reduces computational cost by ~42% compared to classical Savage algorithms  
  - Maintains high-dynamics navigation accuracy  

- **Discrete Integration Framework:**  
  - Explicitly accounts for Earth rotation, Coriolis, centripetal, and gravity effects  
  - Provides accurate updates for attitude, velocity, and position  

- **STEKF Integration:**  
  - Lowers filter prediction rate without compromising accuracy  
  - Further reduces computational overhead  

- **Cross-Platform Validation:**  
  - Experiments conducted on maritime, aerial, and terrestrial datasets  
  - Supports reproducibility for research and development  

## Datasets:
The datasets used for experiments in this work, including maritime, terrestrial, and aerial datasets, can be downloaded from the following links:
- **Maritime dataset**: [https://www.educoder.net/dataset/335/detail](https://www.educoder.net/dataset/335/detail)
- **Aerial dataset**: [https://www.educoder.net/dataset/342/detail](https://www.educoder.net/dataset/342/detail)
- **Terrestrial dataset**: [https://www.educoder.net/dataset/336/detail](https://www.educoder.net/dataset/336/detail)

The most core components of the code are currently open-sourced. A fully functional implementation will be provided upon acceptance of the paper.
