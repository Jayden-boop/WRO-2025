Table of Contents
=====
- [Introduction](#introduction)
- [Engineering Materials](#engineering-materials)
  - [1. Raspberry Pi 5](#1-raspberry-pi-5)
  - [2. Raspberry Pi 5 Expansion Board](#2-raspberry-pi-5-expansion-board)
  - [3. Micro Servo 99 SG90](#3-micro-servo-99-sg90)
  - [4. K989 1/28 WL Toys Chassis](#4-k989-128-wl-toys-chassis)
  - [5. Furitek Micro Komodo Motor](#5-furitek-micro-komodo-motor)
- [Assembly](#assembly)
- [Conclusion](#conclusion)


Engineering materials
=====

This repository contains engineering materials of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2025.


## Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

# JAT | WRO \- Documentation |

## Task

The WRO 2025 Future Engineers Self-Driving Cars Challenge tasks teams with designing an autonomous vehicle to complete dynamic racetrack laps and perform parallel parking, adapting to randomized layouts and traffic signs. Teams must also document their engineering process in a public GitHub repository, showcasing innovation, adaptability, and technical problem-solving.

### Challenge Descriptions:

#### 1\. Open Challenge

Objective: The vehicle must autonomously complete three laps on a racetrack that varies in layout for each round.

**Key Elements:**

The track includes randomized corridor widths (narrow or wide sections) and changes in starting positions.

Adaptability is critical, as the track configuration (e.g., wall placements, starting zones) is determined by a randomization process (e.g., coin tosses, dice rolls) before each round.

#### 2\. Obstacle Challenge

Objective: The vehicle must complete three laps on a track with randomly placed traffic signs and then perform a parallel parking maneuver in a designated area.

**Key Elements:**

Traffic Signs: Red and green pillars indicate required driving lanes (red \= keep right, green \= keep left). The vehicle must obey these rules without displacing the signs.

Parking Task: After completing laps, the vehicle must park within a narrow, variable-sized parking space. Starting inside the parking lot and achieving parallel alignment earns bonus points.

Randomization affects traffic sign positions, parking lot placement, and driving direction (clockwise/counterclockwise).

# 

# Introduction

## Engineering Materials

### 1\. Raspberry Pi 5

Why:  
The Raspberry Pi 5 serves as the vehicle’s "brain," providing the computational power required for autonomous navigation and real-time decision-making. Its quad-core CPU and GPIO capabilities support complex tasks like sensor fusion (e.g., camera data, ultrasonic sensors) and path-planning algorithms.

WRO Compliance:

Rule 11.8 allows Single Board Computers (SBCs), and the Pi 5’s open-source ecosystem aligns with the competition’s focus on open-source hardware.

Enables running advanced libraries (OpenCV, TensorFlow Lite) for computer vision, critical for detecting traffic signs (red/green pillars) and navigating randomized tracks.

### 2\. Raspberry Pi 5 Expansion Board

Why:  
The expansion board simplifies wiring, ensuring stable power distribution and modular sensor/motor integration. This reduces electrical noise and debugging time during high-pressure rounds.

WRO Compliance:

Supports adherence to Rule 11.3 (drive system requirements) by cleanly connecting steering servos and drive motors.

Facilitates compliance with Rule 12.6 (vehicle checks) by ensuring a tidy, inspection-ready build.

### 3\. Micro Servo 99 SG90

Why:  
The SG90 is a lightweight, precise servo for steering control. Its 180° range and torque (1.8 kg/cm) allow accurate turns while adhering to the vehicle’s size/weight limits (Rule 11.1: ≤300x200mm, ≤1.5kg).

WRO Compliance:

Meets Rule 11.3 (steering actuator requirement) and supports Obstacle Challenge tasks (e.g., parallel parking, avoiding traffic signs).

Cost-effective and easy to integrate, freeing resources for other components.

### 4\. K989 1/28 WL Toys Chassis

Why:  
This modular RC chassis provides a pre-built 4-wheeled platform with a differential drive system. While Rule 11.3 prohibits differential wheeled robots, the chassis can be modified to use a single steering actuator (SG90) and compliant drive system (Furitek motor \+ gearbox).

WRO Compliance:

Saves time on drivetrain design, allowing focus on software and sensor integration.

Easily customizable for mounting sensors (e.g., cameras, ultrasonic) and adhering to Rule 11.17 (wired communication only).

### 5\. Furitek Micro Komodo Motor

Why:  
This brushless motor delivers high torque and smooth speed control in a compact package. It powers the drive axle via a gear system, ensuring efficient navigation of variable track widths (600–1000mm) and slopes.

WRO Compliance:

Complies with Rule 11.13 (≤2 driving motors connected via gears).

Enhances reliability for Time Attack laps and parking maneuvers (Obstacle Challenge).

Synergy and Competition-Specific Advantages  
Autonomy and Precision:

The Raspberry Pi 5 processes sensor data (camera, ultrasonic) to navigate randomized tracks and obey traffic signs. The SG90 and Furitek motor enable precise steering and speed adjustments.

GitHub documentation (Rule 7\) can showcase code for adaptive algorithms, sensor calibration, and parking logic.

Size and Weight Compliance:

All components fit within the 300x200mm footprint and 1.5kg weight limit (Rule 11.1).

Scoring Optimization:

The K989 chassis’ agility helps maximize points for lap completion and parking accuracy (Appendix A.6).

The Furitek motor’s efficiency reduces the risk of overheating during repairs (Rule 9.18).

## Assembly

## Conclusion


