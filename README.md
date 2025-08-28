## Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

Table of Contents
=====

- [Introduction](#introduction)
- [The Team](#the-team)
- [Task](##Task)
  - [Obstacle Challenge](#OpenChallenge)
  - [Obstacle Challenge](#ObstacleChallenge)
- [Engineering Materials](#engineering-materials)
  - [1. Raspberry Pi 5](#1-raspberry-pi-5)
  - [2. Raspberry Pi 5 Expansion Board](#2-raspberry-pi-5-expansion-board)
  - [3. Micro Servo 99 SG90](#3-micro-servo-99-sg90)
  - [4. K989 1/28 WL Toys Chassis](#4-k989-128-wl-toys-chassis)
  - [5. Furitek Micro Komodo Motor](#5-furitek-micro-komodo-motor)
  - [Cost Report](https://github.com/Jayden-boop/WRO-2025/blob/main/README.md#cost-report)
- [Assembly](#assembly)
- [Conclusion](#conclusion)



# The Team

### Jayden
Highschool: Anderson CVI

Description: WAAWAAW
#
### Arham
Highschool: Anderson CVI

Description: Syed Arham Wasti is a Junior at Anderson CVI involved in computer science, machine learnin. His ambition drives him to use technology to impact issues he cares aboutWAWAAS
#
### Trevor 
Highschool: Anderson CVI

Description: WA WA WA
#


# JAT | WRO \- Documentation |

## Task

The WRO 2025 Future Engineers Self-Driving Cars Challenge tasks teams with designing an autonomous vehicle to complete dynamic racetrack laps and perform parallel parking, adapting to randomized layouts and traffic signs. Teams must also document their engineering process in a public GitHub repository, showcasing innovation, adaptability, and technical problem-solving.

### Challenge Descriptions:

#### 1\. Open Challenge

Objective: The vehicle must autonomously complete three laps on a racetrack whose layout randomizes each round.

**Key Elements:**

- **Randomized Layouts:** The track may feature randomized corridor widths (narrow or wide sections) and varying starting positions.
  Dynamic configurations:  The track configuration (e.g., wall placements, starting zones) is determined by a randomization process (e.g., coin tosses, dice rolls) before each round.

- **Adaptability requirement:** The navigation algorithms of the vehicle are critical, having to adapt in real time to an unseen wall and course arrangement without human intervention.

**Performance Goal:**
Maximize the consistency and speed of laps under unknown track conditions.

#### 2\. Obstacle Challenge

Objective: The vehicle must complete three laps on a track with randomly placed traffic signs and then perform a parallel parking maneuver in a designated area.

**Key Elements:**

- **Traffic Signs:** Red and green pillars indicate required driving lanes (red = keep right, green = keep left). The vehicle must obey these rules without displacing the signs.
- **Parking Task:** After completing laps, the vehicle must park within a narrow, variable-sized parking space. Starting inside the parking lot and achieving parallel alignment earns bonus points.

**Randomization factors:**

- Traffic sign positions
- Parking lot placement
- Driving direction (clockwise/counterclockwise)

**Performance Goal:**
Complete the 3 laps without faults and complete a precise parallel parking maneuver whilst adapting to randomized factors.

# 

Our Robot 
=====
ADD PHOTOS

ADD VIDEO EMBED


Engineering Materials
=====

## Overview

This section details the specific components and materials selected for our WRO Future Engineers autonomous vehicle, along with the engineering rationale behind each choice.

## Computing and Control Platform

###  Raspberry Pi 5

<table>
  <tr>
    <td>
      <img src="other/readme-images/Raspberry-Pi-5.png" width="100000">
    </td>
    <td style="width: 30px;">
The Raspberry Pi 5 serves as the main computing unit for our robot. I selected this platform for several key reasons: its robust processing power handles real-time computer vision and sensor fusion simultaneously, the extensive GPIO interface provides direct control over all actuators and sensors, and the mature Python ecosystem allows for rapid development of autonomous driving algorithms. The Pi 5's improved performance over previous generations ensures smooth operation of multiple concurrent processes including LIDAR data processing, camera image analysis, and motor control loops. 
    </td>
  </tr>
</table>

### Hiwonder Raspberry Pi 5 Expansion Board

<table>
  <tr>
    <td style="width: 300px;">
To streamline connections and improve reliability, the Raspberry Pi 5 Expansion Board  connects and provides power to the Raspberry Pi. This board provides organized PWM outputs for servo control, regulated power distribution to sensors, and proper signal conditioning for the LIDAR interface. The expansion board design eliminates breadboard connections that could fail during competition runs.
    </td>
    <td>
      <img src="other/readme-images/rrclitecontroller.jpg" width="1250">
    </td>
  </tr>
</table>


## Sensors and Perception

### Pi Camera Module

<table>
  <tr>
    <td>
      <img src="other/readme-images/Diyeeni-Zero-Module-Camera.jpg" width="1000">
    </td>
    <td style="width: 30px;">
The Pi Camera provides the primary vision system for line detection and traffic sign recognition. Its direct CSI interface to the Raspberry Pi ensures low latency image capture essential for real-time autonomous navigation. The compact form factor allows optimal mounting height for field of view while maintaining the robot's low center of gravity.
    </td>
  </tr>
</table


![Camera Holder Drawing](other/readme-images/camera-holder-drawing.jpg)


### LD19 D500 LIDAR

<table>
  <tr>
    <td>
      <img src="other/readme-images/LD19-D500-LIDAR.jpg" width="10000">
    </td>
    <td style="width: 30px;">
This LIDAR unit delivers 360-degree environmental mapping crucial for obstacle detection and wall following. This model was selected for its excellent price-to-performance ratio, reliable UART communication interface, and compact size that fits well within the robot's footprint. The sensor's range and accuracy specifications perfectly match the WRO competition environment requirements.
    </td>
  </tr>
</table>

## Structural Materials

### Custom 3D Printed Chassis (PLA)

The main chassis and structural components are 3D printed in PLA plastic. PLA was selected for its excellent dimensional stability, ease of printing complex geometries, and sufficient strength for the application. The material allows for rapid prototyping iterations during development while providing the precision needed for proper component mounting and alignment.

![Platform Drawing](other/readme-images/platform-drawing.jpg) 

Key printed components include:

- Main chassis baseplate with integrated mounting features
- Electronics platform with built-in component positioning
- Camera mount tower optimized for field of view
- Custom steering linkages designed for precise geometry
- Front wheel rims sized specifically for our tire selection

## Power System

### Gens Ace 2S1P 1300mAh 7.4V battery
<table>
  <tr>
    <td width="50%" style="text-align: left;">
     <img src="other/readme-images/Gens-Ace-2S1P-1300mAh-7.4V-battery.jpg" alt="Gens Ace 2S1P 1300mAh 7.4V battery" width="350"/>
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <ul>
         <li>Capacity: 1300mAh</li>
        <li>Voltage: 7.4V (2S1P)</li>
        <li>Discharge Rate: 45C</li>
        <li>Connector Type: Deans</li>
        <li>Balancer Connector Type: G-tech-3P</li>
        <li>Dimensions (L × W × H): 70.87 × 35.24 × 14.5 mm</li>
        <li>Weight: 90 g (±20 g)</li>
      </ul>
    </td>
  </tr>
</table>



A lithium polymer battery provides reliable power for extended operation periods. The specific capacity and voltage were selected to balance runtime requirements with size constraints.

## Drive and Actuation Components

### HS-5055MG Servo

The HS-5055MG servo was chosen for its precise position control and sufficient torque to overcome tire friction during sharp turns.

<table>
  <tr>
    <td width="50%" style="text-align: left;">
     <img src="other/readme-images/HS-5055MG-Servo.jpg" alt="HS-5055MG Servo" width="350"/>
    </td>
    <td width="50%" style="text-align: left; vertical-align: top; padding: 0;">
      <h3>Specifications:</h3>
      <ul>
        <li>KV: 3450 RPM/Volt</li>
        <li>No-load current @ 10V: 0.7A</li>
        <li>Power: 120W</li>
        <li>Battery: 2–3S LiPo</li>
        <li>Resistance: 0.16 Ω</li>
        <li>Max Current: 10A</li>
        <li>Motor Size: 15.5 × 20.6 mm</li>
        <li>Shaft Size: 1.5 × 6 mm</li>
        <li>Weight: 17.5 g</li>
      </ul>
    </td>
  </tr>
</table>

## Furitek Micro Komodo Motor

This brushless motor controller was chosen for its precise speed control capabilities and compact form factor that fits within our chassis constraints.

<table>
  <tr>
    <td width="60%" style="text-align: left;">
     <img src="other/readme-images/Furitek-Micro-Komodo-Motor.png" alt="Furitek Micro Komodo Motor" width="500"/>
    </td>
    <td width="40%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <ul>
        <li>KV: 3450 RPM/Volt</li>
        <li>No-load current @ 10V: 0.7A</li>
        <li>Power: 120W</li>
        <li>Battery: 2–3S LiPo</li>
        <li>Resistance: 0.16 Ω</li>
        <li>Max Current: 10A</li>
        <li>Motor Size: 15.5 × 20.6 mm</li>
        <li>Shaft Size: 1.5 × 6 mm</li>
        <li>Weight: 17.5 g</li>
      </ul>
    </td>
  </tr>
</table>

### FURITEK LIZARD Pro ESC

The ESC provides the interface between our control system and the drive motor, selected for its low weight and small size. 

## Mechanical Components

### K989 Drivetrain Components

The design adapted select components from the K989 platform including the rear gearbox assembly and steering linkage ball joints. These parts provide proven reliability and appropriate gear ratios for the scale and performance requirements while maintaining cost effectiveness.

### GT24 Carisma Wheels and Tires

The rear wheels utilize standard 1/24 scale components for their proven traction characteristics and availability. Any compatible 1/24 scale tires can be substituted based on surface conditions.

### Custom Front Wheels

Front wheels are 3D printed to exact specifications needed for our steering geometry, paired with stretched 1/28 scale tires that provide optimal grip without requiring adhesive mounting.

## Fasteners and Hardware

### Standard Metric Hardware

All mechanical connections use standard M3 screws and fasteners for reliability and easy maintenance. Double-sided mounting tape provides secure attachment for electronic components while allowing for adjustments during development.

### Ball Joint Steering Links

Precision ball joints from the K989 system ensure smooth steering operation with minimal backlash, critical for accurate autonomous navigation.

## Cost Report

| Component                                       | Quantity | Unit Price (CAD) | Total Cost (CAD) |
| ----------------------------------------------- | -------- | ---------------- | ---------------- |
| Raspberry Pi 5 (8GB)                            | 1        | $139.99          | $139.99          |
| Pi 5 Expansion Board                            | 1        | $55.87           | $55.87           |
| HS-5055MG Servo                                 | 1        | $34.91           | $34.91           |
| K989 1/28 WL Toys Chassis                       | 1        | $105.68          | $105.68          |
| Furitek Micro Komodo Motor                      | 1        | $48.88           | $48.88           |
| LDRobot D500 LiDAR Kit                          | 1        | $99.60           | $99.60           |
| Texas Instruments TPS22918DBVR Power Switch ICs | 1        | $0.64            | $0.64            |
| Battery Pack (2S LiPo 7.4V ~2000mAh)            | 2        | $80.35           | $160.70          |
| Diyeeni Zero Module Camera (5MB, 1080p HD)      | 1        | $46.85           | $46.85           |
| GT24 M-Sport 2022 Hybrid Rally Wheels & Tires   | 1        | $22.00           | $22.00           |

## Measurements

| Part                      | Measurment | Rule Comp        |
| --------------------------|------------| ---------------- |
| Vehicle Length            | 18.2cm     |    30cm          | 
| Vehicle Width             | 10.0cm     |    20cm          | 
| Vehicle Height            | 22.4cm     |    30cm          | 
| Weight                    | 397g       |    1.5kg         | 


# Assembly Guide

## Overview

This guide provides comprehensive step-by-step instructions for assembling our fully custom-designed WRO Future Engineers autonomous vehicle. The robot features a completely custom 3D-printed chassis system with integrated drive components and precision steering mechanism.

## Required Tools

- Phillips head screwdrivers (various sizes)
- Soldering iron and solder (optional for secure connections)
- Wire strippers
- Digital calipers (for precise positioning)

## Required Components

### Custom 3D-Printed Chassis System

- Main chassis baseplate (PLA, 20% infill, 0.2mm layer height)
- Custom front gearbox housing
- Main electronics platform with integrated mounting pins
- Camera mount tower (approximately 80mm height)
- Custom front wheel rims
- Custom platform holders and mounting adapters
- Custom track rod with ball joint connections

### Drive System Components

- Rear gearbox assembly (K989 reference components)
- Custom front gearbox assembly
- FURITEK LIZARD Pro 30A/50A Brushed/Brushless ESC
- Drive motor with Furitek Micro Komodo controller
- Steering servo motor
- K989 steering rod components

### Electronics & Control

- Raspberry Pi 5
- Custom expansion board
- Pi Camera module with CSI ribbon cable
- LD19 D500 LIDAR sensor
- LiPo battery pack (rectangular form factor)

### Wheels and Tires

- Custom 3D-printed front rims with K989 tires (1/28 scale stretched fit)
- Rear wheels with GT24 Carisma tires (or any compatible 1/24 scale tires)

### Hardware & Fasteners

- M3 screws (various lengths for gearbox mounting)
- Double-sided mounting tape (high strength)
- K989 ball joints for steering linkage
- Axle nuts for wheel attachment
- Male-to-female jumper wires (3-wire set for LIDAR)

## Assembly Steps

### Step 1: Chassis Foundation Assembly

1. **Prepare the main chassis baseplate**
   - Ensure all mounting points are clear of support material
   - Test fit all major components before permanent installation
2. **Install rear gearbox assembly**
   - Position K989 rear gearbox in chassis mounting location
   - Secure using double-sided tape for primary attachment
   - Install custom 3D-printed platform holders onto K989 body mounting points
   - Connect platform holders to chassis using vertical M3 screws
3. **Install custom front gearbox housing**
   - Mount front gearbox assembly to chassis front section
   - Ensure proper alignment with steering geometry
   - Secure with double-sided tape and mechanical fasteners

### Step 2: Platform and Electronics Mounting

1. **Install main electronics platform**
   - Align platform with front and rear mounting points
   - Insert platform into custom holders through designed holes
   - Verify platform is level and secure
2. **Mount Raspberry Pi 5 and expansion board**
   - Align Raspberry Pi with integrated mounting pins on platform
   - Press Pi onto pins through standard mounting holes
   - Install expansion board using same pin mounting method
   - Ensure both boards are seated securely
3. **Install LiPo battery**
   - Place rectangular LiPo battery on its side (long edge down)
   - Position between platform sidewalls for friction fit
   - Verify battery is held securely by platform walls

### Step 3: Sensor Integration

1. **Mount LD19 D500 LIDAR**
   - Position LIDAR on platform for optimal 180-degree front coverage
   - Ensure LIDAR is parallel to ground plane
   - Secure using high-strength double-sided tape
   - Verify rotation clearance and mounting stability
2. **Install camera mount tower**
   - Position camera mount at rear of platform
   - Insert rectangular mounting beams into platform holes for friction fit
   - Verify tower is vertical and stable
   - Test camera field of view clearance
3. **Mount Pi Camera**
   - Install Pi Camera in fixed mount position on tower
   - Connect CSI ribbon cable to Raspberry Pi
   - Route cable carefully to avoid interference with moving parts
   - Verify camera orientation and focus adjustment

### Step 4: Drive System Integration

1. **Install Furitek Micro Komodo and ESC**
   - Mount Furitek Micro Komodo controller to chassis
   - Connect drive motor to controller
   - Install FURITEK LIZARD Pro ESC between controller and expansion board
   - Secure all components with appropriate mounting method
2. **Connect drive system wiring**
   - Connect ESC to expansion board using PWM signal pins
   - Ensure proper power distribution from battery
   - Verify all electrical connections are secure

### Step 5: Custom Steering System Assembly

1. **Install steering servo**
   - Mount steering servo in designated chassis location
   - Connect servo to expansion board using PWM pins
   - Verify servo movement range and centering
2. **Assemble steering linkage**
   - Attach K989 steering rod to servo horn
   - Connect custom track rod with ball joints to steering rod
   - Install ball joint connections to front wheel assemblies
   - Verify Ackermann steering geometry implementation
   - Test steering range - track rod should contact front gearbox at maximum turns
3. **Verify steering operation**
   - Check full steering range without binding
   - Ensure proper return to center position
   - Verify ball joint security and movement

### Step 6: Wheel and Tire Installation

1. **Install front wheels**
   - Mount custom 3D-printed front rims to axles
   - Secure with axle nuts
   - Install K989 tires onto rims (stretched fit, no adhesive required)
   - Verify wheel alignment and rotation
2. **Install rear wheels**
   - Mount rear wheels to rear axles
   - Install GT24 Carisma tires or any compatible 1/24 scale tires
   - Secure with appropriate fasteners
   - Verify proper clearance and alignment

### Step 7: System Wiring and Connections

1. **LIDAR connections**
   - Connect 3-wire harness (VCC, GND, Data) using male-to-female jumpers
   - Connect GND to any GPIO ground pin
   - Connect VCC and data signal to appropriate GPIO pins
   - Verify UART communication setup
2. **Power distribution**
   - Connect LiPo battery to main power distribution
   - Verify all components receive proper voltage
   - Check polarity on all connections
3. **Final wiring organization**
   - Route all cables to avoid mechanical interference
   - Secure loose wiring with appropriate methods
   - Verify no wires interfere with steering or drive systems

### Step 8: Final Assembly and Testing

1. **Mechanical verification**
   - Check all mounting points for security
   - Verify steering range and drive system operation
   - Ensure no mechanical binding or interference
2. **Electrical testing**
   - Power on system and verify all components initialize
   - Test LIDAR rotation and data output
   - Verify camera image capture
   - Test steering servo response
   - Verify drive motor operation through ESC
3. **System integration check**
   - Verify all sensors and actuators respond correctly
   - Check wireless connectivity if applicable
   - Perform initial calibration procedures

## Troubleshooting

**Common Assembly Issues:**

- **Steering binding**: Check track rod clearance with front gearbox; verify ball joint installation
- **Platform instability**: Ensure platform holders are properly seated and screwed into gearbox mounting points
- **Sensor mounting**: Verify double-sided tape adhesion; ensure LIDAR has clear rotation path
- **Electrical connections**: Check jumper wire connections; verify PWM pin assignments on expansion board

**Component Fit Issues:**

- **Wheel alignment**: Check axle nut tightness; verify rim installation on axles
- **Electronics mounting**: Ensure mounting pins are properly inserted through board holes
- **Battery fit**: Adjust platform wall spacing if battery is loose or too tight
