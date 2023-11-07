# Auto Braking System (ABS) for Collision Avoidance

## Project Overview

The Auto Braking System (ABS) is a comprehensive project designed to enhance driver safety by proactively responding to critical situations using Real time operating system, where the driver may not react quickly enough. This system aims to identify potential collision risks, calculate the time to collision, and autonomously decide the appropriate actions. The ABS project incorporates a sophisticated simulation environment using PreScan and utilizes the STM32F429 Discovery board for core algorithm implementation. The key components of this system include six RADAR sensors (two for the same lane and four for adjacent lanes) to detect potential obstacles, an intelligent decision-making algorithm, and an integrated driver monitoring system.

## Features

- **Proactive Collision Avoidance:** The ABS system is engineered to identify and address potential collision risks in Real-time before they occur, reducing the chances of accidents and enhancing driver safety.

- **RADAR Sensors:** Utilizes six RADAR sensors to detect objects in the same lane and adjacent lanes, providing comprehensive situational awareness.

- **Time to Collision Calculation:** Employs the equation (relative speed/distance) to calculate the time to collision factor, which is a crucial parameter for decision-making.

- **Dynamic Decision Making:** Based on the time to collision factor, the system classifies scenarios into one of four regions: normal, warning, partial brake, or full brake. This decision-making process is integral to collision avoidance.

- **Lane-Changing Maneuvers:** The system can perform lane-changing maneuvers to explore better driving options, prevent road congestion, and ensure optimal traffic flow.

- **Lane-Keeping Assist:** Incorporates a lane-keeping assist feature to help the vehicle maintain its position within the lane, reducing the risk of unintended lane departure.

- **Driver Monitoring:** Utilizes a Raspberry Pi-based drowsiness and distraction detection module to assess the driver's condition. In situations where the driver may be incapable of driving, the system initiates the parking procedure for enhanced safety.

- **Emergency Assistance:** In critical situations, such as when the driver is unable to operate the vehicle, the ABS system communicates with a mobile application to relay the vehicle's coordinates and numbers to the nearest hospitals, ensuring prompt emergency assistance.

## Project Structure

The Auto Braking System project is structured as follows:

1. **Simulation Environment:** PreScan-based simulation environment to create a realistic testing environment for the system.

2. **STM32F429 Discovery Board:** The core algorithm is implemented on the STM32F429 Discovery board, which interacts with RADAR sensors and decision-making modules using Real time operating system.

3. **RADAR Sensors:** Six RADAR sensors are used for object detection in and around the vehicle.

4. **Driver Monitoring System:** A Raspberry Pi-based module monitors the driver's condition and relays data to the vehicle system.

5. **Communication Protocols:** The system employs UART, CAN, and mobile application communication protocols to exchange data efficiently.

6. **Decision Algorithm:** The core algorithm processes RADAR data, calculates time to collision, and makes decisions for collision avoidance.

## Supervisors

- **Swift-Act**
- **Dr. Omar A. Nasr**


Please click the link to make sure everything works perfectly and passes all tests without any problems.

https://drive.google.com/drive/folders/1K-QwOP_W1QtRm-7QSB5s1nWTsYDH7ODQ?usp=drive_link

## Contribution

Contributions and improvements to the Auto Braking System project are welcome. Please feel free to fork the repository, create pull requests, and enhance the system's safety and functionality. Your contributions can have a significant impact on saving lives and reducing accidents on the roads.

For detailed usage instructions, project documentation, and access to the project files, refer to the provided links and resources.
