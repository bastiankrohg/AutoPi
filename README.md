# AutoPi: Autonomous Mars Rover Project

## Important Links to development and testing repos: 
### Core Components
- **[SmartRover](https://github.com/bastiankrohg/smart-rover)** – Core rover control system.
- **[AutoPi (Current)](https://github.com/bastiankrohg/AutoPi/tree/dev_vision)** – Active development branch of the project.

### Telemetry & Visualization
- **[Telemetry](https://github.com/bastiankrohg/rover-telemetry)** – Rover telemetry data collection and transmission.
- **[MissionControl](https://github.com/bastiankrohg/MissionCtrl)** – PC-based telemetry receiver and dashboard visualization.

### Navigation & Path Planning
- **[2DMap (PyGame)](https://github.com/bastiankrohg/2Dmap/tree/main)** – 2D simulation and visualization for path planning.
- **[Navigation Algorithms & Path Planning](https://github.com/bastiankrohg/rover-coral/tree/autonomy/autonomous)** – Testing of path planning algorithms.

### Computer Vision & Detection
- **[Detection/Computer Vision Testing](https://github.com/bastiankrohg/detection)** – Object detection and resource identification experiments.

### Communication & Streaming
- **[MJPEG Streaming Server](https://github.com/bastiankrohg/AutoPi/blob/dev_vision/mjpeg.py)** – Video streaming for the rover.
- **[CoralCom](https://github.com/bastiankrohg/CoralCom)** – Store-and-forward communication and retransmission for Coral Dev Board.

## Overview
AutoPi is an autonomous rover project designed for exploring unknown terrains, developed for our final-year engineering project at INSA Toulouse. The system integrates a 4tronix Mars Rover with advanced software modules for navigation, obstacle avoidance, path planning, and telemetry. The project leverages both hardware and software innovations to create a semi-autonomous exploration vehicle capable of executing pre-defined patterns, avoiding obstacles, and dynamically adjusting its path. Also, we leverage the Picamera2 coupled with the Raspberry Pi Zero to stream video, and for capturing images that we use during the image processing and vision parts of the projects. 

## Features
- **State Machine**: Manages the rover's various states, including idle, exploring, obstacle avoidance, pursuing resources, and simulation mode.
- **Path Planning**: Implements various path patterns such as straight lines, sine waves, expanding squares, spirals, and zigzags.
- **Obstacle Avoidance**: Detects obstacles using ultrasonic sensors and dynamically replans paths.
- **Simulation Mode**: Allows testing of navigation algorithms (notably A*) in a simulated environment.
- **Telemetry**: Transmits real-time data over UDP for remote monitoring and visualization on a simple dashboard.

## Modules

### 1. Hardware Controllers
#### MotorController
Controls the rover's motors for forward, backward, and turning movements. Interfaces directly with the 4tronix Mars Rover's motor API.

#### SensorController
Manages the rover's sensors, including:
- **Ultrasonic Sensor**: Measures distance to obstacles.
- **Battery Sensor**: Monitors the battery level.

#### NavigationController
Combines motor and sensor data to calculate and execute movements to waypoints. Handles path following and turning to align the rover with the desired direction.

#### VisionController
Interfaces with a TensorFlow Lite (TFLite) model for object detection. Processes the camera feed to identify objects in the rover's frame and triggers state changes when specific objects are detected.

### 2. Planning
Implements path generation algorithms, including:
- **Straight Line**
- **Sine Wave**
- **Expanding Square**
- **Spiral**
- **Zigzag**
- **Random Walk**

These patterns are used for exploration and can dynamically adapt based on the rover's state and environment. We drew inspiration from maritime- and aerial search and rescue (SAR) patterns for optimal coverage of unknown territory to improve the performance of the area coverage during exploration.  

### 3. Obstacle Detector
A dedicated module for detecting obstacles using ultrasonic sensors. Alerts the system when an obstacle is within a predefined range and triggers the state machine to switch to avoidance mode.

### 4. Telemetry
The telemetry module sends real-time data over UDP to the Coral Dev Board. This data includes:
- Current position
- Heading
- Battery level
- Ultrasonic sensor readings
- Current state
- Obstacle proximity alerts

The Coral Dev Board renders a simple dashboard for remote monitoring and analysis, including a simple map element, images from the Picamera, odometry and other relevant datapoints that are interesting to keep track off during a mission.

### 5. State Machine
Manages the rover's behavior by transitioning between states such as:
- **Idle**: Waiting for commands.
- **Exploring**: Following predefined paths and adapting to obstacles.
- **Avoiding Obstacle**: Executing avoidance maneuvers and replanning.
- **Pursuing Resource**: Moving toward detected objects or resources.
- **Simulating**: Testing navigation logic in a controlled environment.

## Hardware Requirements
- 4tronix Mars Rover
- Raspberry Pi Zero
- Picamera 2
- Coral Dev Board (optional, for dashboard rendering, and improved vision model performance)
- Battery pack compatible with the Mars Rover

## Software Requirements
- Python 3.10+
- TensorFlow Lite (for vision processing)
- NumPy, Matplotlib (for path generation and visualization)
- Custom rover API for hardware control

## Setup Instructions
1. Clone the repository:
   ```bash
   git clone https://github.com/BastianKrohg/AutoPi.git
   cd AutoPi
   ```

2. Set up a virtual environment and install dependencies:
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

3. Connect the hardware:
   - Attach the Raspberry Pi Zero to the 4tronix Mars Rover.
   - Connect the ultrasonic sensor to the designated GPIO pins.
   - Ensure the battery pack is fully charged.

4. Run the AutoPi software:
   ```bash
   python autopi.py --debug --path straight_line --sim
   ```

5. For telemetry, ensure the Coral Dev Board is connected to the same network and listening for UDP packets on the specified port.

## Usage
- **Debug Mode**: Visualizes the local map and path planning grid.
- **Simulation Mode**: Simulates the rover's behavior without physical movement.
- **Path Selection**: Choose from various path patterns using the `--path` argument.
- **Telemetry**: Monitors rover data remotely on the Coral Dev Board.

## Acknowledgments
- The 4tronix team for their Mars Rover hardware.
- Our academic supervisor at INSA Toulouse, Tomasz Kloda, for following up the development of the project, and providing research material and sources 
