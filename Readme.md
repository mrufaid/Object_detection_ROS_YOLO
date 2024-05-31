# Object Detection using ROS2, Ultralytics, and OpenCV

This project integrates ROS2, Ultralytics YOLO (You Only Look Once) models, and OpenCV to create a robust object detection system. The system is designed to detect and classify objects in real-time, making it suitable for various robotics applications such as navigation, manipulation, and surveillance.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
  - [Prerequisites](#prerequisites)
  - [Setting up the environment](#setting-up-the-environment)
  - [Installing Dependencies](#installing-dependencies)
- [Usage](#usage)
  - [Running the Object Detection Node](#running-the-object-detection-node)
  
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Introduction

This project aims to leverage the powerful combination of ROS2 for robotics middleware, Ultralytics YOLO for state-of-the-art object detection, and OpenCV for computer vision operations. By combining these technologies, the project provides a real-time object detection solution that can be easily integrated into various robotic systems.

## Features

- **Real-time Object Detection**: Utilizes YOLO models for high-speed object detection.
- **ROS2 Integration**: Fully compatible with ROS2, allowing easy integration with other ROS2-based systems.
- **OpenCV Support**: Utilizes OpenCV for image processing and visualization.
- **Customizable**: Easily configurable to use different YOLO models and parameters.

## Installation

### Prerequisites

Ensure you have the following installed on your system:

- ROS2 (Foxy, Galactic, or Humble recommended)
- Python 3.8+
- OpenCV
- Ultralytics YOLO

### Setting up the Environment

1. **Install ROS2**: Follow the official ROS2 installation guide [here](https://docs.ros.org/en/foxy/Installation.html).

2. **Create a ROS2 Workspace**:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/
    colcon build
    source install/setup.bash
    ```

### Installing Dependencies

3. **Clone the Repository**:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/mrufaid/object_detection_ROS_YOLO.git
    ```

4. **Install Python Dependencies**:
    ```bash
    cd ~/ros2_ws/src/object_detection_ROS_YOLO
    pip install -r requirements.txt
    ```

5. **Build the Workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```

## Usage

### Running the Object Detection Node

1. **Run ROS2 Node**:
    ```bash
    ros2 run object_detection_ROS_YOLO vid_pub.py
    ros2 run object_detection_ROS_YOLO Vid_sub.py
    ```

2. **Viewing Detected Objects**:
    A pop-up window will open displaying video and highlighted detected objects


## Contributing

contributions are welcomed to this project. To contribute:

1. Fork the repository.
2. Create a new branch 
4. Commit your changes.
5. Push to the branch.
6. Create a new Pull Request.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgements

- [ROS2](https://docs.ros.org/en/foxy/index.html) - Robotics middleware framework.
- [Ultralytics YOLO](https://github.com/ultralytics/yolov8) - High-performance object detection models.
- [OpenCV](https://opencv.org/) - Open-source computer vision library.

---
