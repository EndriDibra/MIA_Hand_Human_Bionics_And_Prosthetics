# MIA Hand Project (Prensilia)  
**Author: Endri Dibra**

---

## Project Status

This is an **ongoing research and development project** involving the **Prensilia MIA Hand robotic system**, integrated with ROS 2, Docker, EMG control, and AI-based vision assistance.

The project is not yet finalized and focuses on real-time robotic grasping, human-machine interaction, and explainable AI integration.

---

## Project Goals

### Core Objectives
- Achieve real-time system performance (**100 ms – 400 ms latency**)
- Execute **5 different grasp types**
- Compare two control paradigms:
  - Traditional system: MIA Hand + Wrist + Myo EMG
  - Semi-autonomous system: MIA Hand + Wrist + EMG + AI Vision

---

### AI & Robotics Objectives
- Integrate AI vision for grasp decision support
- Compare:
  - Lightweight vs heavy vision models
  - Local vs cloud AI inference
- Enable dynamic grasp aperture adjustment

---

### Research Objectives
- Evaluate:
  - Accuracy
  - Latency
  - Cognitive burden on user
- Benchmark different AI-assisted grasping strategies
- Apply Explainable AI:
  - **SHAP** → feature importance (sensor/EMG data)
  - **LIME** → vision-based explanation

---

## System Architecture

### Hardware
- MIA Hand (Prensilia)
- Myo EMG armband
- Wrist control system
- USB serial interface (`ttyUSB0`)

### Software
- ROS 2
- Docker container environment
- Python control scripts
- Mujoco simulation
- OpenCV + AI models

---

## Setup Instructions

---

### Step 1: Build the Docker Image

### Dockerfile:

To build the environment from scratch using Docker, i use the following Dockerfile configuration. This installs ROS 2 Jazzy, MuJoCo 3.1.2, and applies the necessary compatibility patches for the MIA Hand packages.

```bash
FROM osrf/ros:jazzy-desktop

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    git \
    wget \
    libglfw3-dev \
    libxext6 \
    libxrender1 \
    libxtst6 \
    libgl1-mesa-dev \
    libserial-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Control and MuJoCo bridge dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-angles \
    ros-jazzy-controller-manager \
    ros-jazzy-transmission-interface \
    ros-jazzy-control-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Install MuJoCo 3.1.2
WORKDIR /opt
RUN wget https://github.com/google-deepmind/mujoco/releases/download/3.1.2/mujoco-3.1.2-linux-x86_64.tar.gz \
    && tar -xvzf mujoco-3.1.2-linux-x86_64.tar.gz \
    && rm mujoco-3.1.2-linux-x86_64.tar.gz

# Set MuJoCo environment variables
ENV MUJOCO_DIR=/opt/mujoco-3.1.2
ENV LD_LIBRARY_PATH=/opt/mujoco-3.1.2/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

# Create Workspace
WORKDIR /ros2_ws/src

# Clone the MIA Hand Repo and the MuJoCo Bridge
RUN git clone https://bitbucket.org/prensilia-ros/mia_hand_ros2_pkgs.git
RUN git clone https://github.com/moveit/mujoco_ros2_control.git

# THE JAZZY COMPATIBILITY PATCHES 

# 1. Fix MIA Hand Code (Update deprecated HardwareComponentInterfaceParams to HardwareInfo)
RUN sed -i 's/HardwareComponentInterfaceParams/HardwareInfo/g' \
    /ros2_ws/src/mia_hand_ros2_pkgs/mia_hand_mujoco/include/mia_hand_mujoco/system_interface.hpp \
    /ros2_ws/src/mia_hand_ros2_pkgs/mia_hand_mujoco/src/mia_hand_mujoco/system_interface.cpp && \
    sed -i 's/params.hardware_info/params/g' \
    /ros2_ws/src/mia_hand_ros2_pkgs/mia_hand_mujoco/src/mia_hand_mujoco/system_interface.cpp

# 2. Fix MuJoCo Bridge (Remove 'load_urdf' which is gone in Jazzy and fix ResourceManager init)
RUN sed -i '/resource_manager->load_urdf/d' \
    /ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_ros2_control.cpp && \
    sed -i 's/hardware.hardware_class_type/hardware.type/g' \
    /ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_ros2_control.cpp && \
    sed -i 's/std::make_unique<hardware_interface::ResourceManager>()/std::make_unique<hardware_interface::ResourceManager>(rclcpp::Clock::SharedPtr(new rclcpp::Clock()), rclcpp::get_logger("mujoco_ros2_control"))/g' \
    /ros2_ws/src/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_ros2_control.cpp

# Build the workspace
WORKDIR /ros2_ws
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

# Source the workspace automatically
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
```

---

### Build Image:

```bash
docker build -t mia_hand_image .
```

---

### Step 2: USB Setup (WSL Integration)

Connect device:
- Plug MIA Hand USB cable into PC
- Set latency timer = 1 in Windows Device Manager

PowerShell (Admin):

```bash
usbipd list
usbipd attach --wsl --busid 1-3
```

---

### Step 3: Open WSL Ubuntu

Install the Ubuntu terminal on Windows and Run on a PowerShell Terminal:

```bash
ubuntu
```

---

### Step 4: Docker Setup

Check containers:

```bash
docker ps -a
```

Start container:

```bash
docker start mia_hand_real
```

Enter container:

```bash
docker exec -it mia_hand_real bash
```

---

### Step 5: USB Verification

```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

If not `1`, fix it:

```bash
sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'
```

---

### Step 6: Mujoco Simulation Mode

XLaunch configuration:
- Multiple windows
- Display: 0
- Disable OpenGL
- Disable access control

**Terminal 1:**

```bash
export DISPLAY=ip:0.0
source /ros2_ws/install/setup.bash

ros2 launch mia_hand_mujoco mia_hand_system_interface_launch.py
```

**Terminal 2:**

```bash
export DISPLAY=ip:0.0
source /ros2_ws/install/setup.bash

chmod +x mia_hand_grasp_control_simulation.py
python3 mia_hand_grasp_control_simulation.py
```

---

### Step 7: Physical MIA Hand Execution

**Terminal 1:**

```bash
source /ros2_ws/install/setup.bash

ros2 launch mia_hand_driver mia_hand_driver_launch.py serial_port:=/dev/ttyUSB0
```

**Terminal 2:**

```bash
source /ros2_ws/install/setup.bash

chmod +x mia_hand_grasp_control_physical.py
python3 mia_hand_grasp_control_physical.py
```

---

### Step 8: VS Code (Inside Docker)

```bash
code mia_hand_grasp_control_physical.py
```

This allows direct editing of ROS2 control scripts inside the container.

---

## Experimental Research Focus

### Control Modes Comparison
- Traditional EMG-based control
- Semi-autonomous AI-assisted control

### AI Evaluation
- Lightweight vs heavy vision models
- Local vs cloud inference systems

### Metrics
- Latency (100–400 ms real-time constraint)
- Grasp accuracy
- System robustness
- User cognitive load

---

## Explainable AI (XAI)

### SHAP
- Explains sensor/EMG feature importance
- Identifies key contributors to grasp decisions

### LIME
- Explains vision-based object detection decisions
- Provides local interpretability for AI outputs

---

## Summary

This project explores a **human-AI hybrid robotic grasping system**, combining:

- Real-time robotic control  
- EMG-based human intent decoding  
- AI-assisted vision grasp selection  
- Explainable AI for transparency  

The goal is to determine the optimal balance between **human control and autonomous intelligence** in prosthetic robotic manipulation systems.
