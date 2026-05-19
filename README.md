# MIA Hand Project (Prensilia)  
**Author: Endri Dibra**

---

## Project Status

This is an **ongoing research and development project** involving the **Prensilia MIA Hand robotic system**, integrated with ROS 2, Docker, EMG control, Wrist motor control (Dynamixel) and AI-based vision assistance.

The project is not yet finalized and focuses on real-time robotic grasping, human-machine interaction, and explainable AI integration.

<h2 align="center">Prensilia  MIA Hand</h2>

<p align="center">
  <img src="https://github.com/user-attachments/assets/8ae28d1e-3fe1-4203-b6c6-202ed537e6b1" width="350" title="MIA Hand">
</p>

- **This is MIA Hand, with the Wrist Motor and the Depth Camera**
- **This Robotic System is used for this Project and Experiments**

---

<h2 align="center">MindRove EMG Data</h2>

<img width="959" height="503" alt="EMG_Data" src="https://github.com/user-attachments/assets/449a70a4-7f0f-43e9-8328-6335b916b6b9" />

- **These are the 8 EMG signal channels, representing my right hand muscles activity**
- **The data from the EMG signals are used to train, fine-tune NaviFlame AI, for predicting my gestures**

---

<h2 align="center">MIA Hand Simple MuJoCo Simulation</h2>

https://github.com/user-attachments/assets/cf530741-d14d-4f3f-a56a-b2b21c569f8f

- **At this setup i am using the keys from the keyboard to control MIA Hand**
- **The simulation is on MuJoCo environment**

---

<h2 align="center">MIA Hand EMG Control MuJoCo Simulation</h2>

https://github.com/user-attachments/assets/219e9298-0da8-4f9e-9323-c59ad1b8cabf

- **On the left video, i am controling the MIA Hand using the MindRove EMG device, shown how it looks like on the right video for educational purposes**
- **The movements on the right video, do not correspond to the gestures on the left video. The right video it is used to present the MindRove EMG device**

---

<h2 align="center">MIA Hand EMG Control Physical</h2>

- **The link below has the video with the implementation involving the Physical Robotic MIA Hand**
- Firstly, i present the functionalities of the MIA Hand
- Secondly, i showcase the 3 grasps of MIA Hand
- Ans lastly, MIA Hand handshakes with another robotic hand 
- https://www.youtube.com/watch?v=SKP5CYs-0fs

---

## Project Goals

### Core Objectives
- Achieve real-time system performance (**100 ms – 400 ms latency**)
- Execute **3 different grasp types**:
  - Cylindrical
  - Pinch
  - Lateral
- Compare two control paradigms:
  - Traditional system: MIA Hand + Wrist + MindRove EMG
  - Semi-autonomous system: MIA Hand + Wrist + EMG + AI Vision + XAI

---

### AI & Robotics Objectives
- Integrate AI vision for grasp decision support
- Compare:
  - Lightweight vs heavy Vision models
  - Local vs cloud Vision models
- Enable dynamic grasp aperture adjustment

---

### Research Objectives
- Evaluate:
  - Accuracy
  - Latency
  - Cognitive burden on user
- Benchmark different AI-assisted grasping strategies
- Apply Explainable AI:
  - **SHAP** → Feature importance (sensor/EMG data)
  - **LIME** → Vision-based explanation

---

## System Architecture

### Hardware
- MIA Hand (Prensilia)
- MindRove EMG armband
- Wrist control system
- USB serial interface (`ttyUSB0`, `ttyUSB1` )

### Software
- ROS 2 (Jazzy)
- Linux (Ubuntu)
- Docker container environment
- Python control scripts
- MuJoCo simulation
- OpenCV + AI models (YOLO)

---

## Setup Instructions for MIA Hand and Wrist Motor (Python based)

### Step 1: Open WSL Ubuntu (in PowerShell)

Install the Ubuntu terminal on Windows and Run on a PowerShell Terminal:

```bash
ubuntu
```

---

### Step 2: Build the Docker Image (in Ubuntu)

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

### Step 3: Create the Docker Container (in Ubuntu)

```bash
docker create -it \
    --name mia_hand_real \
    --privileged \
    --device=/dev/ttyUSB0:/dev/ttyUSB0 \
    --device=/dev/ttyUSB1:/dev/ttyUSB1 \
    --net=host \
    -p 5000:5000 \
    -e DISPLAY=host.docker.internal:0.0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    mia_hand_image
```

---

### Step 4: Run the Docker Container (in Ubuntu)

```bash
docker run -it -p 5000:5000 --name mia_hand_real mia_hand_image bash
```

---

### Step 5: USB Setup - WSL Integration (in PoweShell Admin)

Connect device:
- Plug MIA Hand and Wrist Motor USB cable into PC
- Set latency timer = 1 in Windows Device Manager

Check the list of the USB ports:

```bash
usbipd list
```

Bind the MIA Hand and Wrist motor USB ports:

```bash
usbipd bind --busid <BUSID1>
usbipd bind --busid <BUSID2>
```

Attach the MIA Hand and Wrist motor USB ports to WSL2 environment:

```bash
usbipd attach --wsl --busid <BUSID1>
usbipd attach --wsl --busid <BUSID2>
```

---

### Step 6: Start the Docker Container (For the next time, in Ubuntu)

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

### Step 7: USBs Setup (in Docker Container)

Check the available USB ports:

```bash
ls /dev/ttyUSB*
```

Grant Read/Write permissions to USB ports:

```bash
sudo chmod 666 /dev/ttyUSB0

sudo chmod 666 /dev/ttyUSB1
```

Inject the USB ports manually (Optional, in Ubuntu):

```bash
sudo docker exec -u 0 mia_hand_real mknod -m 666 /dev/ttyUSB0 c 188 0
sudo docker exec -u 0 mia_hand_real mknod -m 666 /dev/ttyUSB1 c 188 1
```

Check the latency timer of the USB ports:

```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

cat /sys/bus/usb-serial/devices/ttyUSB1/latency_timer
```

If their latency timer is not `1`, then to fix it, do:

```bash
sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'

sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB1/latency_timer'
```

---

### Step 8: Mujoco Simulation MIA Hand and Wrist Motor Execution (in Docker Container)

XLaunch configuration:
- Multiple windows
- Display: 0
- Disable OpenGL
- Disable access control

**Terminal 1:**

Check the ip4 address of the device (host, in Windows):

```bash
ipconfig
```

In Ubuntu, the command is:

```bash
ifconfig
```

Run the MuJoCo MIA Hand simulation:

```bash
export DISPLAY=ip4_address:0.0
source /ros2_ws/install/setup.bash

ros2 launch mia_hand_mujoco mia_hand_system_interface_launch.py
```

**Terminal 2:**

Run the ROS2 node to control the MIA Hand in MuJoCo simulation:

```bash
export DISPLAY=ip4_address:0.0
source /ros2_ws/install/setup.bash

chmod +x mia_hand_advanced_simulation.py
python3 mia_hand_advanced_simulation.py
```

---

### Step 9: Physical MIA Hand and Wrist Motor Execution (in Docker Container)

**Terminal 1:**

Run the MIA Hand driver to have access to it:

```bash
source /ros2_ws/install/setup.bash

ros2 launch mia_hand_driver mia_hand_driver_launch.py serial_port:=/dev/ttyUSB0
```

**Terminal 2:**

Run the ROS2 node to control with the EMG gestures the physical MIA Hand and its Wrist motor:

```bash
source /ros2_ws/install/setup.bash

chmod +x mia_hand_EMG_physical.py
python3 mia_hand_EMG_physical.py
```

---

### Step 10: VS Code (in Docker Container)

Example:

```bash
code mia_hand_EMG_physical.py
```

This allows direct editing of ROS2 scripts, inside the container, in VS Code environment.

---

## Setup Instructions for MindRove EMG and NaviFlame AI (Python based)

### EMG Gestures to Control MIA Hand and Wrist motor

- Gesture ID `0`: Hand is relaxed:
  - MIA Hand `Opens`

<p align="center">
  <img width="250" height="250" alt="g0" src="https://github.com/user-attachments/assets/da0f6fdc-b5ad-44fa-a7e0-d54711762c0c" />
</p>

---

- Gesture ID `1`: Thumb is up:
  - MIA Hand rotates in the `Middle`

<p align="center">
  <img width="250" height="250" alt="g1" src="https://github.com/user-attachments/assets/1ec9eecd-f8da-4413-8c90-eba1b2a37c46" />
</p>

---

- Gesture ID `2`: Index and Middle finger are up:
  - MIA Hand performs the `Lateral grasp`  

<p align="center">
  <img width="250" height="250" alt="g2" src="https://github.com/user-attachments/assets/9e538c6f-6e0f-4aee-9ebe-b1ca58a5b617" />
</p>

---

- Gesture ID `3`: All fingers are closed and tight:
  - MIA Hand perform the `Cylindrical` or `Full` grasp

<p align="center">
  <img width="250" height="250" alt="g3" src="https://github.com/user-attachments/assets/e7f0ec52-7321-4759-a786-20b20ae7076e" />
</p>

---

- Gesture ID `4`: Index finger is up:
  - MIA Hand performs `Pinch` or `Precision` grasp 

<p align="center">
  <img width="250" height="250" alt="g4" src="https://github.com/user-attachments/assets/ef5c385f-3cff-460c-8846-c1a87f527ef8" />
</p>

---

- Gesture ID `5`: Wrist flexion upwards:
  - MIA Hand rotates on the `Left` side 

<p align="center">
  <img width="250" height="250" alt="g5" src="https://github.com/user-attachments/assets/c9d09ead-46b5-4265-9e57-771484e6f43a" />
</p>

---

- Gesture ID `6`: Wrist flexion downwards:
  - MIA Hand rotates on the `Right` side   

<p align="center">
  <img width="250" height="250" alt="g6" src="https://github.com/user-attachments/assets/b1751e00-e7fd-40b3-a1a3-fe32babbdced" />
</p>

---

### Requirements

- Python >=3.7 and <3.11.
- Libraries:
  - `tensorflow==2.12.0`
  - `scikit-learn`
  - `opencv-python`
  - `numpy`
  - `mindrove` (MindRove SDK)
 
---

### Step 1: Clone the Naviflame GitHub Repository (VS CODE, in Windows)

Run the fllowing command to clone the repository:

```bash
git clone https://github.com/MindRove/NaviFlame.git
```
---

### Step 2: Install the required packages to control the EMG device with NaviFlame AI (VS Code, in Windows)

Run the following command to install the packages:

```bash
pip install mindrove tensorflow keras scikit-learn opencv-python numpy
```
---

### Step 3: Ensure the MindRove device is properly placed on the Hand and connected to WiFi 2

**Check the ReadMe file of NaviFlame for more information:**

https://github.com/MindRove/NaviFlame/blob/main/README.md

---

### Step 4: Record the Gestures

**To Record the hand gestures do the following:** 

Run the example.py script with the record flag set to True in config.json to capture gestures:

```bash
python example.py
```

---

### Step 5: Fine-Tune MLP model

**To fine-tune the MLP model with the specific recorded gesture data do the following:**

Set fine_tune to True in config.json and run example.py. After recording gestures, the script will fine-tune the MLP model:

```bash
python example.py
```

---

### Step 6: Real-Time Inference

**To check if the MLP model (NaviFlame AI) is reading and responding to the gestures properly, do the following:**

Ensure recorded data exists and the model is fine-tuned. Run the inference example to perform real-time inference:

```bash
python inference_example.py
```

---

### Step 7: Setup the Network Communication between MindRove Device and MIA Hand

At utils.py file, change the send_output_to_socket function to this format:

```bash
def send_output_to_socket(stop_event, output_queue):
    
    # Setting the connection parameters
    host = 'localhost'
    port = 5000

    while not stop_event.is_set(): 
    
        try:
    
            # Creating the socket and connecting to the Docker-mapped port
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    
                s.settimeout(5)  
                s.connect((host, port))
                
                print(f"Connected to MIA Hand Receiver on {port}")

                while not stop_event.is_set():
    
                    try:
    
                        # Getting the gesture ID from the AI queue
                        gesture_id = output_queue.get(timeout=1)
    
                        # Sending the ID as a string (e.g., "3")
                        s.sendall(str(gesture_id).encode('utf-8'))
    
                    except Empty:
    
                        continue  
    
                    except (BrokenPipeError, ConnectionResetError):
    
                        print("Connection lost. Reconnecting...")
    
                        break  

        except (ConnectionRefusedError, socket.timeout):
    
            print("MIA Receiver not ready. Retrying in 5 seconds...")
    
            time.sleep(5)
            
    print("Socket thread stopped.")
```

---

### Step 8: Final Stage

Finally, to start and run the implementation, do the following:

- Run the MIA Hand Driver (`ros2 launch mia_hand_driver mia_hand_driver_launch.py serial_port:=/dev/ttyUSB0`)
- Run the MIA Hand ROS2 node (`mia_hand_full_physical.py`)
- Run the inference script (`inference_example.py`)
- Perform EMG gestures with the MindRove Armband
- Now MIA Hand moves according to the EMG gestures

---

## Setup Instructions for the Semi-Autonomous Level (Python based)

### EMG Gestures to Control MIA Hand

- Gesture ID `0`: Hand is relaxed:
  - MIA Hand returns to its default position, thus, in the middle and with the hand open

<p align="center">
  <img width="250" height="250" alt="g0" src="https://github.com/user-attachments/assets/da0f6fdc-b5ad-44fa-a7e0-d54711762c0c" />
</p>

---

- Gesture ID `2`: Index and Middle finger are up:
  - MIA Hand performs the execution command, to close the hand/grasp  

<p align="center">
  <img width="250" height="250" alt="g2" src="https://github.com/user-attachments/assets/9e538c6f-6e0f-4aee-9ebe-b1ca58a5b617" />
</p>

---

### Requirements

- Python >=3.7 and <3.11.
- Libraries:
  - `pyrealsense2`
  - `ultralytics`
  - `opencv-python`
  - `numpy`
  - `pyttsx3`
  - `speech_recognition`
- `yolo11n.pt` model weights

Install the libraries with this command:

```bash
pip install pyrealsense2 ultralytics opencv-python numpy pyttsx3 speech_recognition
```

---

### Step 1: Depth Camera USB Setup - WSL Integration (in PoweShell Admin)

**The Depth Camera used for this projects is the Intel RealSense D-405**

Connect device:
- Plug the Depth Camera with a type 3.0 USB cable into PC
- Set latency timer = 1 in Windows Device Manager

Check the list of the USB ports:

```bash
usbipd list
```

Bind the Depth Camera USB port:

```bash
usbipd bind --busid <BUSID>
```

Attach the Depth Camera USB port to WSL2 environment:

```bash
usbipd attach --wsl --busid <BUSID>
```
---

### Step 2 (Optional): Run the Base Script for Depth Camera Stream (in Docker Container)

Run this script to verify and see the depth info stream:

```bash
python3 depthCamera.py
```

- As it is shown, the closest objects appear in blue color
- Instead, the distant objects appear in red color
- The objects that are out of the detection range of the depth camera appear in black color

<table align="center">
    <tr>
      <td><img src= "https://github.com/user-attachments/assets/d10da4fd-ebd3-4fdd-a3cd-1f61d2745703" width="300" title="Robot View 2"></td>
      <td><img src="https://github.com/user-attachments/assets/6e12f257-3cf8-4479-a024-15d3d9707d13" width="300" title="Robot View 1"></td>
      <td><img src="https://github.com/user-attachments/assets/c779c125-ea1e-4594-9f67-f3fdfb232cfd" width="300" title="Robot View 3"></td>
    </tr>
</table>

---

### Step 3 (Optional): Run the Object Detector Script with Depth Camera Stream (in Docker Container)

Run this script to detect objects around the scene and check the depth info stream too:

```bash
python3 depthCameraYolo.py
```

- As it is shown, the closest objects appear in green/blue color
- Instead, the distant objects appear in orange/red color

<table align="center">
  <tr>
    <td><img src="https://github.com/user-attachments/assets/0b5c40c4-0cde-4651-9e4a-3c86acb73e6a" width="300" title="Robot View 2"></td>
    <td><img src="https://github.com/user-attachments/assets/39484ae7-8709-4153-bd68-0cd0907121a3" width="300" title="Robot View 1"></td>
  </tr>
</table>

---

### Step 4: Run the Main AI Sript to Perform a Semi-Autonomous Approach (in Docker Container)

- Here the AI Camera will detect the object after the voice commands
- Then, it will predefine the grasp and the rotation type for that specific object
- And lastly, will wait for the execution EMG gesture command from the user, to close the hand and grasp the object
- After that, you can repeat the process by saying again a phrase that includes the words: go, start, begin 

```bash
python3 mia_hand_full_ai_physical.py
```

---

### Step 5: Run the Voice Technology Script to Control the System and MIA Hand (in Windows)

- Now you can start the system by saying, a command phrase that includes the words: start, go, begin
- Then, the system will respond to you that it has started successfully
- It will ask you about what object it should search for
- After that, you just say the name of the object, e.g. bottle
- If there are more that one e.g bottles, it will focus to the one that is closer to the depth camera

```bash
python3 voiceTechnology.py
```

---

### Step 6: Final Stage

So, to perform the Semi-Autonomous approach:
- Run the MIA Hand Driver (`ros2 launch mia_hand_driver mia_hand_driver_launch.py serial_port:=/dev/ttyUSB0`)
- Run the MIA Hand ROS2 node (`mia_hand_full_ai_physical.py`)
- Run the Voice Technology script (`voiceTechnology.py`)
- Run the inference script (`inference_example.py`)

---

## Experimental Research Focus

### Control Modes Comparison
- Traditional EMG-based control
- Semi-autonomous AI-assisted control

### AI Evaluation
- Lightweight vs heavy Vision models
- Local vs cloud Vision models

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

---
