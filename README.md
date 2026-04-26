# MIA Hand Project (Prensilia)  
**Author: Endri Dibra**

---

## 📌 Project Status

This is an **ongoing research and development project** involving the **Prensilia MIA Hand robotic system**, integrated with ROS 2, Docker, EMG control, and AI-based vision assistance.

The project is not yet finalized and focuses on real-time robotic grasping, human-machine interaction, and explainable AI integration.

---

## 🎯 Project Goals

### 🧠 Core Objectives
- Achieve real-time system performance (**100 ms – 400 ms latency**)
- Execute **5 different grasp types**
- Compare two control paradigms:
  - Traditional system: MIA Hand + Wrist + Myo EMG
  - Semi-autonomous system: MIA Hand + Wrist + EMG + AI Vision

---

### 🤖 AI & Robotics Objectives
- Integrate AI vision for grasp decision support
- Compare:
  - Lightweight vs heavy vision models
  - Local vs cloud AI inference
- Enable dynamic grasp aperture adjustment

---

### 📊 Research Objectives
- Evaluate:
  - Accuracy
  - Latency
  - Cognitive burden on user
- Benchmark different AI-assisted grasping strategies
- Apply Explainable AI:
  - **SHAP** → feature importance (sensor/EMG data)
  - **LIME** → vision-based explanation

---

## ⚙️ System Architecture

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

## 🔧 Setup Instructions

---

### 1️⃣ Clone MIA Hand ROS 2 Repository

```bash
git clone https://bitbucket.org/prensilia-ros/mia_hand_ros2_pkgs.git
```

ROS 2 workspace setup:

```bash
cd ~/ros2_ws/src
git clone https://bitbucket.org/prensilia-ros/mia_hand_ros2_pkgs.git
cd ..
colcon build
source install/setup.bash
```

---

### 2️⃣ USB Setup (WSL Integration)

Connect device:
- Plug MIA Hand USB cable into PC
- Set latency timer = 1 in Windows Device Manager

PowerShell (Admin):

```bash
usbipd list
usbipd attach --wsl --busid 1-3
```

---

### 3️⃣ Open WSL Ubuntu

```bash
ubuntu
```

---

### 4️⃣ Docker Setup

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

### 5️⃣ USB Verification

```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

If not `1`, fix it:

```bash
sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'
```

---

### 6️⃣ Mujoco Simulation Mode

XLaunch configuration:
- Multiple windows
- Display: 0
- Disable OpenGL
- Disable access control

Terminal 1:

```bash
export DISPLAY=ip:0.0
source /ros2_ws/install/setup.bash

ros2 launch mia_hand_mujoco mia_hand_system_interface_launch.py
```

Terminal 2:

```bash
export DISPLAY=ip:0.0
source /ros2_ws/install/setup.bash

chmod +x mia_hand_grasp_control_simulation.py
python3 mia_hand_grasp_control_simulation.py
```

---

### 7️⃣ Physical MIA Hand Execution

Terminal 1:

```bash
source /ros2_ws/install/setup.bash

ros2 launch mia_hand_driver mia_hand_driver_launch.py serial_port:=/dev/ttyUSB0
```

Terminal 2:

```bash
source /ros2_ws/install/setup.bash

chmod +x mia_hand_grasp_control_physical.py
python3 mia_hand_grasp_control_physical.py
```

---

### 8️⃣ VS Code (Inside Docker)

```bash
code mia_hand_grasp_control_physical.py
```

This allows direct editing of ROS2 control scripts inside the container.

---

## 🧪 Experimental Research Focus

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

## 🔍 Explainable AI (XAI)

### SHAP
- Explains sensor/EMG feature importance
- Identifies key contributors to grasp decisions

### LIME
- Explains vision-based object detection decisions
- Provides local interpretability for AI outputs

---

## 🧠 Summary

This project explores a **human-AI hybrid robotic grasping system**, combining:

- Real-time robotic control  
- EMG-based human intent decoding  
- AI-assisted vision grasp selection  
- Explainable AI for transparency  

The goal is to determine the optimal balance between **human control and autonomous intelligence** in prosthetic robotic manipulation systems.
