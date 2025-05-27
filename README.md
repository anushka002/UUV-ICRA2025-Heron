# Towards Robotic Trash Removal with Autonomous Surface Vessels 
**IEEE ICRA 2025 - _Robots in the Wild_ Workshop**

**Course: RAS-598 Space Robotics and AI (Grade: A+)**

**Final Project: Heron USV: Opportunistic Trash Collection and Scientific Survey 🌊♻️**
---

Welcome to the **Heron USV Project** repository, showcasing an innovative approach to **autonomous trash detection and collection** integrated into a **scientific survey mission**. This work was developed for **IEEE ICRA 2025** and combines **ROS-Gazebo simulation**, **real-time vision processing**, and **survey-based navigation (boustrophedon pattern)**.

## 📖 Publications & Contributions
- 📄 **ICRA 2025**: _Towards Robotic Trash Removal with Autonomous Surface Vessels_  
  _Presented at IEEE International Conference on Robotics and Automation (Robots in the Wild Track)_

  Read the full paper: https://lnkd.in/dHszBTn3
---
## **Simulation GIF**

![heron final gif](https://github.com/user-attachments/assets/4c426402-e8b2-4624-9c48-aea97ab758ec)  

**Powerpoint presentation**

[Towards Robotic Trash Removal with Autonomous Surface Vessels.pptx](https://github.com/user-attachments/files/20459101/Towards.Robotic.Trash.Removal.with.Autonomous.Surface.Vessels.pptx)

---

## 🌟 Key Highlights
- **Autonomous Surface Vessel (ASV)**
  Based on the Clearpath Heron platform, capable of scientific data collection and environmental cleanup.

- **Vision-based Trash Detection**  
  Incorporates a lightweight color-based proxy in simulation and a fine-tuned YOLOv8 model for field testing.

- **Autonomous Navigation & Detour Logic**  
  The Heron UUV follows a **boustrophedon (lawnmower)** survey pattern and activates a detour mechanism only when:
  - Detection confidence ≥ threshold
  - Target is within sensor scan range
  - Distance from the vehicle is within operational limits
  - ASV dynamically interrupts its survey path to collect detected trash, then seamlessly resumes its mission.

- **Simulation**  
  - **Simulation**: ROS-Gazebo environment with digital twin of the Heron UUV  

- **Future Work**  
  - **Field Testing and Hardware Deployment**: Experiments to be conducted on the R/V Karin Valentine UUV, DREAMSLab.

## 🔧 Features
- 🚀 **ROS-based Control Architecture**: Modular design for perception, planning, and control.
- 📸 **Camera Integration**: Simulated RGB camera stream and real-world camera for trash detection.
- 🧠 **YOLOv8 Detection**: Fine-tuned model for high-confidence trash detection in aquatic environments.
- 🌐 **Path Planning**: Combines boustrophedon coverage and opportunistic detours.
- 📊 **Real-Time Decision Making**: Activation of detours only if detection meets confidence and spatial constraints.


## 🏗️ Setup and Usage
1️⃣ **Clone the Repository**
```
git clone git@github.com:anushka002/UUV-ICRA2025-Heron.git
cd trash_detect_pkg
```

2️⃣ **Install Dependencies**
```
pip install -r requirements.txt
```
3️⃣ **Launch Simulation**
```
roslaunch heron_gazebo trash_floating_world.launch
```

4️⃣ **Run Trash Detection Node**
```
rosrun trash_detect_pkg color_detector_node.py
```

5️⃣ **Run Navigation and Control**
```
rosrun trash_detect_pkg trash_collection_node.py
```

6️⃣ **Visualize in RViz**
```
rviz
```

---
## 🔬 Results & Media Gallery

| Poster | 
|--------|
| [![Poster](https://github.com/user-attachments/assets/4a791fd8-e942-4061-8daf-dd4f06f4a851)](https://github.com/user-attachments/assets/4a791fd8-e942-4061-8daf-dd4f06f4a851) |  

| Results Snapshot |
|------------------|
| ![Results](https://github.com/user-attachments/assets/34755146-175b-4e0b-874a-aacfe5796db6) |


| Field Test |
|------------------|
| ![image](https://github.com/user-attachments/assets/e06ec016-776a-49f1-873a-839387cafbdc) |

✨ **A gallery of the highlights from our project — including design, simulation, field testing, and impactful results.**


## 🌍 Future Work
- Extend to multi-agent coordination for wider area coverage.
- Integrate adaptive path planning with environmental data mapping.
- Enhance detection robustness under varying lighting and water conditions.

## 🤝 Acknowledgments
This work was developed as part of our research at **Arizona State University**, under the guidance of Prof. Jnyaneshwar Das and supported by the DREAMSLab.

Special thanks to:
- Chinmay Amrutkar
- Rodney Jr. Staggers
- Prof. Jnyaneshwar Das
 
---

## 📜 License
This repository is shared for academic purposes. Please contact us for usage inquiries.
