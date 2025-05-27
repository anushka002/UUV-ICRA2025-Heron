# UUV-ICRA2025-Heron 🌊🚤

This repository contains the codebase for our submission to **IEEE ICRA 2025**, titled:

**"Towards Robotic Trash Removal with Autonomous Surface Vessels"**

Our work focuses on enabling **autonomous trash interception** in aquatic environments using the **Heron Unmanned Surface Vehicle (UUV)**. 
This system prioritizes scientific survey integrity while opportunistically collecting floating waste in real-time.

---

## 🚀 Key Highlights
- **Vision-based Trash Detection**  
  Incorporates a lightweight color-based proxy in simulation and a fine-tuned YOLOv8 model for field testing.

- **Autonomous Navigation & Detour Logic**  
  The Heron UUV follows a **boustrophedon (lawnmower)** survey pattern and activates a detour mechanism only when:
  - Detection confidence ≥ threshold
  - Target is within sensor scan range
  - Distance from the vehicle is within operational limits

- **Simulation & Field Validation**  
  - **Simulation**: ROS-Gazebo environment with digital twin of the Heron UUV  
  - **Field Testing**: Experiments conducted on the R/V Karin Valentine

---

## 🏗️ Repository Structure
📁 src/
    ├── perception/         # Vision-based detection modules
    ├── navigation/         # Coverage path planning and detour logic
    ├── control/            # Low-level UUV control and actuation
    ├── simulation/         # Gazebo worlds, models, and launch files
    ├── utils/              # Helper scripts and configuration files
    └── README.md           # Project overview (this file)

---

## 📋 Getting Started
### 1️⃣ Clone the Repository
git clone git@github.com:anushka002/UUV-ICRA2025-Heron.git
cd UUV-ICRA2025-Heron

### 2️⃣ Set Up Dependencies
- ROS (Noetic or Humble recommended)  
- Gazebo simulator  
- Python dependencies (see `requirements.txt`)  
- YOLOv8 model weights (for real-world detection)

### 3️⃣ Launch the Simulation
roslaunch simulation heron_world.launch

---

## 🔬 Results & Media
🔜 **Coming Soon**:  
- Project Poster  
- Simulation GIF  
- Field Test Results & Plots

---

## 🤝 Acknowledgements
This work was developed as part of our research at **Arizona State University**, under the guidance of [Your Advisor’s Name] and supported by the **[Your Lab/Group Name]**.

Special thanks to:
- Chinmay Amrutkar
- Rodney Jr. Staggers
- Prof. Jnyaneshwar Das

---

## 📜 License
This repository is shared for academic purposes. Please contact us for usage inquiries.
