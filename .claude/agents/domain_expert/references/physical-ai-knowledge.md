# Physical AI Domain Knowledge

## Core Concepts

### What is Physical AI?
Physical AI refers to artificial intelligence systems that interact with the physical world through sensors and actuators. Unlike traditional AI that operates purely in digital domains, Physical AI bridges the gap between computation and real-world action.

**Key Components:**
1. **Perception**: Sensors (cameras, lidar, IMUs)
2. **Cognition**: Decision-making systems (planning, learning)
3. **Action**: Actuators (motors, grippers, wheels)
4. **Feedback Loop**: Continuous sensing and adaptation

## Humanoid Robotics

### Definition
Humanoid robots are robots with a body shape resembling the human form, typically including:
- Two legs for bipedal locomotion
- Two arms for manipulation
- A torso
- A head with sensors (cameras, microphones)

### Leading Platforms
- **Atlas** (Boston Dynamics)
- **Optimus** (Tesla)
- **Figure 01** (Figure AI)
- **Digit** (Agility Robotics)
- **H1** (Unitree Robotics)

### Key Challenges
1. **Balance & Locomotion**: Maintaining stability while walking
2. **Manipulation**: Grasping and handling objects
3. **Perception**: Understanding 3D environments
4. **Integration**: Coordinating all systems in real-time

## ROS 2 (Robot Operating System 2)

### Purpose
ROS 2 is a middleware framework for building robot applications. It provides:
- Communication infrastructure (pub/sub, services, actions)
- Hardware abstraction
- Package management
- Tooling for development and debugging

### Key Differences from ROS 1
| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Middleware | Custom TCP/IP | DDS |
| Real-time | Limited | Yes |
| Security | Basic | Built-in |
| Multi-robot | Difficult | Native support |

### Core Concepts
- **Nodes**: Independent programs
- **Topics**: Publish/subscribe messaging
- **Services**: Request/response patterns
- **Actions**: Long-running tasks with feedback
- **Parameters**: Configuration values

## Simulation Environments

### Gazebo
- **Type**: 3D rigid-body physics simulator
- **Use Case**: Testing robot control algorithms
- **Integration**: Native ROS 2 support
- **Strengths**: Accurate physics, sensor simulation

### NVIDIA Isaac Sim
- **Type**: Photorealistic GPU-accelerated simulator
- **Use Case**: Training perception models, sim-to-real
- **Integration**: ROS 2, Isaac ROS
- **Strengths**: Ray-tracing, domain randomization, synthetic data generation

### Comparison
- **Gazebo**: Lightweight, open-source, faster iteration
- **Isaac Sim**: Photorealistic, better for vision, requires powerful GPU

## Control Systems

### Types of Control
1. **Open-Loop**: No feedback (e.g., timed movements)
2. **Closed-Loop**: Uses sensor feedback (e.g., PID control)
3. **Model Predictive Control** (MPC): Predicts future states
4. **Reinforcement Learning**: Learns from trial and error

### PID Control
- **P** (Proportional): React to current error
- **I** (Integral): Account for past errors
- **D** (Derivative): Predict future errors

Common in motor control, position tracking.

## Sensors

### Vision
- **RGB Cameras**: Color images
- **Depth Cameras**: 3D distance (e.g., RealSense, ZED)
- **Lidar**: Laser-based ranging (360° scans)

### Inertial
- **IMU** (Inertial Measurement Unit): Acceleration + gyroscope
- **GPS**: Outdoor positioning

### Proprioception
- **Encoders**: Measure joint angles
- **Force/Torque Sensors**: Measure contact forces

## Conversational AI for Robots

### Voice Recognition
- **Whisper** (OpenAI): Speech-to-text
- **Google Speech API**
-  **Azure Speech Services**

### Natural Language Understanding
- **GPT-4** (OpenAI): Intent recognition, command parsing
- **Dialog Management**: Multi-turn conversations

### Text-to-Speech
- **Coqui TTS** (Open-source)
- **Azure Neural TTS**
- **Google Cloud TTS**

### Integration Pattern
```
User Speech → Whisper (transcription)
     ↓
GPT-4 (understand intent + generate response)
     ↓
TTS (speak response)
     ↓
ROS 2 Action (execute robot command)
```

## Current Trends (2024-2025)

1. **Foundation Models for Robotics**
   - Vision-Language-Action (VLA) models
   - RT-2, RT-X (Google DeepMind)
   - OpenVLA (open-source)

2. **Sim-to-Real Transfer**
   - Domain randomization
   - Photorealistic simulation
   - Zero-shot policy transfer

3. **Whole-Body Control**
   - Unified frameworks for locomotion + manipulation
   - Optimization-based control

4. **Embodied AI**
   - Robots that learn from natural language instructions
   - Multimodal perception (vision + language + touch)

## Recommended Reading

- **Books**:
  - "Modern Robotics" by Lynch & Park
  - "Probabilistic Robotics" by Thrun, Burgard & Fox
  - "A Gentle Introduction to ROS" by Jason O'Kane

- **Courses**:
  - Stanford CS223A (Robotics)
  - MIT 6.832 (Underactuated Robotics)
  - The Construct (ROS 2 courses)

- **Papers**:
  - "RT-2: Vision-Language-Action Models" (Google DeepMind, 2023)
  - "Sim-to-Real Transfer" surveys
  - "Humanoid Locomotion" reviews

## Glossary

- **DOF** (Degrees of Freedom): Number of independent movements
- **URDF**: Unified Robot Description Format
- **TF**: Transform library (coordinate frames)
- **SLAM**: Simultaneous Localization and Mapping
- **MPC**: Model Predictive Control
- **LQR**: Linear Quadratic Regulator
- **ZMP**: Zero Moment Point (for balance)
