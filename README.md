# Autonomous Driving Preparation

Preparation project for UQ Winter Research 2025 - Developing an autonomous driving platform with the Errol vehicle.

## Project Overview

This repository contains practice projects building up to autonomous vehicle development skills:
- C++ programming with sensor data processing
- ROS2 fundamentals (nodes, topics, publishers, subscribers)
- Autonomous vehicle concepts (perception, planning, control)
- Parking space detection and trajectory planning
- **Autoware.auto integration with vehicle control interfaces**

## Research Project Goals

1. **Data Collection**: Collect and publish datasets for autonomous driving development
2. **Valet Parking**: Implement autonomous valet parking in Autoware.auto
3. **Vehicle Control**: Develop steering/speed controllers

## Current Progress

### ✅ Completed (June 2025)

1. **Environment Setup**
   - Ubuntu 24.04 with ROS2 Jazzy (not Humble - important compatibility note)
   - Autoware.auto message packages successfully built
   - Clean workspace structure established

2. **Autoware.auto Analysis**
   - Studied vehicle interface architecture
   - Identified message structures (VehicleControlCommand, GearCommand, VehicleStateCommand)
   - Discovered no existing parking implementation - will build from scratch

3. **Working Implementation**
   - Built three functional parking controllers:
     - Simple ROS2 test node
     - Standard parking controller with state machine
     - **Autoware-compatible parking controller using actual Autoware messages**
   - Successfully tested complete parking sequence with proper state transitions

## Project Structure

```
autonomous-driving-prep-clean/
├── ros2_ws/                          # ROS2 workspace
│   └── src/
│       ├── parking_demo/             # Parking implementation
│       │   ├── src/
│       │   │   ├── simple_parking_test.cpp
│       │   │   ├── parking_controller.cpp
│       │   │   └── autoware_parking_controller.cpp  # Autoware-compatible
│       │   ├── CMakeLists.txt
│       │   └── package.xml
│       └── autoware_auto_msgs/       # Autoware message definitions
└── cpp-practice/                     # Non-ROS C++ practice
    ├── vehicle_dynamics.cpp
    └── dubins_path.cpp
```

## Key Achievements

### Autoware Vehicle Interface Integration

Successfully implemented parking controller using Autoware's standard messages:

```cpp
// Vehicle control with proper Autoware messages
autoware_auto_vehicle_msgs::msg::VehicleControlCommand cmd;
cmd.velocity_mps = -0.5;              // Reverse velocity
cmd.long_accel_mps2 = 0.0;            // Acceleration
cmd.front_wheel_angle_rad = -0.5;     // Steering angle

// Gear control
autoware_auto_vehicle_msgs::msg::GearCommand gear;
gear.command = GearCommand::REVERSE;  // or PARK

// State control (lights, brake)
autoware_auto_vehicle_msgs::msg::VehicleStateCommand state;
state.blinker = VehicleStateCommand::BLINKER_HAZARD;
state.hand_brake = true;
```

### Parking Sequence Implementation

Complete parallel parking maneuver with:
- **Preparation phase**: Hazard lights, shift to reverse
- **Phase 1**: Reverse with right steering (3 seconds)
- **Phase 2**: Reverse with left steering (3 seconds)
- **Phase 3**: Straighten (2 seconds)
- **Completion**: Shift to park, apply parking brake

## How to Run

### Prerequisites
```bash
# Install ROS2 Jazzy (Ubuntu 24.04)
sudo apt install ros-jazzy-desktop

# Source ROS2
source /opt/ros/jazzy/setup.bash
```

### Build and Run
```bash
# Clone repository
git clone https://github.com/yourusername/autonomous-driving-prep.git
cd autonomous-driving-prep/autonomous-driving-prep-clean

# Build workspace
cd ros2_ws
colcon build
source install/setup.bash

# Run Autoware parking controller
ros2 run parking_demo autoware_parking_controller

# In another terminal, trigger parking
ros2 topic pub --once /parking_goal geometry_msgs/msg/PoseStamped "{}"

# Monitor outputs
ros2 topic echo /vehicle_control_command
ros2 topic echo /gear_command
ros2 topic echo /vehicle_state_command
```

## Technical Details

### Message Compatibility
- Using ROS2 Jazzy (newer than Autoware.auto's target)
- Successfully built Autoware IDL message definitions
- Confirmed compatibility with vehicle interface patterns

### Control Architecture
- 50Hz control loop (matching Autoware standards)
- State machine with proper transitions
- Safety features integrated (timeouts, brake application)

## Questions for Project Supervisor

1. **Environment Compatibility**: Errol development uses ROS2 Humble or Jazzy?
2. **Message Formats**: Standard Autoware messages or custom modifications?
3. **CAN Protocol**: Specific IDs and message formats for Errol?
4. **Development Setup**: Docker image or specific environment configuration?
5. **Existing Code**: Access to Errol's current codebase?

## Next Steps

### Before Project Start
- [x] Install and configure ROS2
- [x] Study Autoware.auto architecture
- [x] Build working parking controller
- [x] Test with Autoware messages
- [ ] Get answers about Errol's specific configuration
- [ ] Access existing codebase

### Week 1 Priorities
1. Confirm development environment setup
2. Integrate with Errol's actual vehicle interface
3. Test communication with hardware
4. Begin data collection framework

## Learning Resources Used

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Autoware.auto Documentation](https://autowarefoundation.github.io/autoware.auto/AutowareAuto/)
- Autoware.auto source code analysis (GitLab)

## Contact

**Supervisor**: Dr Tyson Phillips  
**Email**: t.phillips1@uq.edu.au  
**Project Duration**: 4 weeks (30 June - 25 July 2025)

---

Project developed by Isaac Johan Ziebarth for UQ Winter Research 2025

*Last updated: June 2025*