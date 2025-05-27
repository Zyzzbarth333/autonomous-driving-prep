# Autonomous Driving Preparation

Preparation project for UQ Winter Research 2025 - Developing an autonomous driving platform with the Errol vehicle.

## Project Overview

This repository contains practice projects building up to autonomous vehicle development skills:
- C++ programming with sensor data processing
- ROS2 fundamentals (nodes, topics, publishers, subscribers)
- Autonomous vehicle concepts (perception, planning, control)
- Parking space detection and trajectory planning

## Research Project Goals

1. **Data Collection**: Collect and publish datasets for autonomous driving development
2. **Valet Parking**: Implement autonomous valet parking in Autoware.auto
3. **Vehicle Control**: Develop steering/speed controllers

## Project Structure

```
autonomous-driving-prep/
├── cpp-basics/
│   ├── sensor_stats.cpp          # Basic statistics calculator
│   ├── sensor_simulator.cpp      # CSV sensor data generator
│   └── data_processor.cpp        # CSV reader with anomaly detection
└── ros2-tutorials/
    └── sensor_publisher/
        ├── src/
        │   ├── lidar_simulator.cpp            # Simulated lidar publisher
        │   ├── lidar_processor.cpp            # Obstacle detection
        │   ├── obstacle_avoider.cpp           # Velocity control
        │   ├── parking_planner.cpp            # Parking space detection
        │   └── integrated_parking_controller.cpp # Complete parking system
        └── launch/
            ├── autonomous_demo.launch.py      # Basic demo
            └── parking_demo.launch.py         # Parking demo
```

## Setup Instructions

### 1. Install Prerequisites
```bash
# Install Ubuntu 22.04 (WSL2 or dual boot)
# Then install ROS2 Humble:
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

### 2. Clone and Build
```bash
# Clone this repository
git clone https://github.com/Zyzzbarth333/autonomous-driving-prep.git
cd autonomous-driving-prep

# Build C++ examples
cd cpp-basics
g++ -o sensor_stats sensor_stats.cpp -std=c++17
g++ -o sensor_simulator sensor_simulator.cpp -std=c++17 -pthread
g++ -o data_processor data_processor.cpp -std=c++17

# Build ROS2 package
mkdir -p ~/ros2_ws/src
cp -r ros2-tutorials/sensor_publisher ~/ros2_ws/src/
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## How to Run

### C++ Programs

```bash
# Generate sensor data
./sensor_simulator
# Enter: 100

# Process sensor data
./data_processor

# Calculate statistics
./sensor_stats
# Enter values, then Ctrl+D
```

### ROS2 Nodes

#### Individual Nodes
```bash
# Terminal 1: Lidar simulation
ros2 run sensor_publisher lidar_simulator

# Terminal 2: Obstacle detection
ros2 run sensor_publisher lidar_processor

# Terminal 3: Velocity control
ros2 run sensor_publisher obstacle_avoider
```

#### Launch Files
```bash
# Launch complete system
ros2 launch sensor_publisher autonomous_demo.launch.py

# Launch parking demo
ros2 launch sensor_publisher parking_demo.launch.py
```

#### Monitor System
```bash
# View node graph
ros2 run rqt_graph rqt_graph

# View topics
ros2 topic list

# Monitor specific topic
ros2 topic echo /scan
ros2 topic echo /cmd_vel
ros2 topic echo /parking_state
```

## Key Concepts Implemented

### 1. Sensor Data Pipeline
- Timestamp generation with millisecond precision
- CSV file I/O for data logging
- Noise simulation (Gaussian distribution)
- Data anomaly detection

### 2. ROS2 Architecture
- Publisher/Subscriber pattern
- Message types: LaserScan, Twist, Path
- Launch files for system startup
- 10Hz control loops

### 3. Autonomous Behaviors
- Obstacle detection and avoidance
- Parking space identification
- State machine control (SEARCHING → ALIGNING → PARKING → COMPLETE)
- Velocity command generation

## Advanced Enhancements (TODO)

### C++ Enhancements

#### sensor_stats.cpp
- [ ] Add histogram generation for data distribution
- [ ] Implement rolling statistics (last N samples)
- [ ] Add outlier detection using IQR method
- [ ] Export results in JSON format

#### sensor_simulator.cpp
- [ ] Add multiple sensor types (GPS, IMU, wheel encoders)
- [ ] Implement sensor dropout simulation
- [ ] Add configurable noise models per sensor
- [ ] Create realistic motion patterns (sine wave, figure-8)

#### data_processor.cpp
- [ ] Implement Kalman filter for data smoothing
- [ ] Add velocity and acceleration calculations
- [ ] Create data visualization exports for plotting
- [ ] Implement sensor fusion from multiple sources

### ROS2 Enhancements

#### lidar_simulator.cpp
- [ ] Add dynamic obstacles that move
- [ ] Implement multiple obstacle types
- [ ] Add sensor noise based on distance
- [ ] Simulate real lidar patterns (Velodyne, Ouster)

#### obstacle_avoider.cpp
- [ ] Implement PID control for smooth velocity changes
- [ ] Add lateral avoidance (not just stop/turn)
- [ ] Implement dynamic window approach (DWA)
- [ ] Add emergency stop functionality

#### parking_planner.cpp
- [ ] Add perpendicular parking
- [ ] Implement parallel parking trajectory
- [ ] Add A* or RRT* path planning
- [ ] Consider vehicle kinematics (Ackermann steering)

#### integrated_parking_controller.cpp
- [ ] Add recovery behaviors for failed parking
- [ ] Implement multiple parking attempt logic
- [ ] Add obstacle checking during parking
- [ ] Create smooth trajectory following using Pure Pursuit

### System-Level Enhancements
- [ ] Create Gazebo simulation environment
- [ ] Add RVIZ2 visualization configs
- [ ] Implement parameter files for tuning
- [ ] Add unit tests for critical functions
- [ ] Create bag file recording/playback
- [ ] Implement service-based parking requests

## Control Theory Concepts to Implement

### PID Controller Template
```cpp
class PIDController {
private:
    double kp_, ki_, kd_;
    double error_sum_, last_error_;
    double max_integral_;
    
public:
    double calculate(double setpoint, double current) {
        double error = setpoint - current;
        error_sum_ += error;
        
        // Clamp integral term
        error_sum_ = std::clamp(error_sum_, -max_integral_, max_integral_);
        
        double derivative = error - last_error_;
        last_error_ = error;
        
        return kp_ * error + ki_ * error_sum_ + kd_ * derivative;
    }
};
```

### Moving Average Filter
```cpp
class MovingAverageFilter {
private:
    std::vector<double> buffer_;
    size_t window_size_;
    size_t index_ = 0;
    bool filled_ = false;
    
public:
    double filter(double value) {
        buffer_[index_] = value;
        index_ = (index_ + 1) % window_size_;
        if (index_ == 0) filled_ = true;
        
        size_t count = filled_ ? window_size_ : index_;
        return std::accumulate(buffer_.begin(), buffer_.begin() + count, 0.0) / count;
    }
};
```

## Useful Commands Reference

### ROS2 Commands
```bash
# Package management
ros2 pkg list
ros2 pkg executables sensor_publisher

# Topic inspection
ros2 topic list -t          # List with types
ros2 topic info /scan       # Show pub/sub count
ros2 topic hz /scan         # Show publish rate
ros2 topic bw /scan         # Show bandwidth

# Node inspection
ros2 node list
ros2 node info /lidar_simulator

# Parameter management
ros2 param list /node_name
ros2 param get /node_name param_name

# Service calls
ros2 service list
ros2 service call /service_name
```

### Development Commands
```bash
# Clean build
cd ~/ros2_ws
rm -rf build install log
colcon build

# Build single package
colcon build --packages-select sensor_publisher

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Run with GDB
ros2 run --prefix 'gdb -ex run --args' sensor_publisher lidar_simulator
```

## Learning Resources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Autoware.auto Documentation](https://autowarefoundation.github.io/autoware.auto/AutowareAuto/)
- [Modern C++ Features](https://github.com/AnthonyCalandra/modern-cpp-features)
- [PID Control Explained](https://en.wikipedia.org/wiki/PID_controller)

## Next Steps

### Before Starting (Priority Tasks)
- [ ] **Study Autoware.auto architecture**
  - [ ] Read the [Autoware.auto documentation](https://autowarefoundation.github.io/autoware.auto/AutowareAuto/)
  - [ ] Understand the parking module structure
  - [ ] Review the perception-planning-control pipeline

- [ ] **Practice with real sensor data**
  - [ ] Download [KITTI dataset](http://www.cvlibs.net/datasets/kitti/) samples
  - [ ] Modify `data_processor.cpp` to handle real lidar data
  - [ ] Test anomaly detection on real-world noise

- [ ] **Learn vehicle kinematics**
  - [ ] Study Ackermann steering geometry
  - [ ] Implement bicycle model kinematics
  - [ ] Add turning radius constraints to parking planner

- [ ] **Master coordinate transformations**
  - [ ] Complete [TF2 tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
  - [ ] Add frame transformations to your nodes
  - [ ] Practice with `laser_frame` to `base_link` transforms

### Week 1 Preparation Checklist
- [ ] **Environment Setup**
  - [ ] Ensure dual-boot Ubuntu works reliably (not just WSL)
  - [ ] Install Autoware dependencies
  - [ ] Set up IDE with C++ debugging

- [ ] **C++ Skills**
  - [ ] Implement at least 2 advanced enhancements from above
  - [ ] Practice debugging with GDB
  - [ ] Review modern C++ features (smart pointers, lambdas)

- [ ] **ROS2 Skills**
  - [ ] Create a service for parking requests
  - [ ] Implement parameter loading from YAML
  - [ ] Practice with ROS2 bag files

- [ ] **Control Theory**
  - [ ] Implement PID controller in obstacle avoider
  - [ ] Test different PID gains
  - [ ] Add derivative filtering

### Quick Daily Practice (15 mins/day)
- [ ] Monday: Read one Autoware module source code
- [ ] Tuesday: Implement one enhancement from TODO list
- [ ] Wednesday: Practice Git workflows (branching, merging)
- [ ] Thursday: Debug a ROS2 node with GDB
- [ ] Friday: Review research papers on autonomous parking

### Questions to Prepare
- [ ] Draft 5 technical questions about the Errol platform
- [ ] Prepare questions about Autoware.auto architecture
- [ ] List areas where you need clarification

### Optional Advanced Preparation
- [ ] **Simulation**
  - [ ] Install Gazebo Fortress
  - [ ] Create simple robot model
  - [ ] Test your controllers in simulation

- [ ] **Algorithms**
  - [ ] Implement A* path planning
  - [ ] Code RRT for parking trajectories
  - [ ] Study SLAM basics

- [ ] **Hardware Understanding**
  - [ ] Research Ouster/Velodyne lidar specs
  - [ ] Understand drive-by-wire systems
  - [ ] Learn CAN bus basics

## Contact

**Supervisor**: Dr Tyson Phillips  
**Email**: t.phillips1@uq.edu.au

---

Project developed by Isaac Johan Ziebarth for UQ Winter Research 2025
