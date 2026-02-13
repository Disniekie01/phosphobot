# SO-ARM101 ROS2 Bridge: Phospho to Isaac Sim

This project bridges the Phospho teleoperation system with Isaac Sim, allowing you to control the Isaac Sim robot by mirroring the real robot's movements from the Phospho web interface.

## 🎯 Overview

The system consists of:
- **Phospho HTTP Teleop Node**: Reads real robot joint positions from Phospho API
- **ROS2 Topic Relays**: Bridges topics between phospho teleop and Isaac Sim
- **Isaac Sim**: Receives joint commands and mirrors the real robot's movements

## 📋 Prerequisites

- ROS2 Humble
- Isaac Sim (with ROS2 bridge enabled)
- Python 3.10+
- Network access to Phospho API (192.168.1.97:8020)

## 🚀 Quick Start

### 1. Build the ROS2 Workspace

```bash
# Navigate to the workspace
cd /home/celeste/so-arm101-ros2-bridge

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

### 2. Start the Phospho HTTP Teleop Node

```bash
# Activate conda environment (if using)
conda activate lerobot

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start the phospho HTTP teleop node
python3 src/phospho_teleop/phospho_teleop/phospho_http_teleop.py
```

### 3. Start the Topic Relays

In a new terminal:

```bash
# Navigate to workspace
cd /home/celeste/so-arm101-ros2-bridge

# Activate conda environment
conda activate lerobot

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Start the relay script
./src/phospho_teleop/scripts/setup_isaac_relays.sh
```

Or manually start the relay:

```bash
ros2 run topic_tools relay /joint_states /isaac_joint_command
```

### 4. Verify the System

Check if everything is working:

```bash
# List running nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor joint commands being sent to Isaac Sim
ros2 topic echo /isaac_joint_command

# Check if Isaac Sim is receiving commands
ros2 topic info /isaac_joint_command
```

## 🔧 How It Works

### Data Flow

1. **Phospho API** → **Phospho HTTP Teleop Node** → **ROS2 Topics** → **Topic Relays** → **Isaac Sim**

### Key Components

#### 1. Phospho HTTP Teleop Node (`phospho_http_teleop.py`)

- **Purpose**: Reads real robot joint positions from Phospho API
- **API Endpoint**: `http://192.168.1.97:8020/joints/read`
- **Publishes**: `/joint_states` (JointState messages)
- **Update Rate**: 10 Hz (every 0.1 seconds)

#### 2. Topic Relays

- **Purpose**: Bridge topics between phospho teleop and Isaac Sim
- **Main Relay**: `/joint_states` → `/isaac_joint_command`
- **Additional Relays**: Velocity, pose, and gripper commands

#### 3. Isaac Sim Integration

- **Receives**: `/isaac_joint_command` (JointState messages)
- **Joint Names**: `Rotation`, `Pitch`, `Elbow`, `Wrist_Pitch`, `Wrist_Roll`, `Jaw`
- **Mirrors**: Real robot's joint positions in simulation

## 🎮 Usage

### Controlling the Robot

1. **Start the system** (steps 1-3 above)
2. **Open Phospho web interface** in your browser
3. **Control the real robot** using the web interface
4. **Watch Isaac Sim** - the simulated robot will mirror the real robot's movements

### Monitoring

```bash
# Monitor joint commands
ros2 topic echo /isaac_joint_command

# Monitor phospho teleop logs
# (check the terminal running phospho_http_teleop.py)

# Check system status
ros2 node list
ros2 topic list
```

## 🔍 Troubleshooting

### Common Issues

#### 1. Phospho API Connection Failed
```
[WARN] Error polling phospho: Connection refused
```
**Solution**: Ensure Phospho API is running at `192.168.1.97:8020`

#### 2. Isaac Sim Not Moving
```
OmniGraph Warning: 'joint1'
```
**Solution**: The system uses descriptive joint names. Ensure Isaac Sim is configured to accept these names.

#### 3. Topic Relay Not Working
```
ros2 topic info /isaac_joint_command
# Shows no subscribers
```
**Solution**: Restart the topic relay:
```bash
pkill -f relay
ros2 run topic_tools relay /joint_states /isaac_joint_command
```

#### 4. ROS2 Context Errors
```
RCLError: failed to shutdown: rcl_shutdown already called
```
**Solution**: This is normal when stopping the node with Ctrl+C. Just restart the node.

### Debugging Commands

```bash
# Check if phospho API is accessible
curl -s http://192.168.1.97:8020/status

# Check real robot joint positions
curl -s -X POST http://192.168.1.97:8020/joints/read -H "Content-Type: application/json" -d '{}'

# Monitor all topics
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /isaac_joint_command

# Check running processes
ps aux | grep -E "(phospho|relay)"
```

## 📁 File Structure

```
so-arm101-ros2-bridge/
├── src/
│   └── phospho_teleop/
│       ├── phospho_teleop/
│       │   └── phospho_http_teleop.py    # Main teleop node
│       └── scripts/
│           └── setup_isaac_relays.sh      # Relay setup script
├── install/                               # Built packages
└── README.md                             # This file
```

## ⚙️ Configuration

### Phospho API Settings

- **URL**: `http://192.168.1.97:8020`
- **Update Rate**: 10 Hz
- **Timeout**: 1.0 seconds

### Joint Names

The system uses these joint names (matching Isaac Sim expectations):
- `Rotation` (Base rotation)
- `Pitch` (Shoulder pitch)
- `Elbow` (Elbow joint)
- `Wrist_Pitch` (Wrist pitch)
- `Wrist_Roll` (Wrist roll)
- `Jaw` (Gripper)

### Topics

- **Input**: `/joint_states` (from phospho teleop)
- **Output**: `/isaac_joint_command` (to Isaac Sim)
- **Relay**: `/joint_states` → `/isaac_joint_command`

## 🛑 Stopping the System

```bash
# Stop all phospho processes
pkill -f phospho

# Stop all relay processes
pkill -f relay

# Or use the relay script's cleanup
# (Ctrl+C in the relay script terminal)
```

## 📝 Notes

- The system polls the Phospho API every 0.1 seconds for real-time mirroring
- Joint positions are directly mirrored from real robot to Isaac Sim
- The system is designed for the SO-ARM101 robot configuration
- Ensure Isaac Sim is running and the ROS2 bridge is enabled
- Network connectivity to the Phospho API is required

## 🤝 Contributing

To modify the system:
1. Edit `src/phospho_teleop/phospho_teleop/phospho_http_teleop.py`
2. Rebuild: `colcon build`
3. Restart the nodes

## 📞 Support

If you encounter issues:
1. Check the troubleshooting section above
2. Verify network connectivity to Phospho API
3. Ensure Isaac Sim is properly configured
4. Check ROS2 topic connectivity with `ros2 topic list` 