# TB3 Multi-Robot Setup with RSVC

## System Overview

This document provides specific setup instructions for using RSVC with the tb3_multi_robot project in ROS2 Jazzy.

## Requirements

```bash
# ROS2 Jazzy
# Turtlebot3 simulation or hardware
# python3-pip

# Required packages:
pip install numpy
```

## Installation Steps

### 1. Navigate to Speed Controller Package
```bash
cd ~/robot_ws/src/speed_controller/
```

### 2. Ensure Package Structure
```
speed_controller/
├── speed_controller/
│   ├── __init__.py
│   ├── speed_controller_node.py  (MODIFIED with RSVC)
│   └── speed_controller_node - Copy.py
├── RSVC_IMPLEMENTATION.md
├── RSVC_CONFIG_GUIDE.md
└── TB3_MULTI_ROBOT_SETUP.md
```

### 3. Build ROS2 Package
```bash
cd ~/robot_ws
colcon build --packages-select speed_controller
source install/setup.bash
```

## Launch Configuration for tb3_multi_robot

### Option A: Real Robots

Ensure your robots are connected to the network and running:

```bash
# On each TB3 robot's SBC:
ros2 launch turtlebot3_bringup robot.launch.py

# On your main computer:
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
ros2 run speed_controller speed_controller_node.py
```

### Option B: Gazebo Simulation

```bash
# Launch tb3_multi_robot simulation
ros2 launch turtlebot3_gazebo multi_robot_spawn.launch.py

# In another terminal, run speed_controller
export ROS_DOMAIN_ID=0
ros2 run speed_controller speed_controller_node.py
```

### Option C: Create Custom Launch File

Create `multi_robot_rsvc.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    # Speed controller with RSVC
    speed_controller = Node(
        package='speed_controller',
        executable='speed_controller_node.py',
        name='multi_robot_rsvc',
        output='screen',
    )
    
    ld = LaunchDescription([
        speed_controller,
    ])
    
    return ld
```

Launch with:
```bash
ros2 launch speed_controller multi_robot_rsvc.launch.py
```

## Verification Checklist

### Check Node is Running
```bash
ros2 node list
# Should show: /multi_robot_rsvc
```

### Check Topic Connections
```bash
# Check subscriptions
ros2 node info /multi_robot_rsvc

# Should see:
# Subscribers:
#   /tb1/odom
#   /tb1/scan
#   /tb2/odom
#   /tb2/scan
#   /tb3/odom
#   /tb3/scan
#   /tb4/odom
#   /tb4/scan
# Publishers:
#   /tb1/cmd_vel
#   /tb2/cmd_vel
#   /tb3/cmd_vel
#   /tb4/cmd_vel
```

### Monitor Robot Motion
```bash
# Watch TB1's velocity commands
ros2 topic echo /tb1/cmd_vel

# Watch TB1's position
ros2 topic echo /tb1/odom

# Check LIDAR detection
ros2 topic echo /tb1/scan --field ranges
```

## Configuration for tb3_multi_robot

### Default Configuration (Simulation)

```python
# In speed_controller_node.py

ROBOT_NAMES = ['tb1', 'tb2', 'tb3', 'tb4']

self.goals = {
    'tb1': np.array([2.0, 2.0]),
    'tb2': np.array([-2.0, 2.0]),
    'tb3': np.array([-2.0, -2.0]),
    'tb4': np.array([2.0, -2.0]),
}

self.robot_radius = 0.11
self.safety_margin = 0.15
```

### For Real TB3 Robots

TB3 Burger Specifications:
- Robot radius: ~0.105m (bumper edge)
- LIDAR range: 0.12m - 3.5m
- Max linear velocity: 0.26 m/s
- Max angular velocity: 2.84 rad/s

Recommended settings:
```python
self.robot_radius = 0.105
self.safety_margin = 0.20  # More conservative for real hardware
```

## Troubleshooting

### Issue: Node starts but no robots move
```bash
# Check if topics exist
ros2 topic list | grep odom
ros2 topic list | grep scan
ros2 topic list | grep cmd_vel

# If missing, launch tb3_multi_robot first
```

### Issue: Robots not communicating
```bash
# Check ROS domain ID
echo $ROS_DOMAIN_ID  # Should be 0 for simulation, 30 for real robots

# Set domain ID
export ROS_DOMAIN_ID=30
```

### Issue: Stuck or deadlocked robots
```bash
# Restart the node
ros2 run speed_controller speed_controller_node.py

# Or kill and restart
pkill -f speed_controller_node.py
ros2 run speed_controller speed_controller_node.py
```

### Issue: LIDAR not publishing
```bash
# Check if LIDAR node is running
ros2 node list | grep lidar

# Check scan topic
ros2 topic echo /tb1/scan

# May need to publish on different namespace
```

## Performance Monitoring

### CPU and Memory Usage
```bash
# Monitor resource usage
top -p $(pidof -x speed_controller_node.py)
```

### Latency Analysis
```bash
# Check message delays
ros2 topic hz /tb1/odom
ros2 topic hz /tb1/scan
ros2 topic hz /tb1/cmd_vel
```

### Safety Margin Verification
```bash
# Monitor minimum distances between robots
ros2 run rqt_plot rqt_plot  # Plot distances over time
```

## Advanced Usage

### Recording Bag File
```bash
# Record all robot data
ros2 bag record -a -o multi_robot_rsvc_session

# Then analyze:
ros2 bag play multi_robot_rsvc_session
```

### Real-time Visualization
```bash
# Launch RViz
rviz2

# Add displays:
# - Odometry (for positions)
# - LaserScan (for LIDAR)
# - TF (for frame transforms)
# - Markers (for goals)
```

### Modify Goals Dynamically
Edit the goals dictionary and rerun:
```python
self.goals = {
    'tb1': np.array([3.0, 3.0]),    # Changed goal
    'tb2': np.array([-3.0, 3.0]),
    'tb3': np.array([-3.0, -3.0]),
    'tb4': np.array([3.0, -3.0]),
}
```

## Multi-Goal Navigation

For sequential waypoint following, modify `compute_velocity()`:

```python
def compute_velocity(self, robot_name):
    # Add waypoint tracking
    if not hasattr(self, 'waypoints'):
        self.waypoints = {name: [] for name in ROBOT_NAMES}
        self.current_waypoint_idx = {name: 0 for name in ROBOT_NAMES}
    
    pos = self.positions[robot_name]
    waypoints = self.waypoints[robot_name]
    
    if len(waypoints) == 0:
        # Use default goals
        goal = self.goals[robot_name]
    else:
        idx = self.current_waypoint_idx[robot_name]
        goal = waypoints[idx % len(waypoints)]
        
        # Check if reached waypoint
        if np.linalg.norm(goal - pos) < 0.2:
            self.current_waypoint_idx[robot_name] += 1
    
    # Rest of the method...
```

## Expected Behavior

### Normal Operation
- Robots start at origin (0, 0, 0)
- Each robot moves toward its goal
- When robots get close, RSVC activates
- Robots smoothly diverge to avoid collision
- Robots continue to their goals

### Collision Avoidance
- Robots detect each other at ~2.0m
- Safety cones form around each robot
- Velocities are rotated away from cones
- Collisions are prevented
- Motion is smooth (due to smoothing filter)

### Goal Reaching
- Once obstacle is passed, robots return to goal-seeking
- Robots may take longer paths but avoid collisions
- Eventually reach their designated goals

## Simulation vs. Reality

| Aspect | Simulation | Real Hardware |
|--------|-----------|--------------|
| Latency | Low (usually <50ms) | Higher (100-500ms) |
| Noise | None/minimal | Significant |
| LIDAR range | Accurate | May vary |
| Odometry | Perfect | Drifts over time |
| Initial position | Exact | Approximate |
| Recommended safety_margin | 0.15m | 0.20m |
| Max velocity | 0.4 m/s | 0.26 m/s |

## Next Steps

1. **Simulate**: Test in Gazebo first
2. **Configure**: Adjust parameters per environment
3. **Test**: Run in real robots with safety nets
4. **Optimize**: Fine-tune for your application
5. **Deploy**: Run on production robots

## References

- [RSVC Implementation Details](RSVC_IMPLEMENTATION.md)
- [Configuration Guide](RSVC_CONFIG_GUIDE.md)
- [TB3 ROS2 Docs](https://emanual.robotis.com/docs/en/platform/turtlebot3_foxy/)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)

---

## Support

For issues:
1. Check the troubleshooting section above
2. Review configuration guide for parameter tuning
3. Check ROS2 logs: `ros2 run rqt_console rqt_console`
4. Verify all topics are publishing: `ros2 topic list -t`
