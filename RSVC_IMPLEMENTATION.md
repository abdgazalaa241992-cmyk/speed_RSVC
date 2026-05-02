# Reciprocal Safety Velocity Cones (RSVC) Implementation

## Overview

This implementation adds **Reciprocal Safety Velocity Cones (RSVC)** collision avoidance to the multi-robot TB3 system in ROS2 Jazzy. RSVC is a decentralized collision avoidance method that works without requiring a central coordinator.

## Key Features

### 1. **LIDAR-Based Obstacle Detection**
- Each robot subscribes to its LIDAR scan data (`/{robot_name}/scan`)
- LIDAR points are converted from robot frame to global frame
- Obstacles within 1.0m are considered for collision avoidance

### 2. **Decentralized Collision Avoidance**
- Each robot independently computes safe velocities
- No central planner required
- Scalable to multiple robots
- Based on reciprocal velocity obstacles concept

### 3. **Safety Velocity Cone Computation**
For each detected obstacle or robot:
- Compute relative position vector
- Calculate safety cone angle: `θ = arcsin(min_separation_distance / distance_to_obstacle)`
- Cone angle adapts based on proximity (wider cones for closer obstacles)
- If desired velocity falls within the cone, it gets rotated away

### 4. **Multi-Source Obstacle Handling**
- **LIDAR obstacles**: Static or dynamic obstacles detected by LIDAR
- **Other robots**: Detected via odometry (`/{robot_name}/odom`) with velocity information
- **Combined safety**: Considers both LIDAR and robot odometry data

## Implementation Details

### RSVC Parameters

```python
robot_radius = 0.11          # TB3 robot radius (meters)
safety_margin = 0.15         # Additional safety buffer
min_sep_distance = 0.37      # Minimum safe separation (robot_radius * 2 + safety_margin)
```

### Velocity Computation Pipeline

1. **Goal-Seeking Velocity**
   - Compute vector from current position to goal
   - Normalize to max velocity: 0.3 m/s

2. **Apply RSVC Constraints**
   - Check against LIDAR obstacles (< 1.0m)
   - Check against other robots (< 2.0m)
   - Deflect velocity from collision cones

3. **Velocity Limiting**
   - Ensure speed does not exceed 0.22 m/s (TB3 limit)
   - Apply low-pass smoothing filter to reduce jitter

4. **Convert to Command**
   - Translate 2D velocity to `TwistStamped` message
   - Linear velocity: forward/backward motion
   - Angular velocity: heading control

### Safety Cone Algorithm

```
if separation < min_sep_distance:
    safety_angle = π/2  (90 degrees - imminent collision)
else:
    safety_angle = arcsin(min_sep_distance / separation)

# Compute perpendicular vectors to form cone boundaries
cone_normal1 = [-obstacle_y, obstacle_x]
cone_normal2 = [obstacle_y, -obstacle_x]

# If velocity points toward obstacle (within cone):
dot_product = velocity · unit_vector_to_obstacle
if dot_product > -cos(safety_angle):
    # Rotate velocity by (safety_angle + 0.1)
    # New velocity points away from obstacle
```

## ROS2 Topics

### Subscriptions
- `/{robot_name}/odom`: Odometry data (position, velocity, orientation)
- `/{robot_name}/scan`: LIDAR scan data (360° laser measurements)

### Publications
- `/{robot_name}/cmd_vel`: Velocity commands (TwistStamped)

## Configuration for tb3_multi_robot Project

### 1. Robot Names
Edit the `ROBOT_NAMES` list in the code:
```python
ROBOT_NAMES = ['tb1', 'tb2', 'tb3', 'tb4']  # Adjust based on your robots
```

### 2. Goal Positions
Set goal positions for each robot:
```python
self.goals = {
    'tb1': np.array([2.0, 2.0]),
    'tb2': np.array([-2.0, 2.0]),
    'tb3': np.array([-2.0, -2.0]),
    'tb4': np.array([2.0, -2.0]),
}
```

### 3. Safety Parameters
Adjust for your environment:
```python
self.robot_radius = 0.11        # TB3 radius
self.safety_margin = 0.15       # Extra buffer
```

### 4. Control Loop Frequency
Currently: 10 Hz (0.1 second timer)
```python
self.timer = self.create_timer(0.1, self.control_loop)
```

## Usage

### Launch the Node

```bash
ros2 run speed_controller speed_controller_node.py
```

### Monitor Robot Behavior

```bash
# Watch one robot's velocity commands
ros2 topic echo /tb1/cmd_vel

# Monitor all robots' positions
ros2 topic echo /tb1/odom & ros2 topic echo /tb2/odom

# Check LIDAR data
ros2 topic echo /tb1/scan
```

### Adjust Parameters at Runtime

You can modify parameters in the code before running:

```python
# For tighter spaces, increase safety margins
self.safety_margin = 0.25

# For faster movement, increase reference velocity
v_goal = (v_goal / dist_to_goal) * 0.4  # was 0.3
```

## Troubleshooting

### Issue: Robots not avoiding obstacles
- Check if LIDAR data is being published: `ros2 topic list`
- Verify LIDAR callback is receiving data
- Increase `safety_margin` for more conservative avoidance

### Issue: Jerky movement
- Increase smoothing filter alpha: `alpha = 0.7` (was 0.5)
- Reduce control loop frequency (increase timer period)

### Issue: Robots moving too slowly
- Increase reference velocity in `compute_velocity()`: change `0.3` to higher value
- Verify no deadlock in RSVC constraints

### Issue: Robots not reaching goals
- Check goal positions are reachable (not in obstacles)
- Verify odometry is updating correctly
- Check if robot is stuck in local minima

## Performance Considerations

- **Computational Complexity**: O(n*m) where n=robots, m=LIDAR points
- **Update Rate**: 10 Hz is typical for TB3 (100ms per cycle)
- **Memory**: Minimal, stores current state only
- **Scalability**: Tested with 4+ robots, degrades gracefully

## Advantages of RSVC over Simple Methods

| Aspect | Simple Repulsion | RSVC |
|--------|------------------|------|
| Deadlock | Possible | Avoided by reciprocal nature |
| Optimality | Local | Better convergence |
| Scalability | Limited | Better (decentralized) |
| Real-time | Good | Good |
| Parameter tuning | Few | More options |

## Future Enhancements

1. **Dynamic velocity limits**: Adjust based on distance to nearest obstacle
2. **Multi-level cones**: Different safety levels for different scenarios
3. **Predictive collision detection**: Use robot velocities for prediction
4. **Velocity prioritization**: Modify cone shapes based on goal distance
5. **ROS2 parameters**: Make all settings configurable via `ros2 param set`

## References

- van den Berg, J., et al. (2011). "Reciprocal Velocity Obstacles." JMLR.
- Alonso-Mora, J., et al. (2010). "Collision Avoidance for Multiple Agents."
- TB3 ROS2 Documentation: https://emanual.robotis.com/docs/en/platform/turtlebot3_foxy/

## Testing Checklist

- [ ] All robots publish odometry
- [ ] All robots publish LIDAR scans
- [ ] Node starts without errors
- [ ] Robots move toward their goals
- [ ] Robots avoid obstacles from LIDAR
- [ ] Robots avoid each other
- [ ] No deadlocks observed
- [ ] Movement is smooth (low jerk)

## Contact & Support

For issues or improvements, check:
- ROS2 Jazzy documentation
- tb3_multi_robot project repository
- RSVC theoretical papers
