# RSVC Configuration Guide

## Quick Start Configuration

### Scenario 1: 4 Robots in Open Space (Default)
```python
ROBOT_NAMES = ['tb1', 'tb2', 'tb3', 'tb4']

self.goals = {
    'tb1': np.array([2.0, 2.0]),
    'tb2': np.array([-2.0, 2.0]),
    'tb3': np.array([-2.0, -2.0]),
    'tb4': np.array([2.0, -2.0]),
}

self.robot_radius = 0.11
self.safety_margin = 0.15  # Conservative
```

### Scenario 2: Tight Corridor Navigation
Use **wider safety margins** and **slower speeds**:
```python
self.robot_radius = 0.11
self.safety_margin = 0.25  # Increased for safety

# In compute_velocity():
v_goal = (v_goal / dist_to_goal) * 0.2  # Reduced from 0.3
```

### Scenario 3: Crowded Environment (5+ Robots)
Increase **look-ahead distance** and **reaction time**:
```python
# In compute_safety_velocity_cones():
for obs_x, obs_y, obs_dist in obstacles:
    if obs_dist < 1.5:  # Increased from 1.0
        # Process obstacle...

# Check robots up to 2.5m away (was 2.0m)
if separation > 0.01 and separation < 2.5:
```

### Scenario 4: High-Speed Navigation
Balance speed with safety:
```python
# Increase max velocity carefully
v_goal = (v_goal / dist_to_goal) * 0.35  # Slightly faster

# But keep larger safety margin
self.safety_margin = 0.20

# Reduce smoothing (more responsive)
alpha = 0.3  # More reactive control
```

---

## Tuning Parameters

### 1. Safety Margin (`self.safety_margin`)
- **Too small**: Robots collide or graze each other
- **Too large**: Robots take very wide paths, less efficient
- **Recommended range**: 0.10 - 0.30 meters

```python
# Conservative (cluttered spaces)
self.safety_margin = 0.25

# Aggressive (open spaces)
self.safety_margin = 0.10
```

### 2. Reference Velocity (in `compute_velocity()`)
- **Too slow**: Takes forever to reach goals
- **Too fast**: May cause collisions if avoidance is not perfect
- **Recommended range**: 0.2 - 0.35 m/s for TB3

```python
# Slow, safe motion
v_goal = (v_goal / dist_to_goal) * 0.2

# Fast, risky motion
v_goal = (v_goal / dist_to_goal) * 0.4
```

### 3. Control Loop Frequency (in `__init__()`)
- **10 Hz** (0.1s): Good for smooth motion, responsive
- **5 Hz** (0.2s): Lower CPU, but less responsive
- **20 Hz** (0.05s): More responsive but higher CPU

```python
# Standard (recommended)
self.timer = self.create_timer(0.1, self.control_loop)

# Low CPU usage
self.timer = self.create_timer(0.2, self.control_loop)

# High responsiveness
self.timer = self.create_timer(0.05, self.control_loop)
```

### 4. Smoothing Filter Alpha (in `convert_to_cmd()`)
- **α = 1.0**: No smoothing, very jittery
- **α = 0.5**: Medium smoothing (default)
- **α = 0.2**: High smoothing, sluggish
- **Recommended range**: 0.3 - 0.7

```python
# Very smooth motion (sluggish)
alpha = 0.3

# Medium smoothing (balanced)
alpha = 0.5

# Responsive (jittery)
alpha = 0.7
```

### 5. LIDAR Obstacle Detection Range
In `lidar_callback()` and `compute_safety_velocity_cones()`:

```python
# Only consider nearby obstacles (< 1.0m)
if obs_dist < 1.0:

# Extended range (for faster robots)
if obs_dist < 1.5:

# Only immediate vicinity
if obs_dist < 0.8:
```

### 6. Robot Consideration Distance
In `compute_safety_velocity_cones()`:

```python
# Only avoid robots within 2.0m (default)
if separation > 0.01 and separation < 2.0:

# Wider awareness (crowded spaces)
if separation > 0.01 and separation < 2.5:

# Tight focus (open spaces)
if separation > 0.01 and separation < 1.5:
```

---

## Environment-Specific Presets

### Forest/Dense Obstacles
```python
self.robot_radius = 0.11
self.safety_margin = 0.20
self.min_sep_distance = 0.42

# In compute_velocity
v_goal = (v_goal / dist_to_goal) * 0.2
alpha = 0.4  # Smoother

# In lidar processing
if obs_dist < 0.9:  # Aggressive avoidance
```

### Open Corridor
```python
self.robot_radius = 0.11
self.safety_margin = 0.12
self.min_sep_distance = 0.34

# In compute_velocity
v_goal = (v_goal / dist_to_goal) * 0.35
alpha = 0.6  # More responsive

# In lidar processing
if obs_dist < 1.2:
```

### Warehouse/Grid Pattern
```python
self.robot_radius = 0.11
self.safety_margin = 0.18
self.min_sep_distance = 0.40

# In compute_velocity
v_goal = (v_goal / dist_to_goal) * 0.25
alpha = 0.5  # Balanced

# Multi-goal support
self.goals = {
    'tb1': np.array([5.0, 0.0]),
    'tb2': np.array([10.0, 0.0]),
    'tb3': np.array([15.0, 0.0]),
    'tb4': np.array([20.0, 0.0]),
}
```

### Racing/High-Speed
```python
self.robot_radius = 0.11
self.safety_margin = 0.25  # IMPORTANT: Keep safe!
self.min_sep_distance = 0.47

# In compute_velocity
v_goal = (v_goal / dist_to_goal) * 0.38
alpha = 0.7  # Aggressive control

# Extended awareness
if separation > 0.01 and separation < 3.0:  # Longer range
```

---

## Performance Tuning

### For Maximum Speed
1. Increase reference velocity: `0.3 → 0.4`
2. Decrease smoothing: `α = 0.5 → 0.7`
3. Reduce safety margin: `0.15 → 0.10`
4. Increase control frequency: `10 Hz → 20 Hz`

**Warning**: May cause collisions if not careful!

### For Maximum Safety
1. Decrease reference velocity: `0.3 → 0.2`
2. Increase smoothing: `α = 0.5 → 0.3`
3. Increase safety margin: `0.15 → 0.25`
4. Reduce control frequency: `10 Hz → 5 Hz`

**Result**: Very slow but very safe

### For Balanced Performance
Keep defaults:
- Reference velocity: 0.3 m/s
- Smoothing: α = 0.5
- Safety margin: 0.15 m
- Frequency: 10 Hz

---

## Testing Different Configurations

### Test 1: Safety Margin Impact
```bash
# Run with safety_margin = 0.10
# Observe: Robots get closer to obstacles

# Run with safety_margin = 0.25
# Observe: Robots maintain larger distance
```

### Test 2: Velocity Impact
```bash
# Run with v_goal scaling = 0.2
# Observe: Slow, careful motion

# Run with v_goal scaling = 0.4
# Observe: Fast, aggressive motion
```

### Test 3: Smoothing Impact
```bash
# Run with alpha = 0.2
# Observe: Very smooth, slightly delayed response

# Run with alpha = 0.8
# Observe: Jittery, immediate response
```

### Test 4: Multi-Robot Coordination
```bash
# Add 5th robot and observe:
# - Does deadlock occur?
# - Do robots still reach goals?
# - Are there gaps between robots?
```

---

## Diagnostic Commands

```bash
# Monitor collision avoidance decisions
ros2 topic echo /tb1/cmd_vel

# Check LIDAR detection range
ros2 topic echo /tb1/scan | grep ranges

# Verify robot odometry
ros2 topic echo /tb1/odom

# Check all active topics
ros2 topic list

# View computation timing
ros2 node info /multi_robot_rsvc

# Real-time monitoring dashboard
rqt
```

---

## Common Issues & Fixes

| Issue | Likely Cause | Fix |
|-------|--------------|-----|
| Robots collide | Safety margin too small | Increase `safety_margin` |
| Robots move too slow | Reference velocity too low | Increase `v_goal` scaling |
| Jerky movement | Smoothing too high | Decrease `alpha` |
| Robots ignore obstacles | LIDAR range too small | Increase `obs_dist` limit |
| Deadlock between robots | Reciprocal avoidance not working | Check velocity computation |
| High CPU usage | Control frequency too high | Increase timer period |
| Sluggish response | Smoothing too aggressive | Increase `alpha` (up to 1.0) |

---

## Parameter Checklist for New Environment

When deploying to a new environment:

- [ ] Verify robot names in `ROBOT_NAMES`
- [ ] Set appropriate goal positions
- [ ] Measure actual robot radius and set `robot_radius`
- [ ] Choose `safety_margin` based on environment (0.10-0.30)
- [ ] Test with slow velocity first (0.2 m/s)
- [ ] Gradually increase velocity while monitoring safety
- [ ] Adjust smoothing filter to reduce jitter
- [ ] Test with all robots simultaneously
- [ ] Check LIDAR range settings (typically 0.08m - 3.5m for TB3)
- [ ] Monitor CPU and adjust frequency if needed

---

## Reference Implementations

See [RSVC_IMPLEMENTATION.md](RSVC_IMPLEMENTATION.md) for algorithmic details.
