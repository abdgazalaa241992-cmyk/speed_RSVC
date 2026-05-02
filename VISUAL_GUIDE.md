# RSVC Implementation - Visual Guide

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                   Multi-Robot RSVC System (ROS2 Jazzy)              │
└─────────────────────────────────────────────────────────────────────┘

                         ┌─────────────────────┐
                         │  Gazebo Simulation  │
                         │   or Real Robots    │
                         └────────┬────────────┘
                                  │
                ┌─────────────────┼─────────────────┐
                │                 │                 │
           ┌────▼────┐       ┌───▼────┐       ┌───▼────┐
           │   TB1   │       │  TB2   │       │  TB3   │
           │ Robot   │       │ Robot  │       │ Robot  │
           └────┬────┘       └───┬────┘       └───┬────┘
                │                 │                 │
      ┌─────────┴─────────┐  ┌────┴─────────┐  ┌──┴──────────┐
      │ /tb1/odom         │  │ /tb2/odom    │  │ /tb3/odom   │
      │ /tb1/scan (LIDAR) │  │ /tb2/scan    │  │ /tb3/scan   │
      └────┬──────────────┘  └────┬─────────┘  └──┬──────────┘
           │                       │               │
           └───────────────────────┼───────────────┘
                                   │
                    ┌──────────────▼──────────────┐
                    │   MultiRobotRSVC Node      │
                    │ (speed_controller_node.py) │
                    └──────────────┬──────────────┘
                                   │
        ┌──────────────────────────┼──────────────────────────┐
        │                          │                          │
        │   RSVC Computation       │                          │
        │   ───────────────        │                          │
        │  • Position update       │   Control Outputs        │
        │  • LIDAR processing      │   ──────────────        │
        │  • Obstacle detection    │  • Linear velocity       │
        │  • Velocity cone calc    │  • Angular velocity      │
        │  • Safety constraints    │  • Heading control       │
        │  • Velocity rotation     │  • Smoothing filter      │
        │                          │                          │
        └──────────────────────────┼──────────────────────────┘
                                   │
      ┌────────────────────────────┼────────────────────────────┐
      │                            │                            │
    ┌─┴─────────┐            ┌────┴─────┐              ┌──────┴─┐
    │ /tb1/cmd  │            │ /tb2/cmd │              │ /tb3/cmd│
    │   _vel    │            │   _vel   │              │   _vel  │
    └───────────┘            └──────────┘              └─────────┘
```

## RSVC Algorithm Flow

```
START
  │
  ├─ Get robot position (from /odom)
  ├─ Get robot velocity (from /odom)
  ├─ Get robot orientation (from /odom)
  │
  ├─ Process LIDAR scan
  │  └─ Convert LIDAR points to global frame
  │
  ├─ Compute goal-seeking velocity
  │  ├─ Direction: goal - current_position
  │  ├─ Normalize & scale to 0.3 m/s
  │  └─ Store as v_goal
  │
  ├─ Apply RSVC Constraints
  │  │
  │  ├─ For each LIDAR obstacle (dist < 1.0m)
  │  │  ├─ Compute relative position
  │  │  ├─ Calculate safety cone angle
  │  │  ├─ Check if v_goal is in danger zone
  │  │  └─ If yes, rotate velocity away
  │  │
  │  ├─ For each other robot (dist < 2.0m)
  │  │  ├─ Get other robot position & velocity
  │  │  ├─ Compute relative position
  │  │  ├─ Calculate safety cone
  │  │  └─ Apply constraint if needed
  │  │
  │  └─ Return constrained velocity (v_safe)
  │
  ├─ Apply velocity limits
  │  ├─ Max speed: 0.3 m/s
  │  └─ Normalize if exceeded
  │
  ├─ Convert to ROS command
  │  ├─ Compute desired heading angle
  │  ├─ Calculate heading error
  │  ├─ Compute linear & angular velocities
  │  ├─ Apply TB3 hardware limits
  │  └─ Apply smoothing filter
  │
  ├─ Publish to /robot/cmd_vel
  │  └─ TwistStamped message
  │
  ├─ Wait 0.1 seconds (10 Hz)
  │
  └─ LOOP to START
```

## Safety Cone Geometry

```
SIDE VIEW: Robot approaching obstacle

                    Safety Cone
                        ▲
                       /│\
                      / │ \
                     /  │  \
                    /   │   \
                   /    │    \
                  /  Danger  \
                 /      Zone   \
                ───────────────────
                │ Robot │ Obstacle │
                ───────────────────
                   ↑ Safety Margin


TOP VIEW: Velocity cone visualization

          ┌─────────────────┐
          │                 │
          │     OBSTACLE    │
          │                 │
          └─────────────────┘
               ▲▲▲  ← To obstacle direction
              ▲ ▲ ▲
             ▲  |  ▲
            ▲   |   ▲
           ▲    |    ▲  ← Safety cone boundary
          ▲     |     ▲
         ╱  \   |   ╱  ╲
        ╱    \  |  ╱    ╲
       ╱ SAFE \ | ╱ SAFE ╲
      ╱ ZONE 1 \│╱ ZONE 2 ╲
     └─────────────────────┘
              │ Robot │


CONE ANGLE CALCULATION:

θ = arcsin(min_separation / distance_to_obstacle)

Where:
  min_separation = 2 × robot_radius + safety_margin
  
Example for TB3:
  robot_radius = 0.11 m
  safety_margin = 0.15 m
  min_separation = 0.37 m

  If obstacle is 1.0m away:
    θ = arcsin(0.37 / 1.0) ≈ 21.8°
    
  If obstacle is 0.5m away:
    θ = arcsin(0.37 / 0.5) ≈ 46.9°
    
  If obstacle is 0.37m away:
    θ = arcsin(0.37 / 0.37) = 90.0° (imminent collision)
```

## Velocity Deflection Logic

```
INPUT: Current desired velocity, Obstacle direction

┌─────────────────────────────────────────┐
│  Check if velocity is in danger cone    │
└────────────────┬────────────────────────┘
                 │
            ┌────┴────┐
            │          │
         YES│          │NO
            │          │
        ┌───▼─┐    ┌──▼────┐
        │DEFLECT   │PROCEED │
        │ VELOCITY │ AS-IS  │
        └───┬─┐    └────────┘
            │ │
    ┌───────┘ │ Rotate away from
    │         │ obstacle direction
    │    ┌────┴─────────────┐
    │    │ safety_angle +   │
    │    │ 0.1 radians      │
    │    └────┬─────────────┘
    │         │
    │    ┌────▼──────────────────┐
    │    │ Apply rotation matrix │
    │    │ new_vel = R × to_obs  │
    │    └────┬──────────────────┘
    │         │
    │    ┌────▼──────────────────┐
    │    │ Scale to original     │
    │    │ speed (0.9 × speed)   │
    │    └────┬──────────────────┘
    │         │
    └────┬────┘
         │
    ┌────▼──────────┐
    │ OUTPUT:       │
    │ Safe velocity │
    │ away from obs │
    └───────────────┘
```

## Data Flow for Single Robot

```
Time t=0: Robot receives inputs
┌──────────────────────────────────┐
│  Odometry: /tb1/odom             │ Position, velocity, heading
│  LIDAR: /tb1/scan                │ 360 obstacle measurements
│  Static: goals[robot_name]       │ Goal position
└──────────────────────────────────┘
           │
           ├─────────────────────────────────┐
           │                                 │
           ▼                                 ▼
    ┌────────────────┐            ┌─────────────────┐
    │ Extract state: │            │ LIDAR callback: │
    │ • Position     │            │ • Convert to    │
    │ • Velocity     │            │   global frame  │
    │ • Heading      │            │ • Extract obstacles
    │                │            │   (x, y, dist)  │
    └────────┬───────┘            └────────┬────────┘
             │                             │
             └─────────────┬───────────────┘
                           │
                    ┌──────▼──────┐
                    │  compute_   │
                    │ velocity()  │
                    └──────┬──────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
        ▼                  ▼                  ▼
    ┌────────┐      ┌──────────────┐    ┌─────────┐
    │ V_goal │      │compute_safety│    │V_RSVC   │
    │ to     │      │_velocity_    │    │ cone    │
    │goal    │─────►│cones()       │───►│ corrected
    │        │      │              │    │        │
    └────────┘      │• Check LIDAR │    └────┬───┘
                    │• Check robots│         │
                    │• Rotate away │         │
                    │  from obs    │         │
                    └──────────────┘         │
                                           │
                    ┌──────────────────────┘
                    │
                    ▼
            ┌───────────────────┐
            │ Enforce limits:   │
            │ • Max speed 0.3 m/s
            │ • Keep magnitude  │
            └─────────┬─────────┘
                      │
                      ▼
            ┌─────────────────────┐
            │ convert_to_cmd()    │
            │ • Compute heading   │
            │ • Calc linear_x     │
            │ • Calc angular_z    │
            │ • Apply hardware    │
            │   limits (TB3)      │
            │ • Smoothing filter  │
            └─────────┬───────────┘
                      │
                      ▼
            ┌─────────────────────┐
            │ Publish to /tb1/    │
            │ cmd_vel (10Hz)      │
            │                     │
            │ linear.x ← lx       │
            │ angular.z ← az      │
            └─────────┬───────────┘
                      │
Time t=0.1s: Controller repeats
```

## Collision Avoidance Timeline

```
Time    Event                        Action
────────────────────────────────────────────────────────────────

0.0s    Robots start at origin       No velocity
        Move toward goals            Start moving

0.5s    Robot TB1 at (0.5, 0)        Moving toward goal (2, 2)
        Robot TB2 at (0, 0.5)        Moving toward goal (-2, 2)
        Distance between: 0.7m       Still moving normally

0.6s    Distance: 0.5m               RSVC activates!
        v_goal would cause collision Safety cones detected
        Velocities rotated away      Robots diverge

0.7s    Distance: 0.4m               Strong deflection
        Collision imminent            Rotation angle: 90°
        Both robots actively avoid    Mutual avoidance

0.8s    Distance: 0.5m               Gap increasing
        Robots past each other        Normal goal-seeking resume
        
0.9s    Distance: 0.8m               Fully separated
        Moving normally again         Continue to goals

2.0s    All robots reach goals       Stop moving
        Collision avoided             Safe arrival
```

## Parameter Impact Matrix

```
╔═══════════════════════╦════════════╦════════════╦════════════╗
║ Parameter            ║ Decrease   ║ Default    ║ Increase   ║
╠═══════════════════════╬════════════╬════════════╬════════════╣
║ safety_margin        ║ Risky      ║ Balanced   ║ Sluggish   ║
║ (0.10 - 0.30 m)      ║ Collisions ║ 0.15 m     ║ Safe but   ║
║                      ║ Faster     ║            ║ slow       ║
╠═══════════════════════╬════════════╬════════════╬════════════╣
║ reference_velocity   ║ Crawling   ║ Balanced   ║ Aggressive ║
║ (0.2 - 0.4 m/s)     ║ Very safe  ║ 0.3 m/s    ║ May crash  ║
║                      ║ Very slow  ║            ║ Fast       ║
╠═══════════════════════╬════════════╬════════════╬════════════╣
║ smoothing (alpha)    ║ Jittery    ║ Balanced   ║ Sluggish   ║
║ (0.2 - 0.8)          ║ Responsive ║ 0.5        ║ Smooth     ║
║                      ║ Twitchy    ║            ║ Delayed    ║
╠═══════════════════════╬════════════╬════════════╬════════════╣
║ control_frequency    ║ Coarse     ║ Balanced   ║ CPU heavy  ║
║ (5-20 Hz)            ║ Delayed    ║ 10 Hz      ║ Responsive ║
║                      ║ Response   ║            ║ Hot robot  ║
╠═══════════════════════╬════════════╬════════════╬════════════╣
║ obstacle_range       ║ Blind spot ║ Balanced   ║ Reactive   ║
║ (0.8-1.2 m)          ║ Surprises! ║ 1.0 m      ║ Cautious   ║
║                      ║ Crashes    ║            ║ Slow       ║
╚═══════════════════════╩════════════╩════════════╩════════════╝
```

## File Structure

```
robot_ws/
└── src/
    └── speed_controller/
        ├── speed_controller/
        │   ├── __init__.py
        │   ├── speed_controller_node.py         ← MAIN FILE (MODIFIED)
        │   └── speed_controller_node - Copy.py
        │
        ├── README_RSVC_SUMMARY.md               ← THIS README
        ├── RSVC_IMPLEMENTATION.md               ← Algorithm details
        ├── RSVC_CONFIG_GUIDE.md                 ← Parameter tuning
        ├── TB3_MULTI_ROBOT_SETUP.md             ← Setup guide
        └── VISUAL_GUIDE.md                      ← This file
```

## Quick Reference Card

```
╔════════════════════════════════════════════════════════════════╗
║           RSVC COLLISION AVOIDANCE QUICK REFERENCE             ║
╠════════════════════════════════════════════════════════════════╣
║                                                                ║
║ LAUNCH:                                                        ║
║  $ ros2 run speed_controller speed_controller_node.py         ║
║                                                                ║
║ MONITOR:                                                       ║
║  $ ros2 topic echo /tb1/cmd_vel                               ║
║  $ rviz2                                                       ║
║                                                                ║
║ KEY FILES:                                                     ║
║  speed_controller_node.py  - Main RSVC implementation         ║
║  RSVC_CONFIG_GUIDE.md      - Parameter tuning                 ║
║                                                                ║
║ DEFAULT PARAMETERS:                                           ║
║  • robot_radius: 0.11 m                                       ║
║  • safety_margin: 0.15 m                                      ║
║  • reference_velocity: 0.3 m/s                                ║
║  • update_frequency: 10 Hz                                    ║
║                                                                ║
║ SAFETY CONE ANGLE:                                            ║
║  θ = arcsin(min_separation / distance_to_obstacle)            ║
║                                                                ║
║ VELOCITY DEFLECTION:                                          ║
║  if velocity_in_cone: rotate_by(safety_angle + 0.1 rad)       ║
║                                                                ║
║ SOURCES:                                                       ║
║  • LIDAR obstacles (< 1.0m)                                   ║
║  • Robot odometry (< 2.0m)                                    ║
║                                                                ║
╚════════════════════════════════════════════════════════════════╝
```

---

**For more details, see:**
- [RSVC_IMPLEMENTATION.md](RSVC_IMPLEMENTATION.md) - Theory
- [RSVC_CONFIG_GUIDE.md](RSVC_CONFIG_GUIDE.md) - Tuning
- [TB3_MULTI_ROBOT_SETUP.md](TB3_MULTI_ROBOT_SETUP.md) - Setup
