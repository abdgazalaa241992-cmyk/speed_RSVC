"""
Reciprocal Safety Velocity Cones (RSVC) Collision Avoidance for Multi-Robot Systems

This module implements decentralized collision avoidance for multi-agent robot systems
using Reciprocal Safety Velocity Cones based on LIDAR sensor data and odometry.

Key Features:
- LIDAR-based obstacle detection (converted to global frame)
- Reciprocal safety velocity cone computation
- Decentralized collision avoidance (no central coordinator needed)
- Support for multiple TB3 robots in ROS2 Jazzy
- Goal-seeking behavior with collision constraints

RSVC Algorithm:
1. For each detected obstacle/robot, compute relative position and distance
2. Create safety cones around obstacles based on separation distance
3. Compute cone angle using: angle = arcsin(min_separation / distance)
4. Check if desired velocity falls within the danger cone
5. If yes, rotate velocity away from the obstacle
6. Apply rotation proportional to safety margin violation

References:
- van den Berg, J., Guy, S., Lin, M., & Manocha, D. (2011). 
  "Reciprocal Velocity Obstacles". Journal of Machine Learning Research.

Author: Multi-Robot Navigation System
ROS2: Jazzy
Robots: TB3 (Turtlebot3) with LIDAR
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math

ROBOT_NAMES = ['tb3']# , 'tb2', 'tb3', 'tb4'


class MultiRobotRSVC(Node):
    """
    Reciprocal Safety Velocity Cones (RSVC) for decentralized collision avoidance
    with LIDAR-based obstacle detection in multi-agent ROS2 system.
    """

    def __init__(self):
        super().__init__('multi_robot_rsvc')

        # Robot states
        self.positions = {name: np.array([0.0, 0.0]) for name in ROBOT_NAMES}
        self.velocities = {name: np.array([0.0, 0.0]) for name in ROBOT_NAMES}
        self.yaws = {name: 0.0 for name in ROBOT_NAMES}

        # LIDAR data - obstacles detected as (x, y, distance)
        self.lidar_obstacles = {name: [] for name in ROBOT_NAMES}

        # Publishers / Subscribers
        self.odom_subscribers = []
        self.lidar_subscribers = []
        self.cmd_publishers = {}

        # For smoothing
        self.prev_cmd = {}

        # RSVC parameters
        self.robot_radius = 0.11  # TB3 robot radius (meters)
        self.safety_margin = 0.15  # Additional safety margin
        self.min_sep_distance = self.robot_radius * 2 + self.safety_margin

        for name in ROBOT_NAMES:

            self.odom_subscribers.append(
                self.create_subscription(
                    Odometry,
                    f'/{name}/odom',
                    lambda msg, robot=name: self.odom_callback(msg, robot),
                    10
                )
            )

            self.lidar_subscribers.append(
                self.create_subscription(
                    LaserScan,
                    f'/{name}/scan',
                    lambda msg, robot=name: self.lidar_callback(msg, robot),
                    10
                )
            )

            self.cmd_publishers[name] = self.create_publisher(
                TwistStamped,
                f'/{name}/cmd_vel',
                10
            )

        # Goals
        self.goals = {
            'tb1': np.array([5.0, 5.0]),
            'tb2': np.array([-2.0, 2.0]),
            'tb3': np.array([-2.0, -2.0]),
            'tb4': np.array([2.0, -2.0]),
        }

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg, robot_name):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions[robot_name] = np.array([x, y])

        # Extract velocity from odometry
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.velocities[robot_name] = np.array([vx, vy])

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.yaws[robot_name] = yaw

    def lidar_callback(self, msg, robot_name):
        """Process LIDAR scan and extract obstacle positions in global frame."""
        obstacles = []
        
        # Convert LIDAR data to Cartesian coordinates in robot frame
        for i, distance in enumerate(msg.ranges):
            if msg.range_min <= distance <= msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                
                # Robot frame to global frame transformation
                x_robot = distance * math.cos(angle)
                y_robot = distance * math.sin(angle)
                
                # Transform to global frame using robot's pose
                yaw = self.yaws[robot_name]
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                
                x_global = (x_robot * cos_yaw - y_robot * sin_yaw + 
                           self.positions[robot_name][0])
                y_global = (x_robot * sin_yaw + y_robot * cos_yaw + 
                           self.positions[robot_name][1])
                
                obstacles.append((x_global, y_global, distance))
        
        self.lidar_obstacles[robot_name] = obstacles

    def compute_safety_velocity_cones(self, robot_name, desired_velocity, max_iterations=5):
        """
        Compute Reciprocal Safety Velocity Cones (RSVC) and find collision-free velocity.
        Returns a velocity that avoids all detected obstacles.
        """
        pos = self.positions[robot_name]
        vel_cmd = desired_velocity.copy()
        
        # Normalize desired velocity
        desired_speed = np.linalg.norm(desired_velocity)
        if desired_speed > 0:
            desired_unit = desired_velocity / desired_speed
        else:
            desired_unit = np.array([0.0, 0.0])
        
        # Process obstacles from LIDAR
        obstacles = self.lidar_obstacles[robot_name]
        
        for obs_x, obs_y, obs_dist in obstacles:
            if obs_dist < 1.0:  # Only consider nearby obstacles
                obs_pos = np.array([obs_x, obs_y])
                rel_pos = obs_pos - pos
                separation = np.linalg.norm(rel_pos)
                
                if separation > 0.01 and separation < self.min_sep_distance + 0.5:
                    # Compute reciprocal safety velocity cone
                    vel_cmd = self._apply_rsvc_constraint(
                        vel_cmd, rel_pos, separation
                    )
        
        # Process other robots from odometry
        for other_name in ROBOT_NAMES:
            if other_name == robot_name:
                continue
            
            other_pos = self.positions[other_name]
            other_vel = self.velocities[other_name]
            
            rel_pos = other_pos - pos
            separation = np.linalg.norm(rel_pos)
            
            if separation > 0.01 and separation < 2.0:  # Consider robots within 2m
                # Reciprocal velocity
                rel_vel = self.velocities[robot_name] - other_vel
                
                # Apply RSVC constraint
                vel_cmd = self._apply_rsvc_constraint(
                    vel_cmd, rel_pos, separation, rel_vel
                )
        
        return vel_cmd

    def _apply_rsvc_constraint(self, velocity, rel_pos, separation, rel_vel=None):
        """
        Apply a single RSVC constraint to modify the desired velocity.
        
        Args:
            velocity: Current desired velocity [vx, vy]
            rel_pos: Relative position of obstacle [rx, ry]
            separation: Distance to obstacle
            rel_vel: Relative velocity (for moving obstacles)
        
        Returns:
            Modified velocity that avoids collision
        """
        if separation < 0.01:
            return velocity
        
        # Unit vector from robot to obstacle
        to_obstacle = rel_pos / separation
        
        # Compute safety cone angle based on separation distance
        # Closer obstacles = wider cones
        if separation < self.min_sep_distance:
            # Imminent collision - strong deflection
            safety_angle = math.pi / 2
        else:
            # Farther obstacle - narrower cone
            safety_angle = math.asin(min(1.0, self.min_sep_distance / (separation + 0.01)))
        
        # Compute cone boundaries (perpendicular to relative position)
        cone_normal1 = np.array([-to_obstacle[1], to_obstacle[0]])
        cone_normal2 = np.array([to_obstacle[1], -to_obstacle[0]])
        
        # Check if velocity is in the collision cone
        vel_norm = np.linalg.norm(velocity)
        
        if vel_norm > 0.01:
            vel_unit = velocity / vel_norm
            
            # Dot product with cone boundaries
            dot1 = np.dot(vel_unit, cone_normal1)
            dot2 = np.dot(vel_unit, cone_normal2)
            
            # If velocity points toward obstacle, deflect it
            dot_obstacle = np.dot(vel_unit, to_obstacle)
            
            if dot_obstacle > -math.cos(safety_angle):
                # Deflect velocity away from obstacle
                # Rotate velocity by safety angle
                if dot1 > dot2:
                    rotation_angle = safety_angle + 0.1
                else:
                    rotation_angle = -(safety_angle + 0.1)
                
                # Rotation matrix
                cos_a = math.cos(rotation_angle)
                sin_a = math.sin(rotation_angle)
                
                new_vel = np.array([
                    to_obstacle[0] * cos_a - to_obstacle[1] * sin_a,
                    to_obstacle[0] * sin_a + to_obstacle[1] * cos_a
                ])
                
                # Scale to original speed
                new_vel = new_vel * vel_norm * 0.9
                return new_vel
        
        return velocity

    def compute_velocity(self, robot_name):
        """Compute desired velocity using goal-seeking with RSVC collision avoidance."""
        pos = self.positions[robot_name]
        goal = self.goals[robot_name]

        # Compute goal-seeking velocity
        v_goal = goal - pos
        dist_to_goal = np.linalg.norm(v_goal)

        if dist_to_goal > 0.1:
            v_goal = (v_goal / dist_to_goal) * 2
        else:
            v_goal = np.array([0.0, 0.0])

        # Apply RSVC collision avoidance
        v_safe = self.compute_safety_velocity_cones(robot_name, v_goal)

        # Ensure speed limit
        speed = np.linalg.norm(v_safe)
        if speed > 3:
            v_safe = (v_safe / speed) * 3

        return v_safe

    def convert_to_cmd(self, robot_name, v):
        """
        Convert 2D velocity to ROS2 TwistStamped command (linear_x, angular_z).
        Includes heading control and velocity limits for TB3.
        """
        yaw = self.yaws[robot_name]

        desired_angle = math.atan2(v[1], v[0])
        error = desired_angle - yaw

        # normalize angle
        error = math.atan2(math.sin(error), math.cos(error))

        # gains for TB3
        k_lin = 0.4
        k_ang = 1.0

        linear_x = k_lin * np.linalg.norm(v)
        angular_z = k_ang * error

        # TB3 velocity limits
        linear_x = min(linear_x, 0.22)
        angular_z = max(min(angular_z, 1.0), -1.0)

        # stop forward if turning too much
        if abs(error) > 0.6:
            linear_x = 0.0

        # low-pass smoothing filter to reduce jitter
        alpha = 0.7

        prev = self.prev_cmd.get(robot_name, (0.0, 0.0))

        linear_x = alpha * linear_x + (1 - alpha) * prev[0]
        angular_z = alpha * angular_z + (1 - alpha) * prev[1]

        self.prev_cmd[robot_name] = (linear_x, angular_z)

        return linear_x, angular_z

    def control_loop(self):
        """Main control loop: compute RSVC-based velocities for all robots."""
        for robot in ROBOT_NAMES:

            v = self.compute_velocity(robot)
            lx, az = self.convert_to_cmd(robot, v)

            msg = TwistStamped()
            msg.twist.linear.x = float(lx)
            msg.twist.angular.z = float(az)

            self.cmd_publishers[robot].publish(msg)


def main():
    rclpy.init()
    node = MultiRobotRSVC()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()