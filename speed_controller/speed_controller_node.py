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
import time

ROBOT_NAMES = ['tb1', 'tb2', 'tb3', 'tb4']# 


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
        self.safety_margin = 0.20  # Additional safety margin
        self.min_sep_distance = self.robot_radius * 2 + self.safety_margin

        #deadlock_solve
        self.deadlock_en=True
        self.deadlock=False
        self.deadlock_timer=time.time()
        self.dis_arrived=0.0

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
            'tb1': np.array([7.0, 7.0]),
            'tb2': np.array([-7.0, 7.0]),
            'tb3': np.array([-7.0, -7.0]),
            'tb4': np.array([7.0, -7.0]),
        }

        self.timer = self.create_timer(0.1, self.control_loop)
        self.timer2 = self.create_timer(1, self.print_loop)
        self.lx=0.0 
        self.az=0.0

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
        Compute Reciprocal Safety Velocity Cones (RSVC) using half-safety planes.
        Projects desired velocity onto the intersection of half-planes defined by obstacles.
        """
        # Build bearing matrix rows A from neighbors
        # Each row represents a half-plane constraint: a^T u <= 0
        A_rows = []
        
        pos = self.positions[robot_name]
        
        # Process obstacles from LIDAR
        obstacles = self.lidar_obstacles[robot_name]
        
        for obs_x, obs_y, obs_dist in obstacles:
            if obs_dist < 2.0:  # Only consider nearby obstacles
                obs_pos = np.array([obs_x, obs_y])
                rel_pos = obs_pos - pos
                separation = np.linalg.norm(rel_pos)
                
                if separation > 0.01 and separation < self.min_sep_distance + 0.5:
                    # Bearing vector (unit vector from robot to obstacle)
                    bearing = rel_pos / separation
                    A_rows.append(bearing)
        
        # If no constraints, return desired velocity
        if not A_rows:
            return desired_velocity
        
        # Build constraint matrix A (m x 2)
        A = np.vstack(A_rows)
        
        # Project desired velocity onto cone C(A) = {u | A u <= 0}
        v_safe = self._project_onto_cone(desired_velocity, A)
        
        # Ensure speed limit
        speed = np.linalg.norm(v_safe)
        if speed > 0.62:  # TB3 max linear speed
            v_safe = (v_safe / speed) * 0.62
        
        return v_safe

    def _project_onto_cone(self, u0, A):
        """
        Project velocity u0 onto polyhedral cone C(A) = {u | A u <= 0} in 2D.
        The projection lies on a face (one active constraint) or at the apex (zero vector).
        
        Args:
            u0: Desired velocity vector [vx, vy]
            A: Constraint matrix where each row is a bearing (half-plane normal)
        
        Returns:
            Safe velocity projected onto the feasible cone
        """
        # If desired velocity already satisfies all constraints, return it
        if np.all(A @ u0 <= 1e-9):
            return u0
        
        best = np.zeros_like(u0)
        best_dist = np.linalg.norm(u0 - best)
        
        # Try projecting onto each constraint boundary
        for a in A:
            aa = np.dot(a, a)
            if aa < 1e-12:
                continue
            
            # Project u0 onto the boundary line a^T u = 0
            # u_proj = u0 - (a^T u0 / ||a||^2) * a
            u_proj = u0 - (np.dot(a, u0) / aa) * a
            
            # Check if projection satisfies all constraints
            if np.all(A @ u_proj <= 1e-9):
                d = np.linalg.norm(u0 - u_proj)
                if d < best_dist:
                    best = u_proj
                    best_dist = d
        
        return best

    def compute_velocity(self, robot_name,sheft_rotate):
        """Compute desired velocity using goal-seeking with RSVC collision avoidance."""
        pos = self.positions[robot_name]
        goal = self.goals[robot_name]

        # Compute goal-seeking velocity
        v_goal = goal - pos
        dist_to_goal = np.linalg.norm(v_goal)
        self.dis_arrived=dist_to_goal
        theta=math.atan2(v_goal[1],v_goal[0])
        #rotate u0
        u0_R=np.zeros(2)
        u0_R[0]=dist_to_goal*math.cos(theta+sheft_rotate)
        u0_R[1]=dist_to_goal*math.sin(theta+sheft_rotate)
        v_goal=u0_R
        dist_to_goal= 1
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
        k_ang = 2.0

        linear_x = k_lin * np.linalg.norm(v)
        angular_z = k_ang * error

        # TB3 velocity limits
        linear_x = min(linear_x, 0.62)
        angular_z = max(min(angular_z, 2.0), -2.0)

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

            v = self.compute_velocity(robot,0)
            norm_v=np.linalg.norm(v);
            
            if (norm_v < 0.1 and self.dis_arrived > 0.2):
                self.deadlock = True
                self.deadlock_timer = time.time()
            if time.time() - self.deadlock_timer > 0.5:
                self.deadlock = False
                # self.deadlock_R = not self.deadlock_R

            if self.deadlock and self.deadlock_en:
                # print ("robot:",robot)
                # print ("deadlock_timer:",self.deadlock_timer)
                # print ("old_v:",v)
                v_l = self.compute_velocity(robot,math.pi/2)
                v_R = self.compute_velocity(robot,math.pi/2)
                
                # theta_L=math.atan2(v_l[1],v_l[0])
                # theta_R=math.atan2(v_R[1],v_R[0])
                # print ("theta_L-self.yaws[robot]:",theta_L-self.yaws[robot])
                # print ("theta_R-self.yaws[robot]:",theta_R-self.yaws[robot])
                # if (abs(theta_L-self.yaws[robot])>abs(theta_R-self.yaws[robot])):

                if (np.linalg.norm(v_R)>np.linalg.norm(v_l)):
                    v=v_R
                else:
                    v=v_l
                # print ("new_v:",v)
            self.lx, self.az = self.convert_to_cmd(robot, v)

            msg = TwistStamped()
            msg.twist.linear.x = float(self.lx)
            msg.twist.angular.z = float(self.az)

            self.cmd_publishers[robot].publish(msg)
    def print_loop(self):
        """Main print loop: """
        for robot in ROBOT_NAMES:
           print ("robot:",robot)
           print ("velocity:",self.lx, self.az)
           print ("odo:",self.positions[robot],self.yaws[robot])


def main():
    rclpy.init()
    node = MultiRobotRSVC()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()