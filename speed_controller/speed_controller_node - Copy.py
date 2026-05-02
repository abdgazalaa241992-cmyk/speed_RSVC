import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
import math

ROBOT_NAMES = ['tb1', 'tb2', 'tb3', 'tb4']


class MultiRobotRVO(Node):

    def __init__(self):
        super().__init__('multi_robot_rvo_stable')

        # Robot states
        self.positions = {name: np.array([0.0, 0.0]) for name in ROBOT_NAMES}
        self.yaws = {name: 0.0 for name in ROBOT_NAMES}

        # Publishers / Subscribers (IMPORTANT FIX)
        self.odom_subscribers = []
        self.cmd_publishers = {}

        # For smoothing
        self.prev_cmd = {}

        for name in ROBOT_NAMES:

            self.odom_subscribers.append(
                self.create_subscription(
                    Odometry,
                    f'/{name}/odom',
                    lambda msg, robot=name: self.odom_callback(msg, robot),
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
            'tb1': np.array([2.0, 2.0]),
            'tb2': np.array([-2.0, 2.0]),
            'tb3': np.array([-2.0, -2.0]),
            'tb4': np.array([2.0, -2.0]),
        }

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg, robot_name):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.positions[robot_name] = np.array([x, y])

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.yaws[robot_name] = yaw

    def compute_velocity(self, robot_name):
        pos = self.positions[robot_name]
        goal = self.goals[robot_name]

        v = goal - pos
        dist = np.linalg.norm(v)

        if dist > 0:
            v = (v / dist) * 0.3

        # Simple collision avoidance
        for other in ROBOT_NAMES:
            if other == robot_name:
                continue

            diff = pos - self.positions[other]
            d = np.linalg.norm(diff)

            if d < 1.0:
                if d > 0:
                    v += (diff / d) * 0.5

        # limit
        speed = np.linalg.norm(v)
        if speed > 0.3:
            v = (v / speed) * 0.3

        return v

    def convert_to_cmd(self, robot_name, v):
        yaw = self.yaws[robot_name]

        desired_angle = math.atan2(v[1], v[0])
        error = desired_angle - yaw

        # normalize angle
        error = math.atan2(math.sin(error), math.cos(error))

        # gains
        k_lin = 0.4
        k_ang = 1.0

        linear_x = k_lin * np.linalg.norm(v)
        angular_z = k_ang * error

        # limits
        linear_x = min(linear_x, 0.22)
        angular_z = max(min(angular_z, 1.0), -1.0)

        # stop forward if turning too much
        if abs(error) > 0.6:
            linear_x = 0.0

        # smoothing filter
        alpha = 0.5

        prev = self.prev_cmd.get(robot_name, (0.0, 0.0))

        linear_x = alpha * linear_x + (1 - alpha) * prev[0]
        angular_z = alpha * angular_z + (1 - alpha) * prev[1]

        self.prev_cmd[robot_name] = (linear_x, angular_z)

        return linear_x, angular_z

    def control_loop(self):
        for robot in ROBOT_NAMES:

            v = self.compute_velocity(robot)
            lx, az = self.convert_to_cmd(robot, v)

            msg = TwistStamped()
            msg.twist.linear.x = float(lx)
            msg.twist.angular.z = float(az)

            self.cmd_publishers[robot].publish(msg)


def main():
    rclpy.init()
    node = MultiRobotRVO()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()