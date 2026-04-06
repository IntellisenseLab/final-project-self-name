#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math
from std_msgs.msg import Bool


class PIDController:
    """Generic PID controller."""

    def __init__(self, kp: float, ki: float, kd: float, max_output: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output

        self._prev_error = 0.0
        self._integral = 0.0

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        self._integral += error * dt
        self._integral = max(-1.0, min(1.0, self._integral))

        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        output = (
            self.kp * error +
            self.ki * self._integral +
            self.kd * derivative
        )

        return max(-self.max_output, min(self.max_output, output))

    def reset(self):
        self._prev_error = 0.0
        self._integral = 0.0


class PIDControllerNode(Node):

    def __init__(self):
        super().__init__('pid_controller')

        # Parameters
        self.declare_parameter('desired_distance', 0.8)

        self.declare_parameter('linear_kp', 0.6)
        self.declare_parameter('linear_ki', 0.02)
        self.declare_parameter('linear_kd', 0.1)
        self.declare_parameter('max_linear_speed', 0.5)

        self.declare_parameter('angular_kp', 1.5)
        self.declare_parameter('angular_ki', 0.0)
        self.declare_parameter('angular_kd', 0.2)
        self.declare_parameter('max_angular_speed', 1.5)

        self.declare_parameter('control_frequency', 20.0)

        # Read parameters
        self.desired_distance = self.get_parameter('desired_distance').value
        self.control_frequency = self.get_parameter('control_frequency').value

        # PID controllers
        self.linear_pid = PIDController(
            self.get_parameter('linear_kp').value,
            self.get_parameter('linear_ki').value,
            self.get_parameter('linear_kd').value,
            self.get_parameter('max_linear_speed').value
        )

        self.angular_pid = PIDController(
            self.get_parameter('angular_kp').value,
            self.get_parameter('angular_ki').value,
            self.get_parameter('angular_kd').value,
            self.get_parameter('max_angular_speed').value
        )

        # State
        self.leader_pose = None
        self.last_pose_time = self.get_clock().now()
        self.last_time = self.get_clock().now()
        self.emergency_stop = False

        # Subscribers
        self.create_subscription(
            PoseStamped,
            '/leader/pose',
            self.pose_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_stop_callback,
            10
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_loop
        )

        self.get_logger().info('PID Controller Node Started')
        self.get_logger().info(f'Desired distance: {self.desired_distance} m')

    def pose_callback(self, msg: PoseStamped):
        self.leader_pose = msg
        self.last_pose_time = self.get_clock().now()

    def emergency_stop_callback(self, msg: Bool):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().warn('Emergency STOP activated!')
            self.cmd_vel_pub.publish(Twist())

    def compute_errors(self, pose: PoseStamped):
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z

        #Euclidean distance
        actual_distance = math.sqrt(x**2 + y**2 + z**2)
        distance_error = actual_distance - self.desired_distance

        angular_error = math.atan2(y, x)

        return distance_error, angular_error

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        cmd = Twist()

        # Emergency stop
        if self.emergency_stop:
            self.cmd_vel_pub.publish(cmd)
            return

        if self.leader_pose is None:
            self.get_logger().warn('No leader detected', throttle_duration_sec=2.0)
            self.cmd_vel_pub.publish(cmd)
            return

        # Timeout check
        time_since_last = (now - self.last_pose_time).nanoseconds / 1e9
        if time_since_last > 0.5:
            self.get_logger().warn('Leader lost - stopping')
            self.cmd_vel_pub.publish(cmd)
            self.linear_pid.reset()
            self.angular_pid.reset()
            return

        # Compute errors
        distance_error, angular_error = self.compute_errors(self.leader_pose)

        if abs(distance_error) < 0.05:
            distance_error = 0.0

        if abs(angular_error) < 0.02:
            angular_error = 0.0

        # Angular control
        cmd.angular.z = self.angular_pid.compute(angular_error, dt)

        angle_threshold = 0.3  # radians

        if abs(angular_error) > angle_threshold:
            cmd.linear.x = 0.0
        else:
            # Prevent reverse motion
            if distance_error < 0:
                distance_error = 0.0

            cmd.linear.x = self.linear_pid.compute(distance_error, dt)

        # Publish command
        self.cmd_vel_pub.publish(cmd)

        # Debug log
        self.get_logger().info(
            f'Dist err: {distance_error:.3f} | Angle err: {math.degrees(angular_error):.1f} deg | '
            f'Lin: {cmd.linear.x:.2f} | Ang: {cmd.angular.z:.2f}',
            throttle_duration_sec=0.5
        )

    def stop(self):
        self.cmd_vel_pub.publish(Twist())
        self.linear_pid.reset()
        self.angular_pid.reset()
        self.get_logger().info('Controller stopped')


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()