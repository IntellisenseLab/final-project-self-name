#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import math
import numpy as np


class SafetyMonitor(Node):

    # Internal states
    STATE_FOLLOWING = 'following'       # Normal operation
    STATE_EMERGENCY = 'emergency'       # Emergency stop triggered
    STATE_RECOVERING = 'recovering'     # Rotating to reacquire leader
    STATE_LOST = 'lost'                 # Leader lost after recovery failed
    STATE_RETURNING = 'returning'       # Navigating back to home position
    STATE_ARRIVED = 'arrived'           # Safely back at home, resting <-- Added State

    def __init__(self):
        super().__init__('safety_monitor')

        # Parameters
        self.declare_parameter('min_following_distance', 0.3)
        self.declare_parameter('obstacle_distance', 0.4)
        self.declare_parameter('leader_timeout', 3.0)
        self.declare_parameter('recovery_rotation_time', 15.0)
        self.declare_parameter('recovery_angular_speed', 1.0)
        self.declare_parameter('monitor_frequency', 10.0)

        self.min_following_distance = self.get_parameter('min_following_distance').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.leader_timeout = self.get_parameter('leader_timeout').value
        self.recovery_rotation_time = self.get_parameter('recovery_rotation_time').value
        self.recovery_angular_speed = self.get_parameter('recovery_angular_speed').value
        monitor_frequency = self.get_parameter('monitor_frequency').value

        # State
        self.state = self.STATE_FOLLOWING
        self.last_leader_time = None
        self.recovery_start_time = None
        self.latest_depth_image = None
        self.obstacle_detected = False
        self.leader_too_close = False

        # Home position — saved from first amcl_pose received
        self.home_position = None
        self.home_saved = False

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_goal_handle = None
        self._returning = False

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/leader/pose',
            self.pose_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/depth/image_raw',
            self.depth_callback,
            10
        )

        # Subscribe to amcl_pose to get localized position in map frame
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        # Monitor loop timer
        self.timer = self.create_timer(
            1.0 / monitor_frequency,
            self.monitor_loop
        )

        self.get_logger().info('Safety monitor started')

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """Save first received AMCL pose as home position in map frame."""
        if not self.home_saved:
            self.home_position = PoseStamped()
            self.home_position.header.frame_id = 'map'
            self.home_position.header.stamp = self.get_clock().now().to_msg()
            self.home_position.pose = msg.pose.pose
            self.home_saved = True
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.get_logger().info(
                f'Home position saved from AMCL: x={x:.3f}, y={y:.3f} (map frame)'
            )

    def pose_callback(self, msg: PoseStamped):
        """Update last known leader pose and time."""
        self.last_leader_time = self.get_clock().now()

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        distance = math.sqrt(x**2 + y**2 + z**2)

        self.leader_too_close = distance < self.min_following_distance

        # If we see the leader, reset completely and return to following
        if self.state in (self.STATE_RECOVERING, self.STATE_LOST, self.STATE_RETURNING, self.STATE_ARRIVED):
            self.get_logger().info('Leader reacquired — resuming following')
            if self.state == self.STATE_RETURNING:
                self.cancel_navigation()
            self.state = self.STATE_FOLLOWING
            self.recovery_start_time = None
            self._returning = False
            self.publish_emergency_stop(False)

    def depth_callback(self, msg: Image):
        self.latest_depth_image = msg

    def check_obstacle(self) -> bool:
        if self.latest_depth_image is None:
            return False
        try:
            depth_data = np.frombuffer(self.latest_depth_image.data, dtype=np.uint16)
            depth_array = depth_data.reshape(
                self.latest_depth_image.height,
                self.latest_depth_image.width
            ).astype(np.float32) / 1000.0

            h, w = depth_array.shape
            center_region = depth_array[
                int(h * 0.35):int(h * 0.65),
                int(w * 0.35):int(w * 0.65)
            ]
            valid = center_region[np.isfinite(center_region) & (center_region > 0)]
            if len(valid) == 0:
                return False

            min_distance = float(np.min(valid))
            if min_distance < self.obstacle_distance:
                return True
        except Exception as e:
            pass
        return False

    def publish_emergency_stop(self, active: bool):
        msg = Bool()
        msg.data = active
        self.emergency_stop_pub.publish(msg)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def rotate_for_recovery(self):
        cmd = Twist()
        cmd.angular.z = self.recovery_angular_speed
        self.cmd_vel_pub.publish(cmd)

    def navigate_to_home(self):
        """Send Nav2 goal to return to home position in map frame."""
        if self.home_position is None:
            self.get_logger().warn('No home position saved yet.', throttle_duration_sec=2.0)
            return

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 action server not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.home_position
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info('Sending unique return-to-home goal to Nav2')
        send_goal_future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)
        self._returning = True

    def nav_goal_response_callback(self, future):
        self._nav_goal_handle = future.result()
        if not self._nav_goal_handle.accepted:
            self.get_logger().warn('Return-to-home goal rejected by Nav2')
            self._returning = False
            return

        self.get_logger().info('Return-to-home goal accepted by Nav2')
        result_future = self._nav_goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """Triggered once when Nav2 finishes its tracking path."""
        self.get_logger().info('Returned to home position successfully!')
        self._returning = False
        self.stop_robot()
        self.state = self.STATE_ARRIVED # Fixed: Move to ARRIVED instead of LOST to prevent loops

    def nav_feedback_callback(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f'Returning home — distance remaining: {remaining:.2f}m',
            throttle_duration_sec=2.0
        )

    def cancel_navigation(self):
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None
            self._returning = False

    def monitor_loop(self):
        """Main safety monitoring loop."""
        now = self.get_clock().now()
        self.obstacle_detected = self.check_obstacle()

        if self.state == self.STATE_FOLLOWING:
            if self.leader_too_close or self.obstacle_detected:
                self.state = self.STATE_EMERGENCY
                self.publish_emergency_stop(True)
                self.stop_robot()
                return

            if self.last_leader_time is not None:
                time_since_leader = (now - self.last_leader_time).nanoseconds / 1e9
                if time_since_leader > self.leader_timeout:
                    self.get_logger().warn('Leader lost. Starting recovery rotation.')
                    self.state = self.STATE_RECOVERING
                    self.recovery_start_time = now
                    self.publish_emergency_stop(True)

        elif self.state == self.STATE_EMERGENCY:
            self.stop_robot()
            if not self.leader_too_close and not self.obstacle_detected:
                self.state = self.STATE_FOLLOWING
                self.publish_emergency_stop(False)

        elif self.state == self.STATE_RECOVERING:
            recovery_elapsed = (now - self.recovery_start_time).nanoseconds / 1e9
            if recovery_elapsed < self.recovery_rotation_time:
                self.rotate_for_recovery()
            else:
                self.get_logger().warn('Recovery failed — switching to home routing.')
                self.state = self.STATE_LOST
                self.stop_robot()

        elif self.state == self.STATE_LOST:
            if not self._returning:
                self.navigate_to_home()
                if self._returning:
                    self.state = self.STATE_RETURNING
                else:
                    self.stop_robot()

        elif self.state == self.STATE_RETURNING:
            # Let Nav2 work in the background. Do not re-request goals.
            if not self._returning:
                self.state = self.STATE_LOST

        elif self.state == self.STATE_ARRIVED:
            # Safe hold state. Stop wheels completely.
            self.stop_robot()
            self.get_logger().info('Safe at home base. Standing by.', throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()