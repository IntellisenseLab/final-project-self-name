#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
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
 
    def __init__(self):
        super().__init__('safety_monitor')
 
        # Parameters
        self.declare_parameter('min_following_distance', 0.3)   # metres — too close to leader
        self.declare_parameter('obstacle_distance', 0.4)        # metres — obstacle ahead
        self.declare_parameter('leader_timeout', 3.0)           # seconds before declaring leader lost
        self.declare_parameter('recovery_rotation_time', 30.0)   # seconds to rotate before giving up
        self.declare_parameter('recovery_angular_speed', 0.6)   # rad/s rotation speed during recovery
        self.declare_parameter('monitor_frequency', 10.0)       # Hz
 
        self.min_following_distance = self.get_parameter('min_following_distance').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.leader_timeout = self.get_parameter('leader_timeout').value
        self.recovery_rotation_time = self.get_parameter('recovery_rotation_time').value
        self.recovery_angular_speed = self.get_parameter('recovery_angular_speed').value
        monitor_frequency = self.get_parameter('monitor_frequency').value
 
        # State
        self.state = self.STATE_FOLLOWING
        self.last_leader_time = None        # time of last received leader pose
        self.recovery_start_time = None     # time recovery rotation started
        self.latest_depth_image = None      # latest depth image from camera
        self.obstacle_detected = False
        self.leader_too_close = False
 
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/leader/pose',
            self.pose_callback,
            10
        )
 
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',      # update to /rgbd_camera/depth_image for simulation (if simulating)
            self.depth_callback,
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
        self.get_logger().info(
            f'Min following distance: {self.min_following_distance}m | '
            f'Obstacle threshold: {self.obstacle_distance}m | '
            f'Leader timeout: {self.leader_timeout}s'
        )
 
    def pose_callback(self, msg: PoseStamped):
        """Update last known leader pose and time."""
        self.last_leader_time = self.get_clock().now()
 
        # Check if leader is too close
        x = msg.pose.position.x
        y = msg.pose.position.y
        distance = math.sqrt(x**2 + y**2)
 
        self.leader_too_close = distance < self.min_following_distance
 
        if self.leader_too_close:
            self.get_logger().warn(
                f'Leader too close: {distance:.3f}m (min: {self.min_following_distance}m)',
                throttle_duration_sec=1.0
            )
 
        # If we were recovering and now see the leader again, resume following
        if self.state in (self.STATE_RECOVERING, self.STATE_LOST):
            self.get_logger().info('Leader reacquired — resuming following')
            self.state = self.STATE_FOLLOWING
            self.recovery_start_time = None
            self.publish_emergency_stop(False)
 
    def depth_callback(self, msg: Image):
        """Store latest depth image for obstacle checking."""
        self.latest_depth_image = msg
 
    def check_obstacle(self) -> bool:
        """
        Check if an obstacle is within obstacle_distance ahead of the robot.
        Looks at the central region of the depth image.
        Returns True if obstacle detected.
        """
        if self.latest_depth_image is None:
            return False
 
        try:
            # Convert raw depth image bytes to numpy array
            depth_data = np.frombuffer(self.latest_depth_image.data, dtype=np.float32)
            depth_array = depth_data.reshape(
                self.latest_depth_image.height,
                self.latest_depth_image.width
            )
 
            # Look at the central 30% of the image — directly ahead of robot
            h, w = depth_array.shape
            center_region = depth_array[
                int(h * 0.35):int(h * 0.65),
                int(w * 0.35):int(w * 0.65)
            ]
 
            # Filter out NaN and zero values (invalid depth readings)
            valid = center_region[np.isfinite(center_region) & (center_region > 0)]
 
            if len(valid) == 0:
                return False
 
            min_distance = float(np.min(valid))
 
            if min_distance < self.obstacle_distance:
                self.get_logger().warn(
                    f'Obstacle detected at {min_distance:.3f}m',
                    throttle_duration_sec=1.0
                )
                return True
 
        except Exception as e:
            self.get_logger().warn(f'Depth processing error: {str(e)}', throttle_duration_sec=2.0)
 
        return False
 
    def publish_emergency_stop(self, active: bool):
        """Publish emergency stop flag."""
        msg = Bool()
        msg.data = active
        self.emergency_stop_pub.publish(msg)
 
    def stop_robot(self):
        """Publish zero velocity to halt the robot immediately."""
        self.cmd_vel_pub.publish(Twist())
 
    def rotate_for_recovery(self):
        """Publish a rotation command to search for the leader."""
        cmd = Twist()
        cmd.angular.z = self.recovery_angular_speed
        self.cmd_vel_pub.publish(cmd)
 
    def monitor_loop(self):
        """Main safety monitoring loop."""
        now = self.get_clock().now()
 
        # --- Check for obstacles ---
        self.obstacle_detected = self.check_obstacle()
 
        # --- State machine ---
        if self.state == self.STATE_FOLLOWING:
 
            # Trigger emergency stop if leader too close or obstacle detected
            if self.leader_too_close or self.obstacle_detected:
                reason = 'leader too close' if self.leader_too_close else 'obstacle detected'
                self.get_logger().warn(f'Emergency stop triggered: {reason}')
                self.state = self.STATE_EMERGENCY
                self.publish_emergency_stop(True)
                self.stop_robot()
                return
 
            # Check for leader loss
            if self.last_leader_time is not None:
                time_since_leader = (now - self.last_leader_time).nanoseconds / 1e9
                if time_since_leader > self.leader_timeout:
                    self.get_logger().warn(
                        f'Leader lost — last seen {time_since_leader:.1f}s ago. Starting recovery.'
                    )
                    self.state = self.STATE_RECOVERING
                    self.recovery_start_time = now
                    self.publish_emergency_stop(True)
 
        elif self.state == self.STATE_EMERGENCY:
 
            # Stay stopped until conditions clear
            self.stop_robot()
 
            if not self.leader_too_close and not self.obstacle_detected:
                self.get_logger().info('Emergency condition cleared — resuming following')
                self.state = self.STATE_FOLLOWING
                self.publish_emergency_stop(False)
 
        elif self.state == self.STATE_RECOVERING:
 
            # Rotate to search for leader
            recovery_elapsed = (now - self.recovery_start_time).nanoseconds / 1e9
 
            if recovery_elapsed < self.recovery_rotation_time:
                self.rotate_for_recovery()
                self.get_logger().info(
                    f'Recovering — rotating to find leader '
                    f'({recovery_elapsed:.1f}s / {self.recovery_rotation_time}s)',
                    throttle_duration_sec=1.0
                )
            else:
                # Recovery failed — give up and stop
                self.get_logger().warn(
                    'Recovery failed — leader not reacquired after rotation. Stopping.'
                    # TODO: implement return-to-home when Nav2/SLAM is available
                )
                self.state = self.STATE_LOST
                self.stop_robot()
 
        elif self.state == self.STATE_LOST:
            # Hold position and wait for manual intervention
            self.stop_robot()
            self.get_logger().warn(
                'Robot is lost — waiting for manual intervention',
                throttle_duration_sec=5.0
            )
 
 
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