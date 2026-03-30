import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math
 
 
class PIDController:
    """Generic PID controller."""
 
    def __init__(self, kp: float, ki: float, kd: float, max_output: float = 1.0):
        # Adjust kp, ki and kd to balance speed, accuracy and stability
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
 
        self._prev_error = 0.0
        self._integral = 0.0
 
    def compute(self, error: float, dt: float) -> float:
        """Compute PID output given error and time delta."""
        if dt <= 0.0:
            return 0.0
 
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt
        self._prev_error = error
 
        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)
 
        # Clamp output to max
        return max(-self.max_output, min(self.max_output, output))
 
    def reset(self):
        """Reset controller state."""
        self._prev_error = 0.0
        self._integral = 0.0
 
 
class PIDControllerNode(Node):
 
    def __init__(self):
        super().__init__('pid_controller')
 
        # Declare parameters so they can be tuned without recompiling
        self.declare_parameter('desired_distance', 0.8)     # metres to maintain behind leader
 
        self.declare_parameter('linear_kp', 0.5)
        self.declare_parameter('linear_ki', 0.01)
        self.declare_parameter('linear_kd', 0.1)
        self.declare_parameter('max_linear_speed', 0.5)     # m/s
 
        self.declare_parameter('angular_kp', 1.0)
        self.declare_parameter('angular_ki', 0.0)
        self.declare_parameter('angular_kd', 0.1)
        self.declare_parameter('max_angular_speed', 1.0)    # rad/s
 
        self.declare_parameter('control_frequency', 20.0)   # Hz
 
        # Read parameters
        self.desired_distance = self.get_parameter('desired_distance').value
        self.control_frequency = self.get_parameter('control_frequency').value
 
        # Initialise PID controllers
        self.linear_pid = PIDController(
            # Linear PID uses actual_distancec - desired distance as the error
            kp=self.get_parameter('linear_kp').value,
            ki=self.get_parameter('linear_ki').value,
            kd=self.get_parameter('linear_kd').value,
            max_output=self.get_parameter('max_linear_speed').value
        )
        self.angular_pid = PIDController(
            # Angular PID uses the error angle as the error
            kp=self.get_parameter('angular_kp').value,
            ki=self.get_parameter('angular_ki').value,
            kd=self.get_parameter('angular_kd').value,
            max_output=self.get_parameter('max_angular_speed').value
        )
 
        # State
        self.leader_pose: PoseStamped | None = None
        self.last_time = self.get_clock().now()
        self.emergency_stop = False
 
        # Subscriber
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/leader/pose',
            self.pose_callback,
            10
        )
 
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
 
        # Control loop timer
        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_loop
        )
 
        self.get_logger().info('PID controller node started')
        self.get_logger().info(f'Desired following distance: {self.desired_distance}m')
 
    def pose_callback(self, msg: PoseStamped):
        """Receive leader pose from detection node."""
        self.leader_pose = msg
 
    def compute_errors(self, pose: PoseStamped) -> tuple[float, float]:
        """
        Compute distance and angular errors from leader pose.
        The detection node publishes the leader position relative to the robot,
        so x = forward distance, y = lateral offset.
        """
        x = pose.pose.position.x
        y = pose.pose.position.y
 
        actual_distance = math.sqrt(x**2 + y**2)
        distance_error = actual_distance - self.desired_distance
 
        # Angle to leader relative to robot forward axis
        angular_error = math.atan2(y, x)
 
        return distance_error, angular_error
 
    def control_loop(self):
        """Main PID control loop, runs at control_frequency Hz."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
 
        cmd = Twist()
 
        # If no leader pose received yet, or emergency stop, publish zero velocity
        if self.leader_pose is None or self.emergency_stop:
            if self.leader_pose is None:
                self.get_logger().warn('No leader pose received, staying still', throttle_duration_sec=2.0)
            self.cmd_vel_pub.publish(cmd)
            return
 
        distance_error, angular_error = self.compute_errors(self.leader_pose)
 
        # Log errors at a throttled rate to avoid flooding the terminal
        self.get_logger().info(
            f'Distance error: {distance_error:.3f}m | Angular error: {math.degrees(angular_error):.1f}deg',
            throttle_duration_sec=0.5
        )
 
        cmd.linear.x = self.linear_pid.compute(distance_error, dt)
        cmd.angular.z = self.angular_pid.compute(angular_error, dt)
 
        self.cmd_vel_pub.publish(cmd)
 
    def stop(self):
        """Publish zero velocity and reset PID state."""
        self.cmd_vel_pub.publish(Twist())
        self.linear_pid.reset()
        self.angular_pid.reset()
        self.get_logger().info('PID controller stopped')
 
 
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