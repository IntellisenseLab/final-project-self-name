import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
 
 
class MockLeader(Node):
 
    def __init__(self):
        super().__init__('mock_leader')
 
        # Parameters
        self.declare_parameter('mode', 'static')        # 'static' or 'moving'
        self.declare_parameter('static_x', 1.5)         # metres ahead of robot
        self.declare_parameter('static_y', 0.0)         # metres to the side
        self.declare_parameter('move_speed', 0.1)        # metres per second (moving mode)
        self.declare_parameter('publish_rate', 20.0)     # Hz
 
        self.mode = self.get_parameter('mode').value
        self.static_x = self.get_parameter('static_x').value
        self.static_y = self.get_parameter('static_y').value
        self.move_speed = self.get_parameter('move_speed').value
        publish_rate = self.get_parameter('publish_rate').value
 
        # State for moving mode
        self.current_x = self.static_x
        self.current_y = self.static_y
        self.elapsed = 0.0
        self.dt = 1.0 / publish_rate
 
        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/leader/pose', 10)
 
        # Timer
        self.timer = self.create_timer(self.dt, self.publish_pose)
 
        self.get_logger().info(f'Mock leader started in [{self.mode}] mode')
        if self.mode == 'static':
            self.get_logger().info(f'Publishing static pose: x={self.static_x}m, y={self.static_y}m')
        else:
            self.get_logger().info(f'Leader moving forward at {self.move_speed} m/s')
 
    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
 
        if self.mode == 'static':
            msg.pose.position.x = self.static_x
            msg.pose.position.y = self.static_y
 
        elif self.mode == 'moving':
            # Leader moves forward over time, with a gentle side-to-side drift
            # so you can see both linear and angular PID responding
            self.elapsed += self.dt
            self.current_x = self.static_x + (self.move_speed * self.elapsed)
            self.current_y = 0.3 * math.sin(0.5 * self.elapsed)   # gentle sine drift
 
            msg.pose.position.x = self.current_x
            msg.pose.position.y = self.current_y
 
        else:
            self.get_logger().warn(f'Unknown mode [{self.mode}], defaulting to static')
            msg.pose.position.x = self.static_x
            msg.pose.position.y = self.static_y
 
        msg.pose.position.z = 0.0
 
        # Orientation — identity quaternion (no rotation)
        msg.pose.orientation.w = 1.0
 
        self.pose_pub.publish(msg)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = MockLeader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()