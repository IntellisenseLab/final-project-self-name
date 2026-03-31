
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import tf2_ros


class LeaderDetectionNode(Node):
    def __init__(self):
        super().__init__('leader_detection_node')

        self.declare_parameter('leader_tag_id', 0)
        self.declare_parameter('tag_family', '36h11')

        self.leader_id = self.get_parameter('leader_tag_id').value
        self.tag_family = self.get_parameter('tag_family').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, '/leader/pose', 10)

        self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10
        )

        self.get_logger().info('Detection node started')

        self.get_logger().info(
            f'Tracking tag ID {self.leader_id}, family {self.tag_family}'
        )

    def tag_callback(self, msg: AprilTagDetectionArray):

        # Check if leader tag is in this frame
        leader_seen = False
        for detection in msg.detections:
            tag_id = detection.id[0] if isinstance(detection.id, list) else detection.id
            if tag_id == self.leader_id:
                leader_seen = True
                break

        if not leader_seen:
            return

        # TF frame names
        tag_frame = f'{self.tag_family}:{self.leader_id}'
        camera_frame = 'kinect_rgb'  

        try:
            transform = self.tf_buffer.lookup_transform(
                camera_frame,
                tag_frame,
                msg.header.stamp,   
                Duration(seconds=0.1)
            )

        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:

            self.get_logger().warn(
                f'TF lookup failed: {str(e)}',
                throttle_duration_sec=2.0
            )
            return

        # Create pose message
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = camera_frame

        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z

        pose.pose.orientation = transform.transform.rotation

        # Publish
        self.pose_pub.publish(pose)

        self.get_logger().info(
            f'Leader at x={pose.pose.position.x:.3f}, '
            f'y={pose.pose.position.y:.3f}, '
            f'z={pose.pose.position.z:.3f} m',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = LeaderDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()