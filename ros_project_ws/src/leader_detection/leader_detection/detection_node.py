#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped

class LeaderDetectionNode(Node):
    def __init__(self):
        super().__init__('leader_detection_node')

        # Publisher for leader pose
        self.pose_pub = self.create_publisher(PoseStamped, '/leader/pose', 10)

        # Subscriber to AprilTag detections
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10
        )

        self.get_logger().info('Leader Detection Node Started')

    def tag_callback(self, msg: AprilTagDetectionArray):
        if not msg.detections:
            self.get_logger().info('No tags detected')
            return

        tag = msg.detections[0]

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'camera_link'

        # AprilTag pose (relative to camera)
        pose_msg.pose = tag.pose.pose.pose

        # Publish leader pose
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f'Published leader pose for tag ID {tag.id[0]}')

def main(args=None):
    rclpy.init(args=args)
    node = LeaderDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
