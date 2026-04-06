#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import math

# ROS 2 Message Imports
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped

# TF2 Imports
import tf2_ros
import tf2_geometry_msgs 

class LeaderDetectionNode(Node):
    def __init__(self):
        super().__init__('leader_detection_node')

        self.declare_parameter('leader_tag_id', 5)
        self.declare_parameter('tag_family', '36h11')
        self.declare_parameter('base_frame', 'base_link')

        self.leader_id = self.get_parameter('leader_tag_id').value
        self.tag_family = self.get_parameter('tag_family').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, '/leader/pose', 10)

        self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10
        )

        self.get_logger().info(f'--- Leader Detection Node Online ---')
        self.get_logger().info(f'Target: {self.tag_family} ID {self.leader_id}')
        self.get_logger().info(f'Reference Frame: {self.base_frame}')

    def tag_callback(self, msg: AprilTagDetectionArray):
        leader_found = False
        for detection in msg.detections:
            # Handle cases where ID might be a list or a single int
            tag_id = detection.id[0] if isinstance(detection.id, list) else detection.id
            if tag_id == self.leader_id:
                leader_found = True
                break

        if not leader_found:
            return  
        
        tag_frame = f'{self.tag_family}:{self.leader_id}'

        try:

            now = rclpy.time.Time() 
            t = self.tf_buffer.lookup_transform(
                self.base_frame,
                tag_frame,
                msg.header.stamp,
                Duration(seconds=0.1)
            )

            #Construct the PoseStamped message
            leader_pose = PoseStamped()
            leader_pose.header.stamp = msg.header.stamp
            leader_pose.header.frame_id = self.base_frame
            
            leader_pose.pose.position.x = t.transform.translation.x
            leader_pose.pose.position.y = t.transform.translation.y
            leader_pose.pose.position.z = t.transform.translation.z
            leader_pose.pose.orientation = t.transform.rotation

            dist = math.sqrt(
                leader_pose.pose.position.x**2 + 
                leader_pose.pose.position.y**2 + 
                leader_pose.pose.position.z**2
            )

            #Publish and Log
            self.pose_pub.publish(leader_pose)

            self.get_logger().info(
                f'LEADER DETECTED | Dist: {dist:.2f}m | '
                f'Rel (x,y,z): ({leader_pose.pose.position.x:.2f}, '
                f'{leader_pose.pose.position.y:.2f}, {leader_pose.pose.position.z:.2f})',
                throttle_duration_sec=1.0
            )

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().debug(f'TF Lookup failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = LeaderDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down detection node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()