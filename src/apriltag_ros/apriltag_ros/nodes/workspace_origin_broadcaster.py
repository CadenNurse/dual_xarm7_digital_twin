#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from apriltag_msgs.msg import AprilTagDetectionArray


class WorkspaceOriginBroadcaster(Node):
    def __init__(self):
        super().__init__('workspace_origin_broadcaster')

        self.declare_parameter('detections_topic', '/apriltag_detections')
        self.declare_parameter('camera_frame_fallback', 'G_camera_color_optical_frame')
        self.declare_parameter('origin_tag_id', 4)
        self.declare_parameter('origin_frame', 'workspace_origin')
        self.declare_parameter('tag_frame_prefix', 'tag_')
        self.declare_parameter('publish_tag_frame', True)

        self.detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.camera_frame_fallback = self.get_parameter('camera_frame_fallback').get_parameter_value().string_value
        self.origin_tag_id = self.get_parameter('origin_tag_id').get_parameter_value().integer_value
        self.origin_frame = self.get_parameter('origin_frame').get_parameter_value().string_value
        self.tag_frame_prefix = self.get_parameter('tag_frame_prefix').get_parameter_value().string_value
        self.publish_tag_frame = self.get_parameter('publish_tag_frame').get_parameter_value().bool_value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self.detections_callback,
            10,
        )

        self.get_logger().info(
            f'Listening on {self.detections_topic}, using tag {self.origin_tag_id} as {self.origin_frame}'
        )

    def detections_callback(self, msg: AprilTagDetectionArray):
        camera_frame = msg.header.frame_id if msg.header.frame_id else self.camera_frame_fallback

        for detection in msg.detections:
            if int(detection.id) != int(self.origin_tag_id):
                continue

            pose = detection.pose.pose.pose
            stamp = msg.header.stamp

            if self.publish_tag_frame:
                t_tag = TransformStamped()
                t_tag.header.stamp = stamp
                t_tag.header.frame_id = camera_frame
                t_tag.child_frame_id = f'{self.tag_frame_prefix}{self.origin_tag_id}'
                t_tag.transform.translation.x = pose.position.x
                t_tag.transform.translation.y = pose.position.y
                t_tag.transform.translation.z = pose.position.z
                t_tag.transform.rotation = pose.orientation
                self.tf_broadcaster.sendTransform(t_tag)

            t_origin = TransformStamped()
            t_origin.header.stamp = stamp
            t_origin.header.frame_id = camera_frame
            t_origin.child_frame_id = self.origin_frame
            t_origin.transform.translation.x = pose.position.x
            t_origin.transform.translation.y = pose.position.y
            t_origin.transform.translation.z = pose.position.z
            t_origin.transform.rotation = pose.orientation
            self.tf_broadcaster.sendTransform(t_origin)
            return


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceOriginBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# launch with:
# python3 workspace_origin_broadcaster.py
