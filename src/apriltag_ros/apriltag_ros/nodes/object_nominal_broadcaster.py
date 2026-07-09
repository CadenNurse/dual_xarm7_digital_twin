#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class ObjectNominalBroadcaster(Node):
    def __init__(self):
        super().__init__("object_nominal_broadcaster")

        self.declare_parameter("detections_topic", "/apriltag_detections")
        self.declare_parameter("target_tag_id", 21)
        self.declare_parameter("tag_frame_prefix", "tag_")
        self.declare_parameter("object_frame", "object_nominal")
        self.declare_parameter("offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("offset_rpy", [0.0, 0.0, 0.0])

        detections_topic = self.get_parameter("detections_topic").value
        self.target_tag_id = int(self.get_parameter("target_tag_id").value)
        self.tag_frame_prefix = self.get_parameter("tag_frame_prefix").value
        self.object_frame = self.get_parameter("object_frame").value

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            detections_topic,
            self.detections_callback,
            10,
        )

        self.get_logger().info(
            f"Listening on {detections_topic}, publishing "
            f"{self.tag_frame_prefix}{self.target_tag_id} -> {self.object_frame}"
        )

    def get_detection_id(self, det):
        if isinstance(det.id, int):
            return int(det.id)
        if isinstance(det.id, (list, tuple)) and len(det.id) > 0:
            return int(det.id[0])
        return None

    def detections_callback(self, msg):
        seen_target = False
        for det in msg.detections:
            det_id = self.get_detection_id(det)
            if det_id == self.target_tag_id:
                seen_target = True
                break

        if not seen_target:
            return

        offset_xyz = self.get_parameter("offset_xyz").value
        offset_rpy = self.get_parameter("offset_rpy").value
        q = quaternion_from_euler(offset_rpy[0], offset_rpy[1], offset_rpy[2])

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f"{self.tag_frame_prefix}{self.target_tag_id}"
        t.child_frame_id = self.object_frame

        t.transform.translation.x = float(offset_xyz[0])
        t.transform.translation.y = float(offset_xyz[1])
        t.transform.translation.z = float(offset_xyz[2])

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = ObjectNominalBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# launch using:
# ros2 run apriltag_ros object_nominal_broadcaster \
#   --ros-args \
#   -p detections_topic:=/detections \
#   -p target_tag_id:=0 \
#   -p world_frame:=world \
#   -p object_frame:=object_nominal \
#   -p offset_xyz:="[0.0,0.0,0.05]" \
#   -p offset_rpy:="[0.0,0.0,0.0]"