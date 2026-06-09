"""
xarm_joint_bridge — joint_bridge_node.py
========================================
Subscribes to each physical xArm7 joint_states topic, normalizes the
incoming names, reorders joints into canonical xArm7 order, and republishes
Isaac-facing JointState topics.

This version is designed for the case where Isaac Sim's articulation
controller expects the internal DOF names:
    joint1 ... joint7

That means:
- Incoming names may be either unprefixed (joint1) or prefixed (L_joint1,
  R_joint1).
- Outgoing names default to unprefixed canonical names (joint1...joint7).
- Left and right robots are separated by topic, not by joint-name prefix.

Default topics:
  /xarm_left/joint_states   -> /L/joint_states
  /xarm_right/joint_states  -> /R/joint_states

If you really need prefixed output names for a different USD/controller
setup, set rename_joints:=true and provide left_prefix/right_prefix.

Usage:
  ros2 run xarm_joint_bridge joint_bridge_node

Useful overrides:

  ros2 run xarm_joint_bridge joint_bridge_node --ros-args \
      -p rename_joints:=true \
      -p left_prefix:=L_ \
      -p right_prefix:=R_
"""

import math
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState

XARM7_JOINT_NAMES = [
    "joint1", "joint2", "joint3",
    "joint4", "joint5", "joint6", "joint7",
]


def _make_qos(depth: int) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _safe_value(values, idx: int) -> float:
    if idx >= len(values):
        return 0.0
    value = values[idx]
    if isinstance(value, float) and math.isnan(value):
        return 0.0
    return value


class ArmBridgeChannel:
    def __init__(
        self,
        node: Node,
        in_topic: str,
        out_topic: str,
        prefix: str,
        rename: bool,
        qos: QoSProfile,
    ):
        self._node = node
        self._prefix = prefix
        self._rename = rename
        self._pub = node.create_publisher(JointState, out_topic, qos)
        self._sub = node.create_subscription(JointState, in_topic, self._callback, qos)
        node.get_logger().info(
            f"Bridge: {in_topic} -> {out_topic} (prefix='{prefix}', rename={rename})"
        )

    def _candidate_names(self, canonical_name: str):
        names = [canonical_name]
        if self._prefix:
            names.append(f"{self._prefix}{canonical_name}")
        return names

    def _find_index(self, name_to_idx: Dict[str, int], canonical_name: str) -> Optional[int]:
        for candidate in self._candidate_names(canonical_name):
            if candidate in name_to_idx:
                return name_to_idx[candidate]
        return None

    def _output_name(self, canonical_name: str) -> str:
        if self._rename:
            return f"{self._prefix}{canonical_name}"
        return canonical_name

    def _callback(self, msg: JointState) -> None:
        out = JointState()
        out.header = msg.header

        name_to_idx = {name: i for i, name in enumerate(msg.name)}

        for canonical_name in XARM7_JOINT_NAMES:
            idx = self._find_index(name_to_idx, canonical_name)

            out.name.append(self._output_name(canonical_name))

            if idx is None:
                self._node.get_logger().warn(
                    f"Joint '{canonical_name}' not found on incoming topic. "
                    f"Accepted names: {self._candidate_names(canonical_name)}. Padding with 0.0",
                    throttle_duration_sec=5.0,
                )
                out.position.append(0.0)
                out.velocity.append(0.0)
                out.effort.append(0.0)
                continue

            out.position.append(_safe_value(msg.position, idx))
            out.velocity.append(_safe_value(msg.velocity, idx))
            out.effort.append(_safe_value(msg.effort, idx))

        self._pub.publish(out)


class XArmJointBridgeNode(Node):
    def __init__(self):
        super().__init__("xarm_joint_bridge")

        left_in = "/L_xarm/joint_states"
        right_in = "/R_xarm/joint_states"
        left_out = "/L/joint_states"
        right_out = "/R/joint_states"
        left_prefix = "L_"
        right_prefix = "R_"
        rename = False
        depth = 10

        qos = _make_qos(depth)

        self._left = ArmBridgeChannel(self, left_in, left_out, left_prefix, rename, qos)
        self._right = ArmBridgeChannel(self, right_in, right_out, right_prefix, rename, qos)

        self.get_logger().info("xarm_joint_bridge node ready.")
        self.get_logger().info(
            f"Configured inputs: left={left_in}, right={right_in} | outputs: left={left_out}, right={right_out}"
        )
        self.get_logger().info(
            f"rename_joints={rename} (False means Isaac-facing names stay canonical: joint1...joint7)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = XArmJointBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
