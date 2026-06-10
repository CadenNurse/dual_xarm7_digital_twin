#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class SceneObjectAdder(Node):
    def __init__(self):
        super().__init__('scene_object_adder')

        # Back wall
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('wall_id', 'back_wall')
        self.declare_parameter('wall_size', [0.01, 1.2, 0.9])
        self.declare_parameter('wall_position', [-0.28, 0.29, 0.45])

        # Ground / floor
        self.declare_parameter('ground_id', 'ground_plane')
        self.declare_parameter('ground_size', [0.91, 1.2, 0.01])
        self.declare_parameter('ground_position', [0.17, 0.29, -0.01])

        self.pub = self.create_publisher(CollisionObject, 'collision_object', 10)
        self.timer = self.create_timer(3.0, self.add_objects_once)
        self.done = False

    def make_box(self, frame_id, object_id, size, position):
        obj = CollisionObject()
        obj.header.frame_id = frame_id
        obj.id = object_id

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(size[0]), float(size[1]), float(size[2])]

        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])

        obj.primitives = [box]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD
        return obj

    def add_objects_once(self):
        if self.done:
            return

        frame_id = self.get_parameter('frame_id').value

        wall = self.make_box(
            frame_id=frame_id,
            object_id=self.get_parameter('wall_id').value,
            size=self.get_parameter('wall_size').value,
            position=self.get_parameter('wall_position').value,
        )

        ground = self.make_box(
            frame_id=frame_id,
            object_id=self.get_parameter('ground_id').value,
            size=self.get_parameter('ground_size').value,
            position=self.get_parameter('ground_position').value,
        )

        self.pub.publish(wall)
        self.pub.publish(ground)

        self.get_logger().info(
            f'Published collision objects: "{wall.id}" and "{ground.id}" in frame "{frame_id}"'
        )

        self.done = True
        self.timer.cancel()


def main():
    rclpy.init()
    node = SceneObjectAdder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()