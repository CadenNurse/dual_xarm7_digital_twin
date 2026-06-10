#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject, PlanningScene, ObjectColor
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA


class SceneObjectAdder(Node):
    def __init__(self):
        super().__init__('scene_object_adder')

        self.declare_parameter('frame_id', 'world')

        # Shared box dimensions: 3 cm x 3 cm x 1 m
        self.declare_parameter('box_size', [0.03, 0.03, 1.0])

        # Four box positions
        self.declare_parameter('box_1_position', [-0.19, -0.292, 0.5])
        self.declare_parameter('box_2_position', [-0.19,  0.875, 0.5])
        self.declare_parameter('box_3_position', [ 0.61, -0.292, 0.5])
        self.declare_parameter('box_4_position', [ 0.61,  0.875, 0.5])

        # Medium gray, alpha 0.8
        self.declare_parameter('color_rgba', [0.5, 0.5, 0.5, 0.8])

        self.pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
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

    def make_color(self, object_id, rgba):
        c = ObjectColor()
        c.id = object_id
        c.color = ColorRGBA(
            r=float(rgba[0]),
            g=float(rgba[1]),
            b=float(rgba[2]),
            a=float(rgba[3]),
        )
        return c

    def add_objects_once(self):
        if self.done:
            return

        frame_id = self.get_parameter('frame_id').value
        box_size = self.get_parameter('box_size').value
        color_rgba = self.get_parameter('color_rgba').value

        positions = [
            self.get_parameter('box_1_position').value,
            self.get_parameter('box_2_position').value,
            self.get_parameter('box_3_position').value,
            self.get_parameter('box_4_position').value,
        ]

        objects = []
        colors = []

        for i, position in enumerate(positions, start=1):
            obj_id = f'box_{i}'
            obj = self.make_box(frame_id, obj_id, box_size, position)
            color = self.make_color(obj_id, color_rgba)
            objects.append(obj)
            colors.append(color)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects
        scene.object_colors = colors

        self.pub.publish(scene)

        self.get_logger().info(
            f'Published 4 boxes in frame "{frame_id}" with size={box_size} and rgba={color_rgba}'
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