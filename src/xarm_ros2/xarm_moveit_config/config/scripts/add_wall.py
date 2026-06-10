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

        # Wall
        self.declare_parameter('wall_id', 'back_wall')
        self.declare_parameter('wall_size', [0.01, 1.2, 0.9])
        self.declare_parameter('wall_position', [-0.28, 0.29, 0.45])

        # Ground
        self.declare_parameter('ground_id', 'ground_plane')
        self.declare_parameter('ground_size', [0.91, 1.2, 0.01])
        self.declare_parameter('ground_position', [0.17, 0.29, -0.01])

        # Shared color
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
        color_rgba = self.get_parameter('color_rgba').value

        wall = self.make_box(
            frame_id,
            self.get_parameter('wall_id').value,
            self.get_parameter('wall_size').value,
            self.get_parameter('wall_position').value,
        )

        ground = self.make_box(
            frame_id,
            self.get_parameter('ground_id').value,
            self.get_parameter('ground_size').value,
            self.get_parameter('ground_position').value,
        )

        wall_color = self.make_color(wall.id, color_rgba)
        ground_color = self.make_color(ground.id, color_rgba)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [wall, ground]
        scene.object_colors = [wall_color, ground_color]

        self.pub.publish(scene)

        self.get_logger().info(
            f'Published wall "{wall.id}" and ground "{ground.id}" '
            f'with rgba={color_rgba} in frame "{frame_id}"'
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