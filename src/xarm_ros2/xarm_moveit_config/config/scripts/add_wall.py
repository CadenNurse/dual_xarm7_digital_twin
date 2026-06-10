#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class WallAdder(Node):
    def __init__(self):
        super().__init__('wall_adder')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('wall_id', 'back_wall')
        self.declare_parameter('wall_size', [0.01, 1.3, 1.0])
        self.declare_parameter('wall_position', [-0.28, 0.29, 0.5])
        self.timer = self.create_timer(3.0, self.add_wall_once)
        self.done = False

    def add_wall_once(self):
        if self.done:
            return
        frame_id = self.get_parameter('frame_id').value
        wall_id = self.get_parameter('wall_id').value
        wall_size = self.get_parameter('wall_size').value
        wall_position = self.get_parameter('wall_position').value

        wall = CollisionObject()
        wall.header.frame_id = frame_id
        wall.id = wall_id

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(wall_size[0]), float(wall_size[1]), float(wall_size[2])]

        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x = float(wall_position[0])
        pose.position.y = float(wall_position[1])
        pose.position.z = float(wall_position[2])

        wall.primitives = [box]
        wall.primitive_poses = [pose]
        wall.operation = CollisionObject.ADD

        self.pubs = self.create_publisher(CollisionObject, 'collision_object', 10)
        self.pubs.publish(wall)
        
        self.get_logger().info(f'Published wall "{wall_id}" in frame "{frame_id}"')
        self.done = True
        self.timer.cancel()


def main():
    rclpy.init()
    node = WallAdder()
    rclpy.spin(node)


if __name__ == '__main__':
    main()