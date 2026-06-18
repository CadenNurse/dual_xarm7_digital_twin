#!/usr/bin/env python3
import sys
import rospy
from control_msgs.action import GripperCommand
from actionlib import SimpleActionClient

def control_grippers(close=True):
    # Initialize action clients
    client_L = SimpleActionClient('/L_xarm_gripper/gripper_action', GripperCommand)
    client_R = SimpleActionClient('/R_xarm_gripper/gripper_action', GripperCommand)
    
    # Wait for servers
    print("Waiting for left gripper action server...")
    client_L.wait_for_server()
    print("Waiting for right gripper action server...")
    client_R.wait_for_server()
    
    # Create goal
    goal = GripperCommand.Goal()
    goal.command.max_effort = 0.0
    
    if close:
        # Close: position 0.86 rad (fully closed, per xArm docs)
        goal.command.position = 0.86
        print("Closing both grippers...")
    else:
        # Open: position 0.0 rad (fully open)
        goal.command.position = 0.0
        print("Opening both grippers...")
    
    # Send goals to both grippers simultaneously
    print("Sending close/open command to both grippers...")
    client_L.send_goal(goal)
    client_R.send_goal(goal)
    
    # Wait for results
    client_L.wait_for_result()
    client_R.wait_for_result()
    
    print("Gripper action complete!")

if __name__ == '__main__':
    rospy.init_node('gripper_control_node')
    
    # Usage: python gripper_control.py close  OR  python gripper_control.py open
    if len(sys.argv) < 2:
        print("Usage: python gripper_control.py <close|open>")
        sys.exit(1)
    
    command = sys.argv[1].lower()
    if command == 'close':
        control_grippers(close=True)
    elif command == 'open':
        control_grippers(close=False)
    else:
        print("Invalid command. Use 'close' or 'open'")
        sys.exit(1)

    # close grippers (paste in terminal)
    # ros2 action send_goal /L_xarm_gripper/gripper_action control_msgs/action/GripperCommand "{command: {position: 0.86, max_effort: 0.0}}"
    # ros2 action send_goal /R_xarm_gripper/gripper_action control_msgs/action/GripperCommand "{command: {position: 0.86, max_effort: 0.0}}"

    # open grippers (paste in terminal)
    # ros2 action send_goal /L_xarm_gripper/gripper_action control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 0.0}}"
    # ros2 action send_goal /R_xarm_gripper/gripper_action control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 0.0}}"