# dual_xarm7_digital_twin
Utilizing two xarm 7's from UFactory to manipulate objects in a real environment while simultaneously controlling them from Moveit2 using Rviz2 and simulating in Isaac Sim.
To run either Isaac Sim with Moveit2, or just the Moveit2 standalone to operate the manipulators, do the following.

Terminal 1: (Main Moveit/Rviz control GUI)
enter xarm_ws ¨ cd xarm_ws/
enter pixi shell ¨ pixi shell
build ¨ colcon build
source ¨ source install/setup.bash
run code ¨ pixi run launch_process

Terminal 2: (Joint state converter for Isaac Sim. NOT needed for only moving arms)
enter xarm_ws ¨ cd xarm_ws
enter pixi shell ¨ pixi shell
run node bridge ¨ ros2 run xarm_joint_bridge joint_bridge_node

Terminal 3: (Isaac Sim)
enter env_isaaclab ¨ cd env_isaaclab
activate environment ¨ source ~/env_isaaclab/bin/activate
export nessescities to run ROS2Bridge ¨ export ROS_DISTRO=jazzy , export RMW_IMPLEMENTATION=rmw_fastrtps_cpp , export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/home/cadennurse/env_isaaclab/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib" , export ROS_DOMAIN_ID=(Your Domain ID)
call Isaac Sim ¨ isaacsim

or create a .sh file to run in one line using this format: (Sample is named run_isaacsim.sh in a folder named bin)
#!/usr/bin/env bash
set -e
source ~/env_isaaclab/bin/activate
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/home/cadennurse/env_isaaclab/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib"
export ROS_DOMAIN_ID=(Your Domain ID)
isaacsim 

and call in your home directory as: ~/bin/run_isaacsim.sh
(Make sure to link your directory after creating the file by typing in a terminal ¨ chmod -x ~/bin/isaacsim.sh)
