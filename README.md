# Dual Xarm7 Digital Twin
Utilizing two xarm 7's from UFactory to manipulate objects in a real environment while simultaneously controlling them from Moveit2 using Rviz2 and simulating in Isaac Sim.
To run either Isaac Sim with Moveit2, or just the Moveit2 standalone to operate the manipulators, do the following.

Terminal 1: (Main Moveit/Rviz control GUI)  
enter xarm_ws: $ cd xarm_ws/  
enter pixi shell: $ pixi shell  
build: $ colcon build  
source: $ source install/setup.bash  
run code: $ pixi run launch_arms  

Terminal 2: (Joint state converter for Isaac Sim)  
enter xarm_ws: $ cd xarm_ws  
enter pixi shell: $ pixi shell  
run node bridge: $ ros2 run xarm_joint_bridge joint_bridge_node  

Terminal 3: (Isaac Sim)  
enter env_isaaclab: $ cd env_isaaclab  
activate environment: $ source ~/env_isaaclab/bin/activate  
export nessescities to run ROS2Bridge: $ export ROS_DISTRO=jazzy , export RMW_IMPLEMENTATION=rmw_fastrtps_cpp , export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/home/cadennurse/env_isaaclab/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib" , export ROS_DOMAIN_ID=(Your Domain ID)  
call Isaac Sim: $ isaacsim  

SUGGESTED FOR ISAAC SIM
Create a .sh file to run in one line using this format: (Sample code is named "run_isaacsim.sh" in a folder named "bin")

"run_isaacsim.sh"  
```bash
  #!/usr/bin/env bash  
  set -e  
  source ~/env_isaaclab/bin/activate  
  export ROS_DISTRO=jazzy  
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp  
  export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/home/cadennurse/env_isaaclab/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib"  
  export ROS_DOMAIN_ID=(Your Domain ID)  
  isaacsim   
```
(Make sure to link your directory after creating the file by typing in a terminal: $ chmod +x ~/bin/run_isaacsim.sh)
and call in your HOME directory as: $ ~/bin/run_isaacsim.sh


# Moveit Task Constructor Task Planning

To utilize the Moveit task Constructor (MTC) task planning, follow these steps:

Decide which task you wish to test (in /xarm_ws/src/mtc/mtc_tutorial/src)  
pick_place_eff.cpp: Simple left arm pick and place task where the place pose is identical but object pick pose can reside anywhere.  
mtc_node_shoff.cpp: A dual-arm handover using serial containers. Left arm picks object from any pose and brings to handover pose, then the right arm grasps the object during handover, left arm retreats while right arm places the object in a pre-determined pose.  
close_laptop.cpp: A single arm closing of a laptop given the laptop is within reach and the back of the lid is accessible. Closes by pushing the laptop to a hinge angle of 0.3 rads.  

Make sure to edit the launch file (in /xarm_ws/src/mtc/mtc_tutorial/launch)  
pick_place_demo1.launch.py: Use this and edit the ' executable="mtc_node_shoff", ' to, "pick_place_eff", or mtc_node_shoff".  
*If using close_laptop, just ensure the pixi.toml is corrrect: " launch_mtc = 'ros2 launch mtc_tutorial pick_place_demo1.launch.py" ' to ' launch_mtc = "ros2 launch mtc_tutorial laptop_manniulation.launch.py" '  

Terminal 1: (Main Moveit/Rviz control GUI)  
enter xarm_ws: $ cd xarm_ws/  
enter pixi shell: $ pixi shell  
build: $ colcon build  
source: $ source install/setup.bash  
run code: $ pixi run launch_arms  

Terminal 2: (MTC launching)  
enter xarm_ws: $ cd xarm_ws/   
enter pixi shell: $ pixi shell  
run code: $ pixi run launch_mtc  

Once RViz is open and MTC runs its course, you can view the planned path using the "Motion Planning Tasks", and "Motion Planning Tasks - Slider" displays.  
To activate the task and run it on the physical arms, open up a third terminal.  

Terminal 3: (Task launching to physical arms)  
enter xarm_ws: $ cd xarm_ws/  
enter pixi shell: $ pixi shell  
launch task: $ ros2 service call /execute_task std_srvs/srv/Trigger  















