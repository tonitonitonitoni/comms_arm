# How to use.  
- use `display.launch.py` to see the urdf in rviz.
- `gazebo.launch.py` will launch Gazebo and the wrist camera node.
- The gripper is quite rudimentary and does not work.  There is a separate URDF in gripper.urdf if you want to try working with it separately.
- There is a new file with a digger scoop which has been added to the launch file.  Go into the launch file and uncomment the urdfPath of your choice. 
- Unfortunately the arm with digger is not working properly in Gazebo.  

# Ideas for the gripper.
- https://github.com/a-price/robotiq_arg85_description this package is old but it looks similar to our gripper
- https://robotics.snowcron.com/robotics_ros2_c/robotic_arm_gripper_description.htm this is a tutorial which has you build and control a gripper from scratch

Other simulators will have a better time with the parallel linkages present in the current gripper.  I recommend Pybullet.
