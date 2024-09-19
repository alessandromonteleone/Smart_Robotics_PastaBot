## package creation
catkin_create_pkg pastabot_pkg rospy std_msgs gazebo_ros

## build and launch
catkin_make
source devel/setup.bash
roslaunch pastabot_pkg simulation.launch

## sdf and xacro includes
http://sdformat.org/tutorials?tut=composition&ver=1.7&cat=specification&
https://articulatedrobotics.xyz/tutorials/ready-for-ros/urdf#full-example