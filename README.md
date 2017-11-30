# BaxterTrack Repository

This repository contains the packages which include functionality to do basic tracking with Kinect and transforms to Torso space.


## Installation
Install ROS.  http://wiki.ros.org/ROS/Tutorials
git clone "https://github.com/kbhagat6/BaxterTrack.git"  in your home folder




## Steps to running code

cd BaxterTrack
./baxter.sh       --> Sourcing ROS Environment Setup Script

Most of the code will be located in the baxter_setup_tf package. When running a new node, make sure ./baxter.sh is executed in the every new terminal 
that is opened. if this is unclear, visit: http://sdk.rethinkrobotics.com/wiki/Hello_Baxter 

Before running anything with baxter, baxter has to be enabled. 
rosrun baxter_tools enable_robot.py -e




In baxter_pack folder, rosrun freenect_launch freenect.launch   --> launches the kinect launch file
In a new terminal,  rosrun baxter_setup_tf kinect_broadcaster
In a new terminal,  rosrun baxter_setup_tf red_tracking    //will be changed for linemod
--Verify that it's tracking. 
In a new terminal,  rosrun baxter_setup_tf waypoint_test.py -l right -s 0.1 



## Calibration
Have printed alvar marker available, which is located in baxter_setup_tf folder
Have desired arm camera point to the alvar marker and have kinect point to alvar marker. 

Make sure the cameras are open by looking at the list of cameras  (rosrun baxter_tools camera_control.py -l)  if one isn't open do rosrun baxter_tools camera_control.py -o right_hand_camera    or left_hand_camera

visualize on rviz to check if alvar marker is in view of both cameras. 
rosrun rviz rviz.  Change the frame to camera_link vs. right or left_hand camera frame.  
Look up tutorials on rviz if this isn't clear.  

To run calibration node for left camera: 
roslaunch baxter_kinect_calibration baxter_bundle_calibrate.launch

right camera:
roslaunch baxter_kinect_calibration baxter_bundle_calibrate_right.launch

if the terminal is showing left_hand(or right) to ar_marker_8 transformation and coordinates, then the cameras are aligned properly. 

in a new terminal, rosrun tf tf_echo /torso /camera_link  
which shows kinect coordinates w.r.s.t torso. 
when in doubt, check the other transformations from either hand to the alvar_marker (ar_marker_8) frame

Once you get the values, copy and paste them into appropriate function in the kinect_broadcaster file in the baxter_setup_tf package. (should be straightforward). 
You can now exit the calibration node and run the other code. 





## Contact

Krishan Bhagat
kbhagat6@gatech.edu
