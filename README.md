# Implementation of UR5 pick and place in ROS-Gazebo with a USB cam and vacuum grippers. 


## How to use this code
### step 1 

This project was tested in Ubuntu 16.04 with ROS [kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).

### step 2

Make sure you have installed Python2.7 and some useful libraries/packages, such as Numpy, cv2, etc.

### step 3
Install ur_gazebo, moveit, gazebo_ros
  ```
  $ sudo apt-get install ros-kinetic-moveit
  $ sudo apt-get install ros-kinetic-ur-gazebo
  $ sudo apt-get install ros-kinetic-gazebo-ros
  ```

### step 4
- Assuming your universal robot workspace is named as `arm_ws`, copy the package (ur5_ROS-Gazebo,universal_robot,usb_cam) into `arm_ws/src/`
  ```
  $ mkdir -p arm_ws/src 
  $ cd arm_ws/src
  ```

### step 5 
catkin in arm_ws.
  ```
  $ catkin_make
  $ source devel/setup.bash  
  ```
### step 6
Run the code with ROS and Gazebo
  ```
  $ roslaunch ur5_notebook initialize.launch 
  ```
