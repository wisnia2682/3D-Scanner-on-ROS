3D Scanner on ROS
=====================

3D laser based scanner made by studens from Poznań University of Technology

Steps to run the software:

1. Download repository, and place it into your catkin_ws/src as laser_scaner

2. Open terminal and copy those lines:
    
    cd ~/catkin_ws catkin_make
    
    rosparam set cv_camera/device_id 0
    
    - new terminal: roscore
                
    - new terminal: rosrun cv_camera cv_camera_node
                    
3. To calibrate camera:
    
    - new terminal: rosrun laser_scan cam_calib_node
    
4. To run 3D Scanner: 

    - new terminal: rosrun laser_scan laser_scan_node
                    




    
