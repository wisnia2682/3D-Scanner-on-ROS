#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <sstream>



int main(int argc, char **argv)
{

    //initialize node
      ros::init(argc, argv, "test_node"); // to nazwa noda

      // node handler
      ros::NodeHandle n;        // node handler to punkt dostepu do stworzonego noda

      // subsribe topic
      ros::Subscriber sub = n.subscribe("/cv_camera/image_raw", 1000, imageCallback);

      ros::Rate loop_rate(5);

      while (n.ok()) {
       ros::spinOnce();
       //std::cout<<"po spin\n";
       loop_rate.sleep();
      }
    return 0;
}
