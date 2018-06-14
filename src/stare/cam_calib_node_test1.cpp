#include "ros/ros.h"
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
std::cout << "dfff\n";
    Mat temp;
    Mat img[20];
    string nazwa;
    vector<vector<Point2f> > wek;
    vector<Point3f> dobra;
    vector<vector<Point3f> > dobra_wek;

std::cout << "dfff1\n";
    namedWindow("image");

    for (int i = 0; i < 8; i++)
            for (int j = 0; j < 6; j++)
            {
                    dobra.push_back(Point3f(j, i, 0));
    }
    //while (1)
    //{

            // imshow("image", image);
            // if (waitKey(30) >= 0) break;
    //}

    for (int i = 0; i < 20; )
    {



            image.copyTo(img[i]);

            Size patternsize(6, 8);
            vector<Point2f> corners;

            bool patternfound = findChessboardCorners(img[i], patternsize, corners,
                    CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                    + CALIB_CB_FAST_CHECK);

            if (patternfound)
            {
                    Mat subpixel_BW;
                    cvtColor(img[i], subpixel_BW, CV_BGR2GRAY);
                    cornerSubPix(subpixel_BW, corners, Size(11, 11), Size(-1, -1),
                            TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                    //if (i == 0)
                            //cout << corners;
                    wek.push_back(corners);
                    dobra_wek.push_back(dobra);

                    namedWindow("Chessboard");
                    drawChessboardCorners(img[i], patternsize, Mat(corners), patternfound);
//                    imshow("Chessboard", img[i]);
                    i++;
            }

//            while (1)
//            {

//                    imshow("image", image);
//                    if (waitKey(30) >= 0) break;
//            }

    }
std::cout << "dfff2\n";
    Mat cameraMatrix, distCoeffs;
    vector <Mat>rv, tv;
    cout << dobra_wek.size() << endl;
    cout << wek.size() << endl;
    calibrateCamera(dobra_wek, wek,img[0].size(), cameraMatrix, distCoeffs,rv,tv);

    FileStorage file("/home/w/catkin_ws/src/cam_param.yml", cv::FileStorage::WRITE);
    file << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
std::cout << "dfff3\n";
    file.release();
std::cout << "dfff4\n";
    Mat tmp;
    undistort(img[0], tmp, cameraMatrix, distCoeffs);
    std::cout << "dfff5\n";
//    for (;;)
//    {
//            imshow("First Img undistorted", tmp);

//            if (waitKey(1) >= 0) break;

//    }




}

int main(int argc, char **argv)
{
    //initialize node
      ros::init(argc, argv, "cam_calin_node"); // "cam_calin_node" to nazwa noda

      // node handler
      ros::NodeHandle n;        // node handler to punkt dostepu do stworzonego noda

      // subsribe topic
      ros::Subscriber sub = n.subscribe("/cv_camera/image_raw", 1000, imageCallback);

      /*
       * Subscriber sub bedzie odbierał wiadomosci z topicu /cv_camera/image_raw i przesyłał je do funkcji
        imageCallback, która przyjmuję wiadomości typu const sensor_msgs::ImageConstPtr& msg
     */

      ros::Rate loop_rate(5);
      while (n.ok()) {

          ros::spinOnce();
          loop_rate.sleep();

      }

      return 0;

}
