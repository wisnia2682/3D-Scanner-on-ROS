#include "ros/ros.h"
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

using namespace cv;
using namespace std;


cv::Mat img[20];
string nazwa;
vector<vector<Point2f> > wek;
vector<Point3f> dobra;
vector<vector<Point3f> > dobra_wek;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat temp = cv_bridge::toCvShare(msg, "bgr8")->image;

    std::cout<<"obraz\n";

    //namedWindow("image");

    for (int i = 0; i < 8; i++)
            for (int j = 0; j < 6; j++)
            {
                    dobra.push_back(Point3f(j, i, 0));
            }
    std::cout<<"1111111\n";

    // wyswietlanie macierzy
    /*while (1)
    {
            imshow("image", temp);
            if (waitKey(30) >= 0) break;
    }
    */

    imshow("imm",temp);
    cv::waitKey(0);
    for (int i = 0; i < 20; )
    {


           temp.copyTo(img[i]);

           std::cout<<"skopiowano\n";


            Size patternsize(6, 8);
            vector<Point2f> corners;

            bool patternfound = findChessboardCorners(img[i], patternsize, corners,
                    CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                    + CALIB_CB_FAST_CHECK);

             std::cout<<"33333\n";
             //wyswietlanie macierzy








            if (patternfound)
            {
                    cout<<"znaleziono wzór\n";
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
                    imshow("Chessboard", img[i]);
                    i++;
            }

           // else{
                //std::cout<<"nie znaleziono wzoru\n";
           // }

            /*                          // wyswietlanie obrazu z kamery
            while (1)
            {
                    imshow("image", temp);
                    if (waitKey(30) >= 0) break;
            }
            */


    }




    Mat cameraMatrix, distCoeffs;
    vector <Mat>rv, tv;
    cout << dobra_wek.size() << endl;
    cout << wek.size() << endl;
    calibrateCamera(dobra_wek, wek,img[0].size(), cameraMatrix, distCoeffs,rv,tv);

    std::cout<<"zapis pliku\n";

    FileStorage file("/home/w/catkin_ws/src/laser_scan/cam_param.yml", cv::FileStorage::WRITE);
    //Mat cameraMatrix = (Mat_<double>(3, 3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
    //Mat distCoeffs = (Mat_<double>(5, 1) << 0.1, 0.01, -0.001, 0, 0);
    file << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;

    file.release();


    /*FileStorage fs2("cam_param.yml", FileStorage::READ);
    Mat cameraMatrix2, distCoeffs2;
    fs2["cameraMatrix"] >> cameraMatrix2;
    fs2["distCoeffs"] >> distCoeffs2;
    fs2.release();
    Mat tmp;
    cap >> temp;
    undistort(temp, tmp, cameraMatrix2, distCoeffs2);*/

    Mat tmp;
    undistort(img[0], tmp, cameraMatrix, distCoeffs);
    for (;;)
    {
            imshow("First Img undistorted", tmp);

            if (waitKey(1) >= 0) break;

    }




}

int main(int argc, char **argv)
{




        //initialize node
        ros::init(argc, argv, "cam_calib_node"); // "cam_calib_node" to nazwa noda

          // node handler
        ros::NodeHandle n;        // node handler to punkt dostepu do stworzonego noda

          // subsribe topic
        ros::Subscriber sub = n.subscribe("/cv_camera/image_raw", 1000, imageCallback);

        std::cout<<"ros zainicializowano\n";


        ros::Rate loop_rate(5);

       while (n.ok()) {
        ros::spinOnce();
        //std::cout<<"po spin\n";
        loop_rate.sleep();
       }






	return 0;
}
