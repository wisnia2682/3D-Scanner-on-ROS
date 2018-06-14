#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <sstream>

using namespace cv;
using namespace std;

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > vectorVector2d;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vectorVector3d;
typedef std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > vectorVector4d;

/// Global Variables

const int hsv_h_min_max = 255;
const int hsv_s_min_max = 255;
const int hsv_v_min_max = 255;
const int hsv_h_max_max = 255;
const int hsv_s_max_max = 255;
const int hsv_v_max_max = 255;

int hsv_h_min = 166;
int hsv_s_min = 37;
int hsv_v_min = 255;

int hsv_h_max = 255;
int hsv_s_max = 255;
int hsv_v_max = 255;

int checkerboard_size_x = 10;
int checkerboard_size_y = 7;
const int camera_calib_img_count = 40;


cv::Mat temp;
cv::Mat image;
Eigen::Vector4d PlaneEquation;
Eigen::Vector4d best_plane_from_points(Eigen::MatrixXd cloud_of_points);
bool find_laser(Mat frame, vectorVector2d &TestVect2d);
bool calculateCheckerboardVector(Mat frame, Mat cameraMatrix, Mat distCoeffs, Eigen::Vector4d &PlaneAtCheckerboard);
bool projectImagePointsOntoPlane(const vectorVector2d &pts,
        vectorVector3d &pts3d,
        const cv::Mat &cameraMatrix,
        const Eigen::Vector4d &planeEq);


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    temp = cv_bridge::toCvShare(msg, "bgr8")->image;
    std::cout<<"obraz\n";


        FileStorage fs2("cam_param.yml", FileStorage::READ);

                        if (fs2.isOpened())	//if camera calibration file exists
                        {
                            Mat cameraMatrix(3, 3, CV_64F), distCoeffs(3, 3, CV_64F);
                            fs2["cameraMatrix"] >> cameraMatrix;
                            fs2["distCoeffs"] >> distCoeffs;

                            //cout << cameraMatrix2 << endl;
                            //cout << cameraMatrix2.at<float>(0, 0);
                            fs2.release();

                            FileStorage fs_Plane("PlaneEq.yml", FileStorage::READ);
                            if (fs_Plane.isOpened())		//if laser plane equation exists
                            {
                                fs_Plane["PlaneNx"] >> PlaneEquation(0);
                                fs_Plane["PlaneNy"] >> PlaneEquation(1);
                                fs_Plane["PlaneNz"] >> PlaneEquation(2);
                                fs_Plane["PlaneD"] >> PlaneEquation(3);

                                cout << PlaneEquation << endl;
                                fs_Plane.release();

                                Eigen::MatrixXd cloud_of_points = Eigen::MatrixXd::Zero(3, 1);

                                vectorVector2d laserpoints2d;
                                Eigen::Vector4d PlaneAtCheckerboard;
                                imshow("frame",image);
                                if (waitKey(7) == 'n')
                                {
                                    return;
                                }
                                if (waitKey(7) == 's')
                                {
                                    std::ofstream file("cloud.txt");
                                    if (file.is_open())
                                    {
                                        file << cloud_of_points << '\n';
                                        return;
                                    }
                                }


                                if (find_laser(image, laserpoints2d))
                                {
                                    vectorVector3d laserpoints3d;
                                    projectImagePointsOntoPlane(laserpoints2d, laserpoints3d, cameraMatrix, PlaneEquation);
                                    for (int i = 0; i < laserpoints3d.size(); i++)
                                    {
                                        cloud_of_points.conservativeResize(cloud_of_points.rows(), cloud_of_points.cols() + 1);
                                        cloud_of_points.col(cloud_of_points.cols() - 1) = laserpoints3d.at(i);
                                    }
                                }
                             }

                          else{
                                Eigen::MatrixXd cloud_of_points = Eigen::MatrixXd::Zero(3, 1);
                                vectorVector2d laserpoints2d;
                                Eigen::Vector4d PlaneAtCheckerboard;
                                imshow("frame", image);
                                Mat tresholded;
                                inRange(image, Scalar(hsv_h_min, hsv_s_min, hsv_v_min), Scalar(hsv_h_max, hsv_s_max, hsv_v_max), tresholded);
                                imshow("tresholded", tresholded);
                                if (waitKey(7) == 'p')
                                {
                                    return;
                                }

                                else if (waitKey(7) == 's')
                                {
                                    FileStorage file("PlaneEq.yml", cv::FileStorage::WRITE);
                                    file << "PlaneNx" << PlaneEquation(0) << "PlaneNy" << PlaneEquation(1) << "PlaneNz" << PlaneEquation(2) << "PlaneD" << PlaneEquation(3);
                                    file.release();

                                    return;
                                }

                                if (calculateCheckerboardVector(image, cameraMatrix, distCoeffs, PlaneAtCheckerboard) //if checkerboard vector found
                                                && find_laser(image, laserpoints2d))
                                {
                                    vectorVector3d laserpoints3d;
                                    projectImagePointsOntoPlane(laserpoints2d, laserpoints3d, cameraMatrix, PlaneAtCheckerboard);

                                    for (int i = 0; i < laserpoints3d.size(); i++)
                                    {
                                        cloud_of_points.conservativeResize(cloud_of_points.rows(), cloud_of_points.cols() + 1);
                                        cloud_of_points.col(cloud_of_points.cols() - 1) = laserpoints3d.at(i);

                                    }
                                    PlaneEquation = best_plane_from_points(cloud_of_points);
                                    cout << cloud_of_points.size();
                                }

                          }
                        }
                        else
                        {
                           std::cout<<"bad calibrated camera?\n";
                                            //camCalib(camera);?
                        }

                        waitKey(0);




}// end of try






Eigen::Vector4d best_plane_from_points(Eigen::MatrixXd cloud_of_points)
{

        Eigen::MatrixXd cloud_of_points_tmp;
        cloud_of_points_tmp = cloud_of_points.replicate(1, 1);

        // calculate centroid
        Eigen::Vector3d centroid(cloud_of_points.row(0).mean(), cloud_of_points.row(1).mean(), cloud_of_points.row(2).mean());


        // subtract centroid
        for (int i = 0; i < cloud_of_points.cols(); i++)
        {
                cloud_of_points_tmp.block<3, 1>(0, i) -= centroid;
        }

        Eigen::Vector3d plane_normal;

        //Eigen::JacobiSVD<Eigen::MatrixXd> svd = cloud_of_points_tmp.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);		//je¿eli nie dzia³a to tu mo¿e byæ Ÿle
        auto svd = cloud_of_points_tmp.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);		//je¿eli nie dzia³a to tu mo¿e byæ Ÿle
        plane_normal = svd.matrixU().rightCols<1>();
        //cout << plane_normal << endl << endl;

        auto distance = centroid.transpose()*plane_normal;
        Eigen::Vector4d PlaneEq;
        PlaneEq.head<3>() = plane_normal;
        PlaneEq(3) = -distance.value();
        //cout << distance << endl << endl;
        cout << PlaneEq << endl << endl;
        return PlaneEq;
}

//create vector of points from tresholded image
bool find_laser(Mat frame, vectorVector2d &laserpoints2d)
{

        Mat laser;
        inRange(frame, Scalar(hsv_h_min, hsv_s_min, hsv_v_min), Scalar(hsv_h_max, hsv_s_max, hsv_v_max), laser);
        imshow("laser", laser);
        bool flaga = false;
        for (int i = 0; i < frame.rows; ++i)
        {
                for (int j = 0; j < frame.cols; ++j)
                {
                        if (laser.at<uchar>(i, j) > 0)
                        {
                                Eigen::Vector2d punkty2d(j, i);
                                laserpoints2d.push_back(punkty2d);
                                flaga = true;
                        }
                }
        }
        return flaga;
}


bool calculateCheckerboardVector(Mat frame, Mat cameraMatrix, Mat distCoeffs, Eigen::Vector4d &PlaneAtCheckerboard)
{
        Size patternsize(checkerboard_size_y, checkerboard_size_x);
        vector<Point2f> corners;

        bool patternfound = findChessboardCorners(frame, patternsize, corners,
                CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                + CALIB_CB_FAST_CHECK);
        if (patternfound)
        {
                Mat subpixel_BW;
                cvtColor(frame, subpixel_BW, CV_BGR2GRAY);
                cornerSubPix(subpixel_BW, corners, Size(11, 11), Size(-1, -1),
                        TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                //if (i == 0)
                //cout << corners;
                //wek.push_back(corners);
                //dobra_wek.push_back(dobra);

                //namedWindow("Chessboard");
                drawChessboardCorners(frame, patternsize, Mat(corners), patternfound);
                //imshow("Chessboard", frame);

                vector<Point3f> axis;
                axis.push_back(cv::Point3d(0, 0, 0));
                axis.push_back(cv::Point3d(1, 0, 0));
                axis.push_back(cv::Point3d(0, 1, 0));
                axis.push_back(cv::Point3d(0, 0, -1));

                vector<Point3f> objp;
                for (int y = patternsize.height - 1; y >= 0; y--)
                {
                        for (int x = patternsize.width - 1; x >= 0; x--)
                        {
                                objp.push_back(cv::Point3d(x, y, 0));
                        }
                }
                Mat rvec(3, 1, DataType<double>::type);
                Mat tvec(3, 1, DataType<double>::type);
                vector<Point2f> imgpts;
                //# Find the rotation and translation vectors.
                solvePnP(objp, corners, cameraMatrix, distCoeffs, rvec, tvec);

                //cout << "tvec:"<<tvec << endl;

                Mat rotMat;
                Rodrigues(rvec, rotMat);

                Eigen::Matrix<double, 4, 4> Transf;
                Transf << rotMat.at<double>(0, 0), rotMat.at<double>(0, 1), rotMat.at<double>(0, 2), tvec.at<double>(0),
                        rotMat.at<double>(1, 0), rotMat.at<double>(1, 1), rotMat.at<double>(1, 2), tvec.at<double>(1),
                        rotMat.at<double>(2, 0), rotMat.at<double>(2, 1), rotMat.at<double>(2, 2), tvec.at<double>(2),
                        0, 0, 0, 1;

                Eigen::Vector4d PlaneAtCamera;
                PlaneAtCamera << 0, 0, 1, 0;

                PlaneAtCheckerboard = Transf.inverse().transpose()*PlaneAtCamera;

                //# project 3D points to image plane
                projectPoints(axis, rvec, tvec, cameraMatrix, distCoeffs, imgpts);

                int myradius = 5;

                circle(frame, cvPoint(imgpts[0].x, imgpts[0].y), myradius, CV_RGB(255, 0, 255), -1, 8, 0);
                circle(frame, cvPoint(imgpts[1].x, imgpts[1].y), myradius, CV_RGB(255, 0, 0), -1, 8, 0);
                line(frame, cvPoint(imgpts[0].x, imgpts[0].y), cvPoint(imgpts[1].x, imgpts[1].y), CV_RGB(255, 0, 0), 4);
                circle(frame, cvPoint(imgpts[2].x, imgpts[2].y), myradius, CV_RGB(0, 255, 0), -1, 8, 0);
                line(frame, cvPoint(imgpts[0].x, imgpts[0].y), cvPoint(imgpts[2].x, imgpts[2].y), CV_RGB(0, 255, 0), 4);
                circle(frame, cvPoint(imgpts[3].x, imgpts[3].y), myradius, CV_RGB(0, 0, 255), -1, 8, 0);
                line(frame, cvPoint(imgpts[0].x, imgpts[0].y), cvPoint(imgpts[3].x, imgpts[3].y), CV_RGB(0, 0, 255), 4);
                imshow("Axis", frame);

                return true;
        }
        else
        {
                return false;
        }
}

bool projectImagePointsOntoPlane(const vectorVector2d &pts,
        vectorVector3d &pts3d,
        const cv::Mat &cameraMatrix,
        const Eigen::Vector4d &planeEq)
{
        static constexpr double eps = 1e-6;

        float fx = cameraMatrix.at<double>(0, 0);
        float fy = cameraMatrix.at<double>(1, 1);
        float cx = cameraMatrix.at<double>(0, 2);
        float cy = cameraMatrix.at<double>(1, 2);

        // camera center in homogeneous cooridinates
        Eigen::Vector4d C;
        C << 0, 0, 0, 1;

        double den = planeEq.transpose() * C;

        // plane through camera center
        if (abs(den) < eps) {
                return false;
        }
        else {
                // pseudoinverse of the camera matrix P
                Eigen::Matrix<double, 4, 3> Ppinv;
                Ppinv << 1 / fx, 0, -cx / fx,
                        0, 1 / fy, -cy / fy,
                        0, 0, 1,
                        0, 0, 0;

                Eigen::Matrix<double, 1, 3> pPpinv = planeEq.transpose() * Ppinv;

                for (const Eigen::Vector2d &pt2d : pts) {
                        // point in homogeneous cooridinates
                        Eigen::Vector3d pt;
                        pt << pt2d(0), pt2d(1), 1;

                        double lambda = -(pPpinv * pt)(0) / den;

                        Eigen::Vector4d pt3d = Ppinv * pt + lambda * C;

                        // adding point in inhomogeneous coordinates
                        pts3d.push_back(pt3d.head<3>() / pt3d(3));
                }
                return true;
        }
}





int main(int argc, char **argv)
{

    //initialize node
      ros::init(argc, argv, "laser_scan_node"); // to nazwa noda

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

