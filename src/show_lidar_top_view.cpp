#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "structIO.hpp"

#include <unistd.h>

using namespace std;

void showLidarTopview()
{
    std::vector<LidarPoint> lidarPoints;
    readLidarPts("../dat/C51_LidarPts_0000.dat", lidarPoints);

    cv::Size worldSize(10.0, 20.0); // width and height of sensor field in m
    cv::Size imageSize(1000, 2000); // corresponding top view image in pixel

    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Scalar hsv = cv::Scalar(0, 0, 255);
    cv::Mat pixel(1, 1, CV_8UC3, cv::Scalar(0x30, 0x31, 0x41));

    int maxHue = 70;
    int minHue = 0;
    int hue = 0;

    float groundOffset = -1.5;

    // plot Lidar points into image
    for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
    {
        float xw = (*it).x; // world position in m with x facing forward from sensor
        float yw = (*it).y; // world position in m with y facing left from sensor

        int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
        int x = (-yw * imageSize.height / worldSize.height) + imageSize.width / 2;

        // cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);

        // TODO:
        // 1. Change the color of the Lidar points such that
        // X=0.0m corresponds to red while X=20.0m is shown as green.
        // 2. Remove all Lidar points on the road surface while preserving
        // measurements on the obstacles in the scene.

        // not sure if expectation is to apply pcl segmentation as per LIDAR lessons?
        // could do that but I'm assuing that the road is parallel with the LIDAR x - y plane.
        // therefore going to filter out any points that are below a certain offset
        // which will gain by experimenting.... but on Kitti car, the Lidar is at height 1.73m.
        // so ground would be -1.73

        // get height of point
        float zw = (*it).z;

        if (zw > groundOffset)
        {

            // hsv has hue channel that can rotate through colours. Set h channel based on x position
            hue = maxHue - abs(y * maxHue / (imageSize.height));
            cv::Scalar hsv = cv::Scalar(hue, 255, 255);
            pixel = cv::Mat(1, 1, CV_8UC3, hsv);
            cv::cvtColor(pixel, pixel, cv::COLOR_HSV2BGR);
            cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(pixel.at<cv::Vec3b>(0, 0)), -1);
        }
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "Top-View Perspective of LiDAR data";
    cv::namedWindow(windowName, 2);
    cv::imshow(windowName, topviewImg);
    cv::waitKey(0); // wait for key to be pressed
}

int main()
{
    showLidarTopview();
}