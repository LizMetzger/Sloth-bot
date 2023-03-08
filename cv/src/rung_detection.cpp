
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include "control/control_dyn.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk/port_handler.h"
#include "dynamixel_sdk/port_handler_linux.h"

#include <iostream>
using namespace cv;

using namespace std;

int main() {

Mat image;

namedWindow("Display window");

VideoCapture cap(0);

if (!cap.isOpened()) {

cout << "cannot open camera";

}

while (true) {

cap >> image;

imshow("Display window", image);

waitKey(25);

}

return 0;
}