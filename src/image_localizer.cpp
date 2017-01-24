/*!
  \file        speed_calibration.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/1/10

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________
 */
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
// ROS msgs
#include <geometry_msgs/PointStamped.h>
// ROS
#include <ros/node_handle.h>
#include <ros/package.h>
#include <camera_calibration_parsers/parse.h>
#include <image_geometry/pinhole_camera_model.h>

void set_v4l_param(int fd, unsigned int id, int value, const std::string & descr) {
  v4l2_control c;
  c.id = id;
  c.value = value;
  printf("Retval for %s:%i\n", descr.c_str(), v4l2_ioctl(fd, VIDIOC_S_CTRL, &c));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_localizer");
  ros::NodeHandle nh_public, nh_private("~");

  // get params
  bool display = true;
  nh_private.param("display", display, display);

  // read calibration data
  std::string cal_filename = ros::package::getPath("rossumo") + "/data/camcalib.yaml",
      cal_camname = "/camera";
  sensor_msgs::CameraInfo caminfo;
  cv::Mat mapx, mapy, intrinsics, distortion;
  if (!camera_calibration_parsers::readCalibration
      (cal_filename, cal_camname, caminfo)) {
    ROS_FATAL("Could not read camera '%s' in camera_calibration file '%s'",
             cal_camname.c_str(), cal_filename.c_str());
    return -1;
  }
  image_geometry::PinholeCameraModel cam_model;
  if (!cam_model.fromCameraInfo(caminfo)) {
    ROS_FATAL("Could not convert CameraInf --> PinholeCameraModel");
    return -1;
  }
  // Read camera parameters from CamerInfo
  //  printf("K:%i, D:%i\n", caminfo.K.size(), caminfo.D.size());
  //  std::cout << "intrinsics:" << intrinsics << std::endl;
  //  std::cout << "distortion:" << distortion << std::endl;
  intrinsics.create(3, 3, CV_32FC1);
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      intrinsics.at<float>(row, col) = caminfo.K[3*row+col];
  distortion.create(5, 1, CV_32FC1); // rows, cols
  for (int col = 0; col < 5; ++col)
    distortion.at<float>(col, 0) = caminfo.D[col];
  // Computes the undistortion and rectification transformation map.
  // cf http://docs.opencv.org/trunk/modules/imgproc/doc/geometric_transformations.html#initundistortrectifymap
  cv::Mat newCameraMatrix;
  cv::initUndistortRectifyMap(intrinsics, distortion, cv::Mat(), newCameraMatrix,
                              cv::Size(caminfo.width, caminfo.height), CV_32FC1,
                              mapx, mapy);

  // http://docs.opencv.org/master/d1/dc5/tutorial_background_subtraction.html#gsc.tab=0
  // create background substractor and set the background we loaded
  cv::Ptr<cv::BackgroundSubtractor> bgsub = //cv::createBackgroundSubtractorKNN();
      cv::createBackgroundSubtractorMOG2();
  // load background image
  std::string bg_filename = ros::package::getPath("rossumo") + "/data/bg.png";
  cv::Mat3b frame, frame_rect, viz;
  cv::Mat bg = cv::imread(bg_filename, cv::IMREAD_COLOR), fg;
  if (!bg.empty()) {
    ROS_INFO("Successfully loaded background '%s'", bg_filename.c_str());
    bgsub->apply(bg, fg, 1);
  }

  // blob detector
  // Setup SimpleBlobDetector parameters.
  cv::SimpleBlobDetector::Params params;
  // Change thresholds
  params.filterByColor = true;
  params.blobColor = 255;
  //params.minThreshold = 100;
  //params.maxThreshold = 256;
  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 1000;
  params.maxArea = 1E10;
  params.filterByCircularity = false;
  params.filterByConvexity = false;
  params.filterByInertia = false;

  // http://www.learnopencv.com/blob-detection-using-opencv-python-c/
  cv::Ptr<cv::SimpleBlobDetector> blob_detec = cv::SimpleBlobDetector::create(params);
  std::vector<cv::KeyPoint> keypoints;

  // configure camera with V4L2
  // http://www.linuxtv.org/downloads/legacy/video4linux/API/V4L2_API/spec-single/v4l2.html#camera-controls
  int fd = v4l2_open("/dev/video1", O_RDWR);
  set_v4l_param(fd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, "auto exposure disable");
  set_v4l_param(fd, V4L2_CID_EXPOSURE_ABSOLUTE, 300, "exposure control");
  set_v4l_param(fd, V4L2_CID_AUTO_WHITE_BALANCE, V4L2_WHITE_BALANCE_MANUAL, "auto wb disable");
  set_v4l_param(fd, V4L2_CID_WHITE_BALANCE_TEMPERATURE, 5000, "wb temp");

  // open camera with OpenCV
  cv::VideoCapture cap(1); // open the default camera
  //int w = 1280, h = 720;
  int w = caminfo.width, h = caminfo.height;
  printf("w:%i, h:%i\n", w, h);
  assert(cap.isOpened());  // check if we succeeded
  cap.set(CV_CAP_PROP_FRAME_WIDTH, w);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, h);

  // set publisher
  ros::Publisher pt_pub = nh_public.advertise<geometry_msgs::PointStamped>("robot", 1);
  geometry_msgs::PointStamped msg;
  msg.header.frame_id = "camera_frame";

  // loop
  ros::Rate rate(20);
  while(ros::ok()) {
    ros::Time begin_time = ros::Time::now();
    cap >> frame; // get a new frame from camera
    if (bgsub->empty()) { // set background from frame
      bgsub->apply(frame_rect, fg, 1);
    }

    // rectify image
    cv::remap(frame, frame_rect, mapx, mapy, cv::INTER_LINEAR);

    // find object by differenciation - do not update model
    bgsub->apply(frame_rect, fg, 0);

    // find biggest blob
    blob_detec->detect(fg, keypoints);
    ROS_INFO_THROTTLE(1, "%li keypoints found in %i ms",
                      keypoints.size(),
                      (int) (1000 * (ros::Time::now() - begin_time).toSec()));

    // convert pixel to point
    if (keypoints.size() == 1) {
      cv::Point2f pt = keypoints.front().pt;
      cv::Point3d ray = cam_model.projectPixelTo3dRay(pt);
      msg.header.stamp = begin_time;
      msg.point.x = ray.x;
      msg.point.y = ray.y;
      msg.point.z = ray.z;
      pt_pub.publish(msg);
      ros::spinOnce();
    }

    rate.sleep();

    //
    // display
    //
    if (!display)
      continue;
    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    frame_rect.copyTo(viz);
    cv::drawKeypoints( viz, keypoints, viz,
                       cv::Scalar(0,0,255), cv::DrawMatchesFlags::DEFAULT );
    cv::drawKeypoints( viz, keypoints, viz,
                       cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    // display
    //cv::imshow("frame", frame);
    cv::imshow("viz", viz);
    cv::imshow("fg", fg);
    //bgsub->getBackgroundImage(bg); cv::imshow("bg", bg);
    char c = cv::waitKey(5);
    if (c == 's') {
      ROS_INFO("Saving background '%s'", bg_filename.c_str());
      cv::imwrite(bg_filename, frame_rect);
      // set background from frame
      bgsub->apply(frame_rect, fg, 1);
    }
    else if (c == 'q')
      break;
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}
