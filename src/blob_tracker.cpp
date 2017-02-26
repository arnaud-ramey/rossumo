// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

enum State {
  LOST = 0,
  TRACKING = 1
};

State _state;
// HSV filter stuff
std::string _rgbwin = "rgb", _filterwin = "filter";
int hue_min, hue_max, saturation_min, saturation_max, value_min, value_max;
cv::Mat1b hue_, saturation_, value_, _filter;
cv::Mat3b _hsv;
// blob tracker
// http://www.learnopencv.com/blob-detection-using-opencv-python-c/
cv::Ptr<cv::SimpleBlobDetector> _blob_detec;
std::vector<cv::KeyPoint> _keypoints;
// ROS stuff
ros::Publisher _cmd_vel_pub;
geometry_msgs::Twist _cmd_vel;
// speed computation
int _max_blob_size, _min_blob_size;
double _max_v, _max_w, _x_deadzone, _y_deadzone;

////////////////////////////////////////////////////////////////////////////////

inline void HSVfilter(const cv::Mat3b & srcHSV,
                      const int outValue, cv::Mat1b & result,
                      const int Hmin, const int Hmax,
                      const int Smin, const int Smax,
                      const int Vmin, const int Vmax,
                      cv::Mat1b & Hbuffer, cv::Mat1b & Sbuffer, cv::Mat1b & Vbuffer) {
  // init the images if needed
  Hbuffer.create(srcHSV.size());
  Sbuffer.create(srcHSV.size());
  Vbuffer.create(srcHSV.size());
  result.create(srcHSV.size());

  // split
  std::vector< cv::Mat1b > layers;
  layers.push_back(Hbuffer);
  layers.push_back(Sbuffer);
  layers.push_back(Vbuffer);
  cv::split(srcHSV, layers);

  // filter each channel
  cv::threshold(Hbuffer, Hbuffer, Hmax, 255, cv::THRESH_TOZERO_INV);
  cv::threshold(Hbuffer, Hbuffer, Hmin, outValue, cv::THRESH_BINARY);
  cv::threshold(Sbuffer, Sbuffer, Smax, 255, cv::THRESH_TOZERO_INV);
  cv::threshold(Sbuffer, Sbuffer, Smin, outValue, cv::THRESH_BINARY);
  cv::threshold(Vbuffer, Vbuffer, Vmax, 255, cv::THRESH_TOZERO_INV);
  cv::threshold(Vbuffer, Vbuffer, Vmin, outValue, cv::THRESH_BINARY);

  // combine
  cv::min( (cv::Mat) Hbuffer, Sbuffer, result);
  cv::min( (cv::Mat) result, Vbuffer, result);
}

////////////////////////////////////////////////////////////////////////////////

double interlin(const double & x,
                const double & x1, const double & y1,
                const double & x2, const double & y2) {
  double a = (y2 - y1) / (x2 - x1), b = y1 - a * x1;
  return a * x + b;
}

////////////////////////////////////////////////////////////////////////////////

void image_cb(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr bridge = cv_bridge::toCvCopy(msg);
  ros::Time begin_time = ros::Time::now();
  // convert to HSV
  cv::cvtColor(bridge->image, _hsv, CV_BGR2HSV);

  // filter HSV
  HSVfilter(_hsv, 255, _filter,
            hue_min, hue_max,
            saturation_min, saturation_max,
            value_min, value_max,
            hue_, saturation_, value_);

  // find all blobs in filter
  _blob_detec->detect(_filter, _keypoints);

  // find biggest blob in filter
  unsigned int nkpts = _keypoints.size(), biggest_idx = 0, blob_size = 0;
  double curr_v = 0, curr_w = 0;
  std::ostringstream caption;
  if (_keypoints.empty()) {
    caption << "No blob found";
  }
  else {
    for (unsigned int i = 0; i < nkpts; ++i) {
      if (_keypoints[i].size >= blob_size) {
        biggest_idx = i;
        blob_size = _keypoints[i].size;
      }
    } // end for i
    double blob_x = 1. * _keypoints[biggest_idx].pt.x / _filter.cols;
    double blob_y = 1. * _keypoints[biggest_idx].pt.y / _filter.rows;

    // move
    if (blob_x < .5 - _x_deadzone / 2)
      curr_w = interlin(blob_x,  0, _max_v,  .5 - .5 * _x_deadzone, 0);
    else if (blob_x > .5 + _x_deadzone / 2)
      curr_w = interlin(blob_x,  1, -_max_v,  .5 + .5 * _x_deadzone, 0);
    else if ((int) blob_size < _max_blob_size)
      curr_v = interlin(blob_size, _max_blob_size,  0, 0, _max_w);

    caption << '(' << std::setprecision(2) << blob_x << ", "
            << std::setprecision(2) << blob_y << "):" << blob_size
            << "-> (" << curr_v << "," << curr_w << ")";
  }
  // send speed
  _cmd_vel.linear.x = curr_v;
  _cmd_vel.angular.z = curr_w;
  _cmd_vel_pub.publish(_cmd_vel);
  ROS_INFO_THROTTLE(1, "%li keypoints found in %i ms",
                    _keypoints.size(),
                    (int) (1000 * (ros::Time::now() - begin_time).toSec()));


  // display
  cv::drawKeypoints( bridge->image, _keypoints, bridge->image,
                     cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  unsigned int w = bridge->image.cols, h = bridge->image.rows;
  cv::Point speedcenter(w/2, h-10);
  cv::line(bridge->image, speedcenter,
           speedcenter - cv::Point(5*curr_w, 5*curr_v), CV_RGB(255,0,0), 2);
  cv::putText(bridge->image, caption.str(), cv::Point(10, 30),
              CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 255, 0));
  cv::imshow(_rgbwin, bridge->image);
  cv::imshow(_filterwin, _filter);
  cv::waitKey(50);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rossumo");
  ros::NodeHandle nh_public, nh_private("~");

  cv::namedWindow(_filterwin);
  cv::namedWindow(_rgbwin);
  //! parameters of the filter
  nh_private.param("hue_min", hue_min, 0);
  nh_private.param("hue_max", hue_max, 255);
  nh_private.param("saturation_min", saturation_min, 0);
  nh_private.param("saturation_max", saturation_max, 255);
  nh_private.param("value_min", value_min, 0);
  nh_private.param("value_max", value_max, 255);

  nh_private.param("min_blob_size", _min_blob_size, 10);
  nh_private.param("max_blob_size", _max_blob_size, 200);
  nh_private.param("max_v", _max_v, 30.);
  nh_private.param("max_w", _max_w, 30.);
  nh_private.param("x_deadzone", _x_deadzone, .2); // 20% => 40 -> 60% = deadzone
  nh_private.param("y_deadzone", _y_deadzone, .5); // 50% => 50 -> 100% = deadzone

  cv::createTrackbar("HUE_MIN", _filterwin, &hue_min, 255);
  cv::createTrackbar("HUE_MAX", _filterwin, &hue_max, 255);
  cv::createTrackbar("SATURATION_MIN", _filterwin, &saturation_min, 255);
  cv::createTrackbar("SATURATION_MAX", _filterwin, &saturation_max, 255);
  cv::createTrackbar("VALUE_MIN", _filterwin, &value_min, 255);
  cv::createTrackbar("VALUE_MAX", _filterwin, &value_max, 255);

  // blob detector
  // Setup SimpleBlobDetector parameters.
  cv::SimpleBlobDetector::Params params;
  // Change thresholds
  params.filterByColor = true;
  params.blobColor = 255;
  // Filter by Area.
  params.filterByArea = true;
  params.minArea = _min_blob_size;
  params.maxArea = 1E10;
  params.filterByCircularity = false;
  params.filterByConvexity = false;
  params.filterByInertia = false;
  _blob_detec = cv::SimpleBlobDetector::create(params);

  // ROS stuff
  _cmd_vel_pub = nh_public.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  image_transport::ImageTransport it(nh_public);
  image_transport::Subscriber sub =
      it.subscribe("camera/image_raw", 1, image_cb);

  ros::spin();
  return 0;
}
