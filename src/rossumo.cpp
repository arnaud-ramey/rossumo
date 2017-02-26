/*!
  \file        rossumo.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/1/8

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
#include <rossumo/light_sumo.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <camera_calibration_parsers/parse.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class RosSumo : public LightSumo {
public:
  RosSumo() : _nh_private("~") {
    // initial data
    _image_rect.encoding = _image_raw.encoding = "bgr8";
    _image_rect.header.frame_id = _image_raw.header.frame_id = "/camera";
    _caminfo.header = _image_raw.header;
    // read params
    std::string cal_filename = "", cal_camname = "camname";
    _caminfo_read = false;
    _nh_private.param("max_vel_lin", _max_vel_lin, _max_vel_lin);
    _nh_private.param("max_vel_ang", _max_vel_ang, _max_vel_ang);
    _nh_private.param("ip_address", _ip_address, DEFAULT_IP_ADDRESS);
    _nh_private.param("discovery_port", _discovery_port, DEFAULT_DISCOVERY_PORT);
    _nh_private.param("camera_calibration_filename", cal_filename, cal_filename);
    _nh_private.param("camera_calibration_camname", cal_camname, cal_camname);
    ROS_INFO("rossumo '%s': IP:%s:%i, max velocities:%i, %i",
             _nh_public.getNamespace().c_str(),
             _ip_address.c_str(), _discovery_port, _max_vel_lin, _max_vel_ang);
    if (!cal_filename.empty())
      read_calibration(cal_filename, cal_camname);

    // create publishers
    _it = new image_transport::ImageTransport(_nh_public);
    _cmd_vel_sub = _nh_public.subscribe("cmd_vel", 1, &RosSumo::cmd_vel_cb, this);
    _cmd_vel_norm_sub = _nh_public.subscribe("cmd_vel_norm", 1, &RosSumo::cmd_vel_norm_cb, this);
    _anim_sub = _nh_public.subscribe("anim", 1, &RosSumo::anim_cb, this);
    _posture_sub = _nh_public.subscribe("set_posture", 1, &RosSumo::posture_cb, this);
    _sharp_turn_sub = _nh_public.subscribe("sharp_turn", 1, &RosSumo::sharp_turn_cb, this);
    _high_jump_sub = _nh_public.subscribe("high_jump", 1, &RosSumo::high_jump_cb, this);
    _long_jump_sub = _nh_public.subscribe("long_jump", 1, &RosSumo::long_jump_cb, this);
    // create subscribers
    _image_raw_pub = _it->advertise("camera/image_raw", 1);
    _image_rect_pub = _it->advertise("camera/image_rect_color", 1);
    _caminfo_pub = _nh_public.advertise<sensor_msgs::CameraInfo>("camera/camera_info", 1);
    _battery_percentage_pub = _nh_public.advertise<std_msgs::Int16>("battery_percentage", 1);
    _posture_pub = _nh_public.advertise<std_msgs::String>("posture", 1);
    _link_quality_pub = _nh_public.advertise<std_msgs::Int16>("link_quality", 1);
    _alert_pub = _nh_public.advertise<std_msgs::String>("alert", 1);
    _outdoor_pub = _nh_public.advertise<std_msgs::Int16>("outdoor", 1);
  }

  //////////////////////////////////////////////////////////////////////////////

  bool read_calibration(std::string & cal_filename, std::string & cal_camname) {
    _caminfo_read = camera_calibration_parsers::readCalibration
        (cal_filename, cal_camname, _caminfo);
    if (!_caminfo_read) {
      ROS_WARN("Could not read camera '%s' in camera_calibration file '%s'",
               cal_camname.c_str(), cal_filename.c_str());
      return false;
    }
    // Read camera parameters from CamerInfo
    //  printf("K:%i, D:%i\n", _caminfo.K.size(), _caminfo.D.size());
    //  std::cout << "intrinsics:" << intrinsics << std::endl;
    //  std::cout << "distortion:" << distortion << std::endl;
    intrinsics.create(3, 3, CV_32FC1);
    for (int row = 0; row < 3; ++row)
      for (int col = 0; col < 3; ++col)
        intrinsics.at<float>(row, col) = _caminfo.K[3*row+col];
    distortion.create(5, 1, CV_32FC1); // rows, cols
    for (int col = 0; col < 5; ++col)
      distortion.at<float>(col, 0) = _caminfo.D[col];
    // Computes the undistortion and rectification transformation map.
    // cf http://docs.opencv.org/trunk/modules/imgproc/doc/geometric_transformations.html#initundistortrectifymap
    cv::Mat newCameraMatrix;
    cv::initUndistortRectifyMap(intrinsics, distortion, cv::Mat(), newCameraMatrix,
                                cv::Size(_caminfo.width, _caminfo.height), CV_32FC1,
                                mapx, mapy);
    intrinsics_inv = intrinsics.inv();
    ROS_INFO("Succesfully read camera '%s' in camera_calibration file '%s'",
             cal_camname.c_str(), cal_filename.c_str());
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  ~RosSumo() { delete _it; }

  //////////////////////////////////////////////////////////////////////////////

  void spinOnce() {
    if (_image_raw_pub.getNumSubscribers()
        || _image_rect_pub.getNumSubscribers()
        || _caminfo_pub.getNumSubscribers())
      enable_pic_decoding();
    else
      disable_pic_decoding();
  }

  //////////////////////////////////////////////////////////////////////////////
protected:

  //////////////////////////////////////////////////////////////////////////////

  //! check speed calibration: "sumo_speed_calibrate.ods"
  inline void speeds2ticks(const double & v_norm, const double & w_norm,
                    int & v, int & w) {
    // compute v
    if (fabs(v_norm) < 1E-2)
      v = 0;
    else if(v_norm > 0)
      v = (v_norm + 0.1405584209307) / 0.0230341262135;
    else
      v = -(-v_norm + 0.1405584209307) / 0.0230341262135;
    // compute w
    if (fabs(w_norm) < 1E-2)
      w = 0;
    else
      w = -w_norm / 0.05;
  }

  inline void ticks2speeds(const int & v, const int & w,
                    double & v_norm, double & w_norm) {
    // compute v
    if (abs(v) < 10)
      v_norm = 0;
    else if(v_norm > 0)
      v_norm = 0.0230341262135 * v - 0.1405584209307;
    else
      v_norm = -(-0.0230341262135 * v - 0.1405584209307);
    // compute w
    w_norm = -0.05 * w;
  }

  //////////////////////////////////////////////////////////////////////////////

  void cmd_vel_cb(const geometry_msgs::TwistConstPtr & msg) { set_speeds(msg->linear.x, -msg->angular.z); }

  //////////////////////////////////////////////////////////////////////////////

  void cmd_vel_norm_cb(const geometry_msgs::TwistConstPtr & msg) {
    int v, w;
    speeds2ticks(msg->linear.x, msg->angular.z, v, w);
    ROS_INFO_THROTTLE(1, "cmd_vel_norm_cb(%g, %g)->(%i, %i)", msg->linear.x, msg->angular.z, v, w);
    set_speeds(v, w);
  }

  //////////////////////////////////////////////////////////////////////////////

  void posture_cb(const std_msgs::StringConstPtr & msg) { set_posture(msg->data); }

  //////////////////////////////////////////////////////////////////////////////

  void anim_cb(const std_msgs::StringConstPtr & msg) { anim(msg->data); }

  //////////////////////////////////////////////////////////////////////////////

  void sharp_turn_cb(const std_msgs::Float32Ptr & msg) { sharp_turn(msg->data); }

  //////////////////////////////////////////////////////////////////////////////

  void high_jump_cb(const std_msgs::Empty &) { high_jump(); }
  void long_jump_cb(const std_msgs::Empty &) { long_jump(); }

  //////////////////////////////////////////////////////////////////////////////

  //! callback called when a new image is received
  virtual void imageChanged () {
    get_pic(_image_raw.image);
    if (_image_raw.image.empty()) {
      printf("pic empty!\n");
      return;
    }
    _image_raw.header.stamp = ros::Time::now();
    _image_rect.header.stamp = _image_raw.header.stamp;
    // rectify image if needed
    if (_image_rect_pub.getNumSubscribers()) {
      cv::remap(_image_raw.image, _image_rect.image, mapx, mapy, cv::INTER_LINEAR);
      _image_rect_pub.publish(_image_rect.toImageMsg());
    }
    // publish raw pic
    if (_image_raw_pub.getNumSubscribers()) {
      _image_raw_pub.publish(_image_raw.toImageMsg());
    }
    _caminfo.header.stamp  = _image_raw.header.stamp;
    _caminfo_pub.publish(_caminfo);
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual void batteryChanged (uint8_t battery_percentage) {
    LightSumo::batteryChanged(battery_percentage);
    _int_msg.data = battery_percentage;
    _battery_percentage_pub.publish(_int_msg);
    _nh_public.setParam("battery_percentage", battery_percentage);
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual void postureChanged (uint8_t posture) {
    LightSumo::postureChanged(posture);
    _string_msg.data = get_posture2str();
    _posture_pub.publish(_string_msg);
    _nh_public.setParam("posture", _string_msg.data);
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual void volumeChanged (uint8_t volume) {
    LightSumo::volumeChanged(volume);
    _int_msg.data = volume;
    _volume_pub.publish(_int_msg);
    _nh_public.setParam("volume", volume);
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual void alertChanged (uint8_t alert) {
    LightSumo::alertChanged(alert);
    _string_msg.data = get_alert2str();
    _alert_pub.publish(_string_msg);
    _nh_public.setParam("alert", _string_msg.data);
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual void link_qualityChanged (uint8_t link_quality) {
    LightSumo::link_qualityChanged(link_quality);
    _int_msg.data = link_quality;
    _link_quality_pub.publish(_int_msg);
    _nh_public.setParam("link_quality", link_quality);
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual void outdoorChanged (uint8_t outdoor) {
    LightSumo::outdoorChanged(outdoor);
    _int_msg.data = outdoor;
    _outdoor_pub.publish(_int_msg);
    _nh_public.setParam("outdoor", outdoor);
  }

  //////////////////////////////////////////////////////////////////////////////

  image_transport::ImageTransport* _it;
  image_transport::Publisher _image_raw_pub, _image_rect_pub;
  ros::Subscriber _cmd_vel_sub, _cmd_vel_norm_sub, _high_jump_sub, _long_jump_sub, _posture_sub;
  ros::Subscriber _anim_sub, _sharp_turn_sub;
  ros::Publisher _caminfo_pub, _volume_pub, _battery_percentage_pub, _posture_pub;
  ros::Publisher _link_quality_pub, _alert_pub, _outdoor_pub;
  std_msgs::Float32 _float_msg;
  std_msgs::Int16 _int_msg;
  std_msgs::String _string_msg;
  ros::NodeHandle _nh_public, _nh_private;
  sensor_msgs::CameraInfo _caminfo;
  bool _caminfo_read;
  cv_bridge::CvImage _image_raw, _image_rect;
  cv::Mat mapx, mapy;
  cv::Mat intrinsics, distortion, intrinsics_inv;
}; // end class RosSumo

////////////////////////////////////////////////////////////////////////////////

int main (int argc, char **argv) {
#if 1
#endif
  ros::init(argc, argv, "rossumo");
  RosSumo sumo;
  if (!sumo.connect())
    ros::shutdown();
  ros::Rate rate(100);
  while (ros::ok()) {
    sumo.spinOnce();
    ros::spinOnce();
    rate.sleep();
  }
  return EXIT_SUCCESS;
} // end main()
