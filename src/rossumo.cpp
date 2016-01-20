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

class RosSumo : public LightSumo {
public:
  RosSumo() : _nh_private("~") {
    // initial data
    _rgb.encoding = "bgr8";
    _rgb.header.frame_id = "sumo_camera_frame";
    // create publishers and subscribers
    _it = new image_transport::ImageTransport(_nh_public);
    _cmd_vel_sub = _nh_public.subscribe("cmd_vel", 1, &RosSumo::cmd_vel_cb, this);
    _anim_sub = _nh_public.subscribe("anim", 1, &RosSumo::anim_cb, this);
    _posture_sub = _nh_public.subscribe("set_posture", 1, &RosSumo::posture_cb, this);
    _sharp_turn_sub = _nh_public.subscribe("sharp_turn", 1, &RosSumo::sharp_turn_cb, this);
    _high_jump_sub = _nh_public.subscribe("high_jump", 1, &RosSumo::high_jump_cb, this);
    _long_jump_sub = _nh_public.subscribe("long_jump", 1, &RosSumo::long_jump_cb, this);
    _rgb_pub = _it->advertise("rgb", 1);
    _battery_percentage_pub = _nh_public.advertise<std_msgs::Int16>("battery_percentage", 1);
    _posture_pub = _nh_public.advertise<std_msgs::Int16>("posture", 1);
    _link_quality_pub = _nh_public.advertise<std_msgs::Int16>("link_quality", 1);
    _alert_pub = _nh_public.advertise<std_msgs::Int16>("alert", 1);
    _outdoor_pub = _nh_public.advertise<std_msgs::Int16>("outdoor", 1);
  }

  //////////////////////////////////////////////////////////////////////////////

  ~RosSumo() { delete _it; }

  //////////////////////////////////////////////////////////////////////////////

  void spinOnce() {
    if (_rgb_pub.getNumSubscribers())
      enable_pic_decoding();
    else
      disable_pic_decoding();
  }

  //////////////////////////////////////////////////////////////////////////////
protected:

  //////////////////////////////////////////////////////////////////////////////

  void cmd_vel_cb(const geometry_msgs::TwistConstPtr & msg) { set_speeds(msg->linear.x, -msg->angular.z); }

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
    get_pic(_rgb.image);
    if (_rgb.image.empty())
      printf("pic empty!\n");
    else {
      // publish pic
      _rgb.header.stamp = ros::Time::now();
      _rgb_pub.publish(_rgb.toImageMsg());
    }
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
    _int_msg.data = posture;
    _posture_pub.publish(_int_msg);
    _nh_public.setParam("posture", posture);
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
    _int_msg.data = alert;
    _alert_pub.publish(_int_msg);
    _nh_public.setParam("alert", alert);
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
  image_transport::Publisher _rgb_pub;
  ros::Subscriber _cmd_vel_sub, _high_jump_sub, _long_jump_sub, _posture_sub;
  ros::Subscriber _anim_sub, _sharp_turn_sub;
  ros::Publisher _volume_pub, _battery_percentage_pub, _posture_pub;
  ros::Publisher _link_quality_pub, _alert_pub, _outdoor_pub;
  std_msgs::Float32 _float_msg;
  std_msgs::Int16 _int_msg;
  ros::NodeHandle _nh_public, _nh_private;
  cv_bridge::CvImage _rgb;
}; // end class RosSumo

////////////////////////////////////////////////////////////////////////////////

int main (int argc, char **argv) {
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
