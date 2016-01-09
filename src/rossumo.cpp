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
#include <std_msgs/Float32.h>

class RosSumo : public LightSumo {
public:
  RosSumo() : _nh_private("~") {
    // create publishers and subscribers
    _it = new image_transport::ImageTransport(_nh_private);
    _rgb_pub = _it->advertise("rgb", 1);
    _cmd_vel_sub = _nh_private.subscribe("cmd_vel", 1, &RosSumo::cmd_vel_cb, this);
    _rgb.encoding = "bgr8";
  }

  //////////////////////////////////////////////////////////////////////////////

  ~RosSumo() { delete _it; }

  //////////////////////////////////////////////////////////////////////////////
protected:
  //////////////////////////////////////////////////////////////////////////////

  //! callback called when a new image is received
  virtual void imageChanged () {
    if (!_rgb_pub.getNumSubscribers())
      return;
    get_pic(_rgb.image);
    if (_rgb.image.empty())
      printf("pic empty!\n");
    else {
      // publish pic
      _rgb.header.stamp = ros::Time::now();
      _rgb.header.frame_id = "sumo_camera_frame";
      _rgb_pub.publish(_rgb.toImageMsg());
    }
  }

  //////////////////////////////////////////////////////////////////////////////

  void cmd_vel_cb(const geometry_msgs::TwistConstPtr & msg) {
    set_speeds(msg->linear.x, -msg->angular.z);
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual void batteryStateChanged (uint8_t percent) {
    printf("batteryStateChanged(%i%%)\n", percent);
    _nh_private.setParam("battery_percentage", percent);
  }

  //////////////////////////////////////////////////////////////////////////////

  image_transport::ImageTransport* _it;
  image_transport::Publisher _rgb_pub;
  ros::Subscriber _cmd_vel_sub;
  std_msgs::Float32 _float_msg;
  ros::NodeHandle _nh_public, _nh_private;
  cv_bridge::CvImage _rgb;
}; // end class RosSumo

////////////////////////////////////////////////////////////////////////////////

int main (int argc, char **argv) {
  ros::init(argc, argv, "rossumo");
  RosSumo sumo;
  if (!sumo.connect())
    exit(-1);
  ros::spin();
  return EXIT_SUCCESS;
} // end main()
