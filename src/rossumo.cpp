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

LightSumo sumo;

void cmd_vel_cb(const geometry_msgs::TwistConstPtr & msg) {
  sumo.set_speeds(msg->linear.x, -msg->angular.z);
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "rossumo");
  ros::NodeHandle nh_public, nh_private("~");
  // create publishers and subscribers
  image_transport::ImageTransport it(nh_private);
  image_transport::Publisher rgb_pub = it.advertise("rgb", 1);
  ros::Subscriber cmd_vel_sub = nh_private.subscribe("cmd_vel", 1, cmd_vel_cb);
  // create and connect robot
  if (!sumo.connect())
    exit(-1);
  // data
  cv_bridge::CvImage rgb;
  rgb.encoding = "bgr8";
  unsigned int last_pix_idx = -1;
  // data acquisition loop
  ros::Rate rate(100);
  while(ros::ok()) {
    ros::Time now = ros::Time::now();
    if (rgb_pub.getNumSubscribers() && sumo.get_pic_idx() != last_pix_idx) {
      last_pix_idx = sumo.get_pic_idx();
      sumo.get_pic(rgb.image);
      if (rgb.image.empty())
        printf("pic empty!\n");
      else {
        // publish pic
        rgb.header.stamp = now;
        rgb.header.frame_id = "sumo_camera_frame";
        rgb_pub.publish(rgb.toImageMsg());
      }
    }
    ros::spinOnce();
    rate.sleep();
  } // end while (ros::ok())
  return EXIT_SUCCESS;
} // end main()
