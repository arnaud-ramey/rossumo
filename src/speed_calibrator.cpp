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
#include <ros/node_handle.h>
#include <geometry_msgs/PointStamped.h>

void pt_cb(const geometry_msgs::Point & pt) {

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_localizer");
  ros::NodeHandle nh_public, nh_private("~");
  ros::Subscriber sub = nh_private.subscribe("robot", 1, pt_cb);
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}
