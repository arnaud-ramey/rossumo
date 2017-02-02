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
#include <tf/transform_listener.h>

boost::shared_ptr<tf::TransformListener> _tf_listener;
ros::Publisher pospub;
std::string _static_frame = "/world";
geometry_msgs::PointStamped cam_pos, robot_direction, robot_pos; // in static frame
bool cam_pos_converted = false;
double goalz = 0;

////////////////////////////////////////////////////////////////////////////////

template<class Point3> static inline std::string printP(const Point3 & p) {
  std::ostringstream ans;
  ans << "(" << p.x << ", " << p.y << ", " << p.z << ")";
  return ans.str();
}

////////////////////////////////////////////////////////////////////////////////

template<class Point3>
void solve(const Point3 & A, const Point3 & B, double goalz,
           Point3 & ans) {
  double dx = B.x - A.x, dy = B.y - A.y, dz = B.z - A.z;
  // solve A.z + t * dz = goalz
  double t = (goalz - A.z) / dz;
  ans.x = A.x + t * dx;
  ans.y = A.y + t * dy;
  ans.z = goalz;
}

////////////////////////////////////////////////////////////////////////////////

void pt_cb(const geometry_msgs::PointStamped & robot_direction0) {
  if (!cam_pos_converted) { // convert the position of the camera center into the static frame
    geometry_msgs::PointStamped cam_pos0;
    cam_pos0.header = robot_direction0.header;
    try {
      _tf_listener->transformPoint(_static_frame, ros::Time(0),
                                   cam_pos0, _static_frame, cam_pos);
      cam_pos_converted = true;
    } catch (tf::TransformException e) {
      ROS_WARN("Error converting cam center: '%s'", e.what());
      return;
    }
  }
  // convert the direction of the center into the static frame
  try {
    _tf_listener->transformPoint(_static_frame, ros::Time(0),
                                 robot_direction0, _static_frame, robot_direction);
  } catch (tf::TransformException e) {
    ROS_WARN("Error converting robot direction: '%s'", e.what());
    return;
  }
  // find the point on the line (camera, robot_direction) that has the goal z
  solve(cam_pos.point, robot_direction.point, goalz, robot_pos.point);
  robot_pos.header.stamp = robot_direction0.header.stamp;
  pospub.publish(robot_pos);
  ROS_INFO_THROTTLE(1, "Cam center:%s, robot dir:%s -> robot pos:%s",
                    printP(cam_pos.point).c_str(), printP(robot_direction.point).c_str(),
                    printP(robot_pos.point).c_str());
} // end pt_cb();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_localizer");
  ros::NodeHandle nh_public, nh_private("~");
  nh_private.param("goalz", goalz, goalz);
  ros::Subscriber dirsub = nh_public.subscribe("robot_direction", 1, pt_cb);
  pospub = nh_public.advertise<geometry_msgs::PointStamped>("robot_position", 1);
  _tf_listener = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener());
  robot_pos.header.frame_id = _static_frame;
  ros::spin();
  return 0;
}
