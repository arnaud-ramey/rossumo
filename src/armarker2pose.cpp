#include <ros/ros.h>
#include <ar_pose/ARMarker.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher marker_pub;

void marker_cb(const ar_pose::ARMarker & in) {
  geometry_msgs::PoseStamped out;
  out.header = in.header;
  out.pose = in.pose.pose;
  marker_pub.publish(out);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rossumo");
  ros::NodeHandle nh_public;
  ros::Subscriber pose_sub = nh_public.subscribe("in", 1, marker_cb);
  marker_pub = nh_public.advertise<geometry_msgs::PoseStamped>("out", 1);
  ros::spin();
  return 0;
}
