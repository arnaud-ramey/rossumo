#include <ros/ros.h>
#include <ar_pose/ARMarker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

ros::Publisher marker_pub;
tf::TransformListener* tfconv;
std::string out_frame;
geometry_msgs::PoseStamped out, out_good_frame;

void marker_cb(const ar_pose::ARMarker & in) {
  out.header = in.header;
  out.pose = in.pose.pose;
  if (out_frame.empty() || out_frame == in.header.frame_id) {
    marker_pub.publish(out);
    return;
  }
  try { // convert TF
    tfconv->transformPose(out_frame,  ros::Time(0),
                          out, out_frame, out_good_frame);
  } catch (tf::TransformException & e) {
    ROS_WARN("transform error:'%s'", e.what());
    return;
  }
  marker_pub.publish(out_good_frame);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "armarker2pose");
  ros::NodeHandle nh_public, nh_private("~");
  nh_private.param("out_frame", out_frame, std::string());
  tfconv = new tf::TransformListener();
  ros::Subscriber pose_sub = nh_public.subscribe("in", 1, marker_cb);
  marker_pub = nh_public.advertise<geometry_msgs::PoseStamped>("out", 1);
  ros::spin();
  delete tfconv;
  return 0;
}
