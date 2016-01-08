/*!
  \file        rossumo.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/1/7

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

\class Rossumo
  The interface between "libsumo" and ROS.
\see rossumo.cpp for detailed topics and params.
 */
#ifndef ROSsumo_H
#define ROSsumo_H

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "libsumo/src/bluetooth_mac2device.h"
#include "libsumo/src/gattsumo.h"

class Rossumo : public sumo {
public:
  Rossumo() : _nh_private("~") {
    main_loop = g_main_loop_new(NULL, FALSE);
    std::string device_mac, sumo_mac;
    _nh_private.param("device_mac", device_mac, device_mac);
    _nh_private.param("sumo_mac", sumo_mac, sumo_mac);
    if (device_mac.empty() || sumo_mac.empty()) {
      ROS_FATAL("Parameters 'device_mac' and 'sumo_mac' must be set.");
      ros::shutdown();
      exit(-1);
    }
    if (!sumo::connect(main_loop, bluetooth_mac2device(device_mac).c_str(), sumo_mac.c_str())) {
      ROS_FATAL("Could not connect with device MAC '%s' to sumo with MAC '%s'!",
                device_mac.c_str(), sumo_mac.c_str());
      ros::shutdown();
      exit(-1);
    }
    ROS_INFO("Succesfully connected device MAC '%s' to sumo with MAC '%s' :)",
             device_mac.c_str(), sumo_mac.c_str());
    // advertise publishers
    _battery_voltage_pub = _nh_private.advertise<std_msgs::Float32>("battery_voltage", 1);
    _battery_percentage_pub = _nh_private.advertise<std_msgs::Float32>("battery_percentage", 1);
    _status_pub = _nh_private.advertise<std_msgs::String>("status", 1);
    _odometer_reading_pub = _nh_private.advertise<std_msgs::Float32>("odometer_reading", 1);
    _absspeed_pub = _nh_private.advertise<std_msgs::Float32>("absspeed", 1);
    _last_odom = -1;
    // create subscribers
    _speed_sub = _nh_private.subscribe("speed", 1, &Rossumo::speed_cb, this);

    play_sound(23); // oh yeah
  }

  //////////////////////////////////////////////////////////////////////////////

  ~Rossumo() {
    play_sound(19); // see ya
    pump_up_callbacks();
  }

  //////////////////////////////////////////////////////////////////////////////

  bool spinOnce() {
    ros::Time now = ros::Time::now();
    // only request informations where there are subscribers
    if ((_status_pub.getNumSubscribers()
         || _battery_percentage_pub.getNumSubscribers()
         || _battery_voltage_pub.getNumSubscribers())
        && (now - _status_battery_stamp).toSec() > 1) {
      _status_battery_stamp = now;
      request_battery_voltage();
    }
    if ((_odometer_reading_pub.getNumSubscribers()
         || _absspeed_pub.getNumSubscribers())
        && (now - _odometer_reading_stamp).toSec() > .3) {
      _odometer_reading_stamp = now;
      request_odometer_reading();
    }
    pump_up_callbacks();
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

protected:
  //////////////////////////////////////////////////////////////////////////////

  //! extend this function to add behaviours upon reception of a notification
  virtual void notification_post_hook(sumoCommand cmd, const std::vector<int> & /*values*/) {
    ros::Time now = ros::Time::now();
    if (cmd == CMD_sumo_STATUS) {
      // sumo spontaneously sends its status periodically
      _status_battery_stamp = now;
      _float_msg.data = get_battery_voltage();
      _battery_voltage_pub.publish(_float_msg);
      _float_msg.data = get_battery_percentage();
      _battery_percentage_pub.publish(_float_msg);
      _string_msg.data = get_status2str();
      _status_pub.publish(_string_msg);
    }
    if (cmd == CMD_ODOMETER_READING) {
      // compute speed (first derivative)
      if (_last_odom > 0) {
        double speed = (get_odometer_reading() - _last_odom)
            / (now - _last_odom_stamp).toSec();
        _float_msg.data = speed;
        _absspeed_pub.publish(_float_msg);
      }
      _last_odom_stamp = now;
      _last_odom = get_odometer_reading();
      // publish odom
      _float_msg.data = get_odometer_reading();
      _odometer_reading_pub.publish(_float_msg);
    }
  } // end notification_post_hook();

  //////////////////////////////////////////////////////////////////////////////
  //! https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
  template <typename T> int signum(const T val) {
    return (val >= T(0)? 1 : -1);
  }

  bool speed2ticks(const double & v_ms, const double & w_rads,
                   int & v_int, int & w_int) {
    if (fabs(v_ms) > .95 || fabs(w_rads) > 17) {
      ROS_WARN("(v:%g, w:%g) out of bounds!", v_ms, w_rads);
      return false;
    }
    // v
    if (fabs(v_ms) < 0.01)
      v_int = 0;
    else if (fabs(v_ms) < 0.7) // NORMAL: v = 0.0217453422621 * b1
      v_int = clamp(45.98685952820811545096 * v_ms, -32, 32);
    else // CRAZY: v = 0.0290682838088 * b1
      v_int = clamp(34.40175576162719827641 * v_ms, -32, 32) + 32 * signum(v_ms);
    // w
    if (fabs(w_rads) < 0.2)
      w_int = 0;
    else if (fabs(w_rads) < 13) // NORMAL: w = 0.4177416860037 * b2 - 0.6876060987078
      w_int = clamp(2.39382382344084006941 * w_rads + 1.6460078602299454762, -32, 32);
    else // CRAZY: W = 0.7208400618386 * b2 + 0.0191642762841
      w_int = clamp(1.38727028773812161461 * w_rads - 0.02658603107493626709, -32, 32)
           + 32 * signum(w_rads);
    printf("speed2ticks(%g, %g) -> (%i, %i)\n", v_ms, w_rads, v_int, w_int);
    return true;
  } // end speed2ticks

  //////////////////////////////////////////////////////////////////////////////

  void speed_cb(const geometry_msgs::TwistConstPtr & msg) {
    int v_int, w_int;
    if (!speed2ticks(msg->linear.x, msg->angular.z, v_int, w_int)) {
      ROS_WARN("Fail in speed2ticks()");
      return;
    }
    if (!continuous_drive(v_int, w_int)) {
      ROS_WARN("Fail in continuous_drive()");
      return;
    }
  } // end speed_cb();

  //////////////////////////////////////////////////////////////////////////////

  GMainLoop *main_loop;
  ros::NodeHandle _nh_public, _nh_private;
  // publishers
  ros::Publisher _battery_voltage_pub, _battery_percentage_pub;
  ros::Publisher _status_pub;
  ros::Time _status_battery_stamp;
  ros::Publisher _odometer_reading_pub, _absspeed_pub;
  double _last_odom;
  ros::Time _last_odom_stamp;
  ros::Time _odometer_reading_stamp;
  std_msgs::Float32 _float_msg;
  std_msgs::String _string_msg;
  // subscribers
  ros::Subscriber _speed_sub;
}; // end class Rossumo

#endif // ROSsumo_H

