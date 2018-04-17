/**
 * @author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berln.de>
 * @date February 2016
 */


#ifndef XENOINTERFACE_H
#define XENOINTERFACE_H

#include <ros/ros.h>
#include <xeno_can/xeno_can.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

class XenoInterface
{
 public:
  XenoInterface();
  void processPending();
  void publishStatus(const XenoStatus& status) const;
  /**
   * Currently we ignore assume that reading is meant to be produced in the same
   * coordinate system as the physical joystick.
   */
 protected: 
  std::shared_ptr<XenoCAN> xeno_;
  ros::NodeHandle nh_;
  ros::Publisher joystick_publisher_, status_publisher_;
  ros::Subscriber emergency_button_subscriber_;
  ros::Subscriber virtual_joystick_publisher_;

  void virtualJoystickCallback(const sensor_msgs::JoyConstPtr& msg);
  void emergencyButtonCallback(const std_msgs::BoolConstPtr& msg);
  bool emergency_state_;
};


#endif /* XENOINTERFACE_H */
