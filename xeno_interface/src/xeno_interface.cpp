/**
 * @author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berln.de>
 * @date February 2016
 */

#include <xeno_interface/xeno_interface.h>
#include <xeno_interface/XenoStatus.h>

XenoInterface::XenoInterface()
{
  emergency_state_= true;
  joystick_publisher_= nh_.advertise<sensor_msgs::Joy>("joystick", 10);
  status_publisher_= nh_.advertise<xeno_interface::XenoStatus>("status", 5);
  virtual_joystick_publisher_= nh_.
 	subscribe<sensor_msgs::Joy>("virtual_joystick", 1,
								&XenoInterface::virtualJoystickCallback,
 								this);
	emergency_button_subscriber_= nh_.
	subscribe<std_msgs::Bool>("global_stop", 1,
							  &XenoInterface::emergencyButtonCallback,
							  this);
  std::string can_interface;
  nh_.param<std::string>("can_interface", can_interface, "can0");
  xeno_= std::make_shared<XenoCAN>(can_interface);
}

void XenoInterface::publishStatus(const XenoStatus& status) const {
  auto& con= xeno_->getConnection();
  sensor_msgs::Joy joy;
  joy.header.frame_id= "joystick";
  joy.header.stamp= ros::Time::now();
  joy.axes.push_back(status.joystick_command_.angular_);
  joy.axes.push_back(status.joystick_command_.linear_);
  joystick_publisher_.publish(joy);
  xeno_interface::XenoStatus status_msg;
  status_msg.header.stamp= ros::Time::now();
  status_msg.stopped= status.stopped_;
  status_msg.battery_percentage= status.battery_percentage_;
  status_msg.battery_voltage= status.battery_voltage_;
  status_msg.charger_attached= status.charger_attached_;
  status_msg.gear= status.current_gear_;
  status_msg.autonomous_mode_enabled=
  status.control_mode_ == XenoStatus::control_mode_t::OVERRIDE ? 1 : 0;
  status_msg.can_connected= true; // TODO: Update  
  status_msg.can_connected=	!con.readError();
  status_msg.can_transfer_error= con.error();
  status_publisher_.publish(status_msg);  
}

void XenoInterface::processPending() {
  if (!xeno_)
	return;
  if (emergency_state_) 
	xeno_->inhibitMovement();
  else
	xeno_->allowMovement();
  xeno_->processIncomming();
  XenoStatus status= XenoStatus(xeno_->getStatus());
  publishStatus(status);
}

void XenoInterface::virtualJoystickCallback(const sensor_msgs::JoyConstPtr& msg) {
  //TODO: Send 0 0 last message sent is old
  if (msg->axes.size() != 2) // ill-formed message
	return;
  JoystickCommand j;  
  j.angular_= msg->axes[0];
  j.linear_ = msg->axes[1];
  // Don't allow to travel backwards
  if (j.linear_ < 0) 
	j.linear_= 0;
  xeno_->sendJoystickCommand(j);
}

void XenoInterface::emergencyButtonCallback(const std_msgs::BoolConstPtr& msg) {
  emergency_state_= msg->data;
}
