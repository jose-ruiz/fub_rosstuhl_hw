/**
 * @author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berlin.de>
 * @date March 2016
 */


#include <ros/ros.h>
#include <xeno_interface/xeno_controller.h>
#include <controller_manager/controller_manager.h>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "xeno_controller_node");
  ros::NodeHandle nh("/xeno");
  Xeno xeno;
  controller_manager::ControllerManager cm(&xeno, nh);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  Xeno xeno;
  JoystickCommand cmd;
  auto prev_time= ros::Time::now();
  ros::Rate rate(100.0);
  while (ros::ok())
	{
	  const ros::Time time= ros::Time::now();
	  const ros::Duration period= time - prev_time;
	  xeno.read();
	  cm.update(time, period);
	  xeno.write();
	  ros::spinOnce();
	  rate.sleep();
	}
  return 0;
}
