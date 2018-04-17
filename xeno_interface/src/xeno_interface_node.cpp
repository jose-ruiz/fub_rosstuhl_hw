/**
 * @author Jose Alvarez-Ruiz <jose.alvarez-ruiz.fu-berlin.de>
 * @date February 2016
 *
 * @brief This node exposes the basic functionalities and status information of
 * the Xeno wheelchair to the ROS ecosystem.
 */

#include <xeno_interface/xeno_interface.h>
#include <ros/ros.h>
#include <iostream>




int main(int argc, char *argv[])
{
  ros::init(argc, argv, "xeno_interface");
  ros::NodeHandle nh;
  XenoInterface xeno_interface ;  
  while (ros::ok())
 	{
	  xeno_interface.processPending();
	  ros::spinOnce();
	}
  return 0;
}
