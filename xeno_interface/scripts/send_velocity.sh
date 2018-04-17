#!/bin/bash

###
### @author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berlin.de>
### @date August 2016
###
### Send a latched linear and angular velocity tuple (l, a) to the cmd_vel
### topic.  After doing so, pressing <enter> will send velocity (0, 0) to stop
### the robot.
###


usage() {

	echo 'Sends a velocity command (l, a), where l is the linear velocity in m/s
	and a is the angular velocity in rad/s to the wheelchair.'

}


RATE=100

send_latched_velocity()
{
rostopic pub -l -r ${RATE} /xeno/base_controller/cmd_vel geometry_msgs/Twist "linear:
  x: $1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: $2"
	
}

send_velocity()
{
 rostopic pub -r ${RATE} /xeno/base_controller/cmd_vel geometry_msgs/Twist "linear:
  x: $1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: $2"
	
}

if [ $# -ne 2 ]; then
	usage
	exit 1
fi

echo 'Sending latched velocity message ($1 m/s, $2 rad/s). Press <enter> to stop (0 m/s, 0 rad/s)'

send_latched_velocity $1 $2

echo 'Sending stop message'

#send_latched_velocity -$1 -$2
#send_velocity 0.0 0.0



