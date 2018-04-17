/**
 * @author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berlin.de>
 * @date March 2016
 */


#include <xeno_interface/xeno_controller.h>
#include <random>
#include <map>
#include <sensor_msgs/Joy.h>

// TODO: Move to URDF
const double TICKS_PER_REVOLUTION= 124.0 * 4.0; // count

Xeno::Xeno()
{
  stopped= true;
  autonom= false;
  // initialize all state and command variables to zero!
  stop_subscriber= nh.subscribe("global_stop", 5, &Xeno::stopCallback, this);
  ticks_subscriber= nh.subscribe("wheel_rear_ticks", 5, &Xeno::ticksCallback, this);
  status_subscriber= nh.subscribe("status", 5, &Xeno::statusCallback, this);
  joy_publisher= nh.advertise<sensor_msgs::Joy>("virtual_joystick", 10);
  bzero(cmd, sizeof(cmd));
  bzero(pos, sizeof(cmd));
  bzero(vel, sizeof(cmd));
  bzero(eff, sizeof(cmd));
  current_joystick_cmd= std::pair<double, double>(0.0, 0.0);


  linear_pid.initParam("base_controller/pid/linear");
  angular_pid.initParam("base_controller/pid/angular");

  // connect and register the joint state interface
  hi::JointStateHandle state_handle_a("rear_left_wheel", &pos[0], &vel[0], &eff[0]);
  jnt_state_interface.registerHandle(state_handle_a);

  hi::JointStateHandle state_handle_b("rear_right_wheel", &pos[1], &vel[1], &eff[1]);
  jnt_state_interface.registerHandle(state_handle_b);

  registerInterface(&jnt_state_interface);
  hardware_interface::JointHandle li(jnt_state_interface.getHandle("rear_left_wheel"), &cmd[0]);
  jnt_vel_interface.registerHandle(li);
  hardware_interface::JointHandle lr(jnt_state_interface.getHandle("rear_right_wheel"), &cmd[1]);
  jnt_vel_interface.registerHandle(lr);
  registerInterface(&jnt_vel_interface);

  // TODO: set window size equal to that used by diff_driver_controller
  vel_left_acc= std::make_shared<rolling_mean_t>(tag::rolling_window::window_size = 30);
  vel_right_acc= std::make_shared<rolling_mean_t>(tag::rolling_window::window_size = 30);
  
}

void Xeno::statusCallback(const xeno_interface::XenoStatus::ConstPtr& msg)
{

  autonom= msg->autonomous_mode_enabled;
  error= msg->can_transfer_error;

}

void Xeno::stopCallback(const std_msgs::Bool::ConstPtr& msg)
{
  stopped= msg->data;
}

void Xeno::read()
{
}


void Xeno::ticksCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  pending_odom.push(msg);
}

void Xeno:: updateFromTicks()
{
  while (pending_odom.size())
	{
	  auto o= pending_odom.front();
	  pending_odom.pop();
	  if (last_odo != nullptr)
		{ 
		  auto ticks_left= o->vector.x,
			ticks_right= o->vector.y;
		  /* 
		   *The diff_driver_controller infers wheel velocity from differences
		   * in position (rad). Hence we only need to update position, so that
		   * odometry is computed properly.
		   */
		  static auto tmp= (2 * M_PI) / TICKS_PER_REVOLUTION; // rad per tick
		  double dleft= ticks_left * tmp, // rad per tick
			dright= ticks_right * tmp;
		  pos[0]+= dleft; 
		  pos[1]+= dright;
		  /*
		   *We also update velocity use it with the PIDs. 		   
		   */
		  
		  auto dt= (o->header.stamp - last_odo->header.stamp).toSec();
		  vel[0]= dleft / dt;
		  vel[1]= dright / dt;
		  
		  (*vel_left_acc)(vel[0]);		  
		  (*vel_right_acc)(vel[1]);
		}
	  last_odo= o;	
	}  
}

void Xeno::write()
{
  updateFromTicks();
  if (!autonom || error)
	{
	  angular_pid.reset();
  	  linear_pid.reset();
	  return;
	}
  if (stopped)
	{
	  angular_pid.reset();
  	  linear_pid.reset();
 	  stop();
  	  return;
	}
  runPids();
  static auto scaleValue= [](double& v, double min= -100, double max= 100) -> void
  {
	if (v < min)
	   v= min;
	else if (v > max)
	  v= max;
  };

  scaleValue(pid_effort.first);
  scaleValue(pid_effort.second);
  sendJoyCommand();
}


void Xeno::stop()
{
  sensor_msgs::Joy msg;
  msg.header.stamp= ros::Time::now();
  msg.header.frame_id= "joystick";
  msg.axes.push_back(0);
  msg.axes.push_back(0);
  joy_publisher.publish(msg);

}
void Xeno::sendJoyCommand()
{
  sensor_msgs::Joy msg;
  msg.header.stamp= ros::Time::now();
  msg.header.frame_id= "joystick";
  msg.axes.push_back(pid_effort.second);
  msg.axes.push_back(pid_effort.first);
  joy_publisher.publish(msg);
}
void Xeno::runPids()
{
  current_pid_time= ros::Time::now();
  runAngularVelocityPid();
  runLinearVelocityPid();

  last_pid_update= current_pid_time;
}

void Xeno::runAngularVelocityPid()
{  
  double angular_error= (cmd[0] - cmd[1]) - 
	(rolling_mean(*vel_left_acc) - rolling_mean(*vel_right_acc));
  auto def= angular_pid.computeCommand(angular_error, current_pid_time - last_pid_update) /
   10000.0;
  pid_effort.second+= def;
 // std::cout << "Angular "  << " error : " << angular_error <<  " def " << def << std::endl;
}

void Xeno::runLinearVelocityPid()
{
  /*
   * Linear velocity corrections will only be performed if the individual
   * velocities have the same sign.
   */
  //if ((cmd[0] * cmd[1]) < 0) {
//	return;
  //}
  double avg_vel=
(rolling_mean(*vel_right_acc) + rolling_mean(*vel_right_acc)) / 2.0;
  double avg_cmd= (cmd[0] + cmd[1]) / 2.0;
  double linear_error= avg_cmd - avg_vel;
  auto def= linear_pid.computeCommand(linear_error, current_pid_time - last_pid_update)
  / 10000.0;
  pid_effort.first+= def;
//  std::cout << "Linear "  << " error : " << linear_error <<  " def " << def << std::endl;
}

