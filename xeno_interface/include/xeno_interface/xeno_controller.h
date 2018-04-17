/**
 * @author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berlin.de>
 * @date March 2016
 */

#ifndef XENO_CONTROLLER_H
#define XENO_CONTROLLER_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <hardware_interface/robot_hw.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <queue>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <xeno_interface/XenoStatus.h>
#include <std_msgs/Bool.h>
#include <atomic>
#include <mutex>

namespace hi= hardware_interface;
using namespace boost::accumulators;



class Xeno : public hi::RobotHW
{
 public:
  Xeno();
  void read();
  void write();
protected:
  using rolling_mean_t= accumulator_set<double, stats<tag::rolling_mean> >;

  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];

  ros::NodeHandle nh;
  hi::JointStateInterface jnt_state_interface;
  hi::VelocityJointInterface jnt_vel_interface;
  control_toolbox::Pid linear_pid, angular_pid;
  std::shared_ptr<rolling_mean_t> vel_left_acc, vel_right_acc;
  ros::Subscriber ticks_subscriber, stop_subscriber, status_subscriber;
  ros::Publisher joy_publisher;
  std::queue <geometry_msgs::Vector3Stamped::ConstPtr> pending_odom;
  geometry_msgs::Vector3Stamped::ConstPtr last_odo;  
  std::atomic<bool> stopped, autonom, error;
  xeno_interface::XenoStatus last_status;
  std::mutex mutex;
  void stopCallback(const std_msgs::Bool::ConstPtr& msg);
  void statusCallback(const xeno_interface::XenoStatus::ConstPtr& msg);
  void ticksCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
  void updateFromTicks();
  void initializePid(control_toolbox::Pid& pid, const std::string&  name);
  void runPids();
  void runAngularVelocityPid();
  void runLinearVelocityPid();
  void sendJoyCommand();
  void stop();
 
  ros::Time last_pid_update;
  ros::Time current_pid_time;

  std::pair<double, double> current_joystick_cmd;
  std::pair<double, double> pid_effort;
};


#endif /* XENO_CONTROLLER_H */

