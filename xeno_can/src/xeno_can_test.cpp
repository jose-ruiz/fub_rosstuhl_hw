/*
 * This program allows to connect to the xeno wheelchair and send
 * joystick commands directly using the keyboard. It is useful to test
 * the connectivity of the CAN bus without additional layers of
 * cmplexity.
*/

#include <curses.h>
#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <xeno_can/xeno_can.h>

const unsigned int increment= 5;

void update_display(const XenoCAN& xeno, const JoystickCommand& cmd)
{
  move(0,0);
  addstr("XENO wheelchair CAN connectivity test");
  move(1,0);
  addstr("Jose Alvarez-Ruiz 2019 <jose.alvarez-ruiz@fu-berlin.de>");
  std::stringstream ss;
  ss << "Control mode: ";
  auto status= xeno.getStatus();
  if (status.control_mode_ == XenoStatus::control_mode_t::USER) {
    ss << "manual   ";
  }
  else if (status.control_mode_ == XenoStatus::control_mode_t::OVERRIDE) {
    ss << "OVERRIDE";
  }
  move(9, 0);
  addstr(ss.str().c_str());

  ss.str("");
  ss << "Angular: " 
     << std::setw(6)
     << std::setfill(' ') 
     << cmd.angular_ << "\t\tLinear:"
     << std::setw(6)
     << std::setfill(' ')
     << cmd.linear_;


  move(10, 0);
  addstr("Virtual Joystick command:");
  move(11, 0);
  addstr(ss.str().c_str());

  int row= 25;
  move(row++, 0);
  addstr("Key bindings:");
  move(row++, 0);
  addstr("\t<ENTER>>        Switch mode");
  move(row++, 0);
  addstr("\t<LEFT>/<RIGHT>  Angular velocity -/+");
  move(row++, 0);
  addstr("\t<UP>/<DOWN>     Linear velocity -/+");
  move(row++, 0);
  addstr("\t<SPACE>         Set all velocities to zero");
  move(row++, 0);
  addstr("\tq               quit   ");

  
}

bool process_input(XenoCAN& xeno, JoystickCommand& cmd)
{
  /*Returns true if program should quit*/
    auto key= getch();
    auto status= xeno.getStatus();
    switch (key) {
    case KEY_DOWN:
      if (cmd.linear_ < -99)
	break;
      cmd.linear_ -= increment;
      break;
    case KEY_UP:
      if (cmd.linear_ > 99)
	break;
      cmd.linear_ += increment;
      break;
    case KEY_LEFT:
      if (cmd.angular_ < -99)
	break;
      cmd.angular_ -= increment;
       break;
    case KEY_RIGHT:
      if (cmd.angular_ > 99)
	break;
      cmd.angular_ += increment;
       break;
    case ' ':
       cmd.angular_ = 0;
      cmd.linear_ = 0;     
      break;
    case '\n':
      if (status.control_mode_ == XenoStatus::control_mode_t::USER)
	{
	  xeno.enableOverrideMode();
	}
      if (status.control_mode_ == XenoStatus::control_mode_t::OVERRIDE)
	{
	  /*Clear the command for safety*/
	  cmd.linear_= 0;
	  cmd.angular_= 0;
	  xeno.enableUserMode();
	}
      break;
    case 'q':
      return true;
    }
    if (status.control_mode_ == XenoStatus::control_mode_t::OVERRIDE)
      {
	xeno.sendJoystickCommand(cmd);
      }
    return false;
}


void init_curses()
{
  initscr();
  cbreak();
  noecho();
  nodelay(stdscr, TRUE);
  curs_set(0);
  clear();
  keypad(stdscr, TRUE);
}

int main(int argc, char** argv){
  init_curses();
  ros::init(argc, argv, "xeno_controller_node");
  ros::NodeHandle nh;
  XenoCAN xeno("can0");
  xeno.enableUserMode();
  xeno.allowMovement();
  JoystickCommand cmd;
  ros::Rate r(200);
  bool override_mode= false;
  do {
    if (process_input(xeno, cmd))
      break;
    xeno.processIncomming();
    update_display(xeno, cmd);
    r.sleep();
  } while (ros::ok());
 end:
  endwin();
  return 0;
}
