/**
 * @author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berlin.de>
 * @date February 2016
 */

#include "xeno_can/xeno_can.h"
#include <iostream>
#include <net/if.h>
#include <unistd.h>
#include <map>
#include <string.h>
/* PF_CAN and AF_CAN are not in the socketCAN headers for some reason */
#define PF_CAN 29
#define AF_CAN PF_CAN


/******************************************************************************
 * CANMessage implementation
 ******************************************************************************/

void CANMessage::fill(unsigned int id, size_t length, unsigned char data[8])
{
  if (length > 8)
	{
	  std::cout << "CAN payload length too large! " << length << std::endl;
	}
  id_= id;
  length_= length;
  bzero(&data_[0], sizeof(data));
  memcpy(&data_[0], &data[0], length * sizeof(unsigned char));
}

/******************************************************************************
 * CANConnection implementation
 ******************************************************************************/

CANConnection::CANConnection()
{
  descriptor_= -1;
}

CANConnection::~CANConnection() {
  kill();
}

void CANConnection::connect(const std::string iface)
{
  if (descriptor_ != -1)
	{
	  close (descriptor_);
	  descriptor_= -1;
	}
  struct sockaddr_can addr;  
  descriptor_= socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (descriptor_ == -1)
  {
	std::cout << "Error CAN socket descriptor " << std::endl;
  }
  addr.can_family = AF_CAN;  
  addr.can_ifindex = if_nametoindex(iface.c_str());
  auto bind_result= bind(descriptor_, (struct sockaddr *)&addr,
						 sizeof(addr));
  if(bind_result != 0)
	{
	  std::cout << "Error binding CAN socket to " << iface << std::endl;
	}
}

void CANConnection::send(const CANMessage& msg) 
{

  // No data is send in the just listen mode
#ifdef XENO_CAN_JUST_LISTEN
  write_error_= false;
  return;
#endif
  auto result= write(descriptor_, &msg, sizeof(can_frame));
  if (result == -1)
	{
  	  perror("CAN bus write error: ");
	  write_error_= true;
	}
  else
	write_error_= false;
}

bool CANConnection::writeError() const noexcept
{
  return write_error_;
}

bool CANConnection::readError() const noexcept
{
  return read_error_;
}

bool CANConnection::error() const noexcept
{
  return writeError() || readError();
}


int CANConnection::inputTimeout() const
{
  // From https://www.gnu.org/software/libc/manual/html_node/Waiting-for-I_002fO.html
  fd_set set;
  struct timeval timeout;
  /* Initialize the file descriptor set. */
  FD_ZERO (&set);
  FD_SET (descriptor_, &set);  
  /* Initialize the timeout data structure. */
  timeout.tv_sec= 0;
  timeout.tv_usec= 100000; // timeout at 100 ms
  /* select returns 0 if timeout, 1 if input available, -1 if error. */
  return TEMP_FAILURE_RETRY (select (FD_SETSIZE,
                                     &set, NULL, NULL,
                                     &timeout));
}

bool CANConnection::receive(CANMessage& msg) 
{
  if (inputTimeout() == 1)
	{	  
	  read(descriptor_, &msg, sizeof(can_frame));
	  read_error_= false;
	  return true;
	}
  else
	read_error_= true;
  return false;
}

void CANConnection::kill() {
  close(descriptor_);
  descriptor_= -1;
}

/******************************************************************************
 * XenoCANBase implementation
 ******************************************************************************/

XenoCANBase::XenoCANBase(const std::string iface)
{
  connection_.connect(iface);
}

XenoCANBase::~XenoCANBase()
{ 
  enableJoystick();
  connection_.kill();
}

const CANConnection& XenoCANBase::getConnection() const noexcept
{
  return connection_;
}

// TODO: remove?
void XenoCANBase::setGear(unsigned char gear) 
{
  CANMessage msg;
  unsigned char data[8]= {0x01, 0x00, gear, 0x00, 0x00, 0x00, 0x00, 0x00};
  msg.fill(CAN_ID_SET_GEAR, 8, data);
  connection_.send(msg);
}

void XenoCANBase::nextGear()
{
  CANMessage msg;
  unsigned char data[8] = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  msg.fill(CAN_ID_SET_GEAR_BUTTON, 8, data);
  connection_.send(msg);
}

void XenoCANBase::processIncomming() 
{
  CANMessage incomming;
  if(!connection_.receive(incomming))
	  return;
  using handler_t= std::function<void(XenoCANBase*, const CANMessage&)>;
  using handler_map_t= std::map<unsigned int, handler_t>;
  static handler_map_t handlers=
	{
	  {CAN_ID_SWITCH_ON,               &XenoCANBase::callSwitchOnHandler},
	  {CAN_ID_LIGHT_BUTTON,            &XenoCANBase::lightButtonHandler},
	  {CAN_ID_SET_GEAR_BUTTON,         &XenoCANBase::setGearButtonHandler},
	  {CAN_ID_SET_GEAR,                &XenoCANBase::setGearHandler},
	  {CAN_ID_ACTIVE_JOYSTICK_READING, &XenoCANBase::joystickHandler},
	  {CAN_ID_DUMMY_JOYSTICK_READING,  &XenoCANBase::joystickHandler},
	  {CAN_ID_BUZZER_BUTTON,           &XenoCANBase::buzzerEventHandler},
	  {CAN_ID_VOLTAGE_INFO,           &XenoCANBase::voltageInfoHandler}
	};
  auto handler= handlers.find(incomming.id_);
  if (handler != handlers.end()) 
	handler->second(this, incomming);  
    else
    	otherMessageHandler(incomming);
}

void XenoCANBase::callSwitchOnHandler(const CANMessage& msg) noexcept
{
  // System enters operational state after being switched on
  if (msg.data_[0] == 1 && msg.data_[1] == 3 && msg.data_[2] == 1) 
	switchOnHandler(msg);
}

void XenoCANBase::setJoystickCANId(unsigned char new_id)
{
  /* Service data object (SDO) download (write) request.
	 It changes the joystick used for input to COB-ID (CAN_ID) CAN_SWITCH_JOYSTICK

	 * CAN payload format is as follows
	 * + Command specifier: data[0]:  
	 *    value 0x2b = 00101011b
	 *    bit index    76543210
	 *	  meaning:
	 *	   bits  7-5  Constant = 001 (code for download)
	 *	   bit     4  Constant=   0 (empty) 
	 *	   bits  3-2  Number of bytes in CAN message WITHOUT data
	 *     bit     1  Standard= 1 (transfer type= expedite)
	 *	   bit     0  Standard= 1 (size indicated?= true)
	 * + Index: data[1] and data[2] (in the object dictionary)
	 * + Sub-index: data[3]
	 * + Payload: data[4]-data[7]
	 page 20 curtis specification. Read canopen specifications for more details
  */ 	  
  CANMessage msg;
  unsigned char data[8] = {0x2b, 0x0, 0x30, 0x02, new_id, 0x00, 0x00, 0x00};
  msg.fill(CAN_ID_SET_JOYSTICK_ID, 8, data);
  connection_.send(msg);
}

void XenoCANBase::bypassJoystick()  
{
  setJoystickCANId(CAN_ID_DUMMY_JOYSTICK_READING);
}

void XenoCANBase::enableJoystick() 
{
  setJoystickCANId(CAN_ID_ACTIVE_JOYSTICK_READING);
}

bool XenoCANBase::isActiveJoystick(const CANMessage& msg) noexcept
{
  return (msg.id_ == CAN_ID_ACTIVE_JOYSTICK_READING);
}


void XenoCANBase::joystickHandler(const CANMessage& msg) noexcept
{
}

void XenoCANBase::setGearButtonHandler(const CANMessage& msg) noexcept
{
}

void XenoCANBase::setGearHandler(const CANMessage& msg) noexcept
{
}

void XenoCANBase::lightButtonHandler(const CANMessage& msg) noexcept
{
}

void XenoCANBase::otherMessageHandler(const CANMessage& msg) noexcept
{  
}

void XenoCANBase::switchOnHandler(const CANMessage& msg) noexcept
{  
}

void XenoCANBase::buzzerEventHandler(const CANMessage& msg) noexcept
{
}


void XenoCANBase::voltageInfoHandler(const CANMessage& msg) noexcept  
{
}

// void XenoCANBase::hazard_light_button_handler(const CANMessage& msg) {
//   std::cout << " Hazard light button pressed " << std::endl;
// }
// void XenoCANBase::left_indicator_light_button_handler(const CANMessage& msg) { 
// }
// void XenoCANBase::right_indicator_light_button_handler(const CANMessage& msg) { 
// }


/******************************************************************************
 * JoystickCommand implementation
 ******************************************************************************/

JoystickCommand::JoystickCommand()
{
  linear_= 0;
  angular_= 0;
}

/******************************************************************************
 * XenoCAN implementation
 ******************************************************************************/

XenoCAN::XenoCAN(const std::string& iface) :
  XenoCANBase(iface), status_()
{  
  init();
}

void XenoCAN::init()
{
  status_.stopped_= true;
  status_.buzzer_button_pressed_= false;
  setOverrideModeGearMode(XenoStatus::gear_mode_t::FIXED, 2);
  status_.current_gear_= 0; // Invalid value, forces first event to always  considered
  setting_gear_= false;
  enableUserMode();	
}

void XenoCAN::enableOverrideMode()
{
  status_.control_mode_= XenoStatus::control_mode_t::OVERRIDE;
  status_.current_gear_mode_= status_.override_mode_gear_mode_;
}

void XenoCAN::enableUserMode()
{
  status_.control_mode_= XenoStatus::control_mode_t::USER;
  status_.current_gear_mode_= XenoStatus::gear_mode_t::FREE;
}

void XenoCAN::setOverrideModeGearMode(XenoStatus::gear_mode_t mode, unsigned char gear_constraint)
{
  status_.override_mode_gear_mode_= mode;
  status_.gear_constraint_= gear_constraint;
}

void XenoCAN::activeJoystickHandler(const CANMessage& msg) noexcept
{
  bypassJoystick();
}

void XenoCAN::dummyJoystickHandler(const CANMessage& msg) noexcept
{   

  CANMessage new_msg;  
  static unsigned char stop_data[8]= {0x00, 0, 0x00,0x64, 0x0, 0x03, 0x00, 0x00};
  // in case some unhandled condition occurrs
  new_msg.fill(CAN_ID_ACTIVE_JOYSTICK_READING, 8, stop_data);
  // stop the wheelchair
  if (status_.stopped_ ||
	  setting_gear_ ||
	  (status_.control_mode_ == XenoStatus::control_mode_t::OVERRIDE &&
	   status_.current_gear_ != status_.gear_constraint_)) {
	new_msg.fill(CAN_ID_ACTIVE_JOYSTICK_READING, 8, stop_data);
  }
  // forward user's command to the wheelchair
  else if (status_.control_mode_ ==  XenoStatus::control_mode_t::USER) {
	new_msg.fill(CAN_ID_ACTIVE_JOYSTICK_READING, 8, msg.data_);
	}

  else { // Send override command
	unsigned char override_data[8]= {(unsigned char) status_.override_command_.angular_,
							(unsigned char) status_.override_command_.linear_,
							0x00,0x64, 0x0, 0x03, 0x00, 0x00};
	new_msg.fill(CAN_ID_ACTIVE_JOYSTICK_READING, 8, override_data);
  }  
  connection_.send(new_msg);
}

void XenoCAN::joystickHandler(const CANMessage& msg) noexcept
{
  status_.joystick_command_.angular_= (signed char)msg.data_[0];
  status_.joystick_command_.linear_= (signed char)msg.data_[1];
  if (isActiveJoystick(msg))	
	activeJoystickHandler(msg);
  else
	{
	  dummyJoystickHandler(msg);
	  enforceGearPolicy();
  }
}

void XenoCAN::lightButtonHandler(const CANMessage& msg) noexcept
{
}

void XenoCAN::setGearHandler(const CANMessage& msg) noexcept
{
  auto requested_gear= msg.data_[2];
  if (requested_gear != status_.current_gear_)
	{
	  status_.current_gear_= requested_gear;
	  last_set_gear_time= system_clock::now();
	  setting_gear_= true;
	}
}

void XenoCAN::setGearButtonHandler(const CANMessage& msg) noexcept
{
}

void XenoCAN::enforceGearPolicy()
{
   
  /* We don't know what the current gear is, so we'll have to change the gear to
   * find out.
   *
   */
  //  if (status_.control_mode_ == XenoStatus::control_mode_t::USER)
  //	return;
  if (!setting_gear_ && status_.current_gear_ == 0) 
	{
	  setting_gear_= true;
	  nextGear();
	}
  else if (setting_gear_)
	{
	  std::chrono::milliseconds elapsed=
		std::chrono::duration_cast<std::chrono::milliseconds>(system_clock::now() - last_set_gear_time);
	  if (elapsed.count() > 85) 
		setting_gear_= false;	
  }
  else if (status_.control_mode_ == XenoStatus::control_mode_t::OVERRIDE &&
		   status_.override_mode_gear_mode_ == XenoStatus::gear_mode_t::FIXED &&
		   status_.current_gear_ != status_.gear_constraint_) {
	setting_gear_= true;
	nextGear();
  }
}

void XenoCAN::otherMessageHandler(const CANMessage& msg) noexcept
{
  if (status_.buzzer_button_pressed_)
	{
	  auto now= system_clock::now();
	  auto diff= std::chrono::duration_cast<std::chrono::milliseconds>
		(now - buzzer_button_pressed_last_seen_).count();
	  if (diff > 510) 
		status_.buzzer_button_pressed_= false;	  
	}
}

void XenoCAN::switchOnHandler(const CANMessage& msg) noexcept
{
  init();
}

void XenoCAN::buzzerEventHandler(const CANMessage& msg) noexcept
{
  if (msg.data_[1] == 0x01)
	{
	  if (!status_.buzzer_button_pressed_)
		{ // Buzzer was just pressed
		  if (status_.control_mode_ == XenoStatus::control_mode_t::USER)
			enableOverrideMode();
		  else
			enableUserMode();
		}
	  status_.buzzer_button_pressed_= true;	
	  buzzer_button_pressed_last_seen_= system_clock::now();
	}
}

void XenoCAN::voltageInfoHandler(const CANMessage& msg) noexcept  
{
  status_.battery_percentage_ = (signed char) msg.data_[0];
  signed short tmp= (msg.data_ [1] << 8) | msg.data_[2];
  status_.battery_voltage_ = tmp / 100.0f;
  status_.charger_attached_= msg.data_[3];
}

void XenoCAN::sendJoystickCommand(const JoystickCommand& cmd)
{
  // clip the values to valid range [-100..100]
  signed char linear=  std::min(std::max(-100, cmd.linear_),
							   100);
  signed char angular= std::min(std::max(-100, cmd.angular_),
							   100);
  status_.override_command_.linear_= linear;
  status_.override_command_.angular_= angular;
}

const XenoStatus& XenoCAN::getStatus() const
{
  return(status_);
}

void XenoCAN::allowMovement()
{
  status_.stopped_= false;
}
void XenoCAN::inhibitMovement()
{
  status_.stopped_= true;
}

void XenoCAN::printStatus() const
{
  std::cout << "Gear_Mode:  " << (status_.current_gear_mode_ == XenoStatus::gear_mode_t::FIXED? " Fixed " : "Free") << " ";
  std::cout << "  Control_Mode:  " << (status_.control_mode_ == XenoStatus::control_mode_t::USER? " User " : "Override") << " ";
  std::cout << "  Stat_Stop:  " << (status_.stopped_? " Y " : "N") << " ";
  std::cout << "  Setting_gear:  " << (setting_gear_? " Y " : "N") << " ";
  std::cout << std::endl;
}
