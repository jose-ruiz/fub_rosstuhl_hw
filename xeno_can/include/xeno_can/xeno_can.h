/**
 *  @file
 *  @author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berlin.de>
 *  @date February, 2016
 *
 *  @brief Prototypes for accessing enAble50 functionalities to control the Xeno
 *  wheelchair by Otto Brock.
 *
 *  @section LICENSE
 *
 *  This code of the XenoCANBase class is based on the specification of enAble
 *  50 provided to us by Curtis Instruments AG under non-disclosure
 *  agreement. You cannot place it into public domain without permission.
 */

#ifndef XENOCAN_H
#define XENOCAN_H

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>
#include <memory>
#include <atomic>
#include <chrono>


/**
 * \class CANMessage
 *
 * @brief An interface to a CAN message meant to be independant of the
 * underlying CAN library.
 *
 * This class exposes public members id_, length_ and data_ to allow the user to
 * manipulate CAN messages independently of the underlying CAN library used.
 *
 */
class CANMessage : public can_frame
{
public:
  decltype(can_frame::can_id)& id_=      can_frame::can_id;   ///< the CAN ID
  decltype(can_frame::can_dlc)& length_= can_frame::can_dlc;  ///< Length of the payload in bytes
  decltype(can_frame::data)& data_=      can_frame::data;    ///< The payload (unsigned char)

  /**
   * Fills the fields of a CAN message from the passed parameters
   *  
   * @param id the CAN ID of the message
   *
   * @param length the length of the payload (data field). Valued values are
   * between 0 and 8. No boundary checkings is performed when filling the
   * message, so be sure to provide the correct length.
   * @param Payload of the CAN message
  */
  void fill(unsigned int id, size_t length, unsigned char data[8]);
};

/**
 * @class CANConnection
 * @brief a connection to a CAN bus interface that allows to send and receive messages.
 */

class CANConnection
{
 public:
  /**
   * @brief Creates an unconnected CANConnection object.
   */
  CANConnection();
  /**
   * @brief Connects to a CAN interface.
   * @param iface The name of the CAN bus interface to connect to, e.g. "can0";
   */

  /**
   * Disconnects from device and destroys object
   */
  ~CANConnection();

  void connect(const std::string iface);

  /**
   * @brief Sends a CAN message.
   * @param CAN message to send. 
   */
  void send(const CANMessage& msg);
  /**
   * @brief Received a CAN message, if available.
   * @param msg A CANMessage object whose member will be filled with the data of the incomming message.
   * @returns true if a message was read, false otherwise.
   *
   * This function will wait for an incomming CAN message until a timeout
   * occurrs. If an message was received, its contents will be used to fill
   * msg. msg will remain unmodified if there was not incomming message.
   */

  bool writeError() const noexcept;
  bool readError() const noexcept;
  bool error() const noexcept;
  
  bool receive(CANMessage& msg);

  void kill();
   
 protected:
   int descriptor_;    ///< file descriptor for  CAN I/O
   std::string iface_; ///< CAN interface to which the object is connected.

 private:
   /**
    * @brief Returns if there is either incomming CAN data or if a timeout is
    * exceeded.
    * @returns returns 0 if timeout, 1 if input available, -1 if error.
    */
   std::atomic<bool> write_error_;
   std::atomic<bool> read_error_;   
   int inputTimeout() const;
};

/**
 * @class XenoCANBase
 * @brief This class allows abstract access to data events defined in the enAble
 * 50 specification and allows to access the most important components of the
 * wheelchair, e.g. lights on and off, etc.
 *
 * By specializing this class and overriding its virtual methods, a subclass can
 * process button events and joystick data arriving from the wheelchair in an
 * ad-hoc fashion.
 */
class XenoCANBase
{ 
 public:
  XenoCANBase(const std::string iface);
  ~XenoCANBase();
  void setGear(unsigned char gear);
  void nextGear();
  void processIncomming();
  void setJoystickCANId(unsigned char new_id);
  void bypassJoystick();
  void enableJoystick();
  const CANConnection& getConnection() const noexcept;
  
 protected:
  CANConnection connection_;
  /*
   * Relevant CAN IDs
  */
  const unsigned int	
	CAN_ID_SET_GEAR_BUTTON=          0x91, // In the wheelchair's documentation
	CAN_ID_SET_GEAR=                 0x92, // they say "mode" instead of gear.
	CAN_ID_DUMMY_JOYSTICK_READING=   0x84,
	CAN_ID_ACTIVE_JOYSTICK_READING=  0x81,
	CAN_ID_SET_JOYSTICK_ID=         0x702,
	CAN_ID_LIGHT_BUTTON=            0x161,
	CAN_ID_BUZZER_BUTTON=            0x662,	
	CAN_ID_VOLTAGE_INFO=                  0x7a0,
	CAN_ID_SET_WINKER=               0xa2,
	CAN_ID_SWITCH_ON=                0x00;

  void callSwitchOnHandler(const CANMessage& msg) noexcept;
  bool isActiveJoystick(const CANMessage& msg) noexcept;
  virtual void joystickHandler(const CANMessage& msg) noexcept;
  virtual void setGearButtonHandler(const CANMessage& msg) noexcept;
  virtual void setGearHandler(const CANMessage& msg) noexcept;
  virtual void lightButtonHandler(const CANMessage& msg) noexcept;
  virtual void otherMessageHandler(const CANMessage& msg) noexcept;
  virtual void switchOnHandler(const CANMessage& msg) noexcept;
  virtual void buzzerEventHandler(const CANMessage& msg) noexcept;
  virtual void voltageInfoHandler(const CANMessage& msg) noexcept;
};




/**
 * @class JoystickCommand
 * 
 * @brief Represents a velocity command of the wheelchair, either it if is
* syntetic (i.e.) for autonomos drivig or a comming from the physical joystick.
 */
class JoystickCommand {
 public:
  JoystickCommand();
  std::chrono::system_clock::time_point time;
  int linear_, angular_;
  int& x_= angular_;
  int& y_= linear_;
};

/**
 * @class XenoStatus 
 *
 * @brief Represents the overall status of the Xeno wheelchair, inclusing also
* parameters used when controlling the wheelchair via an external agent.
*/

class XenoStatus
{ 
 public:
  using gear_mode_t= enum {FIXED= 0, FREE= 2};
  using control_mode_t= enum {USER=0, OVERRIDE= 1};
  
  std::chrono::system_clock::time_point time;
  JoystickCommand
	joystick_command_,
	override_command_;
  bool buzzer_button_pressed_;
  control_mode_t control_mode_;
  gear_mode_t override_mode_gear_mode_,
	current_gear_mode_;
  unsigned char gear_constraint_;
  unsigned  current_gear_;
  unsigned char battery_charge_;
  bool lights_on_;
  bool hazard_lights_on_;
  bool left_indicator_on_;
  bool right_indicator_on;
  bool connection_up_;
  bool stopped_;
  float battery_voltage_;
  unsigned int battery_percentage_;
  bool charger_attached_;
};

/**
 * @class XenoCAN
 *
 * @defines Defines the basic workflow to interact with the Xeno wheelchair,
 * to safely switch between user and override modes.
 *
 * In user mode, the user drives the wheelchair using the joystick, whereas in
 * override mode, the wheelchair ignores the joystick commands and allows us to
 * control it, for example, to perform autonomous driving.
 *
 * For safety, the wheelchair will be in user mode at startup. To switch the
 * current mode, the user has to push the horn button.
 * 
 */

class XenoCAN : public XenoCANBase
{
 public:
  using time_point_t= std::chrono::system_clock::time_point;
  using system_clock= std::chrono::system_clock; 
 	
  XenoCAN(const std::string& iface);
  void init();
  void enableOverrideMode();
  void enableUserMode();
  void setOverrideModeGearMode(XenoStatus::gear_mode_t mode, unsigned char gear_constraint);
  void sendJoystickCommand(const JoystickCommand& cmd);
  const XenoStatus& getStatus() const;
  void allowMovement();
  void inhibitMovement();
  
 protected:
  time_point_t buzzer_button_pressed_last_seen_;
  time_point_t last_set_gear_time;
  bool setting_gear_;
  XenoStatus status_;
  
  virtual void activeJoystickHandler(const CANMessage& msg) noexcept;
  virtual void dummyJoystickHandler(const CANMessage& msg) noexcept;
  virtual void joystickHandler(const CANMessage& msg) noexcept override;
  virtual void lightButtonHandler(const CANMessage& msg) noexcept override;
  virtual void setGearHandler(const CANMessage& msg) noexcept override;
  virtual void setGearButtonHandler(const CANMessage& msg) noexcept override;
  virtual void otherMessageHandler(const CANMessage& msg) noexcept override;
  virtual void switchOnHandler(const CANMessage& msg) noexcept override;
  virtual void buzzerEventHandler(const CANMessage& msg) noexcept override;
  virtual void voltageInfoHandler(const CANMessage& msg) noexcept override;
  void printStatus() const;
  void enforceGearPolicy();
};

#endif /* XENOCAN_H */
