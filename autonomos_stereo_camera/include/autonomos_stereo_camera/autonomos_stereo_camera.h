/**
 * @author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berlin.de>
 * @date September 2014
*/

#ifndef AUTONOMOS_STEREO_CAMERA_H
#define  AUTONOMOS_STEREO_CAMERA_H

#include <opencv2/opencv.hpp>
#include <boost/thread/mutex.hpp>
#include <autonomos_stereo_camera/Calibration.h>
#include <autonomos_stereo_camera/UDPComm.h>
#include <mutex>

struct SOFHeader {
	uint32_t packetId; 
	uint64_t timeStamp;
	uint32_t frameWidth;
	uint32_t frameHeight;
	uint32_t bytesPerPixel;
	uint32_t byteCount;
	uint32_t bayerPattern;
} __attribute__((__packed__));

typedef struct SOFHeader SOFHeader; 

struct DataHeader {
	uint32_t packetId;
	uint32_t endOfFrameFlag;
	uint32_t byteOffset;
	uint32_t byteCount;
	uint32_t reserved1;
	uint32_t reserved2;
	uint64_t reserved3;
} __attribute__((__packed__));
typedef struct DataHeader DataHeader; 


#define BUFFER_SIZE 3000
#define VISKOS_LEFT_PORT 2000
#define VISKOS_RIGHT_PORT 2001
#define VISKOS_DISPARITY_PORT 2002

using image_fn_t= std::function<void (const std::shared_ptr<Image>&)>;

class ACUDPServer : public UDPServer
{
 public:
  ACUDPServer(unsigned short port);
};



class ViskosFrameReceiver : public ACUDPServer
{ 
  friend class Viskos;
public:
  ViskosFrameReceiver(std::string name, unsigned int port, unsigned int format, image_fn_t fn);
  ~ViskosFrameReceiver();
  void run();
  
 protected:
  char* image_data_ptr= NULL;
  int frame_width, frame_height;
  unsigned int id;
  unsigned int format;
  unsigned int port;
  std::string name;
  //char buffer[BUFFER_SIZE];
  boost::array<char, 3000> recv_buffer;
  unsigned short int bytes_per_pixel;
  Image img;
  SOFHeader sof1;
  DataHeader   dh1;
  SOFHeader *H;
  DataHeader  *D;
  unsigned char *dPtr;
  virtual void handle_receive() override;
  void commit_image();
  image_fn_t callback;
};

class ViskosParam : public std::map<std::string, std::string> {
 public:
  
  static ViskosParam HDR_on();
  static ViskosParam HDR_off();
  static ViskosParam fixed_gain(int gain);
  static ViskosParam variable_gain(int min_gain, int max_gain, int max_gain_change= 10);
  static ViskosParam fixed_shutter(int exposure);
  static ViskosParam variable_shutter(int min_shutter, int max_shutter, int max_shutter_change= 10);
  static ViskosParam stream_distorted();
  static ViskosParam stream_undistorted();
  ViskosParam();
  ViskosParam(std::initializer_list<std::pair<std::string, std::string> > il);
  friend ViskosParam operator+(const ViskosParam& lhs, const ViskosParam& rhs);
  friend std::ostream& operator<<(std::ostream& os, const ViskosParam& obj); 
};

ViskosParam operator+(const ViskosParam& lhs, const ViskosParam& rhs);
std::ostream& operator<<(std::ostream& os, const ViskosParam& obj); 


struct AutonomosStereoCameraOutput
{
  std::shared_ptr<Image>
  left, right, disparity_float, left_rect, right_rect;
  std::shared_ptr<StereoCalibration> stereo_calibration;
};

class ViskosHelper {
 public:
  ViskosHelper(std::function<void(std::shared_ptr<AutonomosStereoCameraOutput>) > fn);
  void set_active_calibration(std::shared_ptr<StereoCalibration> cal);
  void do_it();
  void set_images(const std::shared_ptr<Image>& left,
				  const std::shared_ptr<Image>& right,
				  const std::shared_ptr<Image>& disparity);	
 protected:
  std::shared_ptr<Image> rectify(const cv::Mat& map1, const cv::Mat& map2,
                          const std::string& name, std::shared_ptr<Image> img);
  std::shared_ptr<StereoCalibration> active_calibration;
  std::shared_ptr<Image> left, right, disparity;
  std::function <void(std::shared_ptr<AutonomosStereoCameraOutput>) > callback;
};


 class Viskos
 { 
 public:
   std::shared_ptr<StereoCalibration> stereo_calibration, active_calibration; 
   std::shared_ptr<ViskosFrameReceiver>  
     left,  
     right,  
     disparity; 

   Viskos(std::function<void(std::shared_ptr<AutonomosStereoCameraOutput>)> fn,
		  const ViskosParam& par= ViskosParam(), 
          const std::string& host= "192.168.2.10");    
   void disparity_callback(const std::shared_ptr<Image>& img); 
   void left_callback(const std::shared_ptr<Image>& img); 
   void right_callback(const std::shared_ptr<Image>& img);
   std::mutex mutex;
/*   bool parameters_callback(const std::shared_ptr<ViskosParam> msg); */
   void start_camera(); 
  void setup_streaming(); 
  void stop_camera(); 
  void run();
/*   bool new_calibration_handler(const std::shared_ptr<StereoCalibration> msg); */
  protected: 
  ViskosParam param; 
/*   mode_t mode; */
/*   std::string host; */
  std::map<time_point, std::shared_ptr<Image> > left_imgs, right_imgs, disparity_imgs;
   void send_if_sync(const time_point& t); 
   void delete_older_than(std::map<time_point, std::shared_ptr<Image> >& imgs, 
						  const time_point& t); 
   time_point last_sent_time; 
  std::atomic<bool> first_frame_arrived; 
  bool first_sent; 
  int calibration_streamed_n_times; 
  std::shared_ptr<ViskosHelper> viskos_helper;
  
};

class ViskosDisparity {
 public:
  static void disparityToInt(cv::Mat& disparity, cv::Mat& result);
  static void disparityToFloat(const cv::Mat& disparity, cv::Mat& result);
  static void decode_disparity_map(cv::Mat& encoded, cv::Mat& decoded);  
 protected:
  static cv::Mat3b mColorLut;
};


#endif
