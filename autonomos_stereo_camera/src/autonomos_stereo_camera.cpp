/**
 * @author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berlin.de>
 * @date September 2014
 *
 * @brief This module implements the communication interface to the stereo
 * camera developed by Bennet Fischer. It supports the following features: start
 * the camera (with possibility to setup image acquisition parameters), and
 * receive frames and calibration parameters. The main limitation of this
 * implementation is that the image acquisition parameters cannot be changed
 * dynamically. If such a changes become necessary, the camera will have to be
 * restarted.
*/

#include <autonomos_stereo_camera/autonomos_stereo_camera.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <sys/select.h>
#include <stdio.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <memory>

#define SWAP_UINT32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | ((x) << 24))
void swapInt (void * ptr) {
   uint32_t *n = (uint32_t *)ptr;
   for (int i=0; i<7; i++)
     n[i] = SWAP_UINT32 (n[i]);
}


ACUDPServer::ACUDPServer(unsigned short port) :
  UDPServer(port, 3000)
{
	// run the io_service run in another thread.
  boost::function<void()> f= [this, port]() ->void  {
	handle_receive();
  };

  thread_ptr= std::make_shared<boost::thread>(f);
}

void ViskosFrameReceiver::handle_receive()
{
  size_t SOFHeaderSize= sizeof(SOFHeader);
  SOFHeader* sof1_ptr= (SOFHeader*) &recv_buffer[0];
  DataHeader* data_ptr= (DataHeader*) &recv_buffer[0];

  const int n= 10000;
  while(true) {
	size_t len = socket->receive_from(boost::asio::buffer(recv_buffer), remote_endpoint);
	if (!len) {
	  boost::this_thread::sleep(boost::posix_time::microseconds(n));
	  continue;
	}
	auto prestamp= sof1_ptr->timeStamp;

	swapInt((void*) sof1_ptr);
	if (sof1_ptr->packetId== 0)
	  {
		frame_width= sof1_ptr->frameWidth;
		frame_height= sof1_ptr->frameHeight;
	  // Allocate space for the image TODO: Account for short images
	  if (image_data_ptr == nullptr) 
		image_data_ptr= new char[frame_width * frame_height * 4];	  
	  memcpy (image_data_ptr, (void *)&recv_buffer[SOFHeaderSize], data_ptr->byteCount);
	  img.header.time= time_point() + std::chrono::nanoseconds(prestamp);
	  img.header.frame_name= name;
	  img.header.index++;
	}
  else if(sof1_ptr->packetId == 1)
	{ 
	  if (image_data_ptr == NULL)
		{ // still waiting for SOFHeader
		  //		  start_receive();
		  boost::this_thread::sleep(boost::posix_time::microseconds(n));
		  continue;
		}
	  memcpy (&image_data_ptr[data_ptr->byteOffset],
	  		  (void *)&recv_buffer[sizeof(DataHeader)], data_ptr->byteCount);
	  if (data_ptr->endOfFrameFlag) {
		commit_image();
	  }
	}}
}

void ViskosFrameReceiver::commit_image()
{
  static int dims[]= {frame_height, frame_width};
  img.Mat::operator=(cv::Mat(2, dims, format, image_data_ptr));  
  std::shared_ptr<Image>copy_of= std::make_shared<Image>();
  copy_of->header= img.header;
  img.copyTo(*copy_of);
  callback(copy_of);
}

/************************************************************
ViskosParam implementation
************************************************************/
ViskosParam::ViskosParam() {  
}

ViskosParam::ViskosParam(std::initializer_list<std::pair<std::string, std::string> > il) {
  for (auto const& p: il) 
    (*this)[p.first]= p.second;  
}

ViskosParam operator+(const ViskosParam& lhs, const ViskosParam& rhs) {  
  ViskosParam result= lhs;
  for (auto const& p: rhs) 
      result[p.first]= p.second;
  return (result);  
}

std::ostream& operator<<(std::ostream& os, const ViskosParam& obj) {
  for (auto const& p : obj) {
    os << p.first << "=" << p.second << std::endl;
  }
  return os;
}

ViskosParam ViskosParam::HDR_on() {
  return(ViskosParam({{"HDR", "1"}}));
}

ViskosParam ViskosParam::HDR_off() {
  return(ViskosParam({{"HDR", "0"}}));  
}
ViskosParam ViskosParam::fixed_gain(int gain) { 
    return(ViskosParam({
          {"MinGain", boost::lexical_cast<std::string>(gain)}, 
            {"MaxGain", boost::lexical_cast<std::string>(gain)},
            {"MaxGainChange", "0"}
        }));  
}

ViskosParam ViskosParam::variable_gain(int min_gain, int max_gain, int max_gain_change) {
  return(ViskosParam({
        {"MinGain", boost::lexical_cast<std::string>(min_gain)}, 
          {"MaxGain", boost::lexical_cast<std::string>(max_gain)},
            {"MaxGainChange", boost::lexical_cast<std::string>(max_gain_change)}
      }));    
}
ViskosParam ViskosParam::fixed_shutter(int shutter) {
  return(ViskosParam({
        {"MinShutter", boost::lexical_cast<std::string>(shutter)}, 
          {"MaxShutter", boost::lexical_cast<std::string>(shutter)},
          {"MaxShutterChange", "0"}
      }));  
}

ViskosParam ViskosParam::variable_shutter(int min_shutter, int max_shutter, int max_shutter_change) {
  return(ViskosParam({
        {"MinShutter", boost::lexical_cast<std::string>(min_shutter)}, 
          {"MaxShutter", boost::lexical_cast<std::string>(max_shutter)},
            {"MaxShutterChange", boost::lexical_cast<std::string>(max_shutter_change)}
      }));  
}

ViskosParam ViskosParam::stream_distorted() {
  return(ViskosParam({{"RectificationLevel", "0"}}));
}

ViskosParam ViskosParam::stream_undistorted() {
  return(ViskosParam({{"RectificationLevel", "1"}}));
}


/* ViskosDisparity */

cv::Mat3b ViskosDisparity::mColorLut;

void ViskosDisparity::disparityToInt(cv::Mat& disparity, cv::Mat& result) {
  result.create(disparity.size(), CV_16UC1);
  for (int i = 0; i < disparity.rows; ++i) {
    short* ptr = disparity.ptr<short>(i);
    short* ptr_res = result.ptr<short>(i);
    for (int j = 0; j < disparity.cols; ++j, ++ptr, ++ptr_res) {
      short d = (*ptr);
      if (d < 0)
        (*ptr_res)= -1;
      else {
        (*ptr_res)= d >> 4;
      }
    }
  }
}

void ViskosDisparity::disparityToFloat(const cv::Mat& disparity_raw, cv::Mat& result) {
  result.create(disparity_raw.size(), CV_32FC1);
  for(int i=0; i < disparity_raw.rows; i++) {
    for(int j=0; j < disparity_raw.cols; j++) {
      short value= disparity_raw.at<short>(i, j);
      if (value < 0) 
        result.at<float>(i, j)= -1.0f;      
      else {
        result.at<float>(i, j)=  value >> 4;
        unsigned short fpoint= value & 0x07;
        if (fpoint != 0) {
          result.at<float>(i, j)+= float(fpoint) / pow(10, (floor(log10(fpoint)) + 1));
        }
      }
    }
  }
}

void ViskosDisparity::decode_disparity_map(cv::Mat& disparityImage, cv::Mat& img16bit) {
  if (mColorLut.empty()) {
    mColorLut.create(1, 180);
    cv::Mat_<cv::Vec3b> hsv(1, 180);
    cv::Vec3b * p = hsv.ptr<cv::Vec3b>(0);    
    for (int i = 0; i < mColorLut.cols; ++i, ++p) {
      *p = cv::Vec3b(i, 255, 255);
    }    
    cv::cvtColor(hsv, mColorLut, CV_HSV2BGR);
  }
  
  disparityImage.create(img16bit.rows, img16bit.cols, CV_8UC4);
  int mMaxDisparity= 20;
  float mAlphaValid= 1.0f;
  float mAlphaInvalid= 0.0f;
  int mMinDisparity= 2;
  int mMaxHue= 180;  
  uint8_t aValid = 255 * mAlphaValid;
  uint8_t aInvalid = 255 * mAlphaInvalid;        
  int maxHue = std::min(179, mMaxHue / 2);

  for (int i = 0; i < disparityImage.rows; ++i) {
    float * ptr = img16bit.ptr<float>(i);
    cv::Vec4b * imgPtr = disparityImage.ptr<cv::Vec4b>(i);
    for (int j = 0; j < disparityImage.cols; ++j, ++ptr, ++imgPtr) {
      float d = (*ptr);
      if (d < 0) {
        *imgPtr = cv::Vec4b(0, 0, 0, aInvalid);
        continue;
      }
      else if (d > mMaxDisparity) {
        d= mMaxDisparity;
      }
      else if (d < mMinDisparity) 
        d= mMinDisparity;
      int index = maxHue - int(double(maxHue) * (d - mMinDisparity) /
                               ((mMaxDisparity - mMinDisparity))); // red near, blue far
      cv::Vec3b c = mColorLut(0, index);
      *imgPtr = cv::Vec4b(c[0], c[1], c[2], aValid);
    }
  }
}

 ViskosFrameReceiver::ViskosFrameReceiver(std::string name, unsigned int port, unsigned int format, image_fn_t fn) :
  ACUDPServer(port)
{
  img.header.index= -1;
  this->format= format;
  this->port= port;
  this->name= name;
  this->callback= fn;
}

ViskosFrameReceiver::~ViskosFrameReceiver() {

}

void ViskosFrameReceiver::run() {
//   H= &sof1;
//   D= (DataHeader *)&sof1;
//   dPtr= NULL;
//   uint16_t DepthMap [752][480];
//   dPtr= (unsigned char*) &DepthMap [0][0];  
//   int psize= sizeof(SOFHeader);
//   buffer[0]=0;
//   unsigned int bytes_written=0;
//   unsigned int frame_width= 752;
//   unsigned int frame_height= 480;
//   Styx::Time::time_point hwTimeStamp;
//   CS<Image>* cbs= CSR<Image>(name);
// 
//   while( buffer[0]!='@' ) {
//     buffer[0]=0;
//     //we wait for socket
//     sof1.packetId = 256;
//     int size = inport.msocketRxData(buffer, BUFFER_SIZE);
//     if (!Styx::is_running()) {
//       usleep(30000);
//       continue;
//     }
//     if (size == -1) {
//       usleep(30000);
//       continue;
//     }    
//     memcpy ((void *)&sof1, (void*)&buffer[0], psize);
//     if (H->packetId== 0) {
//       hwTimeStamp= Styx::Time::time_point() + Styx::Time::nanoseconds(H->timeStamp);
//     }
//     swapInt ((void *)&sof1);
//     memcpy ((void *)&dh1, (void*)&sof1, psize);
//     H = &sof1;
//     D = &dh1;
//     if (H->packetId == 0) {
//       frame_width= H->frameWidth;
//       frame_height= H->frameHeight;
//       bzero(dPtr, frame_height * frame_width * sizeof(uint16_t));
//       memcpy (dPtr, (void *)&buffer[psize], H->byteCount);
//       bytes_written= H->byteCount;
//       msg.header.update_timestamp();
//     }
//     else if (H->packetId == 1) {
//       bytes_written+= D->byteCount;
//       memcpy ((void *)&dPtr[D->byteOffset], (void *)&buffer[sizeof(DataHeader)], D->byteCount);
//       if(D->endOfFrameFlag) {
// 	if (bytes_written != frame_width * frame_height * bytes_per_pixel) {
// 	  std::cout << "*D";
//           std::cout.flush();
// 	  continue;
// 	}
//         msg.header.index++;
// 	int dims[]= {frame_height, frame_width};			    
//         msg.Mat::operator=(cv::Mat(2, dims, format, dPtr));
//         std::shared_ptr<Image>copy_of= std::make_shared<Image>();
//         copy_of->header.time= hwTimeStamp;
//         msg.copyTo(*copy_of);
//         copy_of->header.frame_name= msg.header.frame_name;
//         copy_of->header.index= msg.header.index;
//         cbs->send(copy_of);
//       }
//     }
//   } // while
}

Viskos::Viskos(std::function<void(std::shared_ptr<AutonomosStereoCameraOutput>)> fn,
  const ViskosParam& par, const std::string& host)
{
//   first_frame_arrived= false;
  viskos_helper= std::make_shared<ViskosHelper>(fn);
  stop_camera();
  std::cout << "Starting Viskos camera with following overrides: " << std::endl;
  param= par;
  std::cout << par;
  start_camera();
  stereo_calibration= std::make_shared<StereoCalibration>();
  bool res= stereo_calibration->load_from_camera("asc-1");
  viskos_helper->set_active_calibration(stereo_calibration);
  if (!res) {
	std::cout << "Could not load calibration file... Aborting" << std::endl;
  }
//   }
//   CSR<ViskosParam>("Viskos_parameters")->set("update_parameters", &Viskos::parameters_callback, this);
//   sleep(2);
  setup_streaming();
//   // Wait for the first frame to arrive
//   if (Styx::is_online_mode()) 
// 	start(ExecutionThread::ASYNC);

  boost::function<void()> f= [this]() ->void  {
	run();
  };

  auto thread_ptr= std::make_shared<boost::thread>(f);

}

  void Viskos::setup_streaming() {
	left= std::make_shared<ViskosFrameReceiver>("_VL", VISKOS_LEFT_PORT, CV_8UC1,
												[this](const std::shared_ptr<Image>& img) {
												  left_callback(img);
												}
												);
	right= std::make_shared<ViskosFrameReceiver>("_VR", VISKOS_RIGHT_PORT, CV_8UC1,
												 [this](const std::shared_ptr<Image>& img) {
												   right_callback(img);
												 });
	disparity= std::make_shared<ViskosFrameReceiver>("_VD", VISKOS_DISPARITY_PORT, CV_16UC1,
													 [this](const std::shared_ptr<Image>& img) {
													   disparity_callback(img);
													 });
}

void Viskos::stop_camera() {  
  auto cmd= "sshpass -p root ssh -f root@asc-1 "
	"'nohup stopAutoCam.py '";
  system(cmd);
  sleep(2);
}

void Viskos::start_camera() {
  std::string param_str= "";
  for (const auto& p : param) {
    param_str+= p.first + "=" + p.second + " ";
  }
  auto cmd= "sshpass -p root ssh -f -n root@asc-1 "
    "'nohup startAutoCam.py " +  param_str + "'";// + "&'";
  system(cmd.c_str());
  // Might take up to 20 seconds to start!!!
}

void Viskos::delete_older_than(std::map<time_point, std::shared_ptr<Image> >& imgs,
							   const time_point& t) {
  std::map<time_point, std::shared_ptr<Image> >::iterator it;
  for (it= imgs.begin(); it != imgs.end();) {
    if (it->first <= t && std::abs((std::chrono::high_resolution_clock::now() -
									it->first).count()) / 1e9 > 0.5) {
      imgs.erase(it++);
    }
    else
      it++;
  }
}

void Viskos::send_if_sync(const time_point& t) {
  auto right_found= right_imgs.find(t) != right_imgs.end();
  auto left_found= left_imgs.find(t) != left_imgs.end();
  auto disparity_found= disparity_imgs.find(t) != disparity_imgs.end();
  if (!left_found || !right_found || !disparity_found) {
	return;
  }
    std::shared_ptr<Image> left_img= left_imgs[t], right_img= right_imgs[t], disparity_img= disparity_imgs[t];

	viskos_helper->set_images(left_img, right_img, disparity_img);
	viskos_helper->do_it();
    last_sent_time= left_img->header.time;
    first_sent= true;
  }

void Viskos::disparity_callback(const std::shared_ptr<Image>& img) {
  mutex.lock();
  first_frame_arrived= true;
  if (stereo_calibration && calibration_streamed_n_times < 10) {
    stereo_calibration->header.update_timestamp();
    calibration_streamed_n_times++;
    mutex.unlock();
  }
  disparity_imgs[img->header.time]= img;
  send_if_sync(img->header.time);
  mutex.unlock();
  usleep(10000);

}

void Viskos::left_callback(const std::shared_ptr<Image>& img) {
  mutex.lock();
  left_imgs[img->header.time]= img;
  send_if_sync(img->header.time);
  mutex.unlock();
  usleep(10000);
}

void Viskos::right_callback(const std::shared_ptr<Image>& img) {  
  mutex.lock();
  right_imgs[img->header.time]= img;
  send_if_sync(img->header.time);
  mutex.unlock();
  usleep(10000);
}

// bool Viskos::parameters_callback(const std::shared_ptr<ViskosParam> msg) {
//   stop_camera();
//   param= *msg;
//   start_camera();
// }

// bool Viskos::new_calibration_handler(const std::shared_ptr<StereoCalibration> msg) {
//   mutex.lock();
//   Styx::LOG_INFO("New calibration arrived");
//   Styx::LOG_INFO("Stopping camera");
//   stop_camera();
//   Styx::LOG_INFO("Uploading calibration file");
//   write_calibration_file("/tmp/viskos_calibration.xml", *msg);
//   auto cmd= "sshpass -p root scp  /tmp/viskos_calibration.xml root@" + host + ":office_stereo_cam.xml";
//   calibration_streamed_n_times= 0;
//   Styx::LOG_INFO("Starting camera");
//   start_camera();
//   mutex.unlock();
// }

void Viskos::run()
{
  //TODO
  while(true)
	{    
	  if (!mutex.try_lock()) {
		usleep(800000);
		continue;
	  }
	if (first_sent)
	  {
		delete_older_than(left_imgs, last_sent_time);
		delete_older_than(right_imgs, last_sent_time);
		delete_older_than(disparity_imgs, last_sent_time);
	  }
	mutex.unlock();
	usleep(800000);
	}  
}


ViskosHelper::ViskosHelper(std::function<void (std::shared_ptr<AutonomosStereoCameraOutput>) > fn) {

  callback= fn;

}

void ViskosHelper::set_active_calibration(std::shared_ptr<StereoCalibration> cal)
{
  active_calibration= cal;  
}

std::shared_ptr<Image> ViskosHelper::rectify(const cv::Mat& map1,
											 const cv::Mat& map2,
											 const std::string& name, 
											 std::shared_ptr<Image> img)
{
  std::shared_ptr<Image> rectified= std::make_shared<Image>();
  rectified->header= img->header;
  cv::remap(*img, *rectified, 
            map1, 
            map2,
            CV_INTER_LINEAR);
  return(rectified);
}

void ViskosHelper::set_images(const std::shared_ptr<Image>& left,
				  const std::shared_ptr<Image>& right,
				  const std::shared_ptr<Image>& disparity)
{
  this->left= left;
  this->right= right;
  this->disparity= disparity;
}

void ViskosHelper::do_it() {
  if (!(active_calibration && 
        left && right && disparity &&
        left->header.time == right->header.time && 
        left->header.time == disparity->header.time))
    return;
  std::shared_ptr<Image> disparity_float= std::make_shared<Image>();
  std::shared_ptr<Image> point_cloud= std::make_shared<Image>();
  std::shared_ptr<Image> pc_colored= std::make_shared<Image>();

  point_cloud->header= disparity->header;
  disparity_float->header= disparity->header;
  disparity_float->header.frame_name= "Viskos_df";
  ViskosDisparity::disparityToFloat(*disparity, *disparity_float);
  active_calibration->toPointCloud(*disparity_float, *point_cloud);
  // Merge intensity and 3D data into a point cloud
  std::vector<cv::Mat> left_float_channels, pc_colored_channels, pc_channels;
  cv::Mat left_float;
  left->convertTo(left_float, CV_32F);
  left_float*= 1/255.0f; //Scale to [0..1] range.
  cv::split(*point_cloud, pc_channels);
  cv::split(left_float, left_float_channels);
  pc_colored_channels.insert(pc_colored_channels.begin(),
                             pc_channels.begin(), pc_channels.end());
  pc_colored_channels.insert(pc_colored_channels.begin(),
                             left_float_channels.begin(), left_float_channels.end());
  cv::merge(pc_colored_channels, *pc_colored);
  //TODO disparity_float_cs->send(disparity_float);
  //TODO point_cloud_cs->send(point_cloud);
  auto left_rect= rectify(active_calibration->left_map1, active_calibration->left_map2,
                     "VL_unrect", left);
  auto right_rect= rectify(active_calibration->right_map1, active_calibration->right_map2,
                     "VR_unrect", right);

  auto out= std::make_shared<AutonomosStereoCameraOutput>();
  out->left= left;
  out->left_rect= left_rect;
  out->right= right;
  out->right_rect= right_rect;
  out->disparity_float= disparity_float;
  out->stereo_calibration= active_calibration;
  callback(out);
  left= nullptr;
  right= nullptr;
  disparity= nullptr;

}

