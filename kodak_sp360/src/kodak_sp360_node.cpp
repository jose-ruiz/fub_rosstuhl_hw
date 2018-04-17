/**
 * @author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berlin.de>
 * @data March 2016
 *
 * @brief This nodes publishes the images streamed by the Kodak SP360 camera
 * using HTTP and JPEG compression as raw ROS images to the topic "kodak_sp360".
 */

#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/opencv.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <memory>

class KodakSP360 
{
public:
  KodakSP360(std::string host="172.16.0.254", unsigned short port= 9176);
  void run();
protected:
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket socket_;
  boost::asio::ip::tcp::endpoint endpoint_;
  boost::asio::streambuf input_buffer_;
  ros::Publisher img_publisher_, ci_publisher_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::ServiceServer set_camera_info_service;
  std::string camera_info_uri_;
  std::string camera_name_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
  bool calibration_matches_;
  
};

KodakSP360::KodakSP360(std::string host, unsigned short port) :
  nh_private_("~"),
  endpoint_(boost::asio::ip::address::from_string(host), port),
  socket_(io_service_)
{
  nh_private_.param<std::string>("camera_info_uri", camera_info_uri_,
								 "file:///tmp/kodaksp360-1.yaml");
    nh_private_.param<std::string>("camera_info_uri", camera_name_,
								 "kodak_sp360-1");
  cinfo_= boost::make_shared<camera_info_manager::CameraInfoManager>
	(nh_, camera_name_, camera_info_uri_);
  ros::spinOnce();
  img_publisher_= nh_.advertise<sensor_msgs::Image>("image_raw", 5);
  ci_publisher_= nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 5);
  boost::system::error_code error;
  socket_.connect(endpoint_, error);
  if (error != 0) {
    throw("Could not connect to Kodak SP360 camera.");
  }
}

void KodakSP360::run() {
  // Initiate-transfer request
  boost::system::error_code ignored_error;
  std::string message= "GET / HTTP/1.1/\r\nHost: 172.16.0.254:9176\r\nConnection: Keep-Alive\r\nUser-Agent: ROS/0.01\r\nAccept: */*\r\n\r\n";

  boost::asio::write(socket_, boost::asio::buffer(message),
                    boost::asio::transfer_all(), ignored_error);
  std::vector<char> buffer(220);
  boost::asio::read(socket_, boost::asio::buffer(buffer));
  std::vector<char> sub_head_buffer(55 + 20);
  for  (unsigned int frame= 0;; frame++) {
	if (!ros::ok())
	  return;

	boost::asio::read(socket_, boost::asio::buffer(sub_head_buffer));
	std::string n_frames_str(sub_head_buffer.begin() + 55, sub_head_buffer.begin() + 62);
	auto time= ros::Time::now();
	int n_bytes= atoi(n_frames_str.c_str());
	std::vector<char> image_buffer(n_bytes);
	int length_of_n_bytes_str= trunc(log10(n_bytes)) + 1;
	int trailing= (20 - length_of_n_bytes_str - 3);
	n_bytes-= trailing;
	// TODO: Avoid copy and reallocation
	for(int t=0; t < trailing; t++) {
	  image_buffer[t]= sub_head_buffer[t + 55 + length_of_n_bytes_str + 3];
	}
	std::vector<char> image_subbuffer(n_bytes);
	int n= boost::asio::read(socket_, boost::asio::buffer(image_subbuffer));
	for(int t=0; t < n_bytes; t++) 
	  image_buffer[t + trailing]= image_subbuffer[t];
	
	cv::Mat img= cv::imdecode(cv::Mat(image_buffer), 1);
	if (img.cols != 1024) 	
	  continue;
	sensor_msgs::ImagePtr msg= cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	msg->header.frame_id= "omnicam2";
	msg->header.stamp= time;
	
	img_publisher_.publish(msg);

	// Code adapted from:
	// https://github.com/ros-drivers/camera1394/blob/master/src/nodes/driver1394.cpp#L80
	
	sensor_msgs::CameraInfoPtr
      ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
	
	// check whether CameraInfo matches current video mode
	calibration_matches_= (img.cols == ci->width) && (img.rows == ci->height);
	if (!calibration_matches_)
      {
        // image size does not match: publish a matching uncalibrated
        // CameraInfo instead
        if (calibration_matches_)
          {
            // warn user once
            calibration_matches_ = false;
            ROS_WARN_STREAM("[" << camera_name_
                            << "] calibration does not match video mode "
                            << "(publishing uncalibrated data)");
          }
		ci.reset(new sensor_msgs::CameraInfo());
        ci->height = msg->height;
        ci->width = msg->width;
      }
    else if (!calibration_matches_)
      {
        // calibration OK now
        calibration_matches_ = true;
        ROS_WARN_STREAM("[" << camera_name_
                        << "] calibration matches video mode now");
      }
    ci->header.frame_id = msg->header.frame_id;
    ci->header.stamp = msg->header.stamp;
	ci_publisher_.publish(ci);
	ros::spinOnce();
  }
  
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "kodak_sp360_node");
  KodakSP360 k;
  ros::Rate rate(20.0);
  while (ros::ok())
	  k.run(); 
  return 0;
}
