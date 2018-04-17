/**
 * @author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berlin.de>
 * @date   March 2016
 */

#include <ros/ros.h>
#include <autonomos_stereo_camera/autonomos_stereo_camera.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <iostream>



class AutonomosStereoCameraPublisher
{
public:
  AutonomosStereoCameraPublisher();
  void customToRosHeader(const Header& in, std_msgs::Header& out);
  void callback (std::shared_ptr<AutonomosStereoCameraOutput> msg);
  void publishDepth(std::shared_ptr<AutonomosStereoCameraOutput> msg);
  void publishCameraInfo(std::shared_ptr<AutonomosStereoCameraOutput> msg);
  void fillArrayRowMajor(std::vector<double>& result, const cv::Mat& source) const;
  template <typename T>
  void fillArrayRowMajor(T& result, const cv::Mat& source) const;
  bool tf_available;
tf::Transform transform;
protected:
  ros::NodeHandle nh;
  ros::Publisher
  left_p, left_rect_p, left_camerainfo_p,
	right_p, right_rect_p, right_camerainfo_p,
	disparity_p;

};

template <typename T>
void AutonomosStereoCameraPublisher::
fillArrayRowMajor(T& result, const cv::Mat& source) const
{
  unsigned int idx= 0;
  for (int r= 0; r < source.rows; r++)
	for (int c= 0; c < source.cols; c++)
	  {
		result[idx]= source.at<double>(r, c);
		idx++;
	  }
}

void AutonomosStereoCameraPublisher::
fillArrayRowMajor(std::vector<double>& result, const cv::Mat& source) const
{
  for (auto r= 0; r < source.rows; r++)
	for (auto c= 0; c < source.cols; c++)
	  result.push_back(source.at<double>(r, c));
}

void AutonomosStereoCameraPublisher::
publishCameraInfo(std::shared_ptr<AutonomosStereoCameraOutput> msg)
{
  // Fill in camerainfo for left camera
  auto left_caminfo=  boost::make_shared<sensor_msgs::CameraInfo>();
  left_caminfo->height= msg->left->rows;
  left_caminfo->width= msg->left->cols;
  left_caminfo->distortion_model= "plumb_bob";
  fillArrayRowMajor(left_caminfo->D, msg->stereo_calibration->left.distortion.coefficients);
  fillArrayRowMajor(left_caminfo->K, msg->stereo_calibration->left_mono.intrinsic.values);
  fillArrayRowMajor(left_caminfo->R, msg->stereo_calibration->left_R);
  fillArrayRowMajor(left_caminfo->P, msg->stereo_calibration->left_P);    
  customToRosHeader(msg->left->header, left_caminfo->header);
  left_caminfo->header.frame_id= "stereo1_left";
  // Fill in camerainfo for right camera
  auto right_caminfo=  boost::make_shared<sensor_msgs::CameraInfo>();
  right_caminfo->height= msg->right->rows;
  right_caminfo->width= msg->right->cols;
  right_caminfo->distortion_model= "plumb_bob";
  fillArrayRowMajor(right_caminfo->D, msg->stereo_calibration->right.distortion.coefficients);
  fillArrayRowMajor(right_caminfo->K, msg->stereo_calibration->right_mono.intrinsic.values);
  fillArrayRowMajor(right_caminfo->R, msg->stereo_calibration->right_R);
  fillArrayRowMajor(right_caminfo->P, msg->stereo_calibration->right_P);
  customToRosHeader(msg->right->header, right_caminfo->header);
  right_caminfo->header.frame_id= "stereo1_right";
  // Publish 
  right_camerainfo_p.publish(right_caminfo);
  left_camerainfo_p.publish(left_caminfo);
}

void AutonomosStereoCameraPublisher::customToRosHeader(const Header& in, std_msgs::Header& out) {
  auto tot_nsec= in.time.time_since_epoch().count();
  out.stamp.sec= std::trunc(tot_nsec / 1e9);
  out.stamp.nsec= tot_nsec - (out.stamp.sec * 1e9);
  out.frame_id= in.frame_name;
  
}

AutonomosStereoCameraPublisher::AutonomosStereoCameraPublisher()
{
  tf_available= false;
  
  static std::map<std::string, ros::Publisher*> publisher_list=
	{
	  {"left/image_mono", &left_p},
	  {"left/image_rect", &left_rect_p},
	  {"right/image_mono", &right_p},
	  {"right/image_rect", &right_rect_p},
	};
	for (auto& it : publisher_list) {
	  (*it.second)= nh.advertise<sensor_msgs::Image>(it.first, 5);
	}
	disparity_p= nh.advertise<stereo_msgs::DisparityImage>("disparity", 5);
	left_camerainfo_p= nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 5);
	right_camerainfo_p= nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 5);
}

void AutonomosStereoCameraPublisher::publishDepth(std::shared_ptr<AutonomosStereoCameraOutput> msg)
{
  auto enc_img= cv_bridge::CvImage(std_msgs::Header(),
								   sensor_msgs::image_encodings::TYPE_32FC1, 
								   *msg->disparity_float).toImageMsg();

  stereo_msgs::DisparityImagePtr disp_msg =
	// TODO: ensure no copy is performed
	boost::make_shared<stereo_msgs::DisparityImage>();
  disp_msg->image= *enc_img;
  disp_msg->min_disparity= 0;
  disp_msg->max_disparity= 30;
  disp_msg->T= -msg->stereo_calibration->get_baseline();
  disp_msg->f= msg->stereo_calibration->left.intrinsic.get_fx();
  customToRosHeader(msg->disparity_float->header, disp_msg->header);
  disp_msg->header.frame_id= "stereo1_left";
  disparity_p.publish(disp_msg);  
  
}

void AutonomosStereoCameraPublisher::callback (std::shared_ptr<AutonomosStereoCameraOutput> msg)
{
  if (!tf_available)
	{
	  auto& t= msg->stereo_calibration->right.extrinsic.position;
	  auto& R= msg->stereo_calibration->right.extrinsic.rotation;
	  transform.setOrigin(tf::Vector3(t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0)));
	  transform.setBasis(tf::Matrix3x3(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
									   R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
									   R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)));
	  transform= transform.inverse();
	  tf_available= true;
	}  
  struct PublishRecipe {
	std::shared_ptr<Image> data;
	ros::Publisher* publisher;
	std::string encoding;
	std::string frame_id;
  };
  std::vector <struct PublishRecipe> recipes=
 	{
 	  {msg->left, &left_p, "mono8", "stereo1_left"},
	  {msg->left_rect, &left_rect_p, "mono8", "stereo1_left"},
	  {msg->right, &right_p, "mono8", "stereo1_right"},
 	  {msg->right_rect, &right_rect_p, "mono8", "stereo1_right"}
	  
 	};
   for (auto& it : recipes)
 	{
 	  if (!it.publisher->getNumSubscribers ())
 		continue;
	  auto ros_msg= cv_bridge::CvImage(std_msgs::Header(),
									   it.encoding, *
									   it.data).toImageMsg();
	  
	  customToRosHeader(it.data->header, ros_msg->header);
	  ros_msg->header.frame_id= it.frame_id;
 	  it.publisher->publish(ros_msg); 
 	}
   publishDepth(msg);
   publishCameraInfo(msg);
   // TODO publish TF
}



ViskosParam get_parameters()
{
  ros::NodeHandle nh;
  bool HDR;
  int min_shutter, max_shutter,	min_gain, max_gain,
	target_median, max_shutter_change, max_gain_change,
	alpha;

//   nh.param<bool>("/HDR", HDR, true);
//   nh.param<int>("/min_shutter", min_shutter, 10);
//   nh.param<int>("/max_shutter", max_shutter, 80);
//   nh.param<int>("/max_shutter_change", max_shutter_change, 5);  
//   nh.param<int>("/min_gain", min_gain, 10);
//   nh.param<int>("/max_gain", max_gain, 80);
//   nh.param<int>("/max_gain_change", max_gain_change, 5);
//   nh.param<int>("/target_median", target_median, 20);
//   nh.param<int>("/target_median", alpha, 20);
// 
  
  nh.param<bool>("/HDR", HDR, true);
  nh.param<int>("/min_shutter", min_shutter, 1);
  nh.param<int>("/max_shutter", max_shutter, 200);
  nh.param<int>("/max_shutter_change", max_shutter_change, 5);
  
  nh.param<int>("/min_gain", min_gain, 1);
  nh.param<int>("/max_gain", max_gain, 60);
  nh.param<int>("/max_gain_change", max_gain_change, 5);

  nh.param<int>("/target_median", target_median, 40);
  nh.param<int>("/target_median", alpha, 20);

  std::string msg_base= "Autonomos stereo camera configuration error: ";
  if (min_shutter > max_shutter)  
	ROS_ERROR_STREAM(msg_base +
					 "max_shutter must be larger than min_shutter.");

  if (min_gain > max_gain)  
	ROS_ERROR_STREAM(msg_base + "max_gain must be larger than min_gain.");

  if (min_gain < 0 || max_gain < 0 ||
	  min_shutter < 0 || max_shutter < 0)
	ROS_ERROR_STREAM(msg_base +
					 "shutter and gain values must be possitive.");

  if (target_median < 0)
	ROS_ERROR_STREAM(msg_base + "target_median must be larger than or equal to 0");

  if (max_shutter_change < 0 || max_gain_change < 0)
	ROS_ERROR_STREAM(msg_base +
					 "max_gain_change max_shutter_change must be larger than"
					 " or equal to 0 ");

  if (max_gain_change > 10 || max_shutter_change > 10)
	ROS_ERROR_STREAM(msg_base +
					 "max_gain_change and max_shutter_change must be less than 10");
  
  ViskosParam p= {
	{"HDR", (HDR? "1" : "0")},
	{"MinGain", std::to_string(min_gain)},
	{"MaxGain", std::to_string(max_gain)},
	{"MaxGainChange", std::to_string(max_gain_change)},
	{"MinShutter", std::to_string(min_shutter)},
	{"MaxShutter", std::to_string(max_shutter)},
	{"MaxShutterChange", std::to_string(max_shutter_change)},
	{"TargetMedian", std::to_string(target_median)},
	{"Alpha", std::to_string(alpha)},	
  };
  p= p + ViskosParam::stream_distorted();
  return p;
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "autonomous_camera_node");
  ros::NodeHandle nh;
  tf::TransformBroadcaster br;

  auto p= get_parameters();
  AutonomosStereoCameraPublisher pub;
  Viskos vk([&pub] (std::shared_ptr<AutonomosStereoCameraOutput> msg) -> void
			{
			  pub.callback(msg);
			},
			p);
  ros::Rate rate(100);
  while (ros::ok())
	{
	  if (pub.tf_available)
		{
		  br.sendTransform(tf::StampedTransform(pub.transform, ros::Time::now(), "stereo1_left", "stereo1_right"));

		}
	  ros::spinOnce();
	  rate.sleep();
	}
  return 0;
}
