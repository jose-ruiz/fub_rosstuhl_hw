#ifndef TYPES_H
#define TYPES_H

#include <chrono>
#include <string>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <tinyxml.h>


using time_point= std::chrono::system_clock::time_point;

class Header
{
 public:
  Header();
  Header(struct timeval timestamp, std::string frame_name);
  time_point time;
  std::string frame_name;
  unsigned long long index;
  void update_timestamp();
};


class Image : public cv::Mat
{
 public:
  Header header;
};


class IntrinsicParameters
{
 public:
  cv::Mat values;
  void load(TiXmlNode* node);
  const float get_cx() const noexcept;
  const float get_cy() const noexcept;
  const float get_fx() const noexcept;
  const float get_fy() const noexcept;
};

class ExtrinsicParameters
{
 public:
  cv::Mat position, rotation, rotation_quaternion;
  void set_rotation_matrix(const cv::Mat& m);
  void load(TiXmlNode* node);
};

class DistortionCoefficients
{
 public:
  cv::Mat coefficients;
  void load(TiXmlNode* node);
};



class CameraParameters
{
 public:
  Header header;
  ExtrinsicParameters extrinsic;
  IntrinsicParameters intrinsic;
  DistortionCoefficients distortion;
  cv::Size size;
 public:
  void load(TiXmlNode* node);
};

class StereoCalibration
{

 protected:
  cv::Mat Qinv;
  cv::Mat R1, R2;

 public:
  Header header;
  void load(TiXmlNode* node);
  bool load_from_file(std::string path);
  bool load_from_camera(std::string host);
  void updateQ(bool for_undistorted);
  cv::Mat& getQ();
  void someToPointCloud(cv::Mat& points, cv::Mat& result);
  void toPointCloud(cv::Mat& disparity_float, cv::Mat& result);
  void toImage(cv::Mat& points, cv::Mat& result);
  CameraParameters left, right;
  // Monocular camera parameters (e.g. without rectification into account)
  CameraParameters left_mono, right_mono; 
  cv::Mat left_map1, left_map2, 
    right_map1, right_map2;
  cv::Mat Q;
  cv::Mat left_P, right_P;
  cv::Mat left_R, right_R;
  float get_baseline() const;
  
};


#endif /* TYPES_H */
