#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <vector>
#include <boost/lexical_cast.hpp>
#include <autonomos_stereo_camera/types.h>

cv::Mat parse_matrix(TiXmlNode* node, cv::Mat& result);
cv::Mat parse_vector(TiXmlNode* node, cv::Mat& result);

void write_calibration_file(const std::string& path, const StereoCalibration& sc);
void write_vector_xml(std::ofstream& out, const cv::Mat& v);
#endif 

