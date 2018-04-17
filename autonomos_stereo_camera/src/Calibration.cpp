#include <autonomos_stereo_camera/Calibration.h>
#include <Eigen/Geometry>
#include <fstream>
#include <boost/filesystem.hpp>
#include <opencv2/core/eigen.hpp>
#include <boost/format.hpp>

cv::Mat parse_vector(TiXmlNode* node, cv::Mat& result) {
    std::vector<double> result_vec;
    TiXmlElement *data_root= node->FirstChildElement("data");
    if (!data_root) {
      std::cout << "Could not find data root" << std::endl;
    }

    for(TiXmlElement* item= data_root->FirstChildElement("item");
        item != NULL; item= item->NextSiblingElement("item")) {
      double value= boost::lexical_cast<double>(item->GetText());
      result_vec.push_back(value);
    }
    result=cv::Mat(result_vec).clone();
    return result;
 }


cv::Mat parse_matrix(TiXmlNode* node, cv::Mat& result) {
  TiXmlElement *rows_root= node->FirstChildElement("rows");
  if (!rows_root) {
    std::cout << "Could not find the sensors root" << std::endl;
  }
    TiXmlElement *cols_root= node->FirstChildElement("cols");
  if (!cols_root) {
    std::cout << "Could not find the sensors root" << std::endl;
  }
  int rows= boost::lexical_cast<int>(rows_root->GetText());
  int cols= boost::lexical_cast<int>(cols_root->GetText());
  result.create(cv::Size(rows, cols), CV_64FC1);
  TiXmlElement* row_root= node->FirstChildElement("data");
  for(int row= 0; row < rows; row++, row_root= row_root->NextSiblingElement("data")) {
    if (!row_root)
      std::cout << "Could not find the row root" << std::endl;
    TiXmlElement* col_root= row_root->FirstChildElement("item");
    for(int col= 0; col < cols; col++, col_root= col_root->NextSiblingElement("item")) {
      if (!col_root)
        std::cout << "Could not find the item root" << std::endl;      
      double value= boost::lexical_cast<double>(col_root->GetText());
      result.at<double>(row, col)= value;
    }
  }
  return result;  
}

bool StereoCalibration::load_from_file(std::string path) {
  TiXmlDocument doc(path);
  if (!doc.LoadFile()) {
    std::cout << "Stereo camera calibration load failed" << std::endl;
    exit(1);
    return false;
  }
  load(&doc);
  return true;
}

bool StereoCalibration::load_from_camera(std::string host) {  
  // remove old file if there
  boost::filesystem::path path("/tmp/office_stereo_cam.xml");
  if (boost::filesystem::exists(path))
    boost::filesystem::remove(path);
  // get the file
  std::string command= "sshpass -p root scp root@" + host + ":office_stereo_cam.xml /tmp";
  int return_code= system(command.c_str());
  if (return_code) {
    std::cout << "Could not retrieve calibration file Visko Camera. " 
              << std::endl;
    return false;
  }
  bool res= load_from_file("/tmp/office_stereo_cam.xml");
  boost::filesystem::remove(path);
  std::cout << "Successfully loaded calibration file from camera " << std::endl;
  return res;
}

void StereoCalibration::load(TiXmlNode* node) {
    TiXmlElement *serialization_root= node->FirstChildElement("boost_serialization");
    if (!serialization_root) {
      std::cout << "Could not find the sensors root" << std::endl;
    }

    TiXmlElement *sensorType_root= serialization_root->FirstChildElement("sensorType");
    if (!sensorType_root) {
      std::cout << "Could not find the sensorType root" << std::endl;
    }

    TiXmlElement *px_root= sensorType_root->FirstChildElement("px");
    if (!px_root) {
      std::cout << "Could not find the px root" << std::endl;
    }

    TiXmlElement *sensors_root= px_root->FirstChildElement("sensors");
    if (!sensors_root) {
      std::cout << "Could not find the sensors root" << std::endl;
    }
    // Up to here, stuff we traverse "recursively and do not need... simplify the code.."
    // A simple macro would have done wonders here...     
    TiXmlElement* cam1_root= sensors_root->FirstChildElement("item");
    if(!cam1_root) {
      std::cout << "Could not find the item root (cam1)" << std::endl;    
    }        
    left.load(cam1_root);
    TiXmlElement* cam2_root= cam1_root->NextSiblingElement("item");
    if(!cam2_root) {
      std::cout << "Could not find the item root (cam2)" << std::endl;    
    }
    right.load(cam2_root);
    // Viskos seems to have another convention
    right.extrinsic.position= right.extrinsic.position * -1.0;      
    cv::transpose(right.extrinsic.rotation,
                  right.extrinsic.rotation);
    left.header.frame_name= "VL";
    right.header.frame_name= "VR";
    header.frame_name= "VL";
    header.update_timestamp();
    left.header.time= header.time;
    right.header.time= header.time;
    updateQ(false);
  

}

void StereoCalibration::updateQ(bool for_undistorted) {
  left.intrinsic.values.copyTo(left_mono.intrinsic.values);
  right.intrinsic.values.copyTo(right_mono.intrinsic.values);

  left.distortion.coefficients.copyTo(left_mono.distortion.coefficients);
  right.distortion.coefficients.copyTo(right_mono.distortion.coefficients);

  
  cv::stereoRectify(left.intrinsic.values, left.distortion.coefficients, 
                    right.intrinsic.values, right.distortion.coefficients, 
                    left.size, 
                    right.extrinsic.rotation, right.extrinsic.position, 
                    left_R, right_R, 
                    left_P, right_P, 
                    Q,
                    CV_CALIB_ZERO_DISPARITY,
                    0.0);
  
   cv::initUndistortRectifyMap(left.intrinsic.values, left.distortion.coefficients,
							   left_R, left_P, left.size, CV_32FC1,
                               left_map1, left_map2);
   cv::initUndistortRectifyMap(right.intrinsic.values, right.distortion.coefficients,
                              right_R, right_P, left.size, CV_32FC1,
                               right_map1, right_map2);
   invert(Q, Qinv, cv::DECOMP_SVD);
}

void StereoCalibration::someToPointCloud(cv::Mat& points, cv::Mat& result) {
  if(Q.rows != 4 || Q.cols != 4) {
    std::cerr << "StereoCalibration::someToPointCloud: Reprojection matrix"
      " dimensions are incorrect. Did you call updateQ?" << std::endl;
  }
  
  cv::Mat world_points_homo= Q * points;
  result.create(cv::Size(3, points.cols), CV_64FC1);
  for (int i=0; i < world_points_homo.cols; i++) {
    double x= world_points_homo.at<double>(0, i);
    double y= world_points_homo.at<double>(1, i);
    double z= world_points_homo.at<double>(2, i);
    double w= world_points_homo.at<double>(3, i);
    x/= w;
    y/= w;
    z/= w;
    result.at<double>(i, 0)= x;
    result.at<double>(i, 1)= y;
    result.at<double>(i, 2)= z;
  }
}

/*Converts 3D point coordinates w.r.t. camera coordinate system to
  image coordinates */
void StereoCalibration:: toImage(cv::Mat& points, cv::Mat& result) {
  cv::Mat tmp= Qinv * points;
  result.create(cv::Size(2, points.cols), CV_16UC1);
  for(int i= 0; i < tmp.rows; i++) {
    result.at<unsigned short>(i, 0)= round(tmp.at<double>(i, 0) / tmp.at<double>(i, 3));
    result.at<unsigned short>(i, 1)=  round(tmp.at<double>(i, 1) / tmp.at<double>(i, 3));    
  }
}

void StereoCalibration::toPointCloud(cv::Mat& disparity_float, cv::Mat& result) {
  cv::reprojectImageTo3D(disparity_float, result, Q);
}

cv::Mat& StereoCalibration::getQ()  {
  return Q;
}

void CameraParameters::load(TiXmlNode* node) {
  size= cv::Size(752, 480);
  TiXmlElement* abstractSensor_root= node->FirstChildElement("abstractSensor");
  if(!abstractSensor_root) {
    std::cout << "Could not find the abstractSensor root" << std::endl;        
  }
  extrinsic.load(abstractSensor_root);
  intrinsic.load(node);
  distortion.load(node);  
}

void ExtrinsicParameters::load(TiXmlNode* node) {
  TiXmlElement* position_root= node->FirstChildElement("position");
  if(!position_root) {
    std::cout << "Could not find the position root" << std::endl;        
  }

  parse_vector(position_root, position);
  TiXmlElement* rotation_root= node->FirstChildElement("rotation");
  if(!rotation_root) {
    std::cout << "Could not find the rotation root" << std::endl;        
  }
  
  parse_vector(rotation_root, rotation_quaternion);
  Eigen::Quaternion<double> eigen_quaternion(rotation_quaternion.at<double>(0,3), 
                                            rotation_quaternion.at<double>(0,0),
                                            rotation_quaternion.at<double>(0,1),
                                            rotation_quaternion.at<double>(0,2));
  Eigen::Matrix3d eigen_rotation_matrix= eigen_quaternion.matrix();
  cv::eigen2cv(eigen_rotation_matrix, rotation);
}

void ExtrinsicParameters::set_rotation_matrix(const cv::Mat& m) {
  m.copyTo(rotation);  
  Eigen::Matrix3d eigen_mat;
  cv2eigen(m, eigen_mat);
  Eigen::Quaternion<double> quaternion(eigen_mat);
  rotation_quaternion.create(cv::Size(1, 4), CV_64FC1);
  rotation_quaternion.at<double>(0, 0)= quaternion.x();
  rotation_quaternion.at<double>(0, 1)= quaternion.y();
  rotation_quaternion.at<double>(0, 2)= quaternion.z();
  rotation_quaternion.at<double>(0, 3)= quaternion.w();
}


void IntrinsicParameters::load(TiXmlNode* node) {
  TiXmlElement* intrinsic_root= node->FirstChildElement("intrinsic");
  if(!intrinsic_root) {
    std::cout << "Could not find the intrinsic root" << std::endl;        
  }
  parse_matrix(intrinsic_root, values);    
}


void DistortionCoefficients::load(TiXmlNode* node) {
  TiXmlElement* distortion_root= node->FirstChildElement("distortion");
  if(!distortion_root) {
    std::cout << "Could not find the distortion root" << std::endl;        
  }
  parse_matrix(distortion_root, coefficients);  
}

void write_col_xml(std::ofstream& out, const cv::Mat& v) {
  out << "<data>\n";
  for (auto i=0; i < v.rows; i++) {
    out << "<item>" << (boost::format("%0.18f") % v.at<double>(i, 0)).str() << "</item>\n";
  }
  out << "</data>\n";
}

void write_row_xml(std::ofstream& out, const cv::Mat& v) {
  out << "<data>\n";
  for (auto i=0; i < v.cols; i++) {
    out << "<item>" << (boost::format("%0.18f") % v.at<double>(0, i)).str() << "</item>\n";
  }
  out << "</data>\n";
}

void write_matrix_xml(std::ofstream& out, const cv::Mat& m) {
   out << "<rows>" << m.rows << "</rows>\n" <<
     "<cols>" << m.cols << "</cols>\n" <<
     "<flags>1124024326</flags>\n";  
   for (int i= 0; i < m.rows; i++) {
     write_row_xml(out, m.row(i));
   }

}

void write_calibration_file(const std::string& path, const StereoCalibration& sc) {
  std::ofstream out;
  out.open(path);
  out << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\" ?>\n"
    "<!DOCTYPE boost_serialization>\n"
    "<boost_serialization signature=\"serialization::archive\" version=\"9\">\n"
    "<sensorTypes class_id=\"0\" tracking_level=\"0\" version=\"0\">\n"
    "<count>1</count>\n"
    "<item_version>0</item_version>\n"
    "<item>CameraParameter</item>\n"
    "</sensorTypes>\n"
    "<sensorType class_id=\"1\" tracking_level=\"0\" version=\"1\">\n"
    "<px class_id=\"2\" class_name=\"CameraParameterList\" tracking_level=\"1\" version=\"0\" object_id=\"_0\">\n"
    "<sensors class_id=\"3\" tracking_level=\"0\" version=\"0\">\n"
    "<count>2</count>\n"
    "<item_version>0</item_version>\n"
    "<item class_id=\"4\" tracking_level=\"1\" version=\"0\" object_id=\"_1\">\n"
    "<abstractSensor class_id=\"5\" tracking_level=\"0\" version=\"4\">\n"
    "<position class_id=\"6\" tracking_level=\"0\" version=\"0\">\n";
  write_col_xml(out, sc.left.extrinsic.position.col(0));
  //  exit(0);
   out << "</position>\n"
     "<rotation class_id=\"7\" tracking_level=\"0\" version=\"0\">\n";    
   write_col_xml(out, sc.left.extrinsic.rotation_quaternion);
   out << "</rotation>\n"
     "<delay>0</delay>\n"
     "</abstractSensor>\n"
     "<unit>0</unit>\n"
     "<guid>1000</guid>\n"
     "<intrinsic class_id=\"8\" tracking_level=\"0\" version=\"0\">\n";
   write_matrix_xml(out, sc.left.intrinsic.values);
   out << "</intrinsic>\n" <<
     "<distortion>\n";
   cv::Mat tmp;
   //   sc.left.distortion.coefficients.copyTo(tmp);
   //   cv::transpose(sc.left.distortion.coefficients, tmp);
   write_matrix_xml(out, sc.left.distortion.coefficients);
   out << "</distortion>\n"
     "</item>\n";
   /* Right camera */
   out << "<item object_id=\"_2\">\n"
     "<abstractSensor>\n"
     "<position>\n";
   write_col_xml(out, sc.right.extrinsic.position.col(0));
   out << "</position>\n"
        "<rotation>\n";    
   write_col_xml(out, sc.right.extrinsic.rotation_quaternion);
   out << "</rotation>\n"
        "<delay>0</delay>\n";
   out << "</abstractSensor>\n"
     "<unit>1</unit>\n"
     "<guid>1000</guid>\n"
     "<intrinsic>\n";
   write_matrix_xml(out, sc.right.intrinsic.values);
   out << "</intrinsic>\n" <<
     "<distortion>\n";
   //   sc.right.distortion.coefficients.copyTo(tmp);
   //   cv::transpose(sc.right.distortion.coefficients, tmp);
   write_matrix_xml(out, sc.right.distortion.coefficients);
   out << "</distortion>\n";
   out << "</item>\n";
   out << "</sensors>\n"
     "</px>\n"
     "</sensorType>\n"
     "</boost_serialization>";
  out.close();
  std::string cmd= (boost::format("xmllint --pretty 1 --output \"%s\" \"%s\"") % path % path).str();
  std::cout << cmd << std::endl;
  auto ret= system(cmd.c_str());
}


// void StereoCalibration::serialize(cv::FileStorage& storage, const Header& header) const {
//   storage << "{";
//   storage << "header";
//   header.serialize(storage);
//   //  storage << "Q" << Q;
//   storage << "left" << "{";
//   left.serialize(storage);
//   storage << "}";
//   storage << "right" << "{";
//   right.serialize(storage);
//   storage << "}";
//   storage << "}";
// }
// 
// void StereoCalibration::deserialize(const cv::FileNode& node) {
//   header.deserialize(node["header"]);
//   left.deserialize(node["left"]);
//   right.deserialize(node["right"]);
//   updateQ(true);
// }


// void CameraParameters::serialize(cv::FileStorage& storage, const Header& header) const {
//   storage << "header";
//   header.serialize(storage);
//   storage << "size " << size;
//   storage << "intrinsic" << intrinsic.values;
//   storage << "distortion" << distortion.coefficients;
//   storage << "extrinsic" << "{";
//   storage << "position" << extrinsic.position;
//   storage << "rotation_matrix" << extrinsic.rotation;
//   storage << "rotation_quaternion" << extrinsic.rotation_quaternion;
//   storage << "}";
// }
// 
// void CameraParameters::deserialize(const cv::FileNode& node) {  
//   node["size"] >> size;
//   node["intrinsic"] >> intrinsic.values;
//   node["distortion"] >> distortion.coefficients;
//   node["extrinsic"]["position"] >> extrinsic.position;
//   node["extrinsic"]["rotation_matrix"] >> extrinsic.rotation;
//   node["extrinsic"]["rotation_quaternion"] >> extrinsic.rotation_quaternion;
// }
float  StereoCalibration::get_baseline() const {
  return std::abs((right.extrinsic.position.at<double>(0,0)));
}
