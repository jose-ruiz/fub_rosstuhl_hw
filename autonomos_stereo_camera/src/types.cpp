#include <autonomos_stereo_camera/types.h>

Header::Header(struct timeval timestamp, std::string frame_name) {
  this->frame_name= frame_name;
}

Header::Header() {
  update_timestamp();
  index= 0;
  this->frame_name= "";
}

void Header::update_timestamp() {
  time= std::chrono::high_resolution_clock::now();
}


const float IntrinsicParameters::get_cx() const noexcept {
  return (values.at<double>(0, 2));

}
const float IntrinsicParameters::get_cy() const noexcept {
  return (values.at<double>(1, 2));
}
const float IntrinsicParameters::get_fx() const noexcept {
  return (values.at<double>(0, 0));
  
}
const float IntrinsicParameters::get_fy() const noexcept {
  return (values.at<double>(1, 1));
}
