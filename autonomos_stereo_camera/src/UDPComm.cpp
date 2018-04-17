/**
 * @author Jose Alvarez-Ruiz <jose.alvarez-ruiz@fu-berlin.de>
 * @date March 2016
 */
   

#include <autonomos_stereo_camera/UDPComm.h>
#include <boost/thread/thread.hpp>


using boost::asio::ip::udp;

UDPClient::UDPClient(std::string host, unsigned short port) :
  
  endpoint(boost::asio::ip::address::from_string(host), port),
  socket(io_service) {
// TODO: Add hostname resolution
  socket.open(udp::v4());
}

void UDPClient::send(std::string message) {

  socket.send_to(boost::asio::buffer(message), endpoint);
}

std::string UDPClient::receive() {
  boost::system::error_code error;
  std::size_t bytes_received;
  while(!socket.available()) {
    sleep(0.01);
  }
  std::vector<char> recv_buf;
  recv_buf.resize(socket.available());
  bytes_received= socket.receive_from(boost::asio::buffer(recv_buf),
                                      endpoint, 0, error);
  std::string result(recv_buf.begin(), recv_buf.begin() + bytes_received);
  return result;
}


UDPServer::UDPServer(unsigned short port, unsigned int buffer_size) {
  //  std::cout << "Waiting for port " << port << std::endl;
  this->port= port;
  endpoint= std::make_shared<boost::asio::ip::udp::endpoint>(udp::v4(), port);
  socket= std::make_shared<boost::asio::ip::udp::socket>(UDPServer::io_service, *endpoint);  
  //  recv_buffer.resize(buffer_size);
}

void UDPServer::start_receive() {
//   std::cout << " start receive on port " << port << std::endl;
//   socket->async_receive_from(boost::asio::buffer(recv_buffer),
// 							 remote_endpoint,
// 							 boost::bind(&UDPServer::handle_receive,
// 										 this,
// 										 boost::asio::placeholders::error,
//  										 boost::asio::placeholders::bytes_transferred));
//   boost::this_thread::sleep(boost::posix_time::microseconds(2));
}
boost::thread_group UDPServer::threads;
//boost::asio::io_service UDPServer::io_service;
