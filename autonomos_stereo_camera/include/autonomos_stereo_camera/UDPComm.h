/* Freie Universität Berlin
 * \file ControllerComm.h
 * \author José Antonio Álvarez Ruiz
 * \date January 2014
 *
 * \brief Provides functionality to write UDP based clients
 */


#ifndef UDPCOMM_H
#define UDPCOMM_H

#include <boost/asio.hpp>
#include <boost/thread.hpp>

class UDPClient {
 public:
  UDPClient(std::string host, unsigned short port);
  void send(std::string message);
  std::string receive();

 protected:
  boost::asio::io_service io_service;
  boost::asio::ip::udp::socket socket;
  boost::asio::ip::udp::endpoint endpoint;  
};

class UDPServer {
 public:
  UDPServer(unsigned short port, unsigned int buffer_size);
  void start_receive();
  virtual void handle_receive()= 0;
  static boost::thread_group threads;
  boost::asio::io_service io_service;
 protected:
  unsigned short port;
  boost::asio::ip::udp::endpoint remote_endpoint;
  std::shared_ptr<boost::asio::ip::udp::endpoint> endpoint;

  std::shared_ptr<boost::asio::ip::udp::socket> socket;
  //  std::vector<char> recv_buffer;
  std::shared_ptr<boost::thread> thread_ptr;

};

#endif
