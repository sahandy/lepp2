#include "RobotService.h"
#include <boost/thread.hpp>

namespace {
  /**
   * Callback invoked when the async connect operation completes.
   */
  void connect_handler(boost::system::error_code const& error) {
    if (!error) {
      std::cerr << "RobotService: Connected to the robot." << std::endl;
    } else {
      std::cerr << "RobotService: Failed to connect to the robot" << std::endl;
    }
  }

  /**
   * A simple function that is used to spin up the io service event loop in a
   * dedicated thread.
   * Makes sure that the loop does not end when the service does not have any
   * work to do (temporarily).
   */
  void service_thread(boost::asio::io_service* io_service) {
    // Prevent the IO service from running out of work (and exitting when no
    // async operations are queued).
    boost::asio::io_service::work work(*io_service);
    io_service->run();
    std::cout << "RobotService: Exiting service thread..." << std::endl;
  }

  /**
   * A callback for write operations.
   */
  void write_handler(boost::system::error_code const& error,
               std::size_t sent) {
    if (!error) {
      std::cerr << "RobotService: Send complete. "
                << "Sent " << sent << " bytes." << std::endl;
    } else {
      std::cerr << "RobotService: Error sending message." << std::endl;
    }
  }
}

VisionMessage VisionMessage::DeleteMessage(int object_id) {
  std::cout << "Constructing with object_id = " << object_id << std::endl;
  VisionMessage msg;
  msg.id = REMOVE_SSV;
  msg.len = sizeof params;
  memset(msg.params, 0, sizeof msg.params);
  msg.params[1] = object_id;

  std::cout << "Constructed " << msg << std::endl;
  return msg;
}

VisionMessage VisionMessage::SetMessage(
    int type_id, int model_id, double radius, std::vector<double> const& coefs) {
  VisionMessage msg;
  msg.id = SET_SSV;
  memset(msg.params, 0, sizeof msg.params);
  msg.params[0] = type_id;
  msg.params[1] = model_id;
  // 2 - unsused
  msg.params[3] = radius;
  // 4 - unused
  // 5 - unused
  for (size_t i = 0; i < coefs.size(); ++i) msg.params[4 + i] = coefs[i];

  return msg;
}

VisionMessage VisionMessage::ModifyMessage(
    int type_id, int model_id, double radius, std::vector<double> const& coefs) {
  VisionMessage msg;
  msg.id = MODIFY_SSV;
  memset(msg.params, 0, sizeof msg.params);
  msg.params[0] = type_id;
  msg.params[1] = model_id;
  // 2 - unsused
  msg.params[3] = radius;
  // 4 - unused
  // 5 - unused
  for (size_t i = 0; i < coefs.size(); ++i) msg.params[4 + i] = coefs[i];

  return msg;
}

std::ostream& operator<<(std::ostream& out, VisionMessage const& msg) {
  out << "[" << msg.id << " | " << msg.len << " | ";
  for (size_t i = 0; i < 15; ++i) out << msg.params[i] << ", ";
  out << "]";

  return out;
}

void RobotService::start() {
  // Start it up...
  boost::asio::ip::tcp::endpoint endpoint(
    boost::asio::ip::address::from_string(remote_), port_);
  std::cerr << "RobotService: Initiating a connection asynchronously..."
            << std::endl;
  socket_.async_connect(endpoint, connect_handler);
  // Start the service thread in the background...
  boost::thread(boost::bind(service_thread, &io_service_));
}

void RobotService::sendMessage(VisionMessage const& msg) {
  char const* buf = (char const*)&msg;
  std::cerr << "RobotService: Queuing a send of " << (sizeof(VisionMessage))
            << " bytes." << std::endl;
  std::cerr << "RobotService: msg = " << msg << std::endl;
  socket_.async_send(
      boost::asio::buffer(buf, sizeof(VisionMessage)),
      write_handler);
}
