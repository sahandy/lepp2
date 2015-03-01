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
  socket_.async_send(
      boost::asio::buffer(buf, sizeof(VisionMessage)),
      write_handler);
}
