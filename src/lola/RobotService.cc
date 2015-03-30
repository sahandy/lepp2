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

VisionMessage VisionMessage::DeleteMessage(int model_id) {
  std::cout << "Constructing with object_id = " << model_id << std::endl;
  VisionMessage msg;
  msg.id = REMOVE_SSV;
  msg.len = sizeof params;
  memset(msg.params, 0, sizeof msg.params);
  // 0 - unused, no point setting the type
  msg.params[1] = model_id;
  // 2 - unused -- since the entire model is getting removed...
  // 3 - unused, no point setting the radius
  msg.params[4] = VisionMessage::DEL_WHOLE_SEGMENT_FLAG;
  // The rest of the parameters are also irrelevant.
  std::cout << "Constructed " << msg << std::endl;
  return msg;
}

VisionMessage VisionMessage::DeletePartMessage(int model_id, int part_id) {
  std::cout << "Constructing with object_id = " << part_id << std::endl;
  VisionMessage msg;
  msg.id = REMOVE_SSV;
  msg.len = sizeof params;
  memset(msg.params, 0, sizeof msg.params);
  // 0 - unused, no point setting the type
  msg.params[1] = model_id;
  msg.params[2] = part_id;
  // 3 - unused, no point setting the radius
  msg.params[4] = VisionMessage::DEL_ONLY_PART_FLAG;
  // The rest of the parameters are also irrelevant.
  std::cout << "Constructed " << msg << std::endl;
  return msg;
}

VisionMessage VisionMessage::SetMessage(
    int type_id, int model_id, int part_id, double radius, std::vector<double> const& coefs) {
  VisionMessage msg;
  msg.id = SET_SSV;
  memset(msg.params, 0, sizeof msg.params);
  msg.params[0] = type_id;
  msg.params[1] = model_id;
  msg.params[2] = part_id;
  msg.params[3] = radius;
  // 4 - unused
  // 5 - unused
  for (size_t i = 0; i < coefs.size(); ++i) msg.params[6 + i] = coefs[i];

  return msg;
}

VisionMessage VisionMessage::ModifyMessage(
    int type_id, int model_id, int part_id, double radius, std::vector<double> const& coefs) {
  VisionMessage msg;
  msg.id = MODIFY_SSV;
  memset(msg.params, 0, sizeof msg.params);
  msg.params[0] = type_id;
  msg.params[1] = model_id;
  msg.params[2] = part_id;
  msg.params[3] = radius;
  // 4 - unused
  // 5 - unused
  for (size_t i = 0; i < coefs.size(); ++i) msg.params[6 + i] = coefs[i];

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

void RobotService::inner_send(VisionMessage const& next_message) {
  char const* buf = (char const*)&next_message;
  std::cerr << "RobotService: Sending a queued message: "
            << "msg == " << next_message
            << std::endl;
  // Synchronously send the message, i.e. block until the send is complete.
  socket_.send(
      boost::asio::buffer(buf, sizeof(VisionMessage)));
  // After each sent message, we want to wait a pre-defined amount of time
  // before sending the next one.
  // This is because we do not want to overwhelm the robot with a large
  // number of messages all sent in the same time.
  boost::this_thread::sleep(message_timeout_);
}

void RobotService::sendMessage(VisionMessage const& msg) {
  // Just queue another message to be sent by the io_service thread.
  // Since `inner_send` makes sure to wait after it's finished sending
  // each message (i.e. the io_service thread is blocked), the queued
  // messages will all be sent with a preset time delay between subsequent
  // messages.
  io_service_.post(boost::bind(&RobotService::inner_send, this, msg));
}
