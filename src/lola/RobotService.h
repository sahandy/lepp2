#ifndef LOLA_ROBOT_SERVICE_H__
#define LOLA_ROBOT_SERVICE_H__

#include <boost/asio.hpp>

/**
 * A struct representing the raw vision message format that is sent to the
 * robot.
 */
struct VisionMessage {
  uint32_t id;
  uint32_t len;
  float params[15];

  // IDs for particular vision operations.
  static uint32_t const SET_SSV = 0x203;
  static uint32_t const MODIFY_SSV = 0x206;
  static uint32_t const REMOVE_SSV = 0x207;
};

/**
 * A class that implements a service which can send vision-related notifications
 * to the robot.
 *
 * It allows clients to asychronously send vision messages to the robot.
 */
class RobotService {
public:
  /**
   * Creates a new `RobotService` instance that will try to send messages to a
   * robot on the given remote address (host name, port combination).
   */
  RobotService(std::string const& remote, int port)
      : remote_(remote), port_(port), socket_(io_service_) {}
  /**
   * Starts up the service, initiating a connection to the robot.
   *
   * Behind the scenes, this spins up a thread that will handle the IO for this
   * service.
   */
  void start();
  /**
   * Asynchronously sends a message to the robot.
   *
   * The call never blocks.
   */
  void sendMessage(VisionMessage const& msg);
private:
  /**
   * The host name of the robot.
   */
  std::string const remote_;
  /**
   * The port on which the robot is expecting vision messages.
   */
  int const port_;
  /**
   * The io service that handles the async operations of the communication.
   */
  boost::asio::io_service io_service_;
  /**
   * The socket that is connected to the remote robot endpoint.
   */
  boost::asio::ip::tcp::socket socket_;
};

#endif
