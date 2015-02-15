#ifndef LEPP2_LOLA_LOLA_AGGREGATOR_H__
#define LEPP2_LOLA_LOLA_AGGREGATOR_H__

#include "lepp2/ObstacleAggregator.hpp"

#include <boost/array.hpp>
#include <boost/asio.hpp>

namespace {
  /**
   * A dummy WriteHandler for the asnchronous socket write.
   *
   * If send were to fail, there's no point in us retrying, so this handler is
   * just a dummy.
   */
  void handler(
      boost::system::error_code const& error,
      std::size_t bytes_transferred) {}
}

/**
 * A LOLA-specific implementation of an `ObstacleAggregator`.
 *
 * It serializes the received obstacles into a format where each obstacle is
 * represented by 11 integers. Each integer is serialized with machine-specific
 * endianess; no care is taken to perform integer serialization for network
 * transfer, i.e. big-endian. In general, this is not the safest assumption to
 * make, but the LOLA controller has been working under this assumption for now,
 * so enforcing a big-endian encoding would just introduce additional complexity.
 *
 * The serialized representation of each obstacle is packed into a single
 * datagram and sent over UDP to the remote host described by the initial
 * constructor parameters.
 */
class LolaAggregator : public lepp::ObstacleAggregator {
public:
  /**
   * Creates a new `LolaAggregator` where the remote host to which the obstacle
   * list is sent is identified by the given host name and port number.
   */
  LolaAggregator(std::string const& remote_host, int remote_port);
  ~LolaAggregator();
  /**
   * `ObstacleAggregator` interface implementation.
   */
  void updateObstacles(ObjectModelPtrListPtr obstacles);
private:
  /**
   * A helper function that builds the datagram payload based on the given
   * obstacles.
   */
  std::vector<char> buildPayload(ObjectModelPtrList& obstacles) const;

  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint remote_endpoint_;
};

LolaAggregator::LolaAggregator(std::string const& remote_host, int remote_port)
    : socket_(io_service_),
      remote_endpoint_(
        boost::asio::ip::address::from_string(remote_host.c_str()),
        remote_port) {
  socket_.open(boost::asio::ip::udp::v4());
}

LolaAggregator::~LolaAggregator() {
  // RAII
  socket_.close();
}

void LolaAggregator::updateObstacles(ObjectModelPtrListPtr obstacles) {
  std::cerr << "Sending to " << remote_endpoint_ << std::endl;

  // Builds the payload: a raw byte buffer.
  std::vector<char> payload(buildPayload(*obstacles));
  // Once the payload is built, initiate an async send.
  // We don't really care about the result, since there's no point in retrying.
  socket_.async_send_to(
      boost::asio::buffer(payload, payload.size()),
      remote_endpoint_,
      0,
      handler);
}

std::vector<char> LolaAggregator::buildPayload(
    ObjectModelPtrList& obstacles) const {
  size_t sz = obstacles.size();
  // 11 integers (each int is 4 bytes) per obstacle
  std::vector<char> payload(11 * 4 * sz);
  size_t idx = 0;
  for (int i = 0; i < sz; ++i) {
    ObjectModel& model = *obstacles[i];
    // Extracts the model representation into a convenience struct.
    struct {
      int type;
      int radius;
      int rest[9];
    } obstacle;
    memset(&obstacle, 0, sizeof(obstacle));
    obstacle.type = model.getType_nr();
    // LOLA expects the values to be in milimeters.
    obstacle.radius = model.getRadius() * 1000;
    obstacle.rest[3*0 + 0] = model.getP1_notrans().x * 1000;
    obstacle.rest[3*0 + 1] = model.getP1_notrans().y * 1000;
    obstacle.rest[3*0 + 2] = model.getP1_notrans().z * 1000;
    obstacle.rest[3*1 + 0] = model.getP2_notrans().x * 1000;
    obstacle.rest[3*1 + 1] = model.getP2_notrans().y * 1000;
    obstacle.rest[3*1 + 2] = model.getP2_notrans().z * 1000;
    obstacle.rest[3*2 + 0] = model.getP3_notrans().x * 1000;
    obstacle.rest[3*2 + 1] = model.getP3_notrans().y * 1000;
    obstacle.rest[3*2 + 2] = model.getP3_notrans().z * 1000;
    // Now dump the raw bytes extracted from the struct into the payload.
    char* raw = (char*)&obstacle;
    for (int i = 0; i < sizeof(obstacle); ++i) {
      payload[idx++] = raw[i];
    }
  }

  return payload;
}

#endif
