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

using namespace lepp;

class ParametersVisitor : public lepp::ModelVisitor {
public:
  void visitSphere(SphereModel& sphere) {
    params_.push_back(0); // type
    params_.push_back(sphere.radius());
    Coordinate const center = sphere.center();
    params_.push_back(center.x);
    params_.push_back(center.y);
    params_.push_back(center.z);
    // Now the rest is padding
    for (size_t i = 0; i < 6; ++i) params_.push_back(0);
  }

  void visitCapsule(CapsuleModel& capsule) {
    params_.push_back(1); // type
    params_.push_back(capsule.radius());
    Coordinate const first = capsule.first();
    params_.push_back(first.x);
    params_.push_back(first.y);
    params_.push_back(first.z);
    Coordinate const second = capsule.second();
    params_.push_back(second.x);
    params_.push_back(second.y);
    params_.push_back(second.z);
    // The rest is padding
    for (size_t i = 0; i < 3; ++i) params_.push_back(0);
  }

  std::vector<double> params() const { return params_; }
private:
  std::vector<double> params_;
};

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
  void updateObstacles(std::vector<ObjectModelPtr> const& obstacles);
private:
  /**
   * A helper function that builds the datagram payload based on the given
   * obstacles.
   */
  std::vector<char> buildPayload(std::vector<ObjectModelPtr> const& obstacles) const;

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

void LolaAggregator::updateObstacles(std::vector<ObjectModelPtr> const& obstacles) {
  std::cerr << "Sending to " << remote_endpoint_ << std::endl;

  // Builds the payload: a raw byte buffer.
  std::vector<char> payload(buildPayload(obstacles));
  // Once the payload is built, initiate an async send.
  // We don't really care about the result, since there's no point in retrying.
  socket_.async_send_to(
      boost::asio::buffer(payload, payload.size()),
      remote_endpoint_,
      0,
      handler);
}

std::vector<char> LolaAggregator::buildPayload(
    std::vector<ObjectModelPtr> const& obstacles) const {
  std::vector<char> payload;

  size_t const sz = obstacles.size();
  for (int i = 0; i < sz; ++i) {
    ObjectModel& model = *obstacles[i];
    // Get the "flattened" model representation.
    ParametersVisitor parameterizer;
    model.accept(parameterizer);
    std::vector<double> params(parameterizer.params());

    // Since the model could have been a composite, we may have more than 1
    // model's representation in the vector, one after the other.
    for (size_t model_idx = 0; model_idx < params.size() / 11; ++model_idx) {
      // Pack each set of coefficients into a struct that should be shipped off
      // to the viewer.
      struct {
        int type;
        int radius;
        int rest[9];
      } obstacle;
      memset(&obstacle, 0, sizeof(obstacle));
      obstacle.type = params[11*model_idx + 0];
      // LOLA expects the values to be in milimeters.
      obstacle.radius = params[11*model_idx + 1] * 1000;
      for (size_t i = 0; i < 9; ++i) {
        obstacle.rest[i] = params[11*model_idx + 2 + i] * 1000;
      }

      // Now dump the raw bytes extracted from the struct into the payload.
      char* raw = (char*)&obstacle;
      for (size_t i = 0; i < sizeof(obstacle); ++i) {
        payload.push_back(raw[i]);
      }
    }
  }

  return payload;
}

#endif
