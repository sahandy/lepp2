#ifndef LEPP2_LOLA_LOLA_AGGREGATOR_H__
#define LEPP2_LOLA_LOLA_AGGREGATOR_H__

#include "lepp2/ObstacleAggregator.hpp"
#include "lepp2/DiffAggregator.hpp"
#include "lola/RobotService.h"

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

namespace {
/**
 * A `ModelVisitor` implementation used for the implementation of the
 * `LolaAggregator`. Allows us to obtain all information that is required to
 * assemble a message for the visualizer.
 */
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
}  // namespace <anonymous>

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

namespace {
/**
 * A `ModelVisitor` implementation used for the implementation of the
 * `RobotAggregator`. Allows us to obtain all information that is required to
 * assemble a message for the robot.
 *
 * Similar to the `ParametersVisitor`, but for the sake of convenience of
 * the two aggregators' implementations, they are not reconciled.
 */
class CoefsVisitor : public lepp::ModelVisitor {
public:
  void visitSphere(SphereModel& sphere) {
    coefs_.push_back(sphere.center().x);
    coefs_.push_back(sphere.center().y);
    coefs_.push_back(sphere.center().z);
    for (size_t i = 0; i < 6; ++i) coefs_.push_back(0);

    type_id_ = 0;
    radius_ = sphere.radius();
  }

  void visitCapsule(CapsuleModel& capsule) {
    coefs_.push_back(capsule.first().x);
    coefs_.push_back(capsule.first().y);
    coefs_.push_back(capsule.first().z);
    coefs_.push_back(capsule.second().x);
    coefs_.push_back(capsule.second().y);
    coefs_.push_back(capsule.second().z);
    for (size_t i = 0; i < 3; ++i) coefs_.push_back(0);

    type_id_ = 1;
    radius_ = capsule.radius();
  }

  std::vector<double> const& coefs() const { return coefs_; }
  double radius() const { return radius_; }
  int type_id() const { return type_id_; }
private:
  std::vector<double> coefs_;
  int type_id_;
  double radius_;
};
}  // namespace <anonymous>

/**
 * An `ObstacleAggregator` implementation that sends notifications to the robot
 * after every certain amount of frames, informing it of changes in the known
 * obstacles since the previous message.
 *
 * The implementation relies on a `DiffAggregator` to find this diff and uses
 * this to send appropriate messages to the robot. The most notable adjustment
 * that needs to be made is "flattening" a composite model (one made of several
 * primitive models) into its most primitive components and sending msesages for
 * each of those separately to the robot.
 */
class RobotAggregator : public lepp::ObstacleAggregator {
public:
  RobotAggregator(RobotService& service, int freq)
      : service_(service), diff_(freq), next_id_(0) {
    // Set up the callbacks that handle the particular cases.
    diff_.set_new_callback(boost::bind(&RobotAggregator::new_cb_, this, _1));
    diff_.set_modified_callback(boost::bind(&RobotAggregator::mod_cb_, this, _1));
    diff_.set_deleted_callback(boost::bind(&RobotAggregator::del_cb_, this, _1));
  }
  /**
   * `ObstacleAggregator` interface implementation.
   */
  void updateObstacles(std::vector<ObjectModelPtr> const& obstacles) {
    // Just pass it on to find the diff!
    diff_.updateObstacles(obstacles);
  }
private:
  /**
   * The function is passed as a callback to the underlying `DiffAggregator` for
   * when new models are discovered.
   */
  void new_cb_(ObjectModel& model) {
    // Assign an ID to each part of the new model.
    std::vector<ObjectModel*> primitives(getPrimitives(model));
    size_t const sz = primitives.size();
    std::vector<int>& ids = robot_ids_[model.id()];
    for (size_t i = 0; i < sz; ++i) {
      // Assign it a new ID
      int const id = nextId();
      ids.push_back(id);
      // ...and send a message to the robot.
      sendNew(*primitives[i], id);
    }
  }

  /**
   * The function is passed as a callback to the underlying `DiffAggregator` for
   * when models are discovered to be deleted.
   */
  void del_cb_(int obj_id) {
    // Delete all parts of the deleted object.
    {
      std::vector<int>& ids = robot_ids_[obj_id];
      for (size_t i = 0; i < ids.size(); ++i) {
        sendDelete(ids[i]);
      }
    }
    // Remove it from the map too.
    robot_ids_.erase(obj_id);
  }

  /**
   * The function is passed as a callback to the underlying `DiffAggregator` for
   * when a model has been modified.
   */
  void mod_cb_(ObjectModel& model) {
    std::vector<ObjectModel*> primitives(getPrimitives(model));
    size_t const new_size = primitives.size();
    std::vector<int>& ids = robot_ids_[model.id()];
    size_t const old_size = ids.size();

    if (new_size < old_size) {
      // Some parts need to be deleted.
      for (size_t i = 0; i < old_size - new_size; ++i) {
        sendDelete(ids[old_size - i - 1]);
        ids.pop_back();
      }
    }

    // Now we modify what we have left from before
    size_t const sz = ids.size();
    for (size_t i = 0; i < sz; ++i) {
      sendModify(*primitives[i], ids[i]);
    }

    // And finally add new ones, if necessary
    if (new_size > old_size) {
      for (size_t i = 0; i < new_size - old_size; ++i) {
        int const id = nextId();
        ids.push_back(id);
        sendNew(*primitives[old_size + i], id);
      }
    }
  }

  /**
   * Obtains a list of pointers to the primitives that the given model is
   * composed of.
   * This list is safe to use only while the model reference is in scope.
   */
  std::vector<ObjectModel*> getPrimitives(ObjectModel& model) const {
    FlattenVisitor flattener;
    model.accept(flattener);
    return flattener.objs();
  }

  /**
   * Sends a message to the robot informing it of a new model.
   */
  void sendNew(ObjectModel& new_model, int id) {
    CoefsVisitor coefs;
    new_model.accept(coefs);
    VisionMessage msg = VisionMessage::SetMessage(
        coefs.type_id(), id, coefs.radius(), coefs.coefs());
    std::cerr << "RobotAggregator: Creating new primitive ["
              << "type = " << coefs.type_id()
              << "; id = " << id << std::endl;
    service_.sendMessage(msg);
  }

  /**
   * Sends a message to the robot informing it of a deleted model.
   */
  void sendDelete(int id) {
    std::cerr << "RobotAggregator: Deleting a primitive id = "
              << id << std::endl;
    VisionMessage del = VisionMessage::DeleteMessage(id);
    service_.sendMessage(del);
  }

  /**
   * Sends a message to the robot informing it of a modified model.
   */
  void sendModify(ObjectModel& model, int id) {
    CoefsVisitor coefs;
    model.accept(coefs);
    VisionMessage msg = VisionMessage::ModifyMessage(
        coefs.type_id(), id, coefs.radius(), coefs.coefs());
    std::cerr << "RobotAggregator: Modifying existing primitive ["
              << "type = " << coefs.type_id()
              << "; id = " << id << std::endl;
    service_.sendMessage(msg);
  }

  /**
   * Obtains the next ID that should be used for a primitive that the robot is
   * notified of.
   */
  int nextId() { return next_id_++; }

  /**
   * A handle to the service that is used to send notifications to the robot.
   */
  RobotService& service_;
  /**
   * An instance of a `DiffAggregator` that this one delegates to in order to
   * detect the differences between the checkpoint frames.
   */
  DiffAggregator diff_;

  /**
   * Maps the approximation ID to a list of IDs that the robot will know for
   * each primitive SSV object that is found in the model composition.
   */
  std::map<int, std::vector<int> > robot_ids_;
  /**
   * The ID that can be assigned to the next new model (or rather model part).
   */
  int next_id_;
};

#endif
