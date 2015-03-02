#include "lola/LolaAggregator.h"
using namespace lepp;

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

  for (auto& model : obstacles) {
    // Get the "flattened" model representation.
    ParametersVisitor parameterizer;
    model->accept(parameterizer);
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

RobotAggregator::RobotAggregator(RobotService& service, int freq)
    : service_(service), diff_(freq), next_id_(0) {
  // Set up the callbacks that handle the particular cases.
  diff_.set_new_callback(boost::bind(&RobotAggregator::new_cb_, this, _1));
  diff_.set_modified_callback(boost::bind(&RobotAggregator::mod_cb_, this, _1));
  diff_.set_deleted_callback(boost::bind(&RobotAggregator::del_cb_, this, _1));
}
void RobotAggregator::new_cb_(ObjectModel& model) {
  // Assign an ID to each part of the new model.
  std::vector<ObjectModel*> primitives(getPrimitives(model));
  std::vector<int>& ids = robot_ids_[model.id()];
  for (auto& primitive : primitives) {
    // Assign it a new ID
    int const id = nextId();
    ids.push_back(id);
    // ...and send a message to the robot.
    sendNew(*primitive, id);
  }
}

void RobotAggregator::del_cb_(int obj_id) {
  // Delete all parts of the deleted object.
  {
    std::vector<int>& ids = robot_ids_[obj_id];
    for (auto& del_id : ids) {
      sendDelete(del_id);
    }
  }
  // Remove it from the map too.
  robot_ids_.erase(obj_id);
}

void RobotAggregator::mod_cb_(ObjectModel& model) {
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

std::vector<ObjectModel*> RobotAggregator::getPrimitives(ObjectModel& model) const {
  FlattenVisitor flattener;
  model.accept(flattener);
  return flattener.objs();
}

void RobotAggregator::sendNew(ObjectModel& new_model, int id) {
  CoefsVisitor coefs;
  new_model.accept(coefs);
  VisionMessage msg = VisionMessage::SetMessage(
      coefs.type_id(), id, coefs.radius(), coefs.coefs());
  std::cerr << "RobotAggregator: Creating new primitive ["
            << "type = " << coefs.type_id()
            << "; id = " << id << std::endl;
  service_.sendMessage(msg);
}

void RobotAggregator::sendDelete(int id) {
  std::cerr << "RobotAggregator: Deleting a primitive id = "
            << id << std::endl;
  VisionMessage del = VisionMessage::DeleteMessage(id);
  service_.sendMessage(del);
}

void RobotAggregator::sendModify(ObjectModel& model, int id) {
  CoefsVisitor coefs;
  model.accept(coefs);
  VisionMessage msg = VisionMessage::ModifyMessage(
      coefs.type_id(), id, coefs.radius(), coefs.coefs());
  std::cerr << "RobotAggregator: Modifying existing primitive ["
            << "type = " << coefs.type_id()
            << "; id = " << id << std::endl;
  service_.sendMessage(msg);
}
