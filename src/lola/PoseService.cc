#include "lola/PoseService.h"
#include "boost/thread.hpp"

#include <cstring>
#include <iostream>

void PoseService::read_handler(
    boost::system::error_code const& ec,
    std::size_t bytes_transferred) {
  std::cerr << "Pose Service: Received " << bytes_transferred << std::endl;
  if (bytes_transferred != sizeof(HR_Pose)) {
    std::cerr << "Pose Service: Error: Invalid datagram size."
              << "Expected " << sizeof(HR_Pose) << std::endl;
    return;
  }
  boost::shared_ptr<HR_Pose> new_pose(new HR_Pose);
  // The copy is thread safe since nothing can be writing to the recv_buffer
  // at this point. No new async read is queued until this callback is complete.
  memcpy(&*new_pose, &recv_buffer_[0], sizeof(HR_Pose));
  // This performs an atomic update of the pointer, making it a lock-free,
  // thread-safe operation.
  pose_ = new_pose;
  std::cerr << "Pose Service: Updated current pose" << std::endl;
  queue_recv();
}

void PoseService::service_thread() {
  std::cerr << "Pose Service: Thread started" << std::endl;
  io_service_.run();
}

void PoseService::queue_recv() {
  socket_.async_receive(
      boost::asio::buffer(recv_buffer_),
      boost::bind(&PoseService::read_handler, this, _1, _2));
}

void PoseService::bind() {
  boost::system::error_code error;
  socket_.open(boost::asio::ip::udp::v4(), error);
  boost::asio::ip::udp::endpoint local(
      boost::asio::ip::address::from_string(host_),
      port_);
  socket_.bind(local);
}

void PoseService::start() {
  bind();
  queue_recv();
  // Start the thread that will run the associated I/O service.
  boost::thread(boost::bind(&PoseService::service_thread, this));
}


HR_Pose PoseService::getCurrentPose() const {
  // This does an atomic copy of the pointer (the refcount is atomically updated)
  // There can be no race condition since if the service needs to update the
  // pointer, it will do so atomically and the reader also obtains a copy of the
  // pointer atomically.
  boost::shared_ptr<HR_Pose> p = pose_;
  // Now we are safe to manipulate the object itself, since nothing else needs
  // to directly touch the instance itself and we have safely obtained a
  // reference to it.
  // We just return the object (but this involves a non-atomic copy, hence the
  // pointer dance).

  if (p) {
    return *p;
  } else {
    HR_Pose pose = {0};
    return pose;
  }
}
