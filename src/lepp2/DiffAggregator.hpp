#ifndef LEPP2_DIFF_AGGREGATOR_H__
#define LEPP2_DIFF_AGGREGATOR_H__

#include "lepp2/ObstacleAggregator.hpp"

#include <set>
#include <vector>
#include <algorithm>

#include <boost/bind.hpp>
#include <boost/function.hpp>

namespace lepp {

/**
 * An implementation of an `ObstacleAggregator` that finds diffs between
 * obstacles detected in subsequent snapshots.
 *
 * A snapshot is taken after a certain number of frames, provided in the
 * constructor.
 *
 * For each difference between the previous snapshot and the current one,
 * the appropriate callback is fired, if provided, so that the client can
 * take appropriate actions if a diff is detected.
 */
class DiffAggregator : public ObstacleAggregator {
public:
  // Callback typedefs
  typedef boost::function<void (ObjectModel&)> NewObstacleCallback;
  typedef boost::function<void (ObjectModel&)> ModifiedObstacleCallback;
  typedef boost::function<void (int)> DeletedObstacleCallback;

  /**
   * Create a new `DiffAggregator` that will output the diff between frames
   * after every `frequency` frames.
   */
  DiffAggregator(int frequency)
      : freq_(frequency), new_cb_(0), mod_cb_(0), del_cb_(0) {}

  /**
   * Sets a function that will be called for every new obstacle.
   */
  void set_new_callback(NewObstacleCallback new_cb) { new_cb_ = new_cb; }
  /**
   * Sets a function that will be called for every modified obstacle.
   */
  void set_modified_callback(ModifiedObstacleCallback mod_cb) { mod_cb_ = mod_cb; }
  /**
   * Sets a function that will be called for every deleted obstacle.
   */
  void set_deleted_callback(DeletedObstacleCallback del_cb) { del_cb_ = del_cb; }
  /**
   * Implementation of the `ObstacleAggregator` interface.
   */
  void updateObstacles(std::vector<ObjectModelPtr> const& obstacles);
private:
  /**
   * The number of frames after which the difference to the previous snapshot
   * should be found.
   */
  int freq_;
  /**
   * The current frame number.
   */
  int curr_;
  /**
   * A set of model IDs found in the previous snapshot.
   */
  std::set<int> previous_ids_;

  // Callbacks that are invoked in the appropriate event.
  NewObstacleCallback new_cb_;
  ModifiedObstacleCallback mod_cb_;
  DeletedObstacleCallback del_cb_;
};

// TODO This is made inline to facilitate keeping lepp2 header-only for now.
inline void DiffAggregator::updateObstacles(
    std::vector<ObjectModelPtr> const& obstacles) {
  ++curr_;
  if (curr_ % freq_ != 0) return;

  // All IDs in the given list are either new or a modified representation of an
  // obstacle found in the previous snapshot.
  std::set<int> current_ids;
  size_t const sz = obstacles.size();
  for (auto& obstacle : obstacles) {
    int const id = obstacle->id();
    current_ids.insert(id);
    if (previous_ids_.find(id) == previous_ids_.end()) {
      // This is a new obstacle.
      std::cout << "DiffAggregator: New obstacle id = " << id << " (" << *obstacle << ")" << std::endl;
      if (new_cb_) new_cb_(*obstacle);
    } else {
      // This is a modified obstacle.
      std::cout << "DiffAggregator: A modified obstacle id = " << id << " (" << *obstacle << ")" << std::endl;
      if (mod_cb_) mod_cb_(*obstacle);
    }
  }

  // The ids that are in the previous set, but not the current ones are deleted
  // obstacles
  std::vector<int> deleted;
  std::set_difference(previous_ids_.begin(), previous_ids_.end(),
                      current_ids.begin(), current_ids.end(),
                      std::back_inserter(deleted));
  for (auto& del_id : deleted) {
    std::cout << "DiffAggregator: A deleted obstacle id = " << del_id << std::endl;
    if (del_cb_) del_cb_(del_id);
  }

  // Finally, forget the previous set, in favor of the obstacles we know now!
  previous_ids_ = current_ids;
}

}  // namespace lepp
#endif
