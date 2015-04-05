#ifndef BASE_OBSTACLE_DETECTOR_H_
#define BASE_OBSTACLE_DETECTOR_H_

#include <vector>
#include <algorithm>

#include <pcl/visualization/cloud_viewer.h>

#include "lepp2/VideoObserver.hpp"
#include "lepp2/BaseSegmenter.hpp"
#include "lepp2/NoopSegmenter.hpp"
#include "lepp2/EuclideanPlaneSegmenter.hpp"
#include "lepp2/ObstacleAggregator.hpp"
#include "lepp2/ObjectApproximator.hpp"
#include "lepp2/MomentOfInertiaApproximator.hpp"
#include "lepp2/SplitApproximator.hpp"

using namespace lepp;

#include "lepp2/debug/timer.hpp"

template<class PointT>
class BaseObstacleDetector : public lepp::VideoObserver<PointT> {
public:
  BaseObstacleDetector();
  virtual ~BaseObstacleDetector() {}

  /**
   * VideoObserver interface method implementation.
   */
  virtual void notifyNewFrame(
      int idx,
      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud);

  /**
   * Attaches a new ObstacleAggregator, which will be notified of newly detected
   * obstacles by this detector.
   */
  void attachObstacleAggregator(
      boost::shared_ptr<ObstacleAggregator> aggregator);

protected:
  /// Some convenience typedefs
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

  /**
   * Notifies any observers about newly detected obstacles.
   */
  void notifyObstacles(std::vector<ObjectModelPtr> const& models);

private:
  typename pcl::PointCloud<PointT>::ConstPtr cloud_;

  /**
   * Tracks all attached ObstacleAggregators that wish to be notified of newly
   * detected obstacles.
   */
  std::vector<boost::shared_ptr<ObstacleAggregator> > aggregators;

  boost::shared_ptr<BaseSegmenter<PointT> > segmenter_;
  boost::shared_ptr<ObjectApproximator<PointT> > approximator_;

  /**
   * Performs a new update of the obstacle approximations.
   * Triggered when the detector is notified of a new frame (i.e. point cloud).
   */
  void update();
};

template<class PointT>
BaseObstacleDetector<PointT>::BaseObstacleDetector()
    : approximator_(new SplitObjectApproximator<PointT>(
        boost::shared_ptr<ObjectApproximator<PointT> >(
          new MomentOfInertiaObjectApproximator<PointT>))),
      segmenter_(new EuclideanPlaneSegmenter<PointT>()) {
  // TODO Allow for dependency injection.
}


template<class PointT>
void BaseObstacleDetector<PointT>::notifyNewFrame(
    int id,
    const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud) {
  cloud_ = point_cloud;
  try {
    update();
  } catch (...) {
    std::cerr << "ObstacleDetector: Obstacle detection failed ..." << std::endl;
  }
}


template<class PointT>
void BaseObstacleDetector<PointT>::update() {
  Timer t;
  t.start();
  std::vector<PointCloudConstPtr> segments(segmenter_->segment(cloud_));

  // Iteratively approximate the segments
  size_t segment_count = segments.size();
  std::vector<ObjectModelPtr> models;
  for (size_t i = 0; i < segment_count; ++i) {
    models.push_back(approximator_->approximate(segments[i]));
  }
  t.stop();
  std::cerr << "Obstacle detection took " << t.duration() << std::endl;

  notifyObstacles(models);
}

template<class PointT>
void BaseObstacleDetector<PointT>::attachObstacleAggregator(
    boost::shared_ptr<ObstacleAggregator> aggregator) {
  aggregators.push_back(aggregator);
}

template<class PointT>
void BaseObstacleDetector<PointT>::notifyObstacles(
  std::vector<ObjectModelPtr> const& models) {
  size_t sz = aggregators.size();
  for (size_t i = 0; i < sz; ++i) {
    aggregators[i]->updateObstacles(models);
  }
}

#endif
