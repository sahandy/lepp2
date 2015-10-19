#ifndef BASE_STAIR_DETECTOR_H_
#define BASE_STAIR_DETECTOR_H_

#include <vector>
#include <algorithm>

#include <pcl/visualization/cloud_viewer.h>

#include "lepp2/VideoObserver.hpp"
#include "lepp2/BaseSegmenter.hpp"
#include "lepp2/NoopSegmenter.hpp"
#include "lepp2/StairSegmenter.hpp"
#include "lepp2/StairAggregator.hpp"
#include "lepp2/ObjectApproximator.hpp"
#include "lepp2/MomentOfInertiaApproximator.hpp"
#include "lepp2/SplitApproximator.hpp"

using namespace lepp;

#include "lepp2/debug/timer.hpp"

template<class PointT>
class StairDetector : public lepp::VideoObserver<PointT> {

  public:
    StairDetector();
    virtual ~StairDetector() {}

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
    void attachStairAggregator(boost::shared_ptr<StairAggregator<PointT> > aggregator);

  protected:
    /// Helper typedefs to make the implementation cleaner
    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef typename PointCloudT::Ptr PointCloudPtr;
    typedef typename PointCloudT::ConstPtr CloudConstPtr;

    /**
     * Notifies any observers about newly detected stairs.
     */
    void notifyStairs(std::vector<CloudConstPtr> stairs);

  private:

    typename pcl::PointCloud<PointT>::ConstPtr cloud_;

    /**
     * Tracks all attached ObstacleAggregators that wish to be notified of newly
     * detected obstacles.
     */
    std::vector<boost::shared_ptr<StairAggregator<PointT> > > aggregators;

    boost::shared_ptr<BaseSegmenter<PointT> > segmenter_;
    /**
     * Performs a new update of the obstacle approximations.
     * Triggered when the detector is notified of a new frame (i.e. point cloud).
     */
    void update();
};

template<class PointT>
StairDetector<PointT>::StairDetector()
    : segmenter_(new StairSegmenter<PointT>()) {
}

template<class PointT>
void StairDetector<PointT>::notifyNewFrame(
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
void StairDetector<PointT>::update() {

  Timer t;
  t.start();
  std::vector<CloudConstPtr> stairs(segmenter_->segment(cloud_));
  t.stop();
  std::cerr << "Stair segmentation took " << t.duration() << std::endl;

  t.start();
  notifyStairs(stairs);
  t.stop();
  std::cerr << "visualization took " << t.duration() << std::endl;
}

template<class PointT>
void StairDetector<PointT>::attachStairAggregator(
    boost::shared_ptr<StairAggregator<PointT> > aggregator) {
  aggregators.push_back(aggregator);
}

template<class PointT>
void StairDetector<PointT>::notifyStairs(std::vector<CloudConstPtr> stairs) {
  size_t sz = aggregators.size();
  for (size_t i = 0; i < sz; ++i) {
    aggregators[i]->updateStairs(stairs);
  }
}

#endif
