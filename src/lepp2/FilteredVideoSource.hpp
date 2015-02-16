#ifndef LEPP2_FILTERED_VIDEO_SOURCE_H__
#define LEPP2_FILTERED_VIDEO_SOURCE_H__

#include "lepp2/BaseVideoSource.hpp"
#include "lepp2/VideoObserver.hpp"
#include <boost/enable_shared_from_this.hpp>

struct MapPoint {
  int x;
  int y;
  int z;
  MapPoint(int x, int y, int z) : x(x), y(y), z(z) {}
  MapPoint() {}
};

bool operator==(MapPoint const& lhs, MapPoint const& rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

size_t hash_value(MapPoint const& pt) {
  std::size_t seed = 0;
  boost::hash_combine(seed, pt.x);
  boost::hash_combine(seed, pt.y);
  boost::hash_combine(seed, pt.z);
  return seed;
}

/**
 * A VideoSource decorator.  It wraps a given VideoSource instance and emits
 * clouds that are filtered versions of the original raw cloud returned by the
 * wrapped `VideoSource`.
 *
 * The filters applied to the original cloud are hard coded to the implementation
 * (for now).
 */
template<class PointT>
class FilteredVideoSource
    : public VideoSource<PointT>,
      public VideoObserver<PointT>,
      public boost::enable_shared_from_this<FilteredVideoSource<PointT> > {
public:
  /**
   * Creates a new FilteredVideoSource which will perform filtering of each
   * cloud generated by the given source and emit such a filtered cloud to its
   * own observers.
   *
   * The FilteredVideoSource instance does not assume ownership of the given
   * source, but shares it.
   */
  FilteredVideoSource(boost::shared_ptr<VideoSource<PointT> > source)
      : source_(source) {}
  /**
   * Implementation of the VideoSource interface.
   */
  virtual void open();
  /**
   * Implementation of the VideoObserver interface.
   */
  virtual void notifyNewFrame(
      int idx,
      const typename pcl::PointCloud<PointT>::ConstPtr& cloud);
private:
  /**
   * The VideoSource instance that will be filtered by this instance.
   */
  boost::shared_ptr<VideoSource<PointT> > source_;
};

template<class PointT>
void FilteredVideoSource<PointT>::open() {
  // Start the wrapped VideoSource and make sure that this instance is notified
  // when it emits any new clouds.
  source_->attachObserver(this->shared_from_this());
  source_->open();
}

template<class PointT>
void FilteredVideoSource<PointT>::notifyNewFrame(
    int idx,
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud) {
  // Process the cloud received from the wrapped instance
  typename PointCloudType::Ptr cloud_filtered(new PointCloudType());

  // We remove any NaN points and clamp the rest to exactly 2 decimal digits.
  // This helps make the cloud more uniform, since anything after the second
  // decimal is either noise or not useful information.
  cloud_filtered->is_dense = true;
  cloud_filtered->sensor_origin_ = cloud->sensor_origin_;
  for (typename pcl::PointCloud<PointT>::const_iterator it = cloud->begin();
        it != cloud->end();
        ++it) {
    PointT p = *it;
    // Filter out NaN points already, since we're already iterating through the
    // cloud.
    if (!pcl_isfinite(p.x) || !pcl_isfinite(p.y) || !pcl_isfinite(p.z)) {
      continue;
    }
    // Truncate the point to 2 decimals
    double const decimals = 100;
    p.x = static_cast<int>((p.x * decimals)) / decimals;
    p.y = static_cast<int>((p.y * decimals)) / decimals;
    p.z = static_cast<int>((p.z * decimals)) / decimals;
    cloud_filtered->push_back(p);
  }

  // Finally, the cloud that is emitted by this instance is the filtered cloud.
  this->setNextFrame(cloud_filtered);
}


#endif
