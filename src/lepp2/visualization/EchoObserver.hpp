#ifndef LEPP2_VISUALIZATION_ECHO_OBSERVER_H__
#define LEPP2_VISUALIZATION_ECHO_OBSERVER_H__

#include "lepp2/VideoObserver.hpp"

namespace lepp {

/**
 * Simple VideoObserver implementation that echoes the point cloud to a PCL
 * CloudViewer for visualization purposes.
 */
template<class PointT>
class EchoObserver : public VideoObserver<PointT> {
public:
  EchoObserver() : viewer_("PCL Viewer") {}

  virtual void notifyNewFrame(
      int idx,
      const typename pcl::PointCloud<PointT>::ConstPtr& pointCloud) {
    viewer_.showCloud(pointCloud);
  }

private:
  pcl::visualization::CloudViewer viewer_;
};

}

#endif
