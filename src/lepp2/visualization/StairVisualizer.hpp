#ifndef LEPP2_VISUALIZATION_STAIR_VISUALIZER_H__
#define LEPP2_VISUALIZATION_STAIR_VISUALIZER_H__

#include <vector>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "lepp2/VideoObserver.hpp"

namespace lepp{

template<class PointT>
class StairVisualizer
  : public VideoObserver<PointT>,
    public StairAggregator<PointT> {
public:
  StairVisualizer()
    : pcl_visualizer_(new pcl::visualization::PCLVisualizer ("STAIR_VISUALIZER")),
      is_first_cloud(true),
      is_first_addition(true) {

    pcl_visualizer_->setBackgroundColor (0, 0, 0);
//    pcl_visualizer_->addPointCloud < pcl::PointXYZ > (cloud, "sample cloud");

    pcl_visualizer_->addCoordinateSystem (1.0);
    pcl_visualizer_->initCameraParameters ();

    }

  /**
   * VideoObserver interface implementation: processes the current point cloud.
   */
  virtual void notifyNewFrame(
      int idx,
      const typename pcl::PointCloud<PointT>::ConstPtr& pointCloud) {
    if (is_first_cloud) {
      pcl_visualizer_->addPointCloud(pointCloud, "TOTALPC");
      pcl_visualizer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "TOTALPC");
      is_first_cloud = false;
    }
    else
      pcl_visualizer_->updatePointCloud(pointCloud, "TOTALPC");
    pcl_visualizer_->spinOnce(1);
  }

  /**
   * StairAggregator interface implementation: processes detected obstacles.
   */
//  virtual void updateStairs(std::vector<typename pcl::PointCloud<PointT>::ConstPtr> stairs);
  virtual void updateStairs(std::vector<typename pcl::PointCloud<PointT>::ConstPtr> cloud_stairs);

private:
  /**
   * Used for the visualization of the scene.
   */
  boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_visualizer_;
  bool is_first_cloud;
  bool is_first_addition;
};

template<class PointT>
void StairVisualizer<PointT>::updateStairs(std::vector<typename pcl::PointCloud<PointT>::ConstPtr> cloud_stairs) {

    typename pcl::PointCloud<PointT>::Ptr single_cloud(new pcl::PointCloud<PointT>());
    size_t const sz = cloud_stairs.size();
    for(size_t i=0; i<sz; ++i) {
      // first plane is always the floor (a.k.a biggest plane found by StairSegmenter)
      if(i==0)
        continue;

      *single_cloud = *cloud_stairs[i];
      // TODO Assign Different color to each cloud
      pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (single_cloud, 0, 255, 0);
      if(is_first_addition) {
        pcl_visualizer_->addPointCloud(single_cloud, single_color, "STAIRS");
        pcl_visualizer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "STAIRS");

        is_first_addition = false;
      } else
        pcl_visualizer_->updatePointCloud(single_cloud, single_color, "STAIRS");
    }
}

} // namespace lepp

#endif
