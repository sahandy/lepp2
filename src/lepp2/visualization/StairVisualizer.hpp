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
//    if (is_first_cloud) {
//      pcl_visualizer_->addPointCloud(pointCloud, "TOTALPC");
//      pcl_visualizer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "TOTALPC");
//      is_first_cloud = false;
//    }
//    else
//      pcl_visualizer_->updatePointCloud(pointCloud, "TOTALPC");
//    pcl_visualizer_->spinOnce(1);
  }

  /**
   * StairAggregator interface implementation: processes detected obstacles.1
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
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(single_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> green(single_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(single_cloud, 0, 0, 255);

    pcl_visualizer_->removePointCloud("STAIRS1");
    pcl_visualizer_->removePointCloud("STAIRS2");
    pcl_visualizer_->removePointCloud("STAIRS3");
    size_t const sz = cloud_stairs.size();
    for(size_t i=0; i<sz; ++i) {
      single_cloud->clear();
      *single_cloud = *cloud_stairs[i];
      // TODO Assign Different color to each cloud automatically
      if(i==0)
        pcl_visualizer_->addPointCloud(single_cloud, red, "STAIRS1");
      if(i==1)
        pcl_visualizer_->addPointCloud(single_cloud, green, "STAIRS2");
      if(i==2)
        pcl_visualizer_->addPointCloud(single_cloud, blue, "STAIRS3");
    }

    pcl_visualizer_->spinOnce(1);
}

} // namespace lepp

#endif
