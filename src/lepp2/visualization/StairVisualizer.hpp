#ifndef LEPP2_VISUALIZATION_STAIR_VISUALIZER_H__
#define LEPP2_VISUALIZATION_STAIR_VISUALIZER_H__

#include <vector>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "lepp2/VideoObserver.hpp"

namespace lepp{

template<class PointT>
class StairDrawer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StairDrawer(pcl::visualization::PCLVisualizer& pcl_viz)
    : pcl_viz_(pcl_viz), total_(0) {}
  void drawStair(typename pcl::PointCloud<PointT>::ConstPtr cloud_stair);
private:
  pcl::visualization::PCLVisualizer& pcl_viz_;
  int total_;
};

template<class PointT>
void StairDrawer<PointT>::drawStair(
    typename pcl::PointCloud<PointT>::ConstPtr cloud_stair)  {
      ++total_;

      typename pcl::PointCloud<PointT>::Ptr single_cloud (new pcl::PointCloud<PointT> ());
      pcl::visualization::PointCloudColorHandlerCustom<PointT> red (single_cloud, 255, 0, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> green (single_cloud, 0, 255, 0);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> blue (single_cloud, 0, 0, 255);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> pink (single_cloud, 200, 18, 170);

      std::ostringstream ss;
      ss << "STAIR_" << total_;
      std::string const cloud_name = ss.str();

      single_cloud->clear ();
      *single_cloud = *cloud_stair;
      if (total_ == 1)
      {
        std::cout << "show cloud #1" << std::endl;
        pcl_viz_.addPointCloud (single_cloud, red, cloud_name);
      }
      else if (total_ == 2)
      {
        std::cout << "show cloud #2" << std::endl;
        pcl_viz_.addPointCloud (single_cloud, green, cloud_name);
      }
      else if (total_ == 3)
      {
        std::cout << "show cloud #3" << std::endl;
        pcl_viz_.addPointCloud (single_cloud, blue, cloud_name);
      }
      else
      {
        std::cout << "show cloud #" << total_ << std::endl;
        pcl_viz_.addPointCloud (single_cloud, pink, cloud_name);
      }

    }

template<class PointT>
class StairVisualizer
  : public VideoObserver<PointT>,
    public StairAggregator<PointT> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StairVisualizer() : viewer_("StairVisualizer")
                       {} //pclViz (new pcl::visualization::PCLVisualizer ("PCL_VIZ"))

  /**
   * VideoObserver interface implementation: show the current point cloud.
   */
  virtual void notifyNewFrame(
      int idx,
      const typename pcl::PointCloud<PointT>::ConstPtr& pointCloud) {
        viewer_.showCloud(pointCloud);
      }

  /**
   * StairAggregator interface implementation: processes detected obstacles.
   */
  virtual void updateStairs(std::vector<typename pcl::PointCloud<PointT>::ConstPtr> cloud_stairs);

  void drawStairs(std::vector<typename pcl::PointCloud<PointT>::ConstPtr> stairs,
                  pcl::visualization::PCLVisualizer& viewer);

private:
  /**
   * Used for the visualization of the scene.
   */
  pcl::visualization::CloudViewer viewer_;
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViz;
};

template<class PointT>
void StairVisualizer<PointT>::drawStairs(
    std::vector<typename pcl::PointCloud<PointT>::ConstPtr> stairs,
    pcl::visualization::PCLVisualizer& pclViz) {
  // Remove all old stars...
  std::cout << "call removeAllPointClouds" << std::endl;
  pclViz.removeAllPointClouds();
  std::cout << "all pointClouds removed" << std::endl;

  // ...and draw all the new ones.
  StairDrawer<PointT> drawer(pclViz);
  size_t const sz = stairs.size();
  for (size_t i = 0; i < sz; ++i) {
    drawer.drawStair(stairs[i]);
  }

  std::cout << "exiting stair drawer..." << std::endl;
  std::cout << "=======================" << std::endl;
}

template<class PointT>
void StairVisualizer<PointT>::updateStairs(
    std::vector<typename pcl::PointCloud<PointT>::ConstPtr> stairs) {
    std::cout << "entered updateStairs" << std::endl;
  pcl::visualization::CloudViewer::VizCallable stair_visualization =
        boost::bind(&StairVisualizer::drawStairs,
                    this, stairs, _1);

  std::cout << "call runOnVisualizationThreadOnce" << std::endl;
  viewer_.runOnVisualizationThreadOnce(stair_visualization);
  std::cout << "exiting updateStairs..." << std::endl;
}

} // namespace lepp
#endif
