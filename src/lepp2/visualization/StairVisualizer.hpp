#ifndef LEPP2_VISUALIZATION_STAIR_VISUALIZER_H__
#define LEPP2_VISUALIZATION_STAIR_VISUALIZER_H__

#include <vector>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "lepp2/VideoObserver.hpp"
#include "lepp2/StairAggregator.hpp"

namespace lepp{

template<class PointT>
class StairVisualizer
  : public VideoObserver<PointT>,
    public StairAggregator<PointT> {
public:
  StairVisualizer() : viewer_("StairVisualizer")
                       {}

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
};

template<class PointT>
void StairVisualizer<PointT>::drawStairs(
    std::vector<typename pcl::PointCloud<PointT>::ConstPtr> stairs,
    pcl::visualization::PCLVisualizer& pclViz) {
  size_t const sz = stairs.size();
  for (size_t i = 0; i < sz; ++i) {
    // drawer.drawStair(stairs[i]);
    //single_cloud->clear ();
    // *single_cloud = *stairs[i];
    if (i == 0) {
      // pcl::visualization::PointCloudColorHandlerCustom<PointT> red (stairs[i], 255, 0, 0);
      if(!pclViz.updatePointCloud(stairs[i], "STAIRS1"))
        pclViz.addPointCloud (stairs[i], "STAIRS1");
      pclViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0,0, "STAIRS1");
    }
    if (i == 1) {
      // pcl::visualization::PointCloudColorHandlerCustom<PointT> green (stairs[i], 0, 255, 0);
      if(!pclViz.updatePointCloud(stairs[i], "STAIRS2"))
        pclViz.addPointCloud (stairs[i], "STAIRS2");
      pclViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1.0,0, "STAIRS2");
    }
    if (i == 2) {
      // pcl::visualization::PointCloudColorHandlerCustom<PointT> blue (stairs[i], 0, 0, 255);
      if(!pclViz.updatePointCloud(stairs[i], "STAIRS3"))
        pclViz.addPointCloud (stairs[i], "STAIRS3");
      pclViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,0,1.0, "STAIRS3");
    }
    if (i == 3) {
      // pcl::visualization::PointCloudColorHandlerCustom<PointT> pink (stairs[i], 200, 18, 170);
      if(!pclViz.updatePointCloud(stairs[i], "STAIRS4"))
        pclViz.addPointCloud (stairs[i], "STAIRS4");
      pclViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.4,0.5,0.3, "STAIRS4");
    }
  }  
}

template<class PointT>
void StairVisualizer<PointT>::updateStairs(
    std::vector<typename pcl::PointCloud<PointT>::ConstPtr> stairs) {
  pcl::visualization::CloudViewer::VizCallable stair_visualization =
        boost::bind(&StairVisualizer::drawStairs,
                    this, stairs, _1);

  viewer_.runOnVisualizationThread(stair_visualization);
}

} // namespace lepp
#endif
