#ifndef LEPP2_VISUALIZATION_STAIR_VISUALIZER_H__
#define LEPP2_VISUALIZATION_STAIR_VISUALIZER_H__

#include <vector>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "lepp2/VideoObserver.hpp"

namespace lepp
{

  template <class PointT>
  class StairVisualizer : public VideoObserver<PointT>, public StairAggregator<PointT>
  {

    public:

      StairVisualizer () :
          pcl_visualizer_ (new pcl::visualization::PCLVisualizer ("STAIR_VISUALIZER")),
          is_first_cloud (true),
          release_cam_lock (false)
      {

        pcl_visualizer_->setBackgroundColor (0, 0, 0);
        pcl_visualizer_->addCoordinateSystem (1.0);
        pcl_visualizer_->setCameraPosition (-2.31242, -2.76225, 2.44972, 0.207447, 0.350107, 0.91345, 0);
        pcl_visualizer_->initCameraParameters ();
      }

      /**
       * VideoObserver interface implementation: processes the current point cloud.
       * In the case of the StairVisualizer, this cloud is the complete one (no segmentation).
       * It pairs with the #updateStairs, which overlays different stairs found by the StairSegmenter class.
       */
      virtual void
      notifyNewFrame (int idx,
                      const typename pcl::PointCloud<PointT>::ConstPtr& pointCloud)
      {

        std::cout << "Showing Frame #" << idx << std::endl;
        if (is_first_cloud)
        {
          pcl_visualizer_->addPointCloud (pointCloud, "TOTALPC");
          pcl_visualizer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "TOTALPC");
          is_first_cloud = false;
        }
        else
          pcl_visualizer_->updatePointCloud (pointCloud, "TOTALPC");
        // For better visualization, we set the camera position to an angle that the whole cloud is nicely visible.
        // TODO: it makes sense to set the camera position to the position of the actual camera itself.
        if (!release_cam_lock)
        {
          pcl_visualizer_->setCameraPosition (-2.31242, -2.76225, 2.44972, 0.207447, 0.350107, 0.91345, 0);
          release_cam_lock = true;
        }
        pcl_visualizer_->spinOnce (1);
      }

      /**
       * StairAggregator interface implementation: processes detected stairs
       * and tries to visualize them with different color so that they are distinguishable
       * in the visualizer
       */
      virtual void
      updateStairs (std::vector<typename pcl::PointCloud<PointT>::ConstPtr> cloud_stairs);

    private:
      /**
       * Used for the visualization of the scene.
       */
      boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_visualizer_;
      boost::shared_ptr<pcl::visualization::Camera> camera_;
      bool is_first_cloud;
      bool release_cam_lock;
  };

  template <class PointT>
  void
  StairVisualizer<PointT>::updateStairs (std::vector<typename pcl::PointCloud<PointT>::ConstPtr> cloud_stairs)
  {

    typename pcl::PointCloud<PointT>::Ptr single_cloud (new pcl::PointCloud<PointT> ());
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red (single_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> green (single_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> blue (single_cloud, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> pink (single_cloud, 200, 18, 170);

    pcl_visualizer_->removePointCloud ("STAIRS1");
    pcl_visualizer_->removePointCloud ("STAIRS2");
    pcl_visualizer_->removePointCloud ("STAIRS3");
    pcl_visualizer_->removePointCloud ("STAIRS4");
    size_t const sz = cloud_stairs.size ();
    for (size_t i = 0; i < sz; ++i)
    {
      single_cloud->clear ();
      *single_cloud = *cloud_stairs[i];
      // TODO remove hard-coded color assignment
      if (i == 0)
        pcl_visualizer_->addPointCloud (single_cloud, red, "STAIRS1");
      if (i == 1)
        pcl_visualizer_->addPointCloud (single_cloud, green, "STAIRS2");
      if (i == 2)
        pcl_visualizer_->addPointCloud (single_cloud, blue, "STAIRS3");
      if (i == 3)
        pcl_visualizer_->addPointCloud (single_cloud, pink, "STAIRS4");
    }
    pcl_visualizer_->spinOnce (1);
  }

}  // namespace lepp
#endif
