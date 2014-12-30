#ifndef LEPP2_VISUALIZATION_OBSTACLE_VISUALIZER_H__
#define LEPP2_VISUALIZATION_OBSTACLE_VISUALIZER_H__

#include <sstream>

#include "lepp2/VideoObserver.hpp"

namespace lepp {

/**
 * Visualizes the obstacles detected by a particular obstacle detector by
 * overlaying them onto a point cloud feed coming from a video source.
 *
 * Implements the VideoObserver and ObstacleAggregator interfaces.
 */
template<class PointT>
class ObstacleVisualizer
  : public VideoObserver<PointT>,
    public ObstacleAggregator {
public:
  ObstacleVisualizer() : viewer_("ObstacleVisualizer") {}

  /**
   * VideoObserver interface implementation: processes the current point cloud.
   */
  virtual void notifyNewFrame(
      int idx,
      const typename pcl::PointCloud<PointT>::ConstPtr& pointCloud) {
    viewer_.showCloud(pointCloud);
  }

  /**
   * ObstacleAggregator interface implementation: processes detected obstacles.
   */
  virtual void updateObstacles(ObjectModelPtrListPtr obstacles);

private:
  /**
   * Used for the visualization of the scene.
   */
  pcl::visualization::CloudViewer viewer_;

  /**
   * Private member function that is used to set up a callback which is executed
   * by the viewer when the detected obstacles should be visualized.
   */
  void drawShapes(ObjectModelPtrListPtr obstacles,
                  pcl::visualization::PCLVisualizer& viewer);
};

template<class PointT>
void ObstacleVisualizer<PointT>::drawShapes(
    ObjectModelPtrListPtr obstacles,
    pcl::visualization::PCLVisualizer& viewer) {
  viewer.removeAllShapes();

  size_t const sz = obstacles->size();
  for (size_t i = 0; i < sz; ++i) {
    // Generates a unique name for each obstacle within the current frame.
    std::ostringstream ss;
    ss << obstacles->at(i)->getType() << "(" << i << ")";
    std::string const shape_name(ss.str());
    // Delegate to the Model to draw itself on the viewer.
    obstacles->at(i)->draw(viewer, shape_name, 0.5f, 0.5f, 0.0f);
  }
}

template<class PointT>
void ObstacleVisualizer<PointT>::updateObstacles(
    ObjectModelPtrListPtr obstacles) {
  pcl::visualization::CloudViewer::VizCallable obstacle_visualization =
      boost::bind(&ObstacleVisualizer::drawShapes,
                  this, obstacles, _1);
  viewer_.runOnVisualizationThreadOnce(obstacle_visualization);
}

} // namespace lepp

#endif
