#ifndef LEPP2_OBJECT_APPROXIMATOR_H__
#define LEPP2_OBJECT_APPROXIMATOR_H__

#include <pcl/common/projection_matrix.h>

namespace lepp {
  /**
   * Forward declaration of the ObjectModel class that represents an
   * approximation.
   */
  class ObjectModel;

/**
 * An abstract base class that classes that wish to generate approximations
 * for point clouds need to implement.
 */
template<class PointT>
class ObjectApproximator {
public:
  /**
   * Generate the approximations for the given point cloud.
   * The method assumes that the given point cloud segment is a single physical
   * object and tries to find the best approximations for this object, using its
   * own specific approximation method.
   */
  virtual std::vector<boost::shared_ptr<lepp::ObjectModel> > approximate(
      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud) = 0;
};

}  // namespace lepp

#endif
