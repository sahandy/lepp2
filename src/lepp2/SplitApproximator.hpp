#ifndef LEPP2_SPLIT_APPROXIMATOR_H__
#define LEPP2_SPLIT_APPROXIMATOR_H__
#include "lepp2/ObjectApproximator.hpp"
#include "lepp2/models/ObjectModel.h"

#include <deque>
#include <map>

#include <pcl/common/pca.h>
#include <pcl/common/common.h>

namespace lepp {

/**
 * An ABC that represents the strategy for splitting a point cloud used by the
 * `SplitObjectApproximator`.
 *
 * Varying the `SplitStrategy` implementation allows us to change how point
 * clouds are split (or if they are split at all) without changing the logic of
 * the `SplitObjectApproximator` approximator itself.
 */
template<class PointT>
class SplitStrategy {
public:
  /**
   * Performs the split of the given point cloud according to the particular
   * strategy.
   *
   * This method has a default implementation that is a template method, which
   * calls the protected `shouldSplit` method and calls the `doSplit` if it
   * indicated that a split should be made. Since all of the methods are
   * virtual, concrete implementations can override how the split is made or
   * keep the default implementation.
   *
   * This method can also be overridden by concrete implementations if they
   * cannot implement the logic only in terms of the `shouldSplit` and
   * `doSplit` method implementations (although that should be rare).
   *
   * :param split_depth: The current split depth, i.e. the number of times the
   *     original cloud has already been split
   * :param point_cloud: The current point cloud that should be split by the
   *    `SplitStrategy` implementation.
   * :returns: The method should return a vector of point clouds obtained by
   *      splitting the given cloud into any number of parts. If the given
   *      point cloud should not be split, an empty vector should be returned.
   *      Once the empty vector is returned, the `SplitObjectApproximator` will
   *      stop the splitting process for that branch of the split tree.
   */
  virtual std::vector<typename pcl::PointCloud<PointT>::Ptr> split(
      int split_depth,
      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud);
protected:
  /**
   * A pure virtual method that decides whether the given point cloud should be
   * split or not.
   *
   * :param split_depth: The current split depth, i.e. the number of times the
   *     original cloud has already been split
   * :param point_cloud: The current point cloud that should be split by the
   *    `SplitStrategy` implementation.
   * :returns: A boolean indicating whether the cloud should be split or not.
   */
  virtual bool shouldSplit(
      int split_depth,
      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud) = 0;
  /**
   * A helper method that does the actual split, when needed.
   * A default implementation is provided, since that is what most splitters
   * will want to use...
   */
  virtual std::vector<typename pcl::PointCloud<PointT>::Ptr> doSplit(
      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud);
};

template<class PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> SplitStrategy<PointT>::split(
    int split_depth,
    const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud) {
  if (this->shouldSplit(split_depth, point_cloud)) {
    return this->doSplit(point_cloud);
  } else {
    return std::vector<typename pcl::PointCloud<PointT>::Ptr>();
  }
}

template<class PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
SplitStrategy<PointT>::doSplit(
    const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud) {
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  // Compute PCA for the input cloud
  pcl::PCA<PointT> pca;
  pca.setInputCloud(point_cloud);
  Eigen::Vector3f eigenvalues = pca.getEigenValues();
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();

  Eigen::Vector3d main_pca_axis = eigenvectors.col(0).cast<double>();

  // Compute the centroid
  Eigen::Vector4d centroid;
  pcl::compute3DCentroid(*point_cloud, centroid);

  /// The plane equation
  double d = (-1) * (
      centroid[0] * main_pca_axis[0] +
      centroid[1] * main_pca_axis[1] +
      centroid[2] * main_pca_axis[2]
  );

  // Prepare the two parts.
  std::vector<PointCloudPtr> ret;
  ret.push_back(PointCloudPtr(new pcl::PointCloud<PointT>()));
  ret.push_back(PointCloudPtr(new pcl::PointCloud<PointT>()));
  PointCloud& first = *ret[0];
  PointCloud& second = *ret[1];

  // Now divide the input cloud into two clusters based on the splitting plane
  size_t const sz = point_cloud->size();
  for (size_t i = 0; i < sz; ++i) {
    // Boost the precision of the points we are dealing with to make the
    // calculation more precise.
    PointT const& original_point = (*point_cloud)[i];
    Eigen::Vector3f const vector_point = original_point.getVector3fMap();
    Eigen::Vector3d const point = vector_point.cast<double>();
    // Decide on which side of the plane the current point is and add it to the
    // appropriate partition.
    if (point.dot(main_pca_axis) + d < 0.) {
      first.push_back(original_point);
    } else {
      second.push_back(original_point);
    }
  }

  // Return the parts in a vector, as expected by the interface...
  return ret;
}

/**
 * An ABC for classes that provide the functionality of checking whether a
 * point cloud should be split or not.
 */
template<class PointT>
class SplitCondition {
public:
  /**
   * A pure virtual method that decides whether the given point cloud should be
   * split or not.
   *
   * :param split_depth: The current split depth, i.e. the number of times the
   *     original cloud has already been split
   * :param point_cloud: The current point cloud that should be split by the
   *    `SplitStrategy` implementation.
   * :returns: A boolean indicating whether the cloud should be split or not.
   */
  virtual bool shouldSplit(
      int split_depth,
      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud) = 0;
};

/**
 * A `SplitStrategy` implementation that initiates the split iff the depth is
 * less than the given limit.
 */
template<class PointT>
class DepthLimitSplitStrategy : public SplitStrategy<PointT> {
public:
  DepthLimitSplitStrategy(int depth_limit) : limit_(depth_limit) {}
  bool shouldSplit(
      int split_depth,
      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud);
private:
  /**
   * The limit at which splits will stop.
   */
  int const limit_;
};

template<class PointT>
bool DepthLimitSplitStrategy<PointT>::shouldSplit(
    int split_depth,
    const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud) {
  return split_depth < limit_;
}

/**
 * An approximator implementation that will generate an approximation by
 * splitting the given object into multiple parts. Each part approximation is
 * generated by delegating to a wrapped `ObjectApproximator` instance, allowing
 * clients to vary the algorithm used for approximations, while keeping the
 * logic of incrementally splitting up the object.
 */
template<class PointT>
class SplitObjectApproximator : public ObjectApproximator<PointT> {
public:
  /**
   * Create a new `SplitObjectApproximator` that will approximate each part by
   * using the given approximator instance and perform splits decided by the
   * given `SplitStrategy` instance.
   */
  SplitObjectApproximator(
        boost::shared_ptr<ObjectApproximator<PointT> > approx,
        boost::shared_ptr<SplitStrategy<PointT> > splitter)
          : approximator_(approx),
            splitter_(splitter) {}
  /**
   * `ObjectApproximator` interface method.
   */
  boost::shared_ptr<CompositeModel> approximate(
      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud);
private:
  /**
   * An `ObjectApproximator` used to generate approximations for object parts.
   */
  boost::shared_ptr<ObjectApproximator<PointT> > approximator_;
  /**
   * The strategy to be used for splitting point clouds.
   */
  boost::shared_ptr<SplitStrategy<PointT> > splitter_;
};

template<class PointT>
boost::shared_ptr<CompositeModel> SplitObjectApproximator<PointT>::approximate(
    const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud) {
  boost::shared_ptr<CompositeModel> approx(new CompositeModel);
  typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  std::deque<std::pair<int, PointCloudConstPtr> > queue;
  queue.push_back(std::make_pair(0, point_cloud));

  while (!queue.empty()) {
    int const depth = queue[0].first;
    PointCloudConstPtr const current_cloud = queue[0].second;
    queue.pop_front();

    // Delegates to the wrapped approximator for each part's approximation.
    ObjectModelPtr model = approximator_->approximate(current_cloud);
    // TODO Decide whether the model fits well enough for the current cloud.
    // For now we fix the number of split iterations.
    // The approximation should be improved. Try doing it for the split clouds
    std::vector<PointCloudPtr> const splits = splitter_->split(depth, current_cloud);
    // Add each new split section into the queue as children of the current
    // node.
    if (splits.size() != 0) {
      for (size_t i = 0; i < splits.size(); ++i) {
        queue.push_back(std::make_pair(depth + 1, splits[i]));
      }
    } else {
      // Keep the approximation
      approx->addModel(model);
    }
  }

  return approx;
}

}  // namespace lepp
#endif
