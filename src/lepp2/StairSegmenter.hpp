#ifndef LEPP2_STAIR_SEGMENTER_H__
#define LEPP2_STAIR_SEGMENTER_H__

#include "lepp2/BaseSegmenter.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>


namespace lepp {

/**
 * TODO put comments
 */
template<class PointT>
class StairSegmenter : public BaseSegmenter<PointT> {
public:
  StairSegmenter();

  virtual std::vector<typename pcl::PointCloud<PointT>::ConstPtr> segment(
      const typename pcl::PointCloud<PointT>::ConstPtr& cloud);
private:
  // Helper typedefs to make the implementation code cleaner
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef typename PointCloudT::Ptr PointCloudPtr;
  typedef typename PointCloudT::ConstPtr CloudConstPtr;

  // Private helper member functions
  /**
   * Performs some initial preprocessing and filtering appropriate for the
   * segmentation algorithm.
   * Takes the original cloud as a parameter and returns a pointer to a newly
   * created (and allocated) cloud containing the result of the filtering.
   */
  PointCloudPtr preprocessCloud(CloudConstPtr const& cloud);
  /**
   * Removes all planes from the given point cloud.
   */
  void findStairs(PointCloudPtr const& cloud_filtered);
  /**
   * Extracts the Euclidean clusters from the given point cloud.
   * Returns a vector where each element represents the pcl::PointIndices
   * instance representing the corresponding cluster.
   */
  std::vector<pcl::PointIndices> getStairClusters(
      PointCloudPtr const& cloud_filtered);
  /**
   * Convert the clusters represented by the given indices to point clouds,
   * by copying the corresponding points from the cloud to the corresponding
   * new point cloud.
   */
  std::vector<CloudConstPtr> clustersToPointClouds(
      CloudConstPtr const& cloud_filtered,
      std::vector<pcl::PointIndices> const& cluster_indices);


  /**
   * Instance used to extract the planes from the input cloud.
   */
  pcl::SACSegmentation<PointT> segmentation_;
  /**
   * Instance used to extract the actual clusters from the input cloud.
   */
  pcl::EuclideanClusterExtraction<PointT> clusterizer_;
  /**
   * The KdTree will hold the representation of the point cloud which is passed
   * to the clusterizer.
   */
  boost::shared_ptr<pcl::search::KdTree<PointT> > kd_tree_;

  /**
   * The cloud that holds all planar surfaces (Stairs).
   */
  PointCloudPtr cloud_stairs_;
  /**
   * The percentage of the original cloud that should be kept for the
   * clusterization, at the least.
   * We stop removing planes from the original cloud once there are either no
   * more planes to be removed or when the number of points remaining in the
   * cloud dips below this percentage of the original cloud.
   */

  /**
   * TEMP: vector containing clouds for each stair
   */
  std::vector<CloudConstPtr> vec_cloud_stairs_;

  double const min_filter_percentage_;
};

template<class PointT>
StairSegmenter<PointT>::StairSegmenter()
    : min_filter_percentage_(0.2),
      kd_tree_(new pcl::search::KdTree<PointT>()),
      cloud_stairs_(new PointCloudT()) {
  // Parameter initialization of the plane segmentation
  segmentation_.setOptimizeCoefficients(true);
  segmentation_.setModelType(pcl::SACMODEL_PLANE);
  segmentation_.setMethodType(pcl::SAC_RANSAC);
  segmentation_.setMaxIterations(100); // value recognized by Irem
  segmentation_.setDistanceThreshold(0.05);

  // Parameter initialization of the clusterizer
  clusterizer_.setClusterTolerance(17);
  clusterizer_.setMinClusterSize(150);
  clusterizer_.setMaxClusterSize(3100000);
}

template<class PointT>
typename pcl::PointCloud<PointT>::Ptr
StairSegmenter<PointT>::preprocessCloud(
    CloudConstPtr const& cloud) {
  // Remove NaN points from the input cloud.
  // The pcl API forces us to pass in a reference to the vector, even if we have
  // no use of it later on ourselves.
  PointCloudPtr cloud_filtered(new PointCloudT());
  std::vector<int> index;
  pcl::removeNaNFromPointCloud<PointT>(*cloud,
                                       *cloud_filtered,
                                       index);

  return cloud_filtered;
}

template<class PointT>
void StairSegmenter<PointT>::findStairs(
    PointCloudPtr const& cloud_filtered) {

  size_t surface_counter = 0;
  double surface_level = 0;

  vec_cloud_stairs_.clear();
  cloud_stairs_->clear();

  // Instance that will be used to perform the elimination of unwanted points
  // from the point cloud.
  pcl::ExtractIndices<PointT>  extract;
  // Will hold the indices of the next extracted plane within the loop
  pcl::PointIndices::Ptr current_plane_indices(new pcl::PointIndices);
  // Another instance of when the pcl API requires a parameter that we have no
  // further use for.
  pcl::ModelCoefficients coefficients;
  // Remove planes until we reach x % of the original number of points
  size_t const original_cloud_size = cloud_filtered->size();
  size_t const point_threshold = min_filter_percentage_ * original_cloud_size;
  while (cloud_filtered->size() > point_threshold) {
    // Try to obtain the next plane...
    segmentation_.setInputCloud(cloud_filtered);
    segmentation_.segment(*current_plane_indices, coefficients);

    // We didn't get any plane in this run. Therefore, there are no more planes
    // to be removed from the cloud.
    if (current_plane_indices->indices.size() == 0) {
      std::cerr << "cannot find more planes > BREAK" << std::endl;
      break;
    }

    // Cloud that holds a plane in each iteration, to be added to the total cloud.
    PointCloudPtr cloud_planar_surface(new PointCloudT());

    // Add the planar inliers to the cloud holding the stairs
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(current_plane_indices);
    extract.setNegative(false);
    extract.filter(*cloud_planar_surface);

    // ... and remove those inliers from the input cloud
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    // Determine the minimum surface level based on the currently found plane
    if (surface_counter == 0)
        surface_level = cloud_planar_surface->points[0].z;
      else if (surface_level > cloud_planar_surface->points[0].z)
        surface_level = cloud_planar_surface->points[0].z;
      // ... final else??
    surface_counter++;
    // put the newly found plane in the total cloud
    // TODO determine if pcl::concatenateFields is necessary
    // -> [http://pointclouds.org/documentation/tutorials/concatenate_clouds.php]
    *cloud_stairs_ += *cloud_planar_surface;

    vec_cloud_stairs_.push_back(cloud_planar_surface);
  }

  std::cout << "#found Stairs: " << vec_cloud_stairs_.size() << std::endl;

}

template<class PointT>
std::vector<pcl::PointIndices> StairSegmenter<PointT>::getStairClusters(
    PointCloudPtr const& cloud_stairs_) {
  // Extract the clusters from such a filtered cloud.
  kd_tree_->setInputCloud(cloud_stairs_);
  clusterizer_.setSearchMethod(kd_tree_);
  clusterizer_.setInputCloud(cloud_stairs_);
  std::vector<pcl::PointIndices> cluster_indices;
  clusterizer_.extract(cluster_indices);
  return cluster_indices;
}

template<class PointT>
std::vector<typename pcl::PointCloud<PointT>::ConstPtr>
StairSegmenter<PointT>::clustersToPointClouds(
    CloudConstPtr const& cloud_filtered,
    std::vector<pcl::PointIndices> const& cluster_indices) {
  // Now copy the points belonging to each cluster to a separate PointCloud
  // and finally return a vector of these point clouds.
  std::vector<CloudConstPtr> ret;
  size_t const cluster_count = cluster_indices.size();
  for (size_t i = 0; i < cluster_count; ++i) {
    typename PointCloudT::Ptr current(new PointCloudT());
    std::vector<int> const& curr_indices = cluster_indices[i].indices;
    size_t const curr_indices_sz = curr_indices.size();
    for (size_t j = 0; j < curr_indices_sz; ++j) {
      // add the point to the corresponding point cloud
      current->push_back(cloud_filtered->at(curr_indices[j]));
    }

    ret.push_back(current);
  }

  return ret;
}

template<class PointT>
std::vector<typename pcl::PointCloud<PointT>::ConstPtr>
StairSegmenter<PointT>::segment(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud) {
  PointCloudPtr cloud_filtered = preprocessCloud(cloud);
  // extract those planes that are considered as stairs and put them in cloud_stairs_
  findStairs(cloud_filtered);
  return vec_cloud_stairs_;
//  std::vector<pcl::PointIndices> stair_cluster_indices = getStairClusters(cloud_stairs_);
//  return clustersToPointClouds(cloud_stairs_, stair_cluster_indices);
}


} // namespace lepp

#endif
