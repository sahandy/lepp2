#ifndef LEPP2_MOMENT_OF_INERTIA_APPROXIMATOR_H__
#define LEPP2_MOMENT_OF_INERTIA_APPROXIMATOR_H__

#include "lepp2/ObjectApproximator.hpp"

#include <deque>

#include <pcl/common/pca.h>
#include <pcl/common/common.h>

#include "lepp2/legacy/moment_of_inertia_estimation.hpp"
#include "lepp2/legacy/Preallocator.hpp"
#include "lepp2/legacy/models/SphereModel.hpp"
#include "lepp2/legacy/models/CylinderModel.hpp"
#include "lepp2/legacy/models/TriangleModel.hpp"

namespace lepp {

/**
 * An implementation of the ObjectApproximator abstract base class that performs
 * approximations based on a heuristic that chooses one of the object models
 * based on the principal axes of the object.
 *
 * NOTE:
 * Mostly an adaptation of the legacy implementation to the new and improved
 * interface specifications. As such, still depends on some of the legacy
 * constructs, such as ObjectModel(s) and Preallocators.
 */
template<class PointT>
class MomentOfInertiaObjectApproximator : public ObjectApproximator<PointT> {
public:
  std::vector<boost::shared_ptr<ObjectModel> > approximate(
      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud);
private:
  // Legacy preallocators
  Preallocator<SphereModel> m_preallocator_sphere;
  Preallocator<CylinderModel> m_preallocator_cylinder;
  Preallocator<TriangleModel> m_preallocator_triangle;

  // Private helper member functions for fitting individual models.
  // Takes a pointer to a model and a descriptor and sets the parameters of the
  // model so that it describes the point cloud with the given features in the
  // best way.
  // The implementations of the functions are pretty much a copy-paste of the
  // legacy code (ergo not very clean).
  // TODO This improves the legacy fitting *somewhat* in that at least some
  //      sort of of polymorphism is being applied (overloaded functions) so
  //      that the if-then-else is avoided at least for fitting
  // This design isn't even *that* bad, but it would be better to use some sort
  // of double-dispatch so that MoIApproximator doesn't need to be recompiled
  // when another model is added.
  void performFitting(boost::shared_ptr<SphereModel> sphere,
                      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud,
                      Eigen::Vector3f mass_center,
                      std::vector <Eigen::Vector3f> const& axes);
  void performFitting(boost::shared_ptr<CylinderModel> cylinder,
                      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud,
                      Eigen::Vector3f mass_center,
                      std::vector <Eigen::Vector3f> const& axes);
  void performFitting(boost::shared_ptr<TriangleModel> triangle,
                      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud,
                      Eigen::Vector3f mass_center,
                      std::vector <Eigen::Vector3f> const& axes);
  /**
   * Finds an approximation for the given PointCloud using a single ObjectModel.
   */
  boost::shared_ptr<ObjectModel> getSingleApproximation(
      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud);
  /**
   * Splits the given point_cloud into two parts and places them in the
   * ``first`` and ``second`` PointCloud references.
   */
  void splitCloud(
      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud,
      pcl::PointCloud<PointT>& first,
      pcl::PointCloud<PointT>& second);

  /**
   * Returns a point representing an estimation of the position of the center
   * of mass for the given point cloud.
   */
  Eigen::Vector3f estimateMassCenter(
      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud);
};

template<class PointT>
std::vector<boost::shared_ptr<ObjectModel> >
MomentOfInertiaObjectApproximator<PointT>::approximate(
    const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud) {
  typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
  typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  std::vector<boost::shared_ptr<ObjectModel> > ret;

  std::deque<PointCloudConstPtr> queue;
  queue.push_back(point_cloud);

  int iteration = 0;
  while (!queue.empty()) {
    PointCloudConstPtr current_cloud = queue[0];
    queue.pop_front();

    ObjectModelPtr model = getSingleApproximation(current_cloud);
    // TODO Decide whether the model fits well enough for the current cloud.
    // For now we fix the number of iterations.
    if (iteration == 0) {
      // The approximation should be improved. Try doing it for the split clouds
      PointCloudPtr first(new pcl::PointCloud<PointT>());
      PointCloudPtr second(new pcl::PointCloud<PointT>());
      splitCloud(current_cloud, *first, *second);
      queue.push_back(first);
      queue.push_back(second);
    } else {
      // Keep the approximation
      ret.push_back(model);
    }
    ++iteration;
  }

  return ret;
}

template<class PointT>
boost::shared_ptr<ObjectModel>
MomentOfInertiaObjectApproximator<PointT>::getSingleApproximation(
    const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud) {

  // Firstly, obtain the moment of inertia descriptors
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  // Uses a customized/forked MomentOfInertiaEstimation
  pcl::MomentOfInertiaEstimation<PointT> moment_of_inertia_extractor;
  moment_of_inertia_extractor.setInputCloud(point_cloud);
  moment_of_inertia_extractor.compute();
  moment_of_inertia_extractor.getEigenValues(major_value,
                                             middle_value,
                                             minor_value);
  moment_of_inertia_extractor.getEigenVectors(major_vector,
                                              middle_vector,
                                              minor_vector);
  std::vector<Eigen::Vector3f> axes;
  axes.push_back(major_vector);
  axes.push_back(middle_vector);
  axes.push_back(minor_vector);

  // Guesstimate the center of mass
  Eigen::Vector3f mass_center(estimateMassCenter(point_cloud));

  // Based on these descriptors, decide which object type should be used.
  // Code smell: if-then-else chain!
  // TODO Find a way to defer this decision to object models themselves.
  //      Give them a descriptor and ask for a rating of how good that model can
  //      be used to approximate the described cloud.
  //      Allows for future extensibility with other model types.
  //      (Bonus points: make the models independent from the MoI-based
  //       descriptors)
  boost::shared_ptr<ObjectModel> model;
  if ((middle_value / major_value > .6) && (minor_value / major_value > .1)) {
    boost::shared_ptr<SphereModel> sphere = m_preallocator_sphere.newInstance();
    performFitting(sphere, point_cloud, mass_center, axes);
    model = sphere;
  } else if (middle_value / major_value < .25) {
    boost::shared_ptr<CylinderModel> cylinder =
        m_preallocator_cylinder.newInstance();
    performFitting(cylinder, point_cloud, mass_center, axes);
    model = cylinder;
  } else {
    // Triangles were being used as a catch-all when nothing else worked.
    boost::shared_ptr<TriangleModel> triangle =
        m_preallocator_triangle.newInstance();
    performFitting(triangle, point_cloud, mass_center, axes);
    model = triangle;
  }

  return model;
}

template<class PointT>
void MomentOfInertiaObjectApproximator<PointT>::splitCloud(
    const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud,
    pcl::PointCloud<PointT>& first,
    pcl::PointCloud<PointT>& second) {
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

}

template<class PointT>
Eigen::Vector3f MomentOfInertiaObjectApproximator<PointT>::estimateMassCenter(
    const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud) {
  PointT min_pt;
  PointT max_pt;
  // TODO Is this really a good heuristic? (It comes from the legacy code)
  pcl::getMinMax3D(*point_cloud, min_pt, max_pt);
  Eigen::Vector3f mass_center;
  mass_center(0) = (max_pt.x + min_pt.x)/2;
  mass_center(1) = 1.02 * ((max_pt.y + min_pt.y) / 2);
  mass_center(2) = 1.02* ((max_pt.z + min_pt.z) / 2);

  return mass_center;
}

template<class PointT>
void MomentOfInertiaObjectApproximator<PointT>::performFitting(
    boost::shared_ptr<SphereModel> sphere,
    const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud,
    Eigen::Vector3f mass_center,
    std::vector <Eigen::Vector3f> const& axes) {
  Eigen::VectorXf  coeffs;

  Eigen::Vector4f  max_point;
  Eigen::Vector4f  center;

  Eigen::Vector3f   result;

  float radius;
  float dist;
  std::vector <int> nearestPointIndex(1);
  std::vector <float> nearestPointSquaredDistance(1);
  float max_dist;
  Eigen::Vector3f      max_distance_vector;
  pcl::PointXYZ       searchPoint;
  pcl::KdTreeFLANN<pcl::PointXYZ>   kdtree;

  Eigen::Vector3f  radius_vector;

  center(0) = mass_center(0);
  center(1) = mass_center(1);
  center(2) = mass_center(2);
  pcl::getMaxDistance (*point_cloud, center, max_point);

   // calculate radius
  radius_vector(0) = center(0) - max_point(0);
  radius_vector(1) = center(1) - max_point(1);
  radius_vector(2) = center(2) - max_point(2);
  radius = sqrt(radius_vector(0)*radius_vector(0)+radius_vector(1)*radius_vector(1)+radius_vector(2)*radius_vector(2));

  // save results format r, x_notrans, y_notrans, z_notrans, x_trans, y_trans, z_trans
  // save results in cam sys
  coeffs.resize(7);
  coeffs[0] = radius;
  coeffs[1] = center(0);
  coeffs[2] = center(1);
  coeffs[3] = center(2);

  //Transform to OdoCoordinateSystem
  // result = TransformPointToOdo(center);
  // This transformation should not be happening here anyway!

  // save results in odo sys
  coeffs[4] = 0.;
  coeffs[5] = 0.;
  coeffs[6] = 0.;

  // Finally sets the calculated coeffs
  sphere->setModelCoefficients(coeffs);
}

template<class PointT>
void MomentOfInertiaObjectApproximator<PointT>::performFitting(
    boost::shared_ptr<CylinderModel> cylinder,
    const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud,
    Eigen::Vector3f mass_center,
    std::vector <Eigen::Vector3f> const& axes) {
  Eigen::VectorXf  coeffs;

  Eigen::Vector4f  max_point;
  Eigen::Vector4f  center;

  Eigen::Vector3f   result;

  float radius;
  float dist;
  std::vector <int> nearestPointIndex(1);
  std::vector <float> nearestPointSquaredDistance(1);
  float max_dist;
  Eigen::Vector3f      max_distance_vector;
  pcl::PointXYZ       searchPoint;
  pcl::KdTreeFLANN<pcl::PointXYZ>   kdtree;

  float     dist_y;
  float     dist_z;
  pcl::PointXYZ   center_xyz;
  Eigen::Vector3f   point_help;


  // find the point with maximum distance from the center
  center(0) = mass_center(0);
  center(1) = mass_center(1);
  center(2) = mass_center(2);
  pcl::getMaxDistance (*point_cloud, center, max_point);

  // calculate max_distance from center point
  max_distance_vector(0) = center(0) - max_point(0);
  max_distance_vector(1) = center(1) - max_point(1);
  max_distance_vector(2) = center(2) - max_point(2);
  max_dist = sqrt(max_distance_vector(0)*max_distance_vector(0)+max_distance_vector(1)*max_distance_vector(1)+max_distance_vector(2)*max_distance_vector(2));

  // create 1st searchPoint
  searchPoint.x = center(0) + max_dist * (axes.at(0))(0);
  searchPoint.y = center(1) + max_dist * (axes.at(0))(1);
  searchPoint.z = center(2) + max_dist * (axes.at(0))(2);

  // search nearest point in x-direction
  kdtree.setInputCloud(point_cloud);
  kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
  dist = sqrt((((point_cloud->at(nearestPointIndex[0]))).x-center(0))*((point_cloud->at(nearestPointIndex[0])).x-center(0)) + ((point_cloud->at(nearestPointIndex[0])).y-center(1))*((point_cloud->at(nearestPointIndex[0])).y-center(1)) + ((point_cloud->at(nearestPointIndex[0])).z-center(2))*((point_cloud->at(nearestPointIndex[0])).z-center(2)));

  // point1 in camera system
  // 0.75 to make sure that all points are inliers (consider the two hemispheres at the ends)
  point_help(0) = center(0) + 0.75*dist*(axes.at(0))(0);
  point_help(1) = center(1) + 0.75*dist*(axes.at(0))(1);
  point_help(2) = center(2) + 0.75*dist*(axes.at(0))(2);

  // save results format r, p1_notrans, p2_notrans, p1_trans, p2_trans
  // save p1_notrans
  coeffs.resize(13);
  coeffs[1] = point_help(0);
  coeffs[2] = point_help(1);
  coeffs[3] = point_help(2);

  // Transform to OdoCoordinateSystem
  // (Skip this -- shouldn't be a part of the detection...)
  // result = TransformPointToOdo(point_help);
  // save p1_trans
  coeffs[7] = 0.;
  coeffs[8] = 0.;
  coeffs[9] = 0.;

  // point2 in camera system
  // 0.75 to make sure that all points are inliers (consider the two hemispheres at the ends)
  point_help(0) = center(0) - 0.75*dist*(axes.at(0))(0);
  point_help(1) = center(1) - 0.75*dist*(axes.at(0))(1);
  point_help(2) = center(2) - 0.75*dist*(axes.at(0))(2);

  // save p2_notrans
  coeffs[4] = point_help(0);
  coeffs[5] = point_help(1);
  coeffs[6] = point_help(2);

  // Transform to OdoCoordinateSystem
  // result = TransformPointToOdo(point_help);

  // save p2_trans
  coeffs[10] = 0.;
  coeffs[11] = 0.;
  coeffs[12] = 0.;

  // search nearest point in +/- y-direction and +- z-direction and get min distance
  // + y-direction
  searchPoint.x = center(0) + max_dist/1.5 * (axes.at(1))(0);
  searchPoint.y = center(1) + max_dist/1.5 * (axes.at(1))(1);
  searchPoint.z = center(2) + max_dist/1.5 * (axes.at(1))(2);
  kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
  dist_y = sqrt((((point_cloud->at(nearestPointIndex[0]))).x-center(0))*((point_cloud->at(nearestPointIndex[0])).x-center(0)) + ((point_cloud->at(nearestPointIndex[0])).y-center(1))*((point_cloud->at(nearestPointIndex[0])).y-center(1)) + ((point_cloud->at(nearestPointIndex[0])).z-center(2))*((point_cloud->at(nearestPointIndex[0])).z-center(2)));

  // - y-direction
  searchPoint.x = center(0) - max_dist/1.5 * (axes.at(1))(0);
  searchPoint.y = center(1) - max_dist/1.5 * (axes.at(1))(1);
  searchPoint.z = center(2) - max_dist/1.5 * (axes.at(1))(2);
  kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
  dist = sqrt((((point_cloud->at(nearestPointIndex[0]))).x-center(0))*((point_cloud->at(nearestPointIndex[0])).x-center(0)) + ((point_cloud->at(nearestPointIndex[0])).y-center(1))*((point_cloud->at(nearestPointIndex[0])).y-center(1)) + ((point_cloud->at(nearestPointIndex[0])).z-center(2))*((point_cloud->at(nearestPointIndex[0])).z-center(2)));
  if (dist_y > dist)
  {
    dist_y = dist;
  }

  // + z-direction
  searchPoint.x = center(0) + max_dist/1.5 * (axes.at(2))(0);
  searchPoint.y = center(1) + max_dist/1.5 * (axes.at(2))(1);
  searchPoint.z = center(2) + max_dist/1.5 * (axes.at(2))(2);
  kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
  dist_z = sqrt((((point_cloud->at(nearestPointIndex[0]))).x-center(0))*((point_cloud->at(nearestPointIndex[0])).x-center(0)) + ((point_cloud->at(nearestPointIndex[0])).y-center(1))*((point_cloud->at(nearestPointIndex[0])).y-center(1)) + ((point_cloud->at(nearestPointIndex[0])).z-center(2))*((point_cloud->at(nearestPointIndex[0])).z-center(2)));

  // - z-direction
  searchPoint.x = center(0) - max_dist/1.5 * (axes.at(2))(0);
  searchPoint.y = center(1) - max_dist/1.5 * (axes.at(2))(1);
  searchPoint.z = center(2) - max_dist/1.5 * (axes.at(2))(2);
  kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
  dist = sqrt((((point_cloud->at(nearestPointIndex[0]))).x-center(0))*((point_cloud->at(nearestPointIndex[0])).x-center(0)) + ((point_cloud->at(nearestPointIndex[0])).y-center(1))*((point_cloud->at(nearestPointIndex[0])).y-center(1)) + ((point_cloud->at(nearestPointIndex[0])).z-center(2))*((point_cloud->at(nearestPointIndex[0])).z-center(2)));
  if (dist_z > dist)
  {
    dist_z = dist;
  }


  // calculate radius for a safety solution
  radius = sqrt(dist_y*dist_y + dist_z*dist_z);

  // save radius
  coeffs[0] = 0.9*radius;

  cylinder->setModelCoefficients(coeffs);
}

template<class PointT>
void MomentOfInertiaObjectApproximator<PointT>::performFitting(
    boost::shared_ptr<TriangleModel> triangle,
    const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud,
    Eigen::Vector3f mass_center,
    std::vector <Eigen::Vector3f> const& axes) {
  Eigen::VectorXf  coeffs;

  Eigen::Vector4f  max_point;
  Eigen::Vector4f  center;

  Eigen::Vector3f   result;

  float radius;
  float dist;
  std::vector <int> nearestPointIndex(1);
  std::vector <float> nearestPointSquaredDistance(1);
  float max_dist;
  Eigen::Vector3f      max_distance_vector;
  pcl::PointXYZ       searchPoint;
  pcl::KdTreeFLANN<pcl::PointXYZ>   kdtree;

  Eigen::Vector3f  triangle_vertex;
  float     dot_product;
  Eigen::Vector3f  cross_product;
  Eigen::Vector3f  cross_product_help;
  Eigen::Vector3f  help_vector;
  int     q;
  float    dist_1;
  float    dist_2;
  float    radius_1;
  float    radius_2;

  coeffs.resize(19);

  q=0;

  // find the point with maximum distance from the center
  center(0) = mass_center(0);
  center(1) = mass_center(1);
  center(2) = mass_center(2);
  pcl::getMaxDistance (*point_cloud, center, max_point);

  // calculate max_distance from center point
  max_distance_vector(0) = center(0) - max_point(0);
  max_distance_vector(1) = center(1) - max_point(1);
  max_distance_vector(2) = center(2) - max_point(2);
  max_dist = sqrt(max_distance_vector(0)*max_distance_vector(0)+max_distance_vector(1)*max_distance_vector(1)+max_distance_vector(2)*max_distance_vector(2));

  // calculate radius
  // create 1st searchPoint
  searchPoint.x = center(0) + max_dist/1.5 * (axes.at(2))(0);
  searchPoint.y = center(1) + max_dist/1.5 * (axes.at(2))(1);
  searchPoint.z = center(2) + max_dist/1.5 * (axes.at(2))(2);

  // search nearest point in z-direction
  kdtree.setInputCloud(point_cloud);
  kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
  radius_1 = sqrt((((point_cloud->at(nearestPointIndex[0]))).x-center(0))*((point_cloud->at(nearestPointIndex[0])).x-center(0)) + ((point_cloud->at(nearestPointIndex[0])).y-center(1))*((point_cloud->at(nearestPointIndex[0])).y-center(1)) + ((point_cloud->at(nearestPointIndex[0])).z-center(2))*((point_cloud->at(nearestPointIndex[0])).z-center(2)));
  dist_1 = sqrt((point_cloud->at(nearestPointIndex[0])).x*(point_cloud->at(nearestPointIndex[0])).x + (point_cloud->at(nearestPointIndex[0])).y*(point_cloud->at(nearestPointIndex[0])).y + (point_cloud->at(nearestPointIndex[0])).z*(point_cloud->at(nearestPointIndex[0])).z);

  // create 2nd searchPoint
  searchPoint.x = center(0) - max_dist/1.5 * (axes.at(2))(0);
  searchPoint.y = center(1) - max_dist/1.5 * (axes.at(2))(1);
  searchPoint.z = center(2) - max_dist/1.5 * (axes.at(2))(2);

  // search nearest point in -z-direction
  kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
  radius_2 = sqrt(((point_cloud->at(nearestPointIndex[0])).x-center(0))*((point_cloud->at(nearestPointIndex[0])).x-center(0)) + ((point_cloud->at(nearestPointIndex[0])).y-center(1))*((point_cloud->at(nearestPointIndex[0])).y-center(1)) + ((point_cloud->at(nearestPointIndex[0])).z-center(2))*((point_cloud->at(nearestPointIndex[0])).z-center(2)));
  dist_2 = sqrt((point_cloud->at(nearestPointIndex[0])).x*(point_cloud->at(nearestPointIndex[0])).x + (point_cloud->at(nearestPointIndex[0])).y*(point_cloud->at(nearestPointIndex[0])).y + (point_cloud->at(nearestPointIndex[0])).z*(point_cloud->at(nearestPointIndex[0])).z);

  if (dist_1 > dist_2)
  {
    radius = radius_2;
    q=1;
  }
  else
  {
    radius = radius_1;
  }

  if (radius < 40)
  {
    // save radius
    // coeffs[0] = 40.0;
    coeffs[0] = radius;
  }
  else
  {
    // save radius
    coeffs[0] = radius;
  }

  // calculate 1st vertex point
  // 0.7 to make sure that all points are inliers, normally: 0.5
  if(q==1)
  {
    triangle_vertex(0) = max_point(0) + 0.7*radius*(axes.at(2))(0);
    triangle_vertex(1) = max_point(1) + 0.7*radius*(axes.at(2))(1);
    triangle_vertex(2) = max_point(2) + 0.7*radius*(axes.at(2))(2);
  }
  else
  {
    triangle_vertex(0) = max_point(0) - 0.7*radius*(axes.at(2))(0);
    triangle_vertex(1) = max_point(1) - 0.7*radius*(axes.at(2))(1);
    triangle_vertex(2) = max_point(2) - 0.7*radius*(axes.at(2))(2);
  }

  // move triangle_vertex 0.6*radius to the center, normally: 0.5
  dist_1 = sqrt((center(0)-triangle_vertex(0))*(center(0)-triangle_vertex(0))+(center(1)-triangle_vertex(1))*(center(1)-triangle_vertex(1))+(center(2)-triangle_vertex(2))*(center(2)-triangle_vertex(2)));
  triangle_vertex(0) = triangle_vertex(0) + 0.6*radius*((center(0)-triangle_vertex(0))/dist_1);
  triangle_vertex(1) = triangle_vertex(1) + 0.6*radius*((center(1)-triangle_vertex(1))/dist_1);
  triangle_vertex(2) = triangle_vertex(2) + 0.6*radius*((center(2)-triangle_vertex(2))/dist_1);

  // save results format r, p1_notrans, p2_notrans, p3_notrans, p1_trans, p2_trans, p3_trans
  // save p1_notrans
  coeffs[1] = triangle_vertex(0);
  coeffs[2] = triangle_vertex(1);
  coeffs[3] = triangle_vertex(2);

  //Transform to WordCoordinateSystem
  // result = TransformPointToOdo(triangle_vertex);

  // save p1_trans
  coeffs[10] = 0;
  coeffs[11] = 0;
  coeffs[12] = 0;

  // first point in local coordinate system
  triangle_vertex(0) = max_point(0) - center(0);
  triangle_vertex(1) = max_point(1) - center(1);
  triangle_vertex(2) = max_point(2) - center(2);

  // calculate 2nd point
  // rotate triangle_vertex 120 degrees about z-axis (local coordinate system)
  help_vector(0) = (axes.at(2))(0);
  help_vector(1) = (axes.at(2))(1);
  help_vector(2) = (axes.at(2))(2);

  dot_product = triangle_vertex.dot(help_vector);

  cross_product_help = help_vector.cross(triangle_vertex);

  cross_product = cross_product_help.cross(help_vector);

  searchPoint.x = help_vector(0)*dot_product - 0.5*cross_product(0) + 0.86603*cross_product_help(0) + center(0);
  searchPoint.y = help_vector(1)*dot_product - 0.5*cross_product(1) + 0.86603*cross_product_help(1) + center(1);
  searchPoint.z = help_vector(2)*dot_product - 0.5*cross_product(2) + 0.86603*cross_product_help(2) + center(2);

  searchPoint.x = searchPoint.x - (center(0) - searchPoint.x);
  searchPoint.y = searchPoint.y - (center(1) - searchPoint.y);
  searchPoint.z = searchPoint.z - (center(2) - searchPoint.z);

  // find nearest point
  kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);

  if(q==1)
  {
    // 0.7 to make sure that all points are inliers, normally: 0.5
    triangle_vertex(0) = (point_cloud->at(nearestPointIndex[0])).x + 0.7*radius*(axes.at(2))(0);
    triangle_vertex(1) = (point_cloud->at(nearestPointIndex[0])).y + 0.7*radius*(axes.at(2))(1);
    triangle_vertex(2) = (point_cloud->at(nearestPointIndex[0])).z + 0.7*radius*(axes.at(2))(2);
  }
  else
  {
    // 0.7 to make sure that all points are inliers, normally: 0.5
    triangle_vertex(0) = (point_cloud->at(nearestPointIndex[0])).x - 0.7*radius*(axes.at(2))(0);
    triangle_vertex(1) = (point_cloud->at(nearestPointIndex[0])).y - 0.7*radius*(axes.at(2))(1);
    triangle_vertex(2) = (point_cloud->at(nearestPointIndex[0])).z - 0.7*radius*(axes.at(2))(2);
  }

  // move triangle_vertex 0.6*radius to the center
  dist_1 = sqrt((center(0)-triangle_vertex(0))*(center(0)-triangle_vertex(0))+(center(1)-triangle_vertex(1))*(center(1)-triangle_vertex(1))+(center(2)-triangle_vertex(2))*(center(2)-triangle_vertex(2)));
  triangle_vertex(0) = triangle_vertex(0) + 0.6*radius*((center(0)-triangle_vertex(0))/dist_1);
  triangle_vertex(1) = triangle_vertex(1) + 0.6*radius*((center(1)-triangle_vertex(1))/dist_1);
  triangle_vertex(2) = triangle_vertex(2) + 0.6*radius*((center(2)-triangle_vertex(2))/dist_1);

  // save p2_notrans
  coeffs[4] = triangle_vertex(0);
  coeffs[5] = triangle_vertex(1);
  coeffs[6] = triangle_vertex(2);

  // Transform to WordCoordinateSystem
  // result = TransformPointToOdo(triangle_vertex);

  // save p2_trans
  coeffs[13] = 0;
  coeffs[14] = 0;
  coeffs[15] = 0;


  // calculate 3rd point
  // get center between the two calculated vertexes
  searchPoint.x = max_point(0) + 0.5*((point_cloud->at(nearestPointIndex[0])).x - max_point(0));
  searchPoint.y = max_point(1) + 0.5*((point_cloud->at(nearestPointIndex[0])).y - max_point(1));
  searchPoint.z = max_point(2) + 0.5*((point_cloud->at(nearestPointIndex[0])).z - max_point(2));

  // get new search point
  searchPoint.x = searchPoint.x + 5*(center(0) - searchPoint.x);
  searchPoint.y = searchPoint.y + 5*(center(1) - searchPoint.y);
  searchPoint.z = searchPoint.z + 5*(center(2) - searchPoint.z);


  // find nearest point
  kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);

  if(q==1)
  {
    // 0.7 to make sure that all points are inliers, normally: 0.5
    triangle_vertex(0) = (point_cloud->at(nearestPointIndex[0])).x + 0.7*radius*(axes.at(2))(0);
    triangle_vertex(1) = (point_cloud->at(nearestPointIndex[0])).y + 0.7*radius*(axes.at(2))(1);
    triangle_vertex(2) = (point_cloud->at(nearestPointIndex[0])).z + 0.7*radius*(axes.at(2))(2);
  }
  else
  {
    // 0.7 to make sure that all points are inliers, normally: 0.5
    triangle_vertex(0) = (point_cloud->at(nearestPointIndex[0])).x - 0.7*radius*(axes.at(2))(0);
    triangle_vertex(1) = (point_cloud->at(nearestPointIndex[0])).y - 0.7*radius*(axes.at(2))(1);
    triangle_vertex(2) = (point_cloud->at(nearestPointIndex[0])).z - 0.7*radius*(axes.at(2))(2);
  }

  // move triangle_vertex 0.6*radius to the center, normally: 0.5
  dist_1 = sqrt((center(0)-triangle_vertex(0))*(center(0)-triangle_vertex(0))+(center(1)-triangle_vertex(1))*(center(1)-triangle_vertex(1))+(center(2)-triangle_vertex(2))*(center(2)-triangle_vertex(2)));
  triangle_vertex(0) = triangle_vertex(0) + 0.6*radius*((center(0)-triangle_vertex(0))/dist_1);
  triangle_vertex(1) = triangle_vertex(1) + 0.6*radius*((center(1)-triangle_vertex(1))/dist_1);
  triangle_vertex(2) = triangle_vertex(2) + 0.6*radius*((center(2)-triangle_vertex(2))/dist_1);

  // save p3_notrans
  coeffs[7] = triangle_vertex(0);
  coeffs[8] = triangle_vertex(1);
  coeffs[9] = triangle_vertex(2);

  // Transform to WordCoordinateSystem
  // result = TransformPointToOdo(triangle_vertex);

  // save p3_trans
  coeffs[16] = 0;
  coeffs[17] = 0;
  coeffs[18] = 0;

  triangle->setModelCoefficients(coeffs);
}

} // namespace lepp

#endif
