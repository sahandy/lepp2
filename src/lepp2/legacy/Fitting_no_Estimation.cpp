/**
 *  \file
 *  \brief     Fitting_no_Estimation.cpp
 *  \author    Fabian Braeu
 *  \date      2014
 *
 *  \copyright Institute of Applied Mechanics, Technical University of Munich
 */

#include "Fitting_no_Estimation.hpp"
#include "models/ObjectModel.hpp"
#include "../BaseObstacleDetector.hpp"
#include <pcl/kdtree/kdtree_flann.h>


#define TRANSFORMATION

   /**
    * Transformation into odo coordinate system
    * return: point in odo coordinate system
    */
Eigen::Vector3f TransformPointToOdo(Eigen::VectorXf point_cam)
{

#ifdef TRANSFORMATION
    Eigen::Vector3f point_world;

    Eigen::Vector3f point_world_2;

    Eigen::Matrix3f     A_odo_cam;
    Eigen::Vector3f     r_odo_cam;


    // std::cout<<"r_odo_cam "<<r_odo_cam<<std::endl;
    // std::cout<<"A_odo_cam "<<A_odo_cam<<std::endl;
/*
    std::cout<<"point cam "<<point_cam(0)
       <<" "<<point_cam(1)<<" "
       <<point_cam(2)<<" "<<std::endl;
*/
    point_world(0) = r_odo_cam(0) + A_odo_cam(0,0)*point_cam(0) - A_odo_cam(0,1)*point_cam(1) - A_odo_cam(0,2)*point_cam(2);
    point_world(1) = r_odo_cam(1) + A_odo_cam(1,0)*point_cam(0) - A_odo_cam(1,1)*point_cam(1) - A_odo_cam(1,2)*point_cam(2);
    point_world(2) = r_odo_cam(2) + A_odo_cam(2,0)*point_cam(0) - A_odo_cam(2,1)*point_cam(1) - A_odo_cam(2,2)*point_cam(2);


    point_world_2(0) = r_odo_cam(0) + A_odo_cam(0,0)*point_cam(0) - A_odo_cam(1,0)*point_cam(1) + A_odo_cam(2,0)*point_cam(2);
    point_world_2(1) = r_odo_cam(1) + A_odo_cam(0,1)*point_cam(0) - A_odo_cam(1,1)*point_cam(1) - A_odo_cam(2,1)*point_cam(2);
    point_world_2(2) = r_odo_cam(2) + A_odo_cam(0,2)*point_cam(0) - A_odo_cam(1,2)*point_cam(1) - A_odo_cam(2,2)*point_cam(2);

/*
    std::cout<<"point world "<<point_world(0)
       <<" "<<point_world(1)<<" "
       <<point_world(2)<<" "<<std::endl;

    std::cout<<"point world_2 "<<point_world_2(0)
       <<" "<<point_world_2(1)<<" "
       <<point_world_2(2)<<" "<<std::endl;
*/
    return point_world;
#endif

#ifndef TRANSFORMATION
    point_world(0) = 0;
    point_world(1) = 0;
    point_world(2) = 0;

    return point_world;
#endif
}


/*
 * Recalc results
 */
void Fitting_no_Estimation::update() {

    // common parameters
    PointCloudPtrListPtr    clusters = detector_->getSegmentationInstance()->getClusterList();
    ObjectModelPtrListPtr   models = detector_->getIdentificationInstance()->getModelList();
    boost::shared_ptr <std::vector <std::vector <Eigen::Vector3f> > > major_axis = detector_->getIdentificationInstance()->getMajorAxisList();
    boost::shared_ptr < std::vector <Eigen::Vector3f> > mass_centers = detector_->getIdentificationInstance()->getMassCenterList();

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

    // parameters for sphere fitting
    Eigen::Vector3f  radius_vector;

    // parameters for cylinder fitting
    float     dist_y;
    float     dist_z;
    pcl::PointXYZ   center_xyz;
    Eigen::Vector3f   point_help;

    // parameters for triangle fitting
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


    // for all clusters in the list
    #pragma omp parallel for
    for (int c = 0; c < clusters->size(); ++c) {

        // sphere fitting algorithm
        if (models->at(c)->getType() == "sphere") {

           // find the point with maximum distance from the center
      center(0) = (mass_centers->at(c))(0);
      center(1) = (mass_centers->at(c))(1);
      center(2) = (mass_centers->at(c))(2);
      pcl::getMaxDistance (*(clusters->at(c)), center, max_point);

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
      result = TransformPointToOdo(center);

           // save results in odo sys
            coeffs[4] = result(0);
            coeffs[5] = result(1);
            coeffs[6] = result(2);


        }

        // cylinder fitting algorithm
        else if (models->at(c)->getType() == "cylinder") {

           // find the point with maximum distance from the center
      center(0) = (mass_centers->at(c))(0);
      center(1) = (mass_centers->at(c))(1);
      center(2) = (mass_centers->at(c))(2);
      pcl::getMaxDistance (*(clusters->at(c)), center, max_point);

     // calculate max_distance from center point
      max_distance_vector(0) = center(0) - max_point(0);
      max_distance_vector(1) = center(1) - max_point(1);
      max_distance_vector(2) = center(2) - max_point(2);
      max_dist = sqrt(max_distance_vector(0)*max_distance_vector(0)+max_distance_vector(1)*max_distance_vector(1)+max_distance_vector(2)*max_distance_vector(2));

     // create 1st searchPoint
      searchPoint.x = center(0) + max_dist * (major_axis->at(c).at(0))(0);
      searchPoint.y = center(1) + max_dist * (major_axis->at(c).at(0))(1);
      searchPoint.z = center(2) + max_dist * (major_axis->at(c).at(0))(2);

     // search nearest point in x-direction
      kdtree.setInputCloud(clusters->at(c));
      kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
      dist = sqrt((((clusters->at(c)->at(nearestPointIndex[0]))).x-center(0))*((clusters->at(c)->at(nearestPointIndex[0])).x-center(0)) + ((clusters->at(c)->at(nearestPointIndex[0])).y-center(1))*((clusters->at(c)->at(nearestPointIndex[0])).y-center(1)) + ((clusters->at(c)->at(nearestPointIndex[0])).z-center(2))*((clusters->at(c)->at(nearestPointIndex[0])).z-center(2)));

      // point1 in camera system
      // 0.75 to make sure that all points are inliers (consider the two hemispheres at the ends)
      point_help(0) = center(0) + 0.75*dist*(major_axis->at(c).at(0))(0);
      point_help(1) = center(1) + 0.75*dist*(major_axis->at(c).at(0))(1);
      point_help(2) = center(2) + 0.75*dist*(major_axis->at(c).at(0))(2);

      // save results format r, p1_notrans, p2_notrans, p1_trans, p2_trans
      // save p1_notrans
      coeffs.resize(13);
            coeffs[1] = point_help(0);
            coeffs[2] = point_help(1);
            coeffs[3] = point_help(2);

      // Transform to OdoCoordinateSystem
      result = TransformPointToOdo(point_help);

      // save p1_trans
      coeffs[7] = result(0);
            coeffs[8] = result(1);
            coeffs[9] = result(2);

      // point2 in camera system
      // 0.75 to make sure that all points are inliers (consider the two hemispheres at the ends)
      point_help(0) = center(0) - 0.75*dist*(major_axis->at(c).at(0))(0);
      point_help(1) = center(1) - 0.75*dist*(major_axis->at(c).at(0))(1);
      point_help(2) = center(2) - 0.75*dist*(major_axis->at(c).at(0))(2);

      // save p2_notrans
      coeffs[4] = point_help(0);
            coeffs[5] = point_help(1);
            coeffs[6] = point_help(2);

      // Transform to OdoCoordinateSystem
      result = TransformPointToOdo(point_help);

      // save p2_trans
            coeffs[10] = result(0);
            coeffs[11] = result(1);
            coeffs[12] = result(2);


     // search nearest point in +/- y-direction and +- z-direction and get min distance
     // + y-direction
      searchPoint.x = center(0) + max_dist/1.5 * (major_axis->at(c).at(1))(0);
      searchPoint.y = center(1) + max_dist/1.5 * (major_axis->at(c).at(1))(1);
      searchPoint.z = center(2) + max_dist/1.5 * (major_axis->at(c).at(1))(2);
      kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
      dist_y = sqrt((((clusters->at(c)->at(nearestPointIndex[0]))).x-center(0))*((clusters->at(c)->at(nearestPointIndex[0])).x-center(0)) + ((clusters->at(c)->at(nearestPointIndex[0])).y-center(1))*((clusters->at(c)->at(nearestPointIndex[0])).y-center(1)) + ((clusters->at(c)->at(nearestPointIndex[0])).z-center(2))*((clusters->at(c)->at(nearestPointIndex[0])).z-center(2)));

          // - y-direction
      searchPoint.x = center(0) - max_dist/1.5 * (major_axis->at(c).at(1))(0);
      searchPoint.y = center(1) - max_dist/1.5 * (major_axis->at(c).at(1))(1);
      searchPoint.z = center(2) - max_dist/1.5 * (major_axis->at(c).at(1))(2);
      kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
      dist = sqrt((((clusters->at(c)->at(nearestPointIndex[0]))).x-center(0))*((clusters->at(c)->at(nearestPointIndex[0])).x-center(0)) + ((clusters->at(c)->at(nearestPointIndex[0])).y-center(1))*((clusters->at(c)->at(nearestPointIndex[0])).y-center(1)) + ((clusters->at(c)->at(nearestPointIndex[0])).z-center(2))*((clusters->at(c)->at(nearestPointIndex[0])).z-center(2)));
            if (dist_y > dist)
      {
        dist_y = dist;
      }

     // + z-direction
      searchPoint.x = center(0) + max_dist/1.5 * (major_axis->at(c).at(2))(0);
      searchPoint.y = center(1) + max_dist/1.5 * (major_axis->at(c).at(2))(1);
      searchPoint.z = center(2) + max_dist/1.5 * (major_axis->at(c).at(2))(2);
      kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
      dist_z = sqrt((((clusters->at(c)->at(nearestPointIndex[0]))).x-center(0))*((clusters->at(c)->at(nearestPointIndex[0])).x-center(0)) + ((clusters->at(c)->at(nearestPointIndex[0])).y-center(1))*((clusters->at(c)->at(nearestPointIndex[0])).y-center(1)) + ((clusters->at(c)->at(nearestPointIndex[0])).z-center(2))*((clusters->at(c)->at(nearestPointIndex[0])).z-center(2)));

    // - z-direction
      searchPoint.x = center(0) - max_dist/1.5 * (major_axis->at(c).at(2))(0);
      searchPoint.y = center(1) - max_dist/1.5 * (major_axis->at(c).at(2))(1);
      searchPoint.z = center(2) - max_dist/1.5 * (major_axis->at(c).at(2))(2);
      kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
      dist = sqrt((((clusters->at(c)->at(nearestPointIndex[0]))).x-center(0))*((clusters->at(c)->at(nearestPointIndex[0])).x-center(0)) + ((clusters->at(c)->at(nearestPointIndex[0])).y-center(1))*((clusters->at(c)->at(nearestPointIndex[0])).y-center(1)) + ((clusters->at(c)->at(nearestPointIndex[0])).z-center(2))*((clusters->at(c)->at(nearestPointIndex[0])).z-center(2)));
            if (dist_z > dist)
      {
        dist_z = dist;
      }


     // calculate radius for a safety solution
        radius = sqrt(dist_y*dist_y + dist_z*dist_z);

     // save radius
        coeffs[0] = 0.9*radius;


    // triangle fitting algorithm
        } else {

    coeffs.resize(19);

    q=0;

    // find the point with maximum distance from the center
      center(0) = (mass_centers->at(c))(0);
      center(1) = (mass_centers->at(c))(1);
      center(2) = (mass_centers->at(c))(2);
      pcl::getMaxDistance (*(clusters->at(c)), center, max_point);

    // calculate max_distance from center point
      max_distance_vector(0) = center(0) - max_point(0);
      max_distance_vector(1) = center(1) - max_point(1);
      max_distance_vector(2) = center(2) - max_point(2);
      max_dist = sqrt(max_distance_vector(0)*max_distance_vector(0)+max_distance_vector(1)*max_distance_vector(1)+max_distance_vector(2)*max_distance_vector(2));


  // calculate radius
    // create 1st searchPoint
      searchPoint.x = center(0) + max_dist/1.5 * (major_axis->at(c).at(2))(0);
      searchPoint.y = center(1) + max_dist/1.5 * (major_axis->at(c).at(2))(1);
      searchPoint.z = center(2) + max_dist/1.5 * (major_axis->at(c).at(2))(2);

    // search nearest point in z-direction
      kdtree.setInputCloud(clusters->at(c));
      kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
      radius_1 = sqrt((((clusters->at(c)->at(nearestPointIndex[0]))).x-center(0))*((clusters->at(c)->at(nearestPointIndex[0])).x-center(0)) + ((clusters->at(c)->at(nearestPointIndex[0])).y-center(1))*((clusters->at(c)->at(nearestPointIndex[0])).y-center(1)) + ((clusters->at(c)->at(nearestPointIndex[0])).z-center(2))*((clusters->at(c)->at(nearestPointIndex[0])).z-center(2)));
      dist_1 = sqrt((clusters->at(c)->at(nearestPointIndex[0])).x*(clusters->at(c)->at(nearestPointIndex[0])).x + (clusters->at(c)->at(nearestPointIndex[0])).y*(clusters->at(c)->at(nearestPointIndex[0])).y + (clusters->at(c)->at(nearestPointIndex[0])).z*(clusters->at(c)->at(nearestPointIndex[0])).z);

    // create 2nd searchPoint
      searchPoint.x = center(0) - max_dist/1.5 * (major_axis->at(c).at(2))(0);
      searchPoint.y = center(1) - max_dist/1.5 * (major_axis->at(c).at(2))(1);
      searchPoint.z = center(2) - max_dist/1.5 * (major_axis->at(c).at(2))(2);

    // search nearest point in -z-direction
      kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);
      radius_2 = sqrt(((clusters->at(c)->at(nearestPointIndex[0])).x-center(0))*((clusters->at(c)->at(nearestPointIndex[0])).x-center(0)) + ((clusters->at(c)->at(nearestPointIndex[0])).y-center(1))*((clusters->at(c)->at(nearestPointIndex[0])).y-center(1)) + ((clusters->at(c)->at(nearestPointIndex[0])).z-center(2))*((clusters->at(c)->at(nearestPointIndex[0])).z-center(2)));
      dist_2 = sqrt((clusters->at(c)->at(nearestPointIndex[0])).x*(clusters->at(c)->at(nearestPointIndex[0])).x + (clusters->at(c)->at(nearestPointIndex[0])).y*(clusters->at(c)->at(nearestPointIndex[0])).y + (clusters->at(c)->at(nearestPointIndex[0])).z*(clusters->at(c)->at(nearestPointIndex[0])).z);

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
        coeffs[0] = 40.0;
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
        triangle_vertex(0) = max_point(0) + 0.7*radius*(major_axis->at(c).at(2))(0);
        triangle_vertex(1) = max_point(1) + 0.7*radius*(major_axis->at(c).at(2))(1);
        triangle_vertex(2) = max_point(2) + 0.7*radius*(major_axis->at(c).at(2))(2);
      }
      else
      {
        triangle_vertex(0) = max_point(0) - 0.7*radius*(major_axis->at(c).at(2))(0);
        triangle_vertex(1) = max_point(1) - 0.7*radius*(major_axis->at(c).at(2))(1);
        triangle_vertex(2) = max_point(2) - 0.7*radius*(major_axis->at(c).at(2))(2);
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
      result = TransformPointToOdo(triangle_vertex);

    // save p1_trans
            coeffs[10] = result(0);
            coeffs[11] = result(1);
            coeffs[12] = result(2);

    // first point in local coordinate system
      triangle_vertex(0) = max_point(0) - center(0);
      triangle_vertex(1) = max_point(1) - center(1);
      triangle_vertex(2) = max_point(2) - center(2);

      // calculate 2nd point
    // rotate triangle_vertex 120 degrees about z-axis (local coordinate system)
      help_vector(0) = (major_axis->at(c).at(2))(0);
      help_vector(1) = (major_axis->at(c).at(2))(1);
      help_vector(2) = (major_axis->at(c).at(2))(2);

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
        triangle_vertex(0) = (clusters->at(c)->at(nearestPointIndex[0])).x + 0.7*radius*(major_axis->at(c).at(2))(0);
        triangle_vertex(1) = (clusters->at(c)->at(nearestPointIndex[0])).y + 0.7*radius*(major_axis->at(c).at(2))(1);
        triangle_vertex(2) = (clusters->at(c)->at(nearestPointIndex[0])).z + 0.7*radius*(major_axis->at(c).at(2))(2);
      }
      else
      {
        // 0.7 to make sure that all points are inliers, normally: 0.5
        triangle_vertex(0) = (clusters->at(c)->at(nearestPointIndex[0])).x - 0.7*radius*(major_axis->at(c).at(2))(0);
        triangle_vertex(1) = (clusters->at(c)->at(nearestPointIndex[0])).y - 0.7*radius*(major_axis->at(c).at(2))(1);
        triangle_vertex(2) = (clusters->at(c)->at(nearestPointIndex[0])).z - 0.7*radius*(major_axis->at(c).at(2))(2);
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
      result = TransformPointToOdo(triangle_vertex);

    // save p2_trans
            coeffs[13] = result(0);
            coeffs[14] = result(1);
            coeffs[15] = result(2);


      // calculate 3rd point
    // get center between the two calculated vertexes
      searchPoint.x = max_point(0) + 0.5*((clusters->at(c)->at(nearestPointIndex[0])).x - max_point(0));
      searchPoint.y = max_point(1) + 0.5*((clusters->at(c)->at(nearestPointIndex[0])).y - max_point(1));
      searchPoint.z = max_point(2) + 0.5*((clusters->at(c)->at(nearestPointIndex[0])).z - max_point(2));

    // get new search point
      searchPoint.x = searchPoint.x + 5*(center(0) - searchPoint.x);
      searchPoint.y = searchPoint.y + 5*(center(1) - searchPoint.y);
      searchPoint.z = searchPoint.z + 5*(center(2) - searchPoint.z);


    // find nearest point
      kdtree.nearestKSearch(searchPoint, 1, nearestPointIndex, nearestPointSquaredDistance);

      if(q==1)
      {
        // 0.7 to make sure that all points are inliers, normally: 0.5
        triangle_vertex(0) = (clusters->at(c)->at(nearestPointIndex[0])).x + 0.7*radius*(major_axis->at(c).at(2))(0);
        triangle_vertex(1) = (clusters->at(c)->at(nearestPointIndex[0])).y + 0.7*radius*(major_axis->at(c).at(2))(1);
        triangle_vertex(2) = (clusters->at(c)->at(nearestPointIndex[0])).z + 0.7*radius*(major_axis->at(c).at(2))(2);
      }
      else
      {
        // 0.7 to make sure that all points are inliers, normally: 0.5
        triangle_vertex(0) = (clusters->at(c)->at(nearestPointIndex[0])).x - 0.7*radius*(major_axis->at(c).at(2))(0);
        triangle_vertex(1) = (clusters->at(c)->at(nearestPointIndex[0])).y - 0.7*radius*(major_axis->at(c).at(2))(1);
        triangle_vertex(2) = (clusters->at(c)->at(nearestPointIndex[0])).z - 0.7*radius*(major_axis->at(c).at(2))(2);
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
      result = TransformPointToOdo(triangle_vertex);

  // save p3_trans
            coeffs[16] = result(0);
            coeffs[17] = result(1);
            coeffs[18] = result(2);

        }


        // store model coefficients
       models->at(c)->setModelCoefficients(coeffs);

    }

}
