/**
 *  \file
 *  \brief     MomentOfInertiaIdentification.cpp
 *  \author    Fabian Braeu
 *  \date      2014
 *
 *  \copyright Institute of Applied Mechanics, Technical University of Munich
 */

#include "MomentOfInertiaIdentification.hpp"
#include <pcl/common/common.h>
#include "../BaseObstacleDetector.hpp"

// Swith on for PointClouds with no estimated points
#define FITTING_NO_ESTIMATION


/*
 * Updates results.
 */
    void MomentOfInertiaIdentification::update() {

        PointCloudPtrListPtr clusters = detector_->getSegmentationInstance()->getClusterList();
        ObjectModelPtr tmp;

	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;			// Instance for moment of inertia estimation


	// variable for saving MajorAxis
	std::vector <Eigen::Vector3f> axis_help;

        // clear model list
        getModelList()->clear();

	// clear major axis list
	getMajorAxisList()->clear();

	// clear mass_center_list
	getMassCenterList()->clear();

	// variables for saving EigenValues
	float major_value, middle_value, minor_value;

	pcl::PointXYZ min_pt;
	pcl::PointXYZ max_pt;

        // for all clusters, add new sphere model to model list
        for (int i = 0; i < clusters->size(); ++i) {

	  // compute moment of inertia
          feature_extractor.setInputCloud (clusters->at(i));
	  feature_extractor.compute ();


	 // extraction of EigenValues, EigenVectors and MassCenter
	  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
	  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
//	  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
//	  feature_extractor.getMassCenter (mass_center);

#ifdef FITTING_NO_ESTIMATION
	 // Estimate center point for each object in clusterlist and save as mass_center
	  pcl::getMinMax3D (*(clusters-> at(i)), min_pt, max_pt);
	  mass_center(0) = (max_pt.x + min_pt.x)/2;
	  mass_center(1) = 1.02*((max_pt.y + min_pt.y)/2);
	  mass_center(2) = 1.02*((max_pt.z + min_pt.z)/2);
#endif


	 // save features in lists
	  axis_help.clear();
	  axis_help.push_back(major_vector);
	  axis_help.push_back(middle_vector);
	  axis_help.push_back(minor_vector);
	  getMajorAxisList()->push_back (axis_help);
	  getMassCenterList()->push_back (mass_center);


	// Decides wether to use sphere, cylinder or triangle to aproximate the objects
	  if ((middle_value/major_value > 0.60) && (minor_value/major_value > 0.10))
	  {
	    //use spheres
	      tmp = m_preallocator_sphere.newInstance();
	      getModelList()->push_back(tmp);
        std::cout << "Sphere" << std::endl;
	  }

	  else if (middle_value/major_value < 0.25)
	  {
	    //use cylinders
	      tmp = m_preallocator_cylinder.newInstance();
	      getModelList()->push_back(tmp);
        std::cout << "Cylinder" << std::endl;
	  }

	  else
	  {
	    //use triangles
	      tmp = m_preallocator_triangle.newInstance();
	      getModelList()->push_back(tmp);
        std::cout << "Triangle" << std::endl;
	  }


#ifdef VERBOSE
//	  std::cout <<"Massenträgheitsmomente"      <<major_value  <<std::endl    <<middle_value  <<std::endl    <<minor_value    <<std::endl;
//	  std::cout <<"min point"     <<min_point_AABB.x      <<min_point_AABB.y      <<min_point_AABB.z       <<std::endl;
//	  std::cout <<"max point"     <<max_point_AABB.x      <<max_point_AABB.y      <<max_point_AABB.z       <<std::endl;
//	  std::cout <<"y_achse"      <<middle_vector(0)  <<std::endl    <<middle_vector(1)  <<std::endl    <<middle_vector(2)    <<std::endl;
//	  visualize();
#endif


        }

    }
