/**
 *  \file
 *  \brief     MomentOfInertiaIdentification.hpp
 *  \author    Fabian Braeu
 *  \date      2014
 *
 *  \copyright Institute of Applied Mechanics, Technical University of Munich
 */

#ifndef MOMENT_OF_INERTIA_IDENTIFICATION_HPP
#define MOMENT_OF_INERTIA_IDENTIFICATION_HPP

#include "IdentificationAlgorithm.hpp"
#include "models/SphereModel.hpp"
#include "models/CylinderModel.hpp"
#include "models/TriangleModel.hpp"
#include "moment_of_inertia_estimation.hpp"

/**
 * Decides wether to use spheres, cylinders or triangles
 */
class MomentOfInertiaIdentification : public IdentificationAlgorithm {

private:

    Preallocator<SphereModel>     m_preallocator_sphere;

    Preallocator<CylinderModel>   m_preallocator_cylinder;

    Preallocator<TriangleModel>   m_preallocator_triangle;

    // vectors for saving major axis and mass_center
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    // min/max points of each pointcloud in clusterlist
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;

    int q;				// counter for axis-names

public:


    /**
     * Initialize algorithm.
     * This method is executed before the first call to update().
     */
    void init() {

        // init preallocator for model objects spheres
        m_preallocator_sphere.allocate(MODEL_LIST_PREALLOC);

	// init preallocator for model objects cylinders
        m_preallocator_cylinder.allocate(MODEL_LIST_PREALLOC);

	// init preallocator for model objects triangles
        m_preallocator_triangle.allocate(MODEL_LIST_PREALLOC);

	// counter starts with 0
	q = 0;

    }

    /**
     * Updates results.
     * Initiates new execution of the algorithm. This method is called by the ObjectManager.
     */
    void update();

};


#endif
