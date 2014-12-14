/**
 *  \file
 *  \brief     IdentificationAlgorithm.hpp
 *  \author    Felix Sygulla
 *  \date      2014
 *
 *  \copyright Institute of Applied Mechanics, Technical University of Munich
 */

#ifndef IDENTIFICATION_ALGORITHM_HPP
#define IDENTIFICATION_ALGORITHM_HPP

#include "Algorithm.hpp"
#include "models/ObjectModel.hpp"
#include "Preallocator.hpp"

template<class PointT>
class BaseObstacleDetector;

/**
 * Number of models, for which memory will be preallocated
 * to speed up identification.
 */
#define MODEL_LIST_PREALLOC   20

/**
 * Superclass for the identification algorithms.
 * Implements basic interfaces for integration into ObjectManager.
 * All identification algorithm implementations must be derived from this class.
 */
class IdentificationAlgorithm : public Algorithm {

private:

   ObjectModelPtrListPtr        m_models;

   boost::shared_ptr <std::vector <std::vector < Eigen::Vector3f > > >  major_axis_list;			// for saving major axis in a list for each cluster

   boost::shared_ptr <std::vector <Eigen::Vector3f> > mass_center_list;					// for saving mass_center in a list for each cluster

public:
    // TODO Quite an ugly way to get rid of the singleton... Algorithms
    //      hold references to the parent detector.
    //      In the future, the algorithms must make sure to work without having
    //      to reference their containers, but completely self-contained.
    boost::shared_ptr<BaseObstacleDetector<PointType> > detector_;

    /**
     * Default constructor.
     * Memory preallocation.
     */
    IdentificationAlgorithm() {

        // new list
        m_models.reset(new ObjectModelPtrList());

        // preallocate memory
        m_models->reserve(MODEL_LIST_PREALLOC);

	// new major_axis_list
	major_axis_list.reset (new std::vector <std::vector < Eigen::Vector3f > > ());

	// preallocate memory
	major_axis_list->reserve(20);

	// new mass_center_list
	mass_center_list.reset (new std::vector <Eigen::Vector3f> ());

	// preallocate memory
	mass_center_list->reserve(20);

    }

    /**
     * Returns the list of identified geometric models.
     * This list is in the same order as the corresponding PointCloud-Cluster-List
     * generated during segmentation process.
     *
     * @return List of geometric model classes corresponding the object clusters
     */
    const ObjectModelPtrListPtr getModelList() {
        return m_models;
    }

     /**
     * Get list with major axis.
     * Returns a list of major axis for each cluster
     *
     * @param axis [major_axis, middle_axis, minor_axis]
     */
     boost::shared_ptr <std::vector <std::vector < Eigen::Vector3f > > >  getMajorAxisList()
     {
       return major_axis_list;
     }

     /**
     * Get list with mass center.
     * Returns a list of mass center for each cluster
     */
     boost::shared_ptr <std::vector <Eigen::Vector3f> >  getMassCenterList()
     {
       return mass_center_list;
     }

};

#endif
