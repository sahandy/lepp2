/**
 *  \file
 *  \brief     SegmentationAlgorithm.hpp
 *  \author    Felix Sygulla
 *  \date      2014
 *
 *  \copyright Institute of Applied Mechanics, Technical University of Munich
 */

#ifndef SEGMENTATION_ALGORITHM_HPP
#define SEGMENTATION_ALGORITHM_HPP

#include "Algorithm.hpp"
#include "PCLWrapper.hpp"
#include "Preallocator.hpp"

template<class PointT>
class BaseObstacleDetector;

/**
 * Superclass for the segmentation algorithms.
 * Implements basic interfaces for integration into ObjectManager.
 * All segmentation algorithm implementations must be derived from this class.
 */
class SegmentationAlgorithm : public Algorithm
{

private:

    // TODO
    // A pointer to a vector (even if it's a smart pointer) is pretty
    // atrocious...
    // (conveniently hidden behind the typedef)
    PointCloudPtrListPtr    m_clusters;

protected:

    Preallocator<PointCloudType>    m_preallocator;     //!< Preallocator class for new PointClouds.

public:

    // TODO Quite an ugly way to get rid of the singleton... Algorithms
    //      hold references to the parent detector.
    //      In the future, the algorithms must make sure to work without having
    //      to reference their containers, but completely self-contained.
    boost::shared_ptr<BaseObstacleDetector<PointType> > detector_;


    /**
     * Default constructor.
     * Allocates memory for cluster list.
     */
    SegmentationAlgorithm() : m_clusters(new PointCloudPtrList()) {

        /*
        // preallocate list memory
        m_clusters->reserve(CLUSTER_LIST_PREALLOC);

        // init preallocator for point clouds
        m_preallocator.allocate(CLUSTER_LIST_PREALLOC, preallocatePointCloud);
        */
    }


    /**
     * Returns the list of recognized clusters.
     * The clusters are encapsuled in a cloud of 3D points (PCL).
     *
     * @return
     */
    const PointCloudPtrListPtr getClusterList() {

        return m_clusters;

    }

};

#endif
