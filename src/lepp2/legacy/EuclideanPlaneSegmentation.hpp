/**
 *  \file
 *  \brief     EuclideanPlaneSegmentation.hpp
 *  \author    Fabian Braeu
 *  \date      2014
 *
 *  \copyright Institute of Applied Mechanics, Technical University of Munich
 */

#ifndef EUCLIDEAN_PLANE_SEGMENTATION_HPP
#define EUCLIDEAN_PLANE_SEGMENTATION_HPP

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include "SegmentationAlgorithm.hpp"

/**
 * Segmentation algorithm based on RANSAC plane matching and euclidean clustering.
 * This Segmentation Algorithm searches for the largest plane, which is then considered as background and substracted.
 * All remaining points are clustered using the euclidean clustering mechanism.
 */

class EuclideanPlaneSegmentation : public SegmentationAlgorithm {

private:

    pcl::SACSegmentation<PointType>        			m_segmentation;

    boost::shared_ptr<pcl::PointIndices>    			m_validPoints;

    boost::shared_ptr<pcl::PointIndices>    			m_inliers;
    boost::shared_ptr<pcl::PointIndices>    			m_outliers;

    boost::shared_ptr<pcl::search::KdTree<PointType> >          m_tree;
    std::vector<pcl::PointIndices>          			m_clusterIndices;
    pcl::EuclideanClusterExtraction<PointType>  		m_clusterizer;

    boost::shared_ptr<std::vector<pcl::ModelCoefficients> >     m_coefficients;     //!< List of coefficients [normal_x normal_y normal_z hessian_component_d] for the 												     largest (and substracted) planes in the data set


    float                                   			m_relMinForegroundSize;   //!< Plane background substraction will stop if we reach this relative size of 											   the foreground in regard to the complete number of points.
    bool				     			m_firstrun;
public:

    /**
     * Default constructor.
     */
    EuclideanPlaneSegmentation() {

        // allocate new list of coefficients
        m_coefficients.reset(new std::vector<pcl::ModelCoefficients>());

        // allocate outliers point indices
        m_outliers.reset(new pcl::PointIndices());

        // allocate inliers point indices
        m_inliers.reset(new pcl::PointIndices());

        // allocate valid point indices
        m_validPoints.reset(new pcl::PointIndices());

        // allocate new kdTree
        m_tree.reset(new pcl::search::KdTree<PointType>());

        // preallocate memory for 30 plane coefficients
        m_coefficients->reserve(40);

        m_relMinForegroundSize = 0.04;				// Video 320x240     0.10		// Video Objekte: 0.04

	m_firstrun = 1;
    }

    /**
     * Initialize algorithm.
     * This method is executed before the first call to update().
     */
    void init();

    /**
     * Updates results.
     * Initiates new execution of the algorithm. This method is called by the ObjectManager.
     */
    void update();

};


#endif
