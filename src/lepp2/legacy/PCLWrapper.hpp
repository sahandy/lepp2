/**
 *  \file
 *  \brief     PCLWrapper.hpp
 *  \author    Felix Sygulla
 *  \date      2014
 *
 *  \copyright Institute of Applied Mechanics, Technical University of Munich
 */

#ifndef PCL_WRAPPER_HPP
#define PCL_WRAPPER_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/sac.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

/**
 * Simplified Point type
 */
typedef pcl::PointXYZ                                               PointType;

/**
 * Simplified PointCloud type
 */
typedef pcl::PointCloud<PointType>::Ptr                             PointCloudPtr;

/**
 * Simplified PointCloud cluster list type
 */
typedef std::vector<pcl::PointCloud<PointType>::Ptr>                PointCloudPtrList;

/**
 * Simplified PointCloud cluster list type
 */
typedef boost::shared_ptr<std::vector<pcl::PointCloud<PointType>::Ptr> >  PointCloudPtrListPtr;

/**
 * Simplified PointCloud cluster type
 */
typedef pcl::PointCloud<PointType>                                  PointCloudType;

#endif
