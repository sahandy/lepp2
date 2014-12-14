/**
 *  \file
 *  \brief     EuclideanPlaneSegmentation.cpp
 *  \author    Fabian Braeu
 *  \date      2014
 *
 *  \copyright Institute of Applied Mechanics, Technical University of Munich
 */

#include "EuclideanPlaneSegmentation.hpp"
#include "../BaseObstacleDetector.hpp"

/**
 * Switches on OUTPUT mode.
 * Switch on to get important output values
 */
//#define OUTPUT

// Switch on to estimate points behind the object
//#define ESTIMATE_POINTS

// Switch on to use voxel filter
//#define FILTER


/*
 * Init Algorithm
 */
void EuclideanPlaneSegmentation::init() {

    // initialize model based segmentation
    m_segmentation.setOptimizeCoefficients (true);
    m_segmentation.setModelType (pcl::SACMODEL_PLANE);
    m_segmentation.setMethodType (pcl::SAC_RANSAC);
    m_segmentation.setMaxIterations (200);			// Video 320x240     300		// Video stampfen: 1500
    //m_segmentation.setDistanceThreshold (25.0);			// Video 320x240     25.0		// Video stampfen: 8.0
    m_segmentation.setDistanceThreshold(0.02);

    // initialize euclidean clusterization class
    //m_clusterizer.setClusterTolerance(20.0);			// Video 320x240     20.0		// Video stampfen: 15.0
    m_clusterizer.setClusterTolerance(0.02);
    m_clusterizer.setMinClusterSize(100);			// Video 320x240     100.0		// Video stampfen: 620
    m_clusterizer.setMaxClusterSize(310000);


}

/*
 * Update Algorithm, motivated by official pcl tutorial
 * http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction
 */
void EuclideanPlaneSegmentation::update() {

    PointCloudPtr   			cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // TODO For now, the segmentation keeps a reference to the parent detector.
    //      However, the segmentor object should be able to live alone and
    //      perform segmentations on any cloud passed to its ``segment`` method.
    //      The detector instance is even left
    pcl::PointCloud<PointType>::ConstPtr   			cloud = detector_->getPointCloud();
    pcl::ExtractIndices<pcl::PointXYZ> 	extract;
    PointCloudPtr   			tmp;
    pcl::ModelCoefficients  		coeffs;
    int  i,j;
    pcl::PointIndices::Ptr 		tmpOutliers (new pcl::PointIndices);
    pcl::PointXYZ 			min_pt;
    pcl::PointXYZ 			max_pt;
    pcl::PointXYZ 			center;
    pcl::PointXYZ 			object_estimation;

    // clear list of model coefficients, outliers, clusterIndices
    m_coefficients->clear();
    m_outliers->indices.clear();
    m_clusterIndices.clear();


#ifdef FILTER
    // std::cout << "size point cloud before filtering " << cloud->size() << std::endl;


    // Voxel Grid filter for downsampling point cloud
    pcl::VoxelGrid <pcl::PointXYZ> sor;
    PointCloudPtr   cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    sor.setInputCloud (cloud);
    sor.setLeafSize (13.0f, 13.0f, 13.0f);
    sor.filter (*cloud_f);


    // std::cout << "size point cloud after filtering " << cloud_f->size() << std::endl;
#endif

    // find all valid points (!= NaN) in the cloud; save the new cloud to cloud_filtered/cloud_cluster; save the indices to validPoints
    pcl::removeNaNFromPointCloud<PointType>(*cloud,*cloud_filtered, m_validPoints->indices);

#ifdef FILTER
    pcl::removeNaNFromPointCloud<PointType>(*cloud_f,*cloud_filtered, m_validPoints->indices);
#endif

    //  extract all planes until we reach x % of the original number of points
    int nrOfPoints = m_validPoints->indices.size();
    // std::cout << "# of pts " << nrOfPoints << std::endl;

    int inliers = nrOfPoints;

    while (cloud_filtered->size() > m_relMinForegroundSize * nrOfPoints) {

        // std::cout << "Cloud filtered size " << cloud_filtered->size() << std::endl;
        // segment the largest planar component from the cloud
        tmpOutliers->indices.clear();
        m_segmentation.setInputCloud(cloud_filtered);
        m_segmentation.segment(*tmpOutliers, coeffs);

        // add coeffs to list
        m_coefficients->push_back(coeffs);

	// Extract the planar inliers from the input cloud
	extract.setInputCloud (cloud_filtered);
	extract.setIndices (tmpOutliers);
	extract.setNegative (true);
	extract.filter (*cloud_filtered);

/*
        std::cout << "size outliers " << tmpOutliers->indices.size() << std::endl;
	      std::cout << "size point cloud after " << cloud_filtered->size() << std::endl;
        fflush(stdout);
*/
        if (tmpOutliers->indices.size() == 0) {

            // no plane recognized... we're done here
            break;
        }

        // copy list of outliers to the global outliers list
        m_outliers->indices.insert(m_outliers->indices.end(), tmpOutliers->indices.begin(), tmpOutliers->indices.end());

	// std::cout << "size m_outliers " << m_outliers->indices.size() << std::endl;

        // update number of inliers
        inliers = inliers - tmpOutliers->indices.size();

    }

    // std::cout << "Final size " << cloud_filtered->size() << std::endl;
    // set search tree input cloud
    m_tree->setInputCloud(cloud_filtered);
    m_clusterizer.setSearchMethod(m_tree);

    // set euclidean clusterization input cloud and extract all cluster indices
    m_clusterizer.setInputCloud(cloud_filtered);
    m_clusterizer.extract(m_clusterIndices);

    /* The individual clusters are known now.
     * Create PointCloud instances from the clusterIndices.
     */

    // std::cout << "Number of Clusters: " << m_clusterIndices.size() << std::endl;
    fflush(stdout);

    // first purge the cluster list
    getClusterList()->clear();

    // allocate PointCloud for each cluster
    for (i = 0; i < m_clusterIndices.size(); ++i) {

        // retrieve allocated object from the Preallocator Instance
        tmp = m_preallocator.newInstance();

        // reset PointCloud
        tmp->clear();

        // add to cluster list
        getClusterList()->push_back(tmp);
    }


    // Now copy the points to each PointCloud
    #pragma omp parallel for
    for (i = 0; i < m_clusterIndices.size();++i) {

#ifdef OUTPUT
        // std::cout << "Number of Points in cluster " << m_clusterIndices.at(i).indices.size() << std::endl;
        fflush(stdout);
#endif

        // for each index in each cluster
	// here you can choose how many points should be used for the identification
        for (j = 0; j < m_clusterIndices.at(i).indices.size();j+=6) {

            // add the point to the corresponding point cloud
            getClusterList()->at(i)->push_back(cloud_filtered->at(m_clusterIndices.at(i).indices.at(j)));

        }
    }

#ifdef ESTIMATE_POINTS
{
    // std::cout << "ESTIMATE_POINTS is defined" << std::endl;
    // Now estimate unknown object points
    #pragma omp parallel for
    for (i = 0; i < m_clusterIndices.size();++i) {

      // calculate center point of each object
      pcl::getMinMax3D (*(getClusterList()-> at(i)), min_pt, max_pt);

#ifdef OUTPUT
      // std::cout << "min_pt " << min_pt.x   << min_pt.y   << min_pt.z    << std::endl;
      // std::cout << "max_pt " << max_pt.x   << max_pt.y   << max_pt.z    << std::endl;
#endif

      center.x = (max_pt.x + min_pt.x)/2;
      center.y = 1.01*((max_pt.y + min_pt.y)/2);
      center.z = 1.01*((max_pt.z + min_pt.z)/2);

#ifdef OUTPUT
      // std::cout << "center " << center.x   << center.y   << center.z    << std::endl;
#endif

        // for each index in each cluster
	// here you can choose how many points should be added
        for (j = 0; j < m_clusterIndices.at(i).indices.size();j+=6) {

	    // calculate estimated point via center point
	    object_estimation.x = (cloud_filtered->at(m_clusterIndices.at(i).indices.at(j))).x + 2*(center.x - (cloud_filtered->at(m_clusterIndices.at(i).indices.at(j)).x));
	    object_estimation.y = (cloud_filtered->at(m_clusterIndices.at(i).indices.at(j))).y + 2*(center.y - (cloud_filtered->at(m_clusterIndices.at(i).indices.at(j)).y));
	    object_estimation.z = (cloud_filtered->at(m_clusterIndices.at(i).indices.at(j))).z + 2*(center.z - (cloud_filtered->at(m_clusterIndices.at(i).indices.at(j)).z));

            // add the point to the corresponding point cloud
            getClusterList()->at(i)->push_back(object_estimation);
	}

    }
}
#endif
}
