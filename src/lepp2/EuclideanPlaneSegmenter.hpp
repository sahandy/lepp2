#ifndef LEPP2_EUCLIDEAN_PLANE_SEGMENTER_H__
#define LEPP2_EUCLIDEAN_PLANE_SEGMENTER_H__

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

namespace lepp {

/**
 * A segmenter that finds the segments by applying Euclidean plane segmentation.
 *
 * The code for now is just an adaptation of the legacy code to the new
 * interface, with no improvements over the legacy version.
 */
template<class PointT>
class EuclideanPlaneSegmenter : public BaseSegmenter<PointT> {
public:
	EuclideanPlaneSegmenter();
	typedef typename pcl::PointCloud<PointT>::ConstPtr CloudConstPtr;

	virtual std::vector<typename pcl::PointCloud<PointT>::ConstPtr> segment(
			const typename pcl::PointCloud<PointT>::ConstPtr& cloud);
private:
	pcl::SACSegmentation<PointT> m_segmentation;
	pcl::EuclideanClusterExtraction<PointT> m_clusterizer;
	double m_relMinForegroundSize;

	std::vector<pcl::PointIndices> m_clusterIndices;

	boost::shared_ptr<pcl::PointIndices> m_inliers;
	boost::shared_ptr<pcl::PointIndices> m_outliers;

	boost::shared_ptr<pcl::search::KdTree<PointT> > m_tree;
	boost::shared_ptr<std::vector<pcl::ModelCoefficients> > m_coefficients;
};

template<class PointT>
EuclideanPlaneSegmenter<PointT>::EuclideanPlaneSegmenter()
		: m_relMinForegroundSize(0.04),
      m_coefficients(new std::vector<pcl::ModelCoefficients>()),
      m_outliers(new pcl::PointIndices()),
      m_inliers(new pcl::PointIndices()),
      m_tree(new pcl::search::KdTree<PointT>()) {
	m_segmentation.setOptimizeCoefficients (true);
	m_segmentation.setModelType (pcl::SACMODEL_PLANE);
	m_segmentation.setMethodType (pcl::SAC_RANSAC);
	m_segmentation.setMaxIterations (200);			// Video 320x240		 300		// Video stampfen: 1500
	// m_segmentation.setDistanceThreshold (25.0);			// Video 320x240		 25.0		// Video stampfen: 8.0
	m_segmentation.setDistanceThreshold(0.02);

	// initialize euclidean clusterization class
	//m_clusterizer.setClusterTolerance(20.0);			// Video 320x240		 20.0		// Video stampfen: 15.0
	m_clusterizer.setClusterTolerance(0.02);
	m_clusterizer.setMinClusterSize(100);			// Video 320x240		 100.0		// Video stampfen: 620
	m_clusterizer.setMaxClusterSize(310000);
}

template<class PointT>
std::vector<typename pcl::PointCloud<PointT>::ConstPtr>
EuclideanPlaneSegmenter<PointT>::segment(
	const typename pcl::PointCloud<PointT>::ConstPtr& cloud) {
	std::vector<CloudConstPtr> ret;

	typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	pcl::ExtractIndices<PointT>	 extract;
	pcl::ModelCoefficients			coeffs;
	int	i,j;
	pcl::PointIndices::Ptr		 tmpOutliers (new pcl::PointIndices);
	PointT			 min_pt;
	PointT			 max_pt;
	PointT			 center;
	PointT			 object_estimation;

	// clear list of model coefficients, outliers, clusterIndices
	m_coefficients->clear();
	m_outliers->indices.clear();
	m_clusterIndices.clear();

	boost::shared_ptr<pcl::PointIndices> m_validPoints(new pcl::PointIndices());
	// find all valid points (!= NaN) in the cloud; save the new cloud to cloud_filtered/cloud_cluster; save the indices to validPoints
	pcl::removeNaNFromPointCloud<PointT>(*cloud,*cloud_filtered, m_validPoints->indices);

	//	extract all planes until we reach x % of the original number of points
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

	// Now copy the points to each PointCloud
	#pragma omp parallel for
	for (i = 0; i < m_clusterIndices.size();++i) {
		typename pcl::PointCloud<PointT>::Ptr current(new pcl::PointCloud<PointT>());
		for (j = 0; j < m_clusterIndices.at(i).indices.size();j+=6) {
			// add the point to the corresponding point cloud
			current->push_back(cloud_filtered->at(m_clusterIndices.at(i).indices.at(j)));
		}

		ret.push_back(current);
	}

	return ret;
}


} // namespace lepp

#endif
