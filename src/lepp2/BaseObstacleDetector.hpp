#ifndef BASE_OBSTACLE_DETECTOR_H_
#define BASE_OBSTACLE_DETECTOR_H_

#include <pcl/visualization/cloud_viewer.h>

#include "lepp2/VideoObserver.hpp"

#include "lepp2/legacy/SegmentationAlgorithm.hpp"
#include "lepp2/legacy/EuclideanPlaneSegmentation.hpp"
#include "lepp2/legacy/MomentOfInertiaIdentification.hpp"
#include "lepp2/legacy/Fitting_no_Estimation.hpp"


template<class PointT>
class BaseObstacleDetector : public lepp::VideoObserver<PointT> {
public:
  BaseObstacleDetector();
  virtual ~BaseObstacleDetector() {}

  /**
   * VideoObserver interface method implementation.
   */
  virtual void notifyNewFrame(
      int idx,
      const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud);

  /**
   * Returns the point cloud that the detector is currently working with.
   *
   * NOTE: Needed only for the legacy code that relies of pulling the point
   *       cloud from the detector, instead of getting it as a parameter.
   */
  virtual typename pcl::PointCloud<PointT>::ConstPtr getPointCloud() const {
    return cloud_;
  }

  /**
   * Return a pointer to the SegmentationAlgorithm that the detector uses.
   * NOTE: Needed only for the legacy code.
   */
  SegmentationAlgorithm* getSegmentationInstance() const {
    return segmenter_;
  }

  /**
   * Return a pointer to the IdentificationAlgorithm that the detector uses.
   * NOTE: Needed only for the legacy code.
   */
  IdentificationAlgorithm* getIdentificationInstance() const {
    return identifier_;
  }

private:
  typename pcl::PointCloud<PointT>::ConstPtr cloud_;

  // TODO These members are naked pointers for now because the legacy code
  //      requires naked pointers... That needs fixing before these can be
  //      wrapped in a safe structure...
  SegmentationAlgorithm* segmenter_;
  IdentificationAlgorithm* identifier_;
  FittingAlgorithm* fitter_;

  /**
   * Performs a new update of the obstacle approximations.
   * Triggered when the detector is notified of a new frame (i.e. point cloud).
   */
  void update();
};

template<class PointT>
BaseObstacleDetector<PointT>::BaseObstacleDetector() {
  // Instantiate all the dependencies of an obstacle detector
  // TODO Allow for dependency injection.
  segmenter_ = new EuclideanPlaneSegmentation();
  segmenter_->detector_.reset(this);
  // TODO This should really be done on instantiation of the segmenter,
  //      automatically...
  segmenter_->init();

  identifier_ = new MomentOfInertiaIdentification();
  identifier_->detector_.reset(this);

  fitter_ = new Fitting_no_Estimation();
  fitter_->detector_.reset(this);
}


template<class PointT>
void BaseObstacleDetector<PointT>::notifyNewFrame(
    int id,
    const typename pcl::PointCloud<PointT>::ConstPtr& point_cloud) {
  cloud_ = point_cloud;
  update();
}


template<class PointT>
void BaseObstacleDetector<PointT>::update() {
  std::cout << "Running the obstacle detector on the frame..." << std::endl;
  segmenter_->update();
  identifier_->update();
  fitter_->update();
}


#endif
