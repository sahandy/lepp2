#ifndef LEPP2_LOLA_COORDINATE_TRANSFORMER_H_
#define LEPP2_LOLA_COORDINATE_TRANSFORMER_H_

#include <pcl/common/pca.h>
#include <pcl/common/common.h>


namespace lepp {

template<class PointT>
class CoordinateTransformer {
public:
  /**
   * Performs the transformation of the given point in the camera system to the
   * coordinate system of the transformer itself.
   *
   * Only affects the x, y, z components of the original point.
   */
  virtual PointT transformPoint(PointT const& original) = 0;
  /**
   * Instructs the transformer to prepare for converting the next frame.
   * This method is needed because transformers may want to change their
   * transformation between different frames.
   */
  virtual void prepareNext() = 0;
};

}  // namespace lepp

#endif
