#ifndef LEPP2_FILTER_SENSOR_CALIBRATION_FILTER_H__
#define LEPP2_FILTER_SENSOR_CALIBRATION_FILTER_H__

#include "lepp2/filter/PointFilter.hpp"

/**
 * Applies a linear function of the form `scale * z + offset` to the
 * z-coordinate of the point in order to fix the error that may occur because of
 * sensor inacuracies at low distances.
 *
 * Other coordinates are also adapted appropriately (based on how they are
 * originally calculated.
 */
template<class PointT>
class SensorCalibrationFilter : public PointFilter<PointT> {
public:
  /**
   * Creates a new sensor calibration filter that will use the given scale and
   * offset for its linear function applied to the z-coordinate.
   */
  SensorCalibrationFilter(double scale, double offset)
      : scale_(scale), offset_(offset) {}
  /**
   * Implementation of the `PointFilter` interface.
   */
  bool apply(PointT& pt) {
    pt.z = scale_*pt.z + offset_;
    pt.x *= scale_ + offset_/pt.z;
    pt.y *= scale_ + offset_/pt.z;
    return true;
  }

  void prepareNext() {}
private:
  double const scale_;
  double const offset_;
};

#endif
