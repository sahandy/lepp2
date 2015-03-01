#ifndef LEPP2_MODELS_OBJECT_MODEL_H__
#define LEPP2_MODELS_OBJECT_MODEL_H__
#include <iostream>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>

namespace lepp {

/**
 * A struct representing a spatial coordinate.
 */
struct Coordinate {
  double x;
  double y;
  double z;
  Coordinate() {}
  Coordinate(double x, double y, double z) : x(x), y(y), z(z) {}
  // Convenience constructors to seamlessly convert from Eigen::Vectors.
  Coordinate(Eigen::Vector3f const& vec) : x(vec(0)), y(vec(1)), z(vec(2)) {}
  Coordinate(Eigen::Vector3d const& vec) : x(vec(0)), y(vec(1)), z(vec(2)) {}
  Coordinate(pcl::PointXYZ const& pt) : x(pt.x), y(pt.y), z(pt.z) {}
};

inline Coordinate operator-(Coordinate const& obj) {
  return Coordinate(-obj.x, -obj.y, -obj.z);
}

inline Coordinate operator+(Coordinate const& lhs, Coordinate const& rhs) {
  return Coordinate(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}

inline Coordinate operator-(Coordinate const& lhs, Coordinate const& rhs) {
  return lhs + (-rhs);
}

inline Coordinate operator/(Coordinate const& lhs, double coef) {
  return Coordinate(lhs.x / coef, lhs.y / coef, lhs.z / coef);
}

inline std::ostream& operator<<(std::ostream& out, Coordinate const& coord) {
  out << "(" << coord.x << ", " << coord.y << ", " << coord.z << ")";

  return out;
}

/**
 * The base class for all geometrical models that can be used to represent
 * objects.
 */
class ObjectModel {
public:
  virtual Coordinate center_point() const = 0;

  virtual ~ObjectModel() {}
};

typedef boost::shared_ptr<ObjectModel> ObjectModelPtr;

}  // namespace lepp
#endif
