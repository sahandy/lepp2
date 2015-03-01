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

// Forward declarations.
class ModelVisitor;
class SphereModel;
class CapsuleModel;

/**
 * The base class for all geometrical models that can be used to represent
 * objects.
 */
class ObjectModel {
public:
  virtual void accept(ModelVisitor& visitor) = 0;

  virtual Coordinate center_point() const = 0;

  virtual ~ObjectModel() {}
};

typedef boost::shared_ptr<ObjectModel> ObjectModelPtr;

class ModelVisitor {
public:
  virtual void visitSphere(SphereModel& sphere) = 0;
  virtual void visitCapsule(CapsuleModel& capsule) = 0;
  virtual ~ModelVisitor() {}
};

/**
 * Model class representing a sphere.
 */
class SphereModel : public ObjectModel {
public:
  SphereModel(double radius, Coordinate const& center);
  /**
   * Returns a model-specific representation of its coefficients packed into
   * an std::vector.
   */
  void accept(ModelVisitor& visitor) { visitor.visitSphere(*this); }
  Coordinate center_point() const { return center_; }

  double radius() const { return radius_; }
  Coordinate const& center() const { return center_; }
  void set_radius(double radius) { radius_ = radius; }
  void set_center(Coordinate const& center) { center_ = center; }

  friend std::ostream& operator<<(std::ostream& out, SphereModel const& sphere);
private:
  double radius_;
  Coordinate center_;
};

inline SphereModel::SphereModel(double radius, Coordinate const& center)
    : radius_(radius), center_(center) {}

inline std::ostream& operator<<(std::ostream& out, SphereModel const& sphere) {
  out << "[sphere; "
      << "radius = " << sphere.radius_ << "; "
      << "center = " << sphere.center_ << "]";

  return out;
}

/**
 * Model class that represents a capsule: a cylinder with two spheres at either
 * end of it.
 */
class CapsuleModel : public ObjectModel {
public:
  /**
   * Creates a new capsule with the given radius and two points representing the
   * centers of the two spheres at the ends of the capsule.
   */
  CapsuleModel(double radius, Coordinate const& first, Coordinate const& second)
      : radius_(radius), first_(first), second_(second) {}

  void accept(ModelVisitor& visitor) { visitor.visitCapsule(*this); }
  Coordinate center_point() const { return (second_ + first_) / 2; }

  double radius() const { return radius_; }
  Coordinate const& first() const { return first_; }
  Coordinate const& second() const { return second_; }

  void set_radius(double radius) { radius_ = radius; }
  void set_first(Coordinate const& first) { first_ = first; }
  void set_second(Coordinate const& second) { second_ = second; }

  friend std::ostream& operator<<(std::ostream& out, CapsuleModel const& caps);
private:
  double radius_;
  Coordinate first_;
  Coordinate second_;
};

inline std::ostream& operator<<(std::ostream& out, CapsuleModel const& capsule) {
  out << "[capsule; "
      << "radius = " << capsule.radius_ << "; "
      << "first = " << capsule.first_ << "; "
      << "second = " << capsule.second_ << "]";

  return out;
}

}  // namespace lepp
#endif
