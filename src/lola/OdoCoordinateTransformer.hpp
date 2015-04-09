#ifndef LEPP2_LOLA_ODO_COORDINATE_TRANSFORMER_H_
#define LEPP2_LOLA_ODO_COORDINATE_TRANSFORMER_H_
#include "lepp2/filter/PointFilter.hpp"

#include "lola/PoseService.h"

#include <fstream>
#include <sstream>
#include <cmath>

#include <pcl/common/pca.h>
#include <pcl/common/common.h>

namespace {

/**
 * A struct wrapping the parameters LOLA-provided kinematics parameters that are
 * used to construct the transformation matrices between the camera frame and
 * the world coordinate system as LOLA knows it.
 */
struct LolaKinematicsParams {
  double t_wr_cl[3];
  double R_wr_cl[3][3];
  double t_stance_odo[3];
  double phi_z_odo;
  double stance;
  int frame_num;
  int stamp;
};

std::ostream& operator<<(std::ostream& out, LolaKinematicsParams const& param) {
  out << "Frame #" << param.frame_num << std::endl
      << "stamp #" << param.stamp << std::endl
      << "phi_z_odo = " << param.phi_z_odo << std::endl
      << "stance = " << param.stance << std::endl;
  out << "t_wr_cl = ";
  for (int i = 0; i < 3; ++i) out << param.t_wr_cl[i] << " "; out << std::endl;

  out << "R_Wr_cl = " << std::endl;
  for (int i = 0; i < 3; ++i) {
    out << "  ";
    for (int j = 0; j < 3; ++j) {
      out << param.R_wr_cl[i][j] << " ";
    }
    out << std::endl;
  }
  out << "t_stance_odo = ";
  for (int i = 0; i < 3; ++i) out << param.t_stance_odo[i] << " "; out << std::endl;

  return out;
}

/**
 * A struct wrapping the parameters for performing a transformation between the
 * camera coordinate system and the LOLA world coordinate system.
 */
struct OdoTransformParameters {
  double r_odo_cam[3];
  double A_odo_cam[3][3];
};

std::ostream& operator<<(std::ostream& out, OdoTransformParameters const& param) {
  out << "r_odo_cam = ";
  for (int i = 0; i < 3; ++i) out << param.r_odo_cam[i] << " "; out << std::endl;
  out << "A_odo_cam = " << std::endl;
  for (int i = 0; i < 3; ++i) {
    out << "  ";
    for (int j = 0; j < 3; ++j) {
      out << param.A_odo_cam[i][j] << " ";
    }
    out << std::endl;
  }

  return out;
}

}

/**
 * A `PointFilter` implementation that performs a transformation from the camera
 * coordinate system to the LOLA world coordinate system based on the currently
 * known kinematics.
 *
 * Concrete implementations of this class need to provide the kinematics info;
 * this ABC provides a way to use the kinematics to construct the transformation
 * and perform the transformation.
 *
 */
template<class PointT>
class OdoCoordinateTransformer : public lepp::PointFilter<PointT> {
public:
  OdoCoordinateTransformer() : current_frame_(0) {}
  /**
   * `PointFilter` interface method.
   */
  void prepareNext();
  /**
   * `PointFilter` interface method.
   */
  bool apply(PointT& original);
protected:
  /**
   * Gets the kinematics parameters that should be used for constructing the
   * transformation for the next frame, effectively locking the transformation
   * for all points for which the `apply` method is called until the next
   * `getNextParams` call (which happens when the next frame is being processed).
   *
   * Concrete implementations need to provide the implementation of this method.
   */
  virtual LolaKinematicsParams getNextParams() = 0;

  /**
   * Tracks the current frame number. Exposed to concrete implementations.
   */
  int current_frame_;

private:
  /**
   * Sets the parameters that should be used for the upcoming transformations,
   * based on the kinematics data given as a parameter.
   * These parameters will be considered valid until the next `setNext` call.
   */
  void setNext(LolaKinematicsParams const& params);

  /**
   * Parameters currently used for point transformations (i.e. by the `apply`
   * method).
   */
  OdoTransformParameters transform_params_;
};

namespace {
  /**
   * Puts a rotation matrix (around the z-axis) for the given angle in the given
   * matrix `matrix`.
   * It is assumed that the given matrix points to a matrix of dimensions 3x3.
   */
  void rotationmatrix(double angle, double matrix[][3]) {
    double s = sin(angle);
    double c = cos(angle);

    matrix[0][0] = c; matrix[0][1] = -s; matrix[0][2] = 0;
    matrix[1][0] = s; matrix[1][1] = c; matrix[1][2] = 0;
    matrix[2][0] = 0; matrix[2][1] = 0; matrix[2][2] = 1;
  }

  /**
   * Transposes the given matrix `matrix` and puts the transpose result into the
   * given `transpose` matrix.
   *
   * The matrices are assumed to be 3x3.
   */
  void transpose(double matrix[][3], double transpose[][3]) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        transpose[j][i] = matrix[i][j];
      }
    }
  }
}

template<class PointT>
void OdoCoordinateTransformer<PointT>::prepareNext() {
  ++current_frame_;
  LolaKinematicsParams new_params = this->getNextParams();
  this->setNext(new_params);
}

template<class PointT>
void OdoCoordinateTransformer<PointT>::setNext(LolaKinematicsParams const& params) {
//  std::cerr << "Setting new transformation for frame " << current_frame_
//            << " based on parameters:" << std::endl
//            << params << std::endl;
  double rotation_matrix[3][3];
  rotationmatrix(params.phi_z_odo, rotation_matrix);

  // In pseudo-code (if matrix operations were supported):
  // r_odo_cam = transpose(rotation_matrix) * (t_wr_cl + t_stance_odo)
  double transposed_matrix[3][3];
  transpose(rotation_matrix, transposed_matrix);
  for (int i = 0; i < 3; ++i) {
    transform_params_.r_odo_cam[i] = 0;
    for (int j = 0; j < 3; ++j) {
      transform_params_.r_odo_cam[i] +=
          transposed_matrix[i][j] * (params.t_wr_cl[j] + params.t_stance_odo[j]);
    }
  }

  // In pseudo-code (if matrix operations were supported):
  // A_odo_cam = transpose(R_wr_cl * rotation_matrix)
  double A_odo_cam_no_trans[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      A_odo_cam_no_trans[i][j] = 0;
      for (int k = 0; k < 3; ++k) {
        A_odo_cam_no_trans[i][j] += params.R_wr_cl[i][k] * rotation_matrix[k][j];
      }
    }
  }
  transpose(A_odo_cam_no_trans, transform_params_.A_odo_cam);

//  std::cerr << "New transformaion matrices calculated:" << std::endl;
//  std::cerr << transform_params_ << std::endl;
}

template<class PointT>
bool OdoCoordinateTransformer<PointT>::apply(PointT& original) {
  // This checks if we have a "null" transform. This would cause all points to
  // be mapped to (0, 0, 0) so we exclude each such point from the output all
  // together.
  bool all = true;
  for (int i = 0; i < 3; ++i) {
    all = all && (transform_params_.r_odo_cam[i] == 0);
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      all = all && (transform_params_.A_odo_cam[i][j] == 0);
    }
  }
  if (all) return false;
  // world_point = r_odo_cam + (A_odo_cam * original)
  PointT world_point = original;
  world_point.x = (transform_params_.r_odo_cam[0])
                               + transform_params_.A_odo_cam[0][0] * original.x
                               + transform_params_.A_odo_cam[0][1] * original.y
                               + transform_params_.A_odo_cam[0][2] * original.z;
  world_point.y = (transform_params_.r_odo_cam[1])
                               + transform_params_.A_odo_cam[1][0] * original.x
                               + transform_params_.A_odo_cam[1][1] * original.y
                               + transform_params_.A_odo_cam[1][2] * original.z;
  world_point.z = (transform_params_.r_odo_cam[2])
                               + transform_params_.A_odo_cam[2][0] * original.x
                               + transform_params_.A_odo_cam[2][1] * original.y
                               + transform_params_.A_odo_cam[2][2] * original.z;

  // Now replace the original with only the x, y, z components modified.
  original = world_point;
  return true;
}

/**
 * A concrete implementation of the transformer, which obtains its kinematics
 * information by reading from a log file. The path to the log file is provided
 * at construction time.
 */
template<class PointT>
class FileOdoTransformer : public OdoCoordinateTransformer<PointT> {
public:
  FileOdoTransformer(std::string const& file_name);
protected:
  LolaKinematicsParams getNextParams();
private:
  LolaKinematicsParams readNextParams();

  /**
   * The name of the file from which the kinematics data is to be read.
   */
  std::string const file_name_;
  /**
   * A handle to the file from which kinematics is being read.
   */
  std::ifstream fin_;

  LolaKinematicsParams current_;
  LolaKinematicsParams next_;
};

template<class PointT>
FileOdoTransformer<PointT>::FileOdoTransformer(
    std::string const& file_name)
      : file_name_(file_name), fin_(file_name.c_str()) {
  // Initialize the current and next values
  current_ = this->readNextParams();
  next_ = this->readNextParams();
}

template<class PointT>
LolaKinematicsParams FileOdoTransformer<PointT>::getNextParams() {
  LolaKinematicsParams use = current_;
  if (this->current_frame_ + 1 == next_.frame_num) {
    current_ = next_;
    next_ = this->readNextParams();
  }

  return use;
}

template<class PointT>
LolaKinematicsParams
FileOdoTransformer<PointT>::readNextParams() {
  std::string line;
  // Ignore the comments in the log file
  do {
    std::getline(fin_, line);
  } while (line[0] == '#');

  // Parse next frame's transformation parameters from the read line...
  LolaKinematicsParams params;

  std::stringstream ss(line);
  for (size_t i = 0; i < 3; ++i) { ss >> params.t_wr_cl[i]; }
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      ss >> params.R_wr_cl[i][j];
    }
  }
  for (size_t i = 0; i < 3; ++i) { ss >> params.t_stance_odo[i]; }
  ss >> params.phi_z_odo;
  ss >> params.stance;
  ss >> params.frame_num;
  ss >> params.stamp;

  return params;
}

/**
 * A concrete implementation of the transformer, which obtains its kinematics
 * information from the robot. Relies on a `PoseService` instance that it can
 * ask for the newest robot kinematics info when needed.
 */
template<class PointT>
class RobotOdoTransformer : public OdoCoordinateTransformer<PointT> {
public:
  RobotOdoTransformer(boost::shared_ptr<PoseService> service)
      : service_(service) {}
protected:
  LolaKinematicsParams getNextParams();
private:
  boost::shared_ptr<PoseService> service_;
};

template<class PointT>
LolaKinematicsParams RobotOdoTransformer<PointT>::getNextParams() {
  HR_Pose pose = service_->getCurrentPose();
  // Now convert the current raw pose to parameters that are of relevance to the
  // transformation.
  LolaKinematicsParams params;
  for (int i = 0; i < 3; ++i) {
    params.t_wr_cl[i] = pose.t_wr_cl[i];
    params.t_stance_odo[i] = pose.t_stance_odo[i];
    for (int j = 0; j < 3; ++j) {
      params.R_wr_cl[i][j] = pose.R_wr_cl[3*i + j];
    }
  }
  params.phi_z_odo = pose.phi_z_odo;
  params.stance = pose.stance;
  params.frame_num = this->current_frame_;
  params.stamp = pose.stamp;

  return params;
}

#endif
