/**
 *  \file
 *  \brief     SphereModel.hpp
 *  \author    Fabian Braeu
 *  \date      2014
 *
 *  \copyright Institute of Applied Mechanics, Technical University of Munich
 */

#ifndef SPHERE_MODEL_HPP
#define SPHERE_MODEL_HPP

#include "ObjectModel.hpp"
#include <pcl/sample_consensus/sac_model_sphere.h>

/**
 * Model of a sphere collision object.
 * Represents information and data associated with a sphere collision model.
 */
class SphereModel : public ObjectModel {

private:

    float radius;				//!< Sphere: radius
    pcl::PointXYZ p1_notrans;   //!< Sphere: center point (in cam sys)
    pcl::PointXYZ p1_trans;		//!< Sphere: center point (in odo sys)
public:

    /**
     * Macro for Eigen block alignment
     * This is needed to use Eigen variables as members
     */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
    * Sets the calculated coefficients.
    * Used to set/update the model coefficients after
    * successful fitting.
    *
    * @param coeffs The Eigen-vector containing the model coefficients: [x,y,z, radius] in cam and odo sys
    */
    void setModelCoefficients(const Eigen::VectorXf& coeffs) {

      radius = static_cast<int>(coeffs[0] * 100.) / 100.;
      p1_notrans.x = static_cast<int>(coeffs[1] * 100 ) / 100.;
      p1_notrans.y = static_cast<int>(coeffs[2] * 100) / 100.;
      p1_notrans.z = static_cast<int>(coeffs[3] * 100) / 100.;
      p1_trans.x = coeffs[4];
      p1_trans.y = coeffs[5];
      p1_trans.z = coeffs[6];


    }

    /**
    * Returns radius
    */
    float getRadius() {
      return radius;
    }

    /**
    * Returns p1_notrans
    */
    pcl::PointXYZ getP1_notrans() {
      return p1_notrans;
    }

    /**
    * Returns p2_notrans
    */
      pcl::PointXYZ getP2_notrans() {
	pcl::PointXYZ p2_notrans(0.0f, 0.0f, 0.0f);
	return p2_notrans;
    }

    /**
    * Returns p3_notrans
    */
      pcl::PointXYZ getP3_notrans() {
	pcl::PointXYZ p3_notrans(0.0f, 0.0f, 0.0f);
	return p3_notrans;
    }

    /**
    * Returns p1_trans
    */
    pcl::PointXYZ getP1_trans() {
      return p1_trans;
    }

    /**
    * Returns p2_trans
    */
    pcl::PointXYZ getP2_trans() {
      pcl::PointXYZ p2_trans(0.0f, 0.0f, 0.0f);
      return p2_trans;
    }

    /**
    * Returns p3_trans
    */
    pcl::PointXYZ getP3_trans() {
      pcl::PointXYZ p3_trans(0.0f, 0.0f, 0.0f);
      return p3_trans;
    }

    /**
     * Returns the identification string
     *
     * @return The identification string "sphere"
     */
    const std::string getType() const {
        return std::string("sphere");
    }

    /**
     * Returns the identification number
     *
     * @return The identification number 0
     */
    const int getType_nr() const {
        return 0;
    }

    /**
     * Draws a model representation within the given pcl::visualizer.
     * This method draws a sphere with the calculated radius and position
     * to the visualizer.
     *
     * @param viewer The PCLVisualizer instance to be used.
     * @param name The unique name of the shape for later identification.
     * @param r The red channel of the color used for the model
     * @param g The green channel of the color used for the model
     * @param b The blue channel of the color used for the model
     */
    void draw(pcl::visualization::PCLVisualizer& viewer, const std::string& name, const float r, const float g, const float b) {

        // update sphere. add if not existant
        // std::cout << "Sphere radius " << radius << std::endl;
        if (!viewer.updateSphere(p1_notrans, radius, r,g,b, name)) {
            viewer.addSphere(p1_notrans,radius , r, g, b, name);
        }

        // set opacity to 30 %
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);


    }
};



#endif
