/**
 *  \file
 *  \brief     CylinderModel.hpp
 *  \author    Fabian Braeu
 *  \date      2014
 *
 *  \copyright Institute of Applied Mechanics, Technical University of Munich
 */

#ifndef CYLINDER_MODEL_HPP
#define CYLINDER_MODEL_HPP

#include "ObjectModel.hpp"
#include <pcl/sample_consensus/sac_model_sphere.h>

/**
 * Model of a Cylinder collision object.
 * Represents information and data associated with a Cylinder collision model.
 */
class CylinderModel : public ObjectModel {

private:
    
    pcl::PointXYZ p1_notrans;		//!< Cylinder coefficients: center point of circle1 in cam sys
    pcl::PointXYZ p2_notrans;   	//!< Cylinder coefficients: center point of circle2 in cam sys
    pcl::PointXYZ p1_trans;			//!< Cylinder coefficients: center point of circle1 in odo sys
    pcl::PointXYZ p2_trans;	   		//!< Cylinder coefficients: center point of circle2 in odo sys
    float radius;   	   			//!< Cylinder coefficients: radius
    
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
    * @param coeffs The Eigen-vector containing the model coefficients: [x1, y1 z1, x2 ,y2, z2, radius] in cam and odo sys
    */
    void setModelCoefficients(const Eigen::VectorXf& coeffs) {
      
	radius = coeffs[0];
	p1_notrans.x = coeffs[1];
	p1_notrans.y = coeffs[2];
	p1_notrans.z = coeffs[3];
	p2_notrans.x = coeffs[4];
	p2_notrans.y = coeffs[5];
	p2_notrans.z = coeffs[6];
	p1_trans.x = coeffs[7];
	p1_trans.y = coeffs[8];
	p1_trans.z = coeffs[9];
	p2_trans.x = coeffs[10];
	p2_trans.y = coeffs[11];
	p2_trans.z = coeffs[12];
	
    }

    /**
     * Returns the identification string
     *
     * @return The identification string "cylinder"
     */
    const std::string getType() const {
        return std::string("cylinder");
    }
    
    /**
     * Returns the identification number
     *
     * @return The identification number 1
     */
    const int getType_nr() const {
        return 1;
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
     * Draws a model representation within the given pcl::visualizer.
     * This method draws a cylinder with the calculated radius and position 
     * to the visualizer.
     *
     * @param viewer The PCLVisualizer instance to be used.
     * @param name The unique name of the shape for later identification.
     * @param r The red channel of the color used for the model
     * @param g The green channel of the color used for the model
     * @param b The blue channel of the color used for the model
     */
    void draw(pcl::visualization::PCLVisualizer& viewer, const std::string& name, const float r, const float g, const float b) {                          
      
      std::string name_sphere;
      name_sphere = name;
      
      pcl::ModelCoefficients cylinder_coeff;
      cylinder_coeff.values.resize (7); // We need 7 values
      cylinder_coeff.values[0] = p1_notrans.x;
      cylinder_coeff.values[1] = p1_notrans.y;
      cylinder_coeff.values[2] = p1_notrans.z;
      cylinder_coeff.values[3] = p2_notrans.x - p1_notrans.x;
      cylinder_coeff.values[4] = p2_notrans.y - p1_notrans.y;
      cylinder_coeff.values[5] = p2_notrans.z - p1_notrans.z;
      cylinder_coeff.values[6] = radius;
      
      //generate cylinder
      viewer.addCylinder (cylinder_coeff, name);
      viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, name);
      viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r,g,b, name);
      
      //generate spheres
      name_sphere.append("sphere1");
      viewer.addSphere(p1_notrans, radius , r, g, b, name_sphere);
      viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, name_sphere);
      name_sphere.append("sphere2");
      viewer.addSphere(p2_notrans, radius , r, g, b, name_sphere);
      viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, name_sphere);
      
    }
};



#endif