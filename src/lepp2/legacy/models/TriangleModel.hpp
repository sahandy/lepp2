/**
 *  \file
 *  \brief     TriangleModel.hpp
 *  \author    Fabian Braeu
 *  \date      2014
 *
 *  \copyright Institute of Applied Mechanics, Technical University of Munich
 */

#ifndef TRIANGLE_MODEL_HPP
#define TRIANGLE_MODEL_HPP

#include "ObjectModel.hpp"
#include <pcl/sample_consensus/sac_model_sphere.h>

/**
 * Model of a Triangle collision object.
 * Represents information and data associated with a Triangle collision model.
 */
class TriangleModel : public ObjectModel {

private:
    
    pcl::PointXYZ	p1_notrans;   	//!< Triangle coefficients: point at edge 1 in cam sys
    pcl::PointXYZ 	p2_notrans;   	//!< Triangle coefficients: point at edge 2 in cam sys
    pcl::PointXYZ 	p3_notrans;   	//!< Triangle coefficients: point at edge 3 in cam sys
    pcl::PointXYZ	p1_trans;   	//!< Triangle coefficients: point at edge 1 in odo sys
    pcl::PointXYZ 	p2_trans;   	//!< Triangle coefficients: point at edge 2 in odo sys
    pcl::PointXYZ 	p3_trans;   	//!< Triangle coefficients: point at edge 3 in odo sys
    float radius;   				//!< Triangle coefficients: radius
    
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
    * @param coeffs The Eigen-vector containing the model coefficients: [x_1, y_1, z_1, x_2 ,y_2, z_2, x_3 ,y_3, z_3, radius] in cam and odo sys
    */
    void setModelCoefficients(const Eigen::VectorXf& coeffs) {
        
	radius = coeffs[0];
	p1_notrans.x = coeffs[1];
	p1_notrans.y = coeffs[2];
	p1_notrans.z = coeffs[3];
	p2_notrans.x = coeffs[4];
	p2_notrans.y = coeffs[5];
	p2_notrans.z = coeffs[6];
	p3_notrans.x = coeffs[7];
	p3_notrans.y = coeffs[8];
	p3_notrans.z = coeffs[9];
	p1_trans.x = coeffs[10];
	p1_trans.y = coeffs[11];
	p1_trans.z = coeffs[12];
	p2_trans.x = coeffs[13];
	p2_trans.y = coeffs[14];
	p2_trans.z = coeffs[15];
	p3_trans.x = coeffs[16];
	p3_trans.y = coeffs[17];
	p3_trans.z = coeffs[18];
        
    }

    /**
     * Returns the identification string
     *
     * @return The identification string "triangle"
     */
    const std::string getType() const {
        return std::string("triangle");
    }
    
    /**
     * Returns the identification number
     *
     * @return The identification number 2
     */
    const int getType_nr() const {
        return 2;
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
      return p3_trans;
    }

    /**
     * Draws a model representation within the given pcl::visualizer.
     * This method draws a triangle with the calculated radius and position 
     * to the visualizer.
     *
     * @param viewer The PCLVisualizer instance to be used.
     * @param name The unique name of the shape for later identification.
     * @param r The red channel of the color used for the model
     * @param g The green channel of the color used for the model
     * @param b The blue channel of the color used for the model
     */
    void draw(pcl::visualization::PCLVisualizer& viewer, const std::string& name, const float r, const float g, const float b) {                          
      
      std::string name_obstacle;
      name_obstacle = name;
      
      //generate cylinder 1
      pcl::ModelCoefficients cylinder_coeff;
      cylinder_coeff.values.resize (7); // We need 7 values
      cylinder_coeff.values[0] = p1_notrans.x;
      cylinder_coeff.values[1] = p1_notrans.y;
      cylinder_coeff.values[2] = p1_notrans.z;
      cylinder_coeff.values[3] = p2_notrans.x - p1_notrans.x;
      cylinder_coeff.values[4] = p2_notrans.y - p1_notrans.y;
      cylinder_coeff.values[5] = p2_notrans.z - p1_notrans.z;
      cylinder_coeff.values[6] = radius;
      
      viewer.addCylinder (cylinder_coeff, name);
      viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, name);
      viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r,g,b, name);
      
      //generate cylinder 2
      name_obstacle.append("cylinder2");
      
      cylinder_coeff.values[0] = p1_notrans.x;
      cylinder_coeff.values[1] = p1_notrans.y;
      cylinder_coeff.values[2] = p1_notrans.z;
      cylinder_coeff.values[3] = p3_notrans.x - p1_notrans.x;
      cylinder_coeff.values[4] = p3_notrans.y - p1_notrans.y;
      cylinder_coeff.values[5] = p3_notrans.z - p1_notrans.z;
      cylinder_coeff.values[6] = radius;
      
      viewer.addCylinder (cylinder_coeff, name_obstacle);
      viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, name_obstacle);
      viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r,g,b, name_obstacle);
      
      //generate cylinder 3
      name_obstacle.append("cylinder3");
      
      cylinder_coeff.values[0] = p2_notrans.x;
      cylinder_coeff.values[1] = p2_notrans.y;
      cylinder_coeff.values[2] = p2_notrans.z;
      cylinder_coeff.values[3] = p3_notrans.x - p2_notrans.x;
      cylinder_coeff.values[4] = p3_notrans.y - p2_notrans.y;
      cylinder_coeff.values[5] = p3_notrans.z - p2_notrans.z;
      cylinder_coeff.values[6] = radius;
      
      viewer.addCylinder (cylinder_coeff, name_obstacle);
      viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, name_obstacle);
      viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r,g,b, name_obstacle);
      
      //generate sphere 1
      name_obstacle.append("sphere1");
      viewer.addSphere(p1_notrans, radius , r, g, b, name_obstacle);
      viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, name_obstacle);
      
      //generate sphere 2
      name_obstacle.append("sphere2");
      viewer.addSphere(p2_notrans, radius , r, g, b, name_obstacle);
      viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, name_obstacle);
      
      //generate sphere 3
      name_obstacle.append("sphere3");
      viewer.addSphere(p3_notrans, radius , r, g, b, name_obstacle);
      viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, name_obstacle);
      
    }
};



#endif