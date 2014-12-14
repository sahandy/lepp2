/**
 *  \file
 *  \brief     ObjectModel.hpp
 *  \author    Felix Sygulla
 *  \date      2014
 *
 *  \copyright Institute of Applied Mechanics, Technical University of Munich
 */

#ifndef OBJECT_MODEL_HPP
#define OBJECT_MODEL_HPP

#include "../PCLWrapper.hpp"
#include <string.h>

/**
 * Smart Pointer to pcL::SampleConsensusModel
 */
typedef pcl::SampleConsensusModel<PointType>::Ptr    SACModelPtr;

/**
 * Simplified type for pcl::SampleConsensusModel
 */
typedef pcl::SampleConsensusModel<PointType>         SACModel;


/**
 * Represents the geometric model of an object.
 * This is a superclass for all collision geometry objects. Used for cluster <-> object
 * fitting and identification.
 */
class ObjectModel {

private:


public:

    /**
     * Sets the calculated coefficients.
     * Pure virtual method to set/update the model coefficients after
     * successful fitting. The respective subclass then finds an ideal model-specific representation
     * for the given data.
     *
     * @param coeffs The Eigen-vector containing the model coefficients.
     */
    virtual void setModelCoefficients(const Eigen::VectorXf& coeffs) = 0;

    /**
     * Returns the identification string of a model.
     * Pure virtual method, needs to be implemented in subclasses.
     *
     * @return Model identification string.
     */
    virtual const std::string getType() const = 0;

    /**
     * Returns the identification number of a model.
     * Pure virtual method, needs to be implemented in subclasses.
     *
     * @return Model identification number.
     */
    virtual const int getType_nr() const = 0;

    /**
    * Returns radius
    * Pure virtual method, needs to be implemented in subclasses.
    *
    * @return Model radius.
    */
    virtual float getRadius() = 0;

    /**
    * Returns p1 in cam sys
    * Pure virtual method, needs to be implemented in subclasses.
    *
    * @return Model point1_notrans.
    */
    virtual pcl::PointXYZ getP1_notrans() = 0;

    /**
    * Returns p2 in cam sys if existent else 0
    * Pure virtual method, needs to be implemented in subclasses.
    *
    * @return Model point2_notrans or 0.
    */
    virtual pcl::PointXYZ getP2_notrans() = 0;

    /**
    * Returns p3 in cam sys if existent else 0
    * Pure virtual method, needs to be implemented in subclasses.
    *
    * @return Model point3_notrans or 0.
    */
    virtual pcl::PointXYZ getP3_notrans() = 0;

    /**
    * Returns p1 in odo sys
    * Pure virtual method, needs to be implemented in subclasses.
    *
    * @return Model point1_trans.
    */
    virtual pcl::PointXYZ getP1_trans() = 0;

    /**
    * Returns p2 in odo sys
    * Pure virtual method, needs to be implemented in subclasses.
    *
    * @return Model point2_trans.
    */
    virtual pcl::PointXYZ getP2_trans() = 0;

    /**
    * Returns p3 in odo sys
    * Pure virtual method, needs to be implemented in subclasses.
    *
    * @return Model point3_trans.
    */
    virtual pcl::PointXYZ getP3_trans() = 0;

    /**
     * Draws a model representation within the given pcl::visualizer.
     * This pure virtual method is intended to draw a shape corresponding to the model and
     * the calculated coefficients.
     *
     * @param viewer The PCLVisualizer instance to be used.
     * @param name The unique name of the shape for later identification.
     * @param r The red channel of the color used for the model.
     * @param g The green channel of the color used for the model.
     * @param b The blue channel of the color used for the model.
     */
    virtual void draw(pcl::visualization::PCLVisualizer& viewer, const std::string& name, const float r, const float g, const float b) = 0;
};

/**
 * Smart Pointer to ObjectModel.
 */
typedef boost::shared_ptr<ObjectModel>          ObjectModelPtr;

/**
 * List of ObjectModel-smart pointers
 */
typedef std::vector<ObjectModelPtr>             ObjectModelPtrList;

/**
 * Smart pointer to List of ObjectModel-smart pointers
 */
typedef boost::shared_ptr<ObjectModelPtrList>   ObjectModelPtrListPtr;

#endif
