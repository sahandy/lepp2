/**
 *  \file
 *  \brief     FittingAlgorithm.hpp
 *  \author    Felix Sygulla
 *  \date      2014
 *
 *  \copyright Institute of Applied Mechanics, Technical University of Munich
 */

#ifndef FITTING_ALGORITHM_HPP
#define FITTING_ALGORITHM_HPP

#include "Algorithm.hpp"
#include "PCLWrapper.hpp"

template<class PointT>
class BaseObstacleDetector;

/**
 * Superclass for the fitting algorithms.
 * Implements basic interfaces for integration into ObjectManager.
 * All fitting algorithm implementations must be derived from this class.
 */
class FittingAlgorithm : public Algorithm
{

public:

// TODO Quite an ugly way to get rid of the singleton... Algorithms
//      hold references to the parent detector.
//      In the future, the algorithms must make sure to work without having
//      to reference their containers, but completely self-contained.
boost::shared_ptr<BaseObstacleDetector<PointType> > detector_;



};

#endif
