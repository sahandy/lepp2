/**
 *  \file
 *  \brief     Fitting_no_Estimation.hpp
 *  \author    Fabian Braeu
 *  \date      2014
 *
 *  \copyright Institute of Applied Mechanics, Technical University of Munich
 */

#ifndef FITTING_NO_ESTIMATION_HPP
#define FITTING_NO_ESTIMATION_HPP

#include "FittingAlgorithm.hpp"
#include <pcl/common/common.h>

/**
 * Fitting_with_Estimation Algorithm for fast Fitting_with_Estimation.
 */
class Fitting_no_Estimation : public FittingAlgorithm {

private:

public:
  Fitting_no_Estimation() {
      // Nothing ...
  }
   /**
    * Initialize algorithm.
    * This method is executed before the first call to update().
    */
   void init() { }

   /**
    * Updates results.
    * Initiates new execution of the algorithm. This method is called by the ObjectManager.
    */
   void update();


};


#endif
