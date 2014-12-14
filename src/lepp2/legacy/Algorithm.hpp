/**
 *  \file
 *  \brief     Algorithm.hpp
 *  \author    Felix Sygulla
 *  \date      2014
 *
 *  \copyright Institute of Applied Mechanics, Technical University of Munich
 */

#ifndef ALGORITHM_HPP
#define ALGORITHM_HPP

#include "PCLWrapper.hpp"

/**
 * Superclass for the algorithms.
 * Implements basic interfaces for integration into ObjectManager.
 * All algorithm implementations must be derived from this class.
 */
class Algorithm {

private:

     boost::mutex    m_mtx;

public:

    /**
     * Init.
     * Executed before first call to update(). This method is called by the ObjectManager and is a pure virtual function, which
     * needs to be implemented in any subclass.
     */
    virtual void init() = 0;

    /**
     * Update.
     * Initiates new execution of the algorithm. This method is called by the ObjectManager and is a pure virtual function, which
     * needs to be implemented in any subclass.
     */
    virtual void update() = 0;

    /**
     * Thread-safe update method.
     * Uses scoped lock of internal mutex to enable multithreading between visualization thread and update method.
     * This method is called by the ObjectManager and itself calls the update() method of each algorithm.
     */
    void safeUpdate() {

        boost::mutex::scoped_lock mylock(m_mtx);
        update();
    }

};

#endif
