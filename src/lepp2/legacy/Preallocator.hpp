/**
 *  \file
 *  \brief     Preallocator.hpp
 *  \author    Felix Sygulla
 *  \date      2014
 *
 *  \copyright Institute of Applied Mechanics, Technical University of Munich
 */

#ifndef PREALLOCATOR_HPP
#define PREALLOCATOR_HPP

#include <assert.h>
#include <boost/shared_ptr.hpp>

/**
 * Preallocation template class.
 * Allocates a pool of instances of type T.
 * The newInstance() method returns a preallocated and currently not used instance of this pool. However this
 * class cannot clear internal data of the instances. You should call appropriate reset methods
 * after retrieving an instance from the newInstance() method.^
 */
template <typename T> class Preallocator {
    
  
private:

    std::vector<boost::shared_ptr<T> >  m_instanceList; 
    
public:
    
    /**
     * (Pre)allocates new pool of instances of type T.
     * This creates a new pool of preallocated instances and calls the given callback
     * for each new isntance. If a pool has already been allocated, it is complete removed and a new pool is created.
     * 
     * @param numElements Number of preallocated elements in the pool. This will be automatically raised, if the program needs more instances.
     * @param fun Callback function for initialization of the pool instances (e.g. for memory reservation).
     */
    void allocate(const unsigned int numElements, void (*fun) (const boost::shared_ptr<T> element) = NULL);

    /**
     * Returns an instance from the preallocated pool.
     * If there are not enough free instances in the pool, this method will allocate a new one (with performance drawback).
     * To ensure maximum performance, you should experiment with the number of preallocated elements in the pool.
     *
     *@return A smart pointer to a new (preallocated) instance of type T
     */
    const boost::shared_ptr<T> newInstance ();
    
};

/*
 * Preallocate memory
 */
template <typename T> void Preallocator<T>::allocate(const unsigned int numElements, void (*fun) (const boost::shared_ptr<T> element)) {

    boost::shared_ptr<T>    smart_ptr;
       
    // check arguments
    assert(numElements != 0);

    // clear list and (re)allocate all elements
    m_instanceList.clear();

    for (int i = 0; i < numElements; ++i) {

       smart_ptr.reset(new T());
       m_instanceList.push_back(smart_ptr);
    }

    // call given function for each new element
    if (fun != NULL) {

       for (int i = 0; i < m_instanceList.size(); ++i) {
           fun(m_instanceList.at(i));
       }

    }
       
}

/*
 * Return new instance of Type T
 */
template <typename T> const boost::shared_ptr<T> Preallocator<T>::newInstance () {
        
    // search for free element in list
    for (int i = 0; i < m_instanceList.size();++i) {
    
        // check ref counter if this object is unique
        if (m_instanceList[i].unique()) {
     
            return m_instanceList[i];
        
        }
    }
    
    boost::shared_ptr<T>    smart_ptr;

    // failback solution: reallocate new entry
    smart_ptr.reset(new T());
    m_instanceList.push_back(smart_ptr);
    return m_instanceList.at(m_instanceList.size()-1);

}



#endif
