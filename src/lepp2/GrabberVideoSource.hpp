#ifndef GRABBER_VIDEO_SOURCE_H_
#define GRABBER_VIDEO_SOURCE_H_

#include "BaseVideoSource.hpp"
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <boost/circular_buffer.hpp>

namespace lepp {

/**
 * An implementation of the VideoSource abstract base class, based on PCL's
 * Grabber class.
 *
 * It allows clients to wrap a PCL Grabber to the VideoSource interface.
 * The grabber which is wrapped needs to be injected at construction of the
 * source instance.
 */
template<class PointT>
class GeneralGrabberVideoSource : public VideoSource<PointT> {
public:
  /**
   * Instantiate a video source which wraps the given Grabber instance.
   * The VideoSource instance takes ownership of the given Grabber instance.
   */
  GeneralGrabberVideoSource(boost::shared_ptr<pcl::Grabber> interface)
      : interface_(interface),
        callback_frame_counter(0){

    incoming_cloud_buffer_.set_capacity(120);
  }
  virtual ~GeneralGrabberVideoSource();
  virtual void open();
private:
  /**
   * A reference to the Grabber instance that the VideoSource wraps.
   */
  const boost::shared_ptr<pcl::Grabber> interface_;
  boost::circular_buffer<std::pair<int, typename pcl::PointCloud<PointT>::ConstPtr> > incoming_cloud_buffer_;
  boost::mutex cloud_mutex;
  int callback_frame_counter;

  /**
   * Member function which is registered as a callback of the Grabber.
   * Acts as the bond between the VideoSource and the Grabber, allowing the
   * adaptation of the interface.
   */
  void cloud_cb_(const typename pcl::PointCloud<PointT>::ConstPtr& cloud);
  void processBuffer(void);
};

template<class PointT>
GeneralGrabberVideoSource<PointT>::~GeneralGrabberVideoSource() {
  // RAII: make sure to stop any running Grabber
  interface_->stop();
}

template<class PointT>
void GeneralGrabberVideoSource<PointT>::cloud_cb_(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud) {

    ++callback_frame_counter;
    std::cout << " ==================================================== " << std::endl;
    std::cout << "Callback cloud #" << callback_frame_counter << std::endl;

    std::pair<int, typename pcl::PointCloud<PointT>::ConstPtr> p;
    incoming_cloud_buffer_.push_back(std::make_pair(callback_frame_counter, cloud));
    std::cout << "#Clouds waiting in queue: " << incoming_cloud_buffer_.size() << std::endl;
}

template<class PointT>
void GeneralGrabberVideoSource<PointT>::open() {
  // Register the callback and start grabbing frames...
  typedef void (callback_t)(const typename pcl::PointCloud<PointT>::ConstPtr&);
  boost::function<callback_t> f = boost::bind(
      &GeneralGrabberVideoSource::cloud_cb_,
      this, _1);
  interface_->registerCallback(f);

  boost::thread buffer_processor_thread(&GeneralGrabberVideoSource<PointT>::processBuffer, this);

  interface_->start();

  buffer_processor_thread.join();
}

template<class PointT>
void GeneralGrabberVideoSource<PointT>::processBuffer() {
    while (true)
    {
      while (!incoming_cloud_buffer_.empty())
      {
        //int last_index = incoming_cloud_buffer_.size() - 1;
        std::cout << "processing frame #" << incoming_cloud_buffer_[0].first << std::endl;
        this->setNextFrame( incoming_cloud_buffer_[0].second );
        incoming_cloud_buffer_.pop_front();
      }
      // If the buffer is empty, then wait for 33 milliseconds for the next frame to come (based on a 30 Hz video)
      boost::this_thread::sleep(boost::posix_time::milliseconds(33));
    }
}

/**
 * A convenience class for a live stream captured from a local RGB-D sensor.
 *
 * The implementation leverages the GeneralGrabberVideoSource wrapping a
 * PCL-based OpenNIGrabber instance.
 */
template<class PointT>
class LiveStreamSource : public GeneralGrabberVideoSource<PointT> {
public:
  LiveStreamSource()
      : GeneralGrabberVideoSource<PointT>(boost::shared_ptr<pcl::Grabber>(
            new pcl::OpenNIGrabber("", pcl::OpenNIGrabber::OpenNI_QVGA_30Hz))) {


    // Empty... All work performed in the initializer list.
  }
};

}  // namespace lepp

#endif
