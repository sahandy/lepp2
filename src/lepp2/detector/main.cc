/**
 * A program performing detection of obstacles in the given file feed
 * and visualizing their approximations.
 */
#include <iostream>

#include <pcl/io/openni2_grabber.h>
#include <pcl/io/pcd_grabber.h>

#include "lepp2/BaseObstacleDetector.hpp"
#include "lepp2/GrabberVideoSource.hpp"
#include "lepp2/BaseVideoSource.hpp"
#include "lepp2/VideoObserver.hpp"
#include "lepp2/FilteredVideoSource.hpp"
#include "lepp2/SmoothObstacleAggregator.hpp"

#include "lepp2/visualization/EchoObserver.hpp"
#include "lepp2/visualization/ObstacleVisualizer.hpp"

using namespace lepp;

/**
 * Prints out the expected CLI usage of the program.
 */
void PrintUsage() {
  std::cout << "usage: detector [--pcd file | --oni file | --stream]"
      << std::endl;
  std::cout << "--pcd    : " << "read the input from a .pcd file" << std::endl;
  std::cout << "--oni    : " << "read the input from an .oni file" << std::endl;
  std::cout << "--stream : " << "read the input from a live stream based on a"
      << " sensor attached to the computer" << std::endl;
}

/**
 * Parses the command line arguments received by the program and chooses
 * the appropriate video source based on those.
 */
boost::shared_ptr<SimpleVideoSource> GetVideoSource(int argc, char* argv[]) {
  if (argc < 2) {
    return boost::shared_ptr<SimpleVideoSource>();
  }

  std::string const option = argv[1];
  if (option == "--stream" && argc == 2) {
    return boost::shared_ptr<SimpleVideoSource>(
        new LiveStreamSource<SimplePoint>());
  } else if (option == "--pcd" && argc == 3) {
    std::string const file_path = argv[2];
    boost::shared_ptr<pcl::Grabber> interface(new pcl::PCDGrabber<SimplePoint>(
      file_path,
      20.,
      true));
    return boost::shared_ptr<SimpleVideoSource>(
        new GeneralGrabberVideoSource<SimplePoint>(interface));
  } else if (option == "--oni" && argc == 3) {
    std::string const file_path = argv[2];
    boost::shared_ptr<pcl::Grabber> interface(new pcl::io::OpenNI2Grabber(
      file_path,
      pcl::io::OpenNI2Grabber::OpenNI_Default_Mode,
      pcl::io::OpenNI2Grabber::OpenNI_Default_Mode));
    return boost::shared_ptr<SimpleVideoSource>(
        new GeneralGrabberVideoSource<SimplePoint>(interface));
  }

  // Unknown option: return a "null" pointer.
  return boost::shared_ptr<SimpleVideoSource>();
}

int main(int argc, char* argv[]) {
  // Obtain a video source based on the command line arguments received
  boost::shared_ptr<SimpleVideoSource> raw_source(GetVideoSource(argc, argv));
  if (!raw_source) {
    PrintUsage();
    return 1;
  }
  // Wrap the raw source in a filter
  boost::shared_ptr<SimpleVideoSource> source(
      new FilteredVideoSource<SimplePoint>(raw_source));

  // Prepare the detector
  boost::shared_ptr<BaseObstacleDetector<SimplePoint> > detector(
      new BaseObstacleDetector<SimplePoint>());
  // Attaching the detector to the source: process the point clouds obtained
  // by the source.
  source->attachObserver(detector);

  // Prepare the result visualizer...
  boost::shared_ptr<ObstacleVisualizer<SimplePoint> > visualizer(
      new ObstacleVisualizer<SimplePoint>());
  // Attaching the visualizer to the source: allow it to display the original
  // point cloud.
  source->attachObserver(visualizer);
  // Attaching the visualizer to the detector: allow it to display the obstacle
  // approximations.
  // The visualizer is additionally decorated by the "smoothener" to smooth out
  // the output...
  boost::shared_ptr<SmoothObstacleAggregator> smooth_decorator(
      new SmoothObstacleAggregator(*visualizer));
  detector->attachObstacleAggregator(smooth_decorator);

  // Starts capturing new frames and forwarding them to attached observers.
  source->open();

  std::cout << "Waiting forever..." << std::endl;
  std::cout << "(^C to exit)" << std::endl;
  while (true)
    boost::this_thread::sleep(boost::posix_time::milliseconds(8000));

  return 0;
}
