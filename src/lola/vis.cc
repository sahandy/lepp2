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

#include "lepp2/filter/TruncateFilter.hpp"
#include "lepp2/filter/SensorCalibrationFilter.hpp"

#include "lola/OdoCoordinateTransformer.hpp"
#include "lola/LolaAggregator.h"
#include "lola/PoseService.h"
#include "lola/RobotService.h"

using namespace lepp;

/**
 * Prints out the expected CLI usage of the program.
 */
void PrintUsage() {
  std::cout << "usage: detector [--pcd file | --oni file | --stream] [--live]"
      << std::endl;
  std::cout << "--pcd    : " << "read the input from a .pcd file" << std::endl;
  std::cout << "--oni    : " << "read the input from an .oni file" << std::endl;
  std::cout << "--stream : " << "read the input from a live stream based on a"
      << " sensor attached to the computer" << std::endl;
  std::cout << "--live   : " << "whether kinematics data is obtained from the robot"
      << std::endl;
}

/**
 * Builds a `FilteredVideoSource` instance that wraps the given raw source.
 */
template<class PointT>
boost::shared_ptr<FilteredVideoSource<PointT> >
buildFilteredSource(boost::shared_ptr<VideoSource<PointT> > raw, bool live) {
  // Wrap the given raw source.
  boost::shared_ptr<FilteredVideoSource<SimplePoint> > source(
      new SimpleFilteredVideoSource<SimplePoint>(raw));
  // Now set the point filters that should be used.
  {
    double const a = 1.0117;
    double const b = -0.0100851;
    boost::shared_ptr<PointFilter<SimplePoint> > filter(
        new SensorCalibrationFilter<SimplePoint>(a, b));
    source->addFilter(filter);
  }
  if (!live) {
    boost::shared_ptr<PointFilter<SimplePoint> > filter(
        new FileOdoTransformer<SimplePoint>("in.log"));
    source->addFilter(filter);
  } else {
    // TODO The reference to the service should eventually be obtained from some
    //      sort of IOC container.
    boost::shared_ptr<PoseService> service(new PoseService("127.0.0.1", 5000));
    service->start();
    boost::shared_ptr<PointFilter<SimplePoint> > filter(
        new RobotOdoTransformer<SimplePoint>(service));
    source->addFilter(filter);
  }
  {
    boost::shared_ptr<PointFilter<SimplePoint> > filter(
        new TruncateFilter<SimplePoint>(2));
    source->addFilter(filter);
  }

  return source;
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
  if (option == "--stream") {
    return boost::shared_ptr<SimpleVideoSource>(
        new LiveStreamSource<SimplePoint>());
  } else if (option == "--pcd" && argc >= 3) {
    std::string const file_path = argv[2];
    boost::shared_ptr<pcl::Grabber> interface(new pcl::PCDGrabber<SimplePoint>(
      file_path,
      20.,
      true));
    return boost::shared_ptr<SimpleVideoSource>(
        new GeneralGrabberVideoSource<SimplePoint>(interface));
  } else if (option == "--oni" && argc >= 3) {
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

/**
 * Checks whether the CLI parameters indicate that the detector should run with
 * a live robot.
 *
 * This is true iff a `--live` flag was passed to the executable.
 */
bool isLive(int argc, char* argv[]) {
  for (int i = 0; i < argc; ++i) {
    if (std::string(argv[i]) == "--live") return true;
  }
  return false;
}

int main(int argc, char* argv[]) {
  // Obtain a video source based on the command line arguments received
  boost::shared_ptr<SimpleVideoSource> raw_source(GetVideoSource(argc, argv));
  if (!raw_source) {
    PrintUsage();
    return 1;
  }
  bool live = isLive(argc, argv);
  // Wrap the raw source in a filter
  boost::shared_ptr<FilteredVideoSource<SimplePoint> > source(
      buildFilteredSource(raw_source, live));
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

  // Attach the obstacle postprocessor.
  boost::shared_ptr<SmoothObstacleAggregator> smooth_decorator(
      new SmoothObstacleAggregator);
  detector->attachObstacleAggregator(smooth_decorator);
  // Now attach various aggregators that are only interested in postprocessed
  // obstacles.
  boost::shared_ptr<LolaAggregator> lola(
      new LolaAggregator("127.0.0.1", 53250));
  smooth_decorator->attachObstacleAggregator(lola);
  smooth_decorator->attachObstacleAggregator(visualizer);

  RobotService robot_service("127.0.0.1", 1337, 10);
  robot_service.start();
  boost::shared_ptr<RobotAggregator> robot(new RobotAggregator(robot_service, 10));
  smooth_decorator->attachObstacleAggregator(robot);

  // Starts capturing new frames and forwarding them to attached observers.
  source->open();

  std::cout << "Waiting forever..." << std::endl;
  std::cout << "(^C to exit)" << std::endl;
  while (true)
    boost::this_thread::sleep(boost::posix_time::milliseconds(8000));

  return 0;
}
