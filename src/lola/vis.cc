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
  std::cout << "usage: lola [--pcd file | --oni file | --stream] [--live]"
      << std::endl;
  std::cout << "--pcd    : " << "read the input from a .pcd file" << std::endl;
  std::cout << "--oni    : " << "read the input from an .oni file" << std::endl;
  std::cout << "--stream : " << "read the input from a live stream based on a"
      << " sensor attached to the computer" << std::endl;
  std::cout << "--live   : " << "whether kinematics data is obtained from the robot"
      << std::endl;
}

/**
 * An ABC that represents the context of the execution. Essentially, it is a
 * container for all parts of the robot's vision pipeline. The parts are
 * exposed via public accessor methods.
 *
 * The ABC provides convenience methods for building up the context so that
 * different concrete implementations can be provided in a simple and
 * straightforward manner.
 */
template<class PointT>
class Context {
public:
  /// VideoSource-related accessors
  boost::shared_ptr<VideoSource<PointT> > raw_source() { return raw_source_; }
  boost::shared_ptr<FilteredVideoSource<PointT> > filtered_source() { return filtered_source_; }
  boost::shared_ptr<VideoSource<PointT> > source() {
    if (filtered_source_) {
      return filtered_source_;
    } else {
      return raw_source_;
    }
  }

  /// Robot-related accessors
  boost::shared_ptr<Robot> robot() { return robot_; }
  boost::shared_ptr<PoseService> pose_service() { return pose_service_; }
  boost::shared_ptr<RobotService> robot_service() { return robot_service_; }

  /// The obstacle detector accessor
  boost::shared_ptr<IObstacleDetector> detector() { return detector_; }

protected:
  /**
   * A template method for performing the initialization of a Context.
   *
   * The concrete implementations that opt into using this helper only need to
   * provide implementations of methods that initialize some parts of the
   * context, rather than worrying about all of them. It is still possible to
   * perform completely custom initialization by avoiding the use of this
   * convenience method.
   */
  void init() {
    // Prepare all the robot parts
    buildRobot();
    // Now get our video source ready...
    initRawSource();
    // ...along with any possible filters
    buildFilteredSource();
    // Initialize the obstacle detector
    initDetector();

    // Attach additional video source observers...
    addObservers();
    // ...and additional obstacle processors.
    addAggregators();

    // Finally, optionally visualize everything in a local GUI
    initVisualizer();
  }

  /// Robot initialization
  virtual void buildRobot() {
    initPoseService();
    initVisionService();
    initRobot();
  }
  /// Initialize the PoseService. Must set the `pose_service_` member.
  virtual void initPoseService() = 0;
  /// Initialize the `RobotService`. Must set the `robot_service_` member.
  virtual void initVisionService() = 0;
  /**
   * Initialize the `Robot`. Must se the `robot_` member.
   * A default implementation pieces a default robot together based on the
   * previously created `PoseService` and `RobotService`.
   */
  virtual void initRobot() {
    robot_.reset(new Robot(*pose_service(), 1.44));
  }

  /// Video source initialization
  /// Initialize a raw video source. Must set the `raw_source_` member.
  virtual void initRawSource() = 0;
  /**
   * A template method for building up a filtered video source. First
   * initializes a new filtered video source and then attaches a number of
   * point-wise filters to it.
   */
  virtual void buildFilteredSource() {
    // First create the basic filtered video source instance...
    initFilteredVideoSource();
    // Now set the point filters that should be used.
    addFilters();
  }

  /// Initialize a `FilteredVideoSource`. Must set the `filtered_source_` member.
  virtual void initFilteredVideoSource() = 0;
  /**
   * Add point-wise filters to the `filtered_source_`.
   * The default implementation does not add any pointwise filters.
   */
  virtual void addFilters() {}

  /// Obstacle detector initialization
  /// Initialize the `ObstacleDetector`. Must set the `detector_` member.
  virtual void initDetector() = 0;

  /// Provide hooks for adding more observers and aggregators.
  /// By default no extra observers or aggregators are added.
  virtual void addObservers() {}
  virtual void addAggregators() {}

  /// A hook for conveniently adding a visualizer, if required.
  /// Provides a default implementation that does not initialize any local
  /// visualization.
  virtual void initVisualizer() {}

protected:
  /// The members are exposed directly to concrete implementations for
  /// convenience.
  boost::shared_ptr<VideoSource<PointT> > raw_source_;
  boost::shared_ptr<FilteredVideoSource<PointT> > filtered_source_;

  boost::shared_ptr<PoseService> pose_service_;
  boost::shared_ptr<RobotService> robot_service_;
  boost::shared_ptr<Robot> robot_;

  boost::shared_ptr<IObstacleDetector> detector_;

  boost::shared_ptr<ObstacleVisualizer<PointT> > visualizer_;
};

/**
 * An implementation of the `Context` base class.
 *
 * It provides a hardcoded pipeline configuration, with only a relatively small
 * number of parameters that are configurable by passing command line options.
 *
 * The CLI arguments need to be passed to the `HardcodedContext` at
 * construct-time.
 */
template<class PointT>
class HardcodedContext : public Context<PointT> {
public:
  /**
   * Creates a new `HardcodedContext` based on the given CLI arguments.
   */
  HardcodedContext(char* argv[], int argc) : argv(argv), argc(argc) {
    live_ = checkLive();
    // Perform the initialization in terms of the provided template init.
    this->init();
  }

  /**
   * Returns whether the run is within a live-context.
   */
  bool isLive() { return live_; }
protected:
  /// Implementations of initialization of various parts of the pipeline.
  void initRawSource() {
    this->raw_source_ = GetVideoSource();
    if (!this->raw_source_) {
      throw "Unable to initialize the video source";
    }
  }

  void initFilteredVideoSource() {
    this->filtered_source_.reset(
        new SimpleFilteredVideoSource<PointT>(this->raw_source_));
  }

  void addFilters() {
    {
      double const a = 1.0117;
      double const b = -0.0100851;
      boost::shared_ptr<PointFilter<PointT> > filter(
          new SensorCalibrationFilter<PointT>(a, b));
      this->filtered_source_->addFilter(filter);
    }
    if (isLive()) {
      boost::shared_ptr<PointFilter<PointT> > filter(
          new RobotOdoTransformer<PointT>(this->pose_service_));
      this->filtered_source_->addFilter(filter);
    }
    {
      boost::shared_ptr<PointFilter<PointT> > filter(
          new TruncateFilter<PointT>(2));
      this->filtered_source_->addFilter(filter);
    }
  }

  void initPoseService() {
    this->pose_service_.reset(new PoseService("127.0.0.1", 5000));
    this->pose_service_->start();
  }

  void initVisionService() {
    boost::shared_ptr<AsyncRobotService> async_robot_service(
        new AsyncRobotService("127.0.0.1", 1337, 10));
    async_robot_service->start();
    this->robot_service_ = async_robot_service;
  }

  void initDetector() {
    // Prepare the base detector...
    base_detector_.reset(new BaseObstacleDetector<PointT>());
    this->source()->attachObserver(base_detector_);
    // Smooth out the basic detector by applying a smooth detector to it
    boost::shared_ptr<SmoothObstacleAggregator> smooth_detector(
        new SmoothObstacleAggregator);
    base_detector_->attachObstacleAggregator(smooth_detector);
    // Now the detector that is exposed via the context is a smoothed-out
    // base detector.
    this->detector_ = smooth_detector;
  }

  void addAggregators() {
    boost::shared_ptr<LolaAggregator> lola_viewer(
        new LolaAggregator("127.0.0.1", 53250));
    this->detector_->attachObstacleAggregator(lola_viewer);

    boost::shared_ptr<RobotAggregator> robot_aggregator(
        new RobotAggregator(*this->robot_service(), 30, *this->robot()));
    this->detector_->attachObstacleAggregator(robot_aggregator);
  }

  void initVisualizer() {
    // Factor out to a member ...
    bool visualization = true;
    if (visualization) {
      this->visualizer_.reset(new ObstacleVisualizer<PointT>());
      // Attach the visualizer to both the point cloud source...
      this->source()->attachObserver(this->visualizer_);
      // ...as well as to the obstacle detector
      this->detector_->attachObstacleAggregator(this->visualizer_);
    }
  }

private:
  /// Private helper member functions
  /**
   * Checks whether the CLI parameters indicate that the run should be "live",
   * i.e. whether the communication with the robot should be enabled.
   */
  bool checkLive() {
    for (int i = 0; i < argc; ++i) {
      if (std::string(argv[i]) == "--live") return true;
    }
    return false;
  }

  /**
   * Gets a `VideoSource` instance that corresponds to the CLI parameters.
   */
  boost::shared_ptr<VideoSource<PointT> > GetVideoSource() {
    if (argc < 2) {
      return boost::shared_ptr<VideoSource<PointT> >();
    }

    std::string const option = argv[1];
    if (option == "--stream") {
      return boost::shared_ptr<VideoSource<PointT> >(
          new LiveStreamSource<PointT>());
    } else if (option == "--pcd" && argc >= 3) {
      std::string const file_path = argv[2];
      boost::shared_ptr<pcl::Grabber> interface(new pcl::PCDGrabber<PointT>(
            file_path,
            20.,
            true));
      return boost::shared_ptr<VideoSource<PointT> >(
          new GeneralGrabberVideoSource<PointT>(interface));
    } else if (option == "--oni" && argc >= 3) {
      std::string const file_path = argv[2];
      boost::shared_ptr<pcl::Grabber> interface(new pcl::io::OpenNI2Grabber(
            file_path,
            pcl::io::OpenNI2Grabber::OpenNI_Default_Mode,
            pcl::io::OpenNI2Grabber::OpenNI_Default_Mode));
      return boost::shared_ptr<VideoSource<PointT> >(
          new GeneralGrabberVideoSource<PointT>(interface));
    }

    // Unknown option: return a "null" pointer.
    return boost::shared_ptr<VideoSource<PointT> >();
  }

  /// Private member variables

  // The CLI arguments
  char** const argv;
  int const argc;

  /**
   * Whether the run is "live".
   */
  bool live_;

  /**
   * The base detector that we attach to the video source and to which, in
   * turn, the "smooth" detector is attached. The `Context` maintains a
   * reference to it to make sure it doesn't get destroyed, although it is
   * never exposed to any outside clients.
   */
  boost::shared_ptr<BaseObstacleDetector<PointT> > base_detector_;
};

int main(int argc, char* argv[]) {
  // Initialize the context container
  boost::shared_ptr<Context<SimplePoint> > context;
  try {
      context.reset(new HardcodedContext<SimplePoint>(argv, argc));
  } catch (char const* exc) {
    std::cerr << exc << std::endl;
    PrintUsage();
    return 1;
  }
  // Get the video source and start it up
  context->source()->open();

  std::cout << "Waiting forever..." << std::endl;
  std::cout << "(^C to exit)" << std::endl;
  while (true)
    boost::this_thread::sleep(boost::posix_time::milliseconds(8000));

  return 0;
}
