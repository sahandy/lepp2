/**
 * A program performing detection of obstacles in the given file feed
 * and visualizing their approximations.
 */
#include <iostream>

#include <pcl/io/openni2_grabber.h>

#include "lepp2/BaseObstacleDetector.hpp"
#include "lepp2/GrabberVideoSource.hpp"
#include "lepp2/BaseVideoSource.hpp"
#include "lepp2/VideoObserver.hpp"

#include "lepp2/visualization/EchoObserver.hpp"


using namespace lepp;

/**
 * Prints out the expected CLI usage of the program.
 */
void PrintUsage() {
  std::cout << "The program expects a single argument: a path to an .oni file"
      << " which will serve as the input to the obstacle detector" << std::endl;
}


int main(int argc, char* argv[]) {
  if (argc != 2) {
    PrintUsage();
    return 1;
  }

  std::string const file_name = argv[1];

  char const* file_path = argv[1];

  boost::shared_ptr<pcl::Grabber> interface(new pcl::io::OpenNI2Grabber(
    file_path,
    pcl::io::OpenNI2Grabber::OpenNI_Default_Mode,
    pcl::io::OpenNI2Grabber::OpenNI_Default_Mode));
  boost::shared_ptr<SimpleVideoSource> source(
      new GeneralGrabberVideoSource<SimplePoint>(interface));

  boost::shared_ptr<SimpleVideoSource::ObserverType> observer(
      new EchoObserver<SimplePoint>());
  source->attachObserver(observer);

  boost::shared_ptr<BaseObstacleDetector<SimplePoint> > detector(
      new BaseObstacleDetector<SimplePoint>());
  source->attachObserver(detector);

  // Starts capturing new frames and forwarding them to attached observers.
  source->open();

  std::cout << "Waiting for 8 seconds..." << std::endl;
  boost::this_thread::sleep(boost::posix_time::milliseconds(8000));
  std::cout << "Example finished." << std::endl;

  return 0;
}
