# lepp2
[ ![Codeship Status for mlalic/lepp2](https://codeship.com/projects/37907730-6977-0132-2878-1a44e1163757/status?branch=master)](https://codeship.com/projects/53558)

LOLA Environment Perception Package v2: understanding the environment sensed by
a Kinect-like sensor.

# Goals

The goal of the project is to approximate arbitrary objects in the video feed
of a Kinect-like sensor, using a number of basic geometric shapes, such as
spheres and cylinders, *in real time*.

The approximations can then be used as a basis for a robot control system,
providing it with precise information on the positions of obstacles.

As such, in order to meet the real-time requirement, precision in the
generated approximations is sometimes sacrificed, but with the goal of
never under-approximating an object, so that the robot does not
accidentaly end up colliding with it.

# Compiling

The project depends on the [PCL](http://pointclouds.org/) library. You should
first install it following PCL's directions.

Then, you can compile the `detector` using `cmake`. In order to compile the
project with only the default options set, use the `build.sh` script.

# Usage

The `detector` executable is built by default and has a number of flags
that can be passed to it from the CLI. Check its usage by running it with
no flags.

For some examples of how to use the library itself, you may check the
`examples` directory.
