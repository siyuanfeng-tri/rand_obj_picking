##################################################################
To build
bazel run --config snopt --config gurobi --copt -g :install ~/drake_bazel_build
1. Change set(drake_DIR "$ENV{HOME}/drake_bazel_build/lib/cmake/drake") to
the correct location in CMakeLists.txt

2.
$ git submodule update --init
$ mkdir build
$ cd build
$ cmake ..
$ make -j

##################################################################
iiwa_controller sets its control thread to real time priority through
some system call. In order to run it without sudoing everytime, you need to
set some bits on the host system.

Add the following line:
*                -       rtprio         90
to
/etc/security/limits.conf
* means for everybody.
rtprio means the real time scheduler, and you can ask for priority up to 90.
