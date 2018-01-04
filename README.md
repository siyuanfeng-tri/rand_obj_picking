To build
bazel run --config snopt --config gurobi --copt -g :install ~/drake_bazel_build
1. Point set(drake_DIR "$ENV{HOME}/drake_bazel_build/lib/cmake/drake") correctly in CMakeLists.txt

2.
$ mkdir build
$ cd build
$ cmake ..
$ make -j
