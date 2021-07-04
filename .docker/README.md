# Using a docker container to build ontologenius

This folder contains scripts and configuration files to use a docker container to build and run
ontologenius. It is based on ROS melodic with perception packages, but can be easily adapted to
other versions of ROS.

## Instructions

* ensure ontologenius is located in a path like *catkin_ws/src/ontologenius/*
* go into *ontologenius/.docker* folder
* run ```./docker-run.sh catkin_make```


# Using the sanitizers to detect bugs

Ontologenius supports sanitizers from GCC and Clang to detect bugs like memory leaks, use after free
and such. To activate a sanitizer, you must run CMake with an option (-D) saying which sanitizer you
want to activate and for which target. You can use multiple sanitizers for a given target, and you
can also configure multiple targets at once.

The option has the following format: -D<target_name>_SANITIZE_<sanitizer_name>=ON

For instance, to enable Address Sanitizer (ASAN) for ontologenius_ontoGraphs_lib module, use the
following option: -Dontologenius_ontoGraphs_lib_SANITIZE_address=ON

The supported sanitizers are:
* Address Sanitizer (address)
* Memory Sanitizer (memory)
* Undefined Sanitizer (undefined)
* Thread Sanitizer (thread)

For more information on sanitizers, see -fsanitize options on GCC manual
https://gcc.gnu.org/onlinedocs/gcc/Instrumentation-Options.html

## Instructions

* build ontologenius with sanitizers activated for the desired targets:
```
catkin_make -Dontologenius_ontoGraphs_lib_SANITIZE_address=ON
```
* run your tests, for example:
```
catkin_make run_tests_ontologenius_rostest
```
