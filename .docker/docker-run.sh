#!/bin/bash

if [[ "$(docker images -q ontologenius-melodic 2> /dev/null)" == "" ]]; then
    ./docker-build.sh
fi

docker run -it -v $(pwd)/../../..:/root/catkin_ws ontologenius-melodic "$@"
