#!/bin/bash

source ../env/env.sh

CURDIR=`pwd`

cd ${ROSWORK_DIR}
source ./devel/setup.bash

roslaunch ./launch/hakoniwa.launch