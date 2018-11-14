#!/bin/bash

source ../env/env.sh

CURDIR=`pwd`
### CLIB GENERATION ###
#DO
cd ${GENERATOR_DIR}
bash generate.bash

#DONE
cd ${CURDIR}

### ROS INSTALL ###
#PREPARE
TARGET_NAME=virtual_ecu_proxy
SRC_FILES=${GENERATED_DIR}/${TARGET_NAME}*
DST_DIR=${ROSWORK_VECU_PROXY_SRC_DIR}
if [ -d ${DST_DIR} ]
then
	:
else
	mkdir -p ${DST_DIR}
fi
rm -f ${DST_DIR}/${TARGET_NAME}*
cp ${SRC_FILES} ${DST_DIR}/

#DO
cd ${ROSWORK_DIR}
rm -rf devel
rm -rf build

catkin_make --pkg virtual_can_bus
catkin_make --pkg virtual_ecu_proxy

#DONE
cd ${CURDIR}
