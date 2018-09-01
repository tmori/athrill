#!/bin/bash

TARGET_NAME=virtual_ecu_proxy
SRC_FILES=../library/c/generated/${TARGET_NAME}*
DST_DIR=src/virtual_ecu_proxy/src
rm -f ${DST_DIR}/${TARGET_NAME}*
cp ${SRC_FILES} ${DST_DIR}/

rm -rf devel
rm -rf build

catkin_make --pkg virtual_can_bus
catkin_make --pkg virtual_ecu_proxy
