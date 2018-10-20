#!/bin/bash

ROOT_DIR=../..
export CLIB_DIR=${ROOT_DIR}/library
export GENERATOR_DIR=${CLIB_DIR}/generator
export GENERATED_DIR=${CLIB_DIR}/c/generated
export ROSWORK_DIR=${ROOT_DIR}/ros_ws
export ROSWORK_LAUNC_DIR=${ROSWORK_DIR}/launch
export ROSWORK_VECU_PROXY_SRC_DIR=${ROSWORK_DIR}/src/virtual_ecu_proxy/src

export GENERATED_MMAP_FILE_PREFIX="athrill"
export GENERATED_MMAP_PATH=$(cd ${ROOT_DIR}/command; pwd)
