#!/bin/sh

GROOVY_PATH=$(dirname $(which $0))
groovy ${GROOVY_PATH}/variable_conflict_check.groovy $*
