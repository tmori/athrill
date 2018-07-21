#!/bin/bash

. ./test/env.sh

cd trunk/src/build/target/linux_v850e2m
make clean;make

if [ -f ${ATHRILL_HOME}/bin/linux/athrill2 ]
then
	exit 0
else
	exit 1
fi
