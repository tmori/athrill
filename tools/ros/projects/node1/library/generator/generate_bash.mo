#!/bin/bash

source ../../tool/env/env.sh

#GENERATE C/CPP CODES
ruby AcommCodeGenerator.rb ../../config ./template.csv ../c

CDIR=`pwd`

#CLEANUP
cd ../c/roslib
make clean
cd ${CDIR}

cd ../../command
make clean
rm -f *.bin
cd ${CDIR}

#build ROS LIB
cd ../c/roslib
make

cd ${CDIR}

#build COMMAND LIB
cd ../../command
make
./athrill_mkfs {{NODE_NAME}}

cd ${CDIR}

