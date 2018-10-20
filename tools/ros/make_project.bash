#!/bin/bash

if [ $# -ne 1 ]
then
    echo "Usage: $0 <nodename>"
    exit 1
fi

NODE_NAME=${1}

if [ -d projects/${NODE_NAME} ]
then
    :
else
    mkdir projects/${NODE_NAME}
fi

cp -rp project_template/* projects/${NODE_NAME}/
