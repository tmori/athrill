#!/bin/bash

if [ $# -ne 1 ]
then
    echo "Usage: $0 <nodename>"
    exit 1
fi

export NODE_NAME=${1}
MO_SCRIPT=project_template/bash_template_engine/mo

if [ -d projects/${NODE_NAME} ]
then
    :
else
    mkdir projects/${NODE_NAME}
fi

cp -rp project_template/* projects/${NODE_NAME}/
mkdir -p projects/${NODE_NAME}/library/c/generated


function call_template_engine()
{
    org_template=${1}
    out_file=${2}
    bash ${MO_SCRIPT} ${org_template} > ${out_file}
}

ORG_TEMPLATE=project_template/ros_ws/launch/hakoniwa_launch.mo
OUT_FILE=projects/${NODE_NAME}/ros_ws/launch/hakoniwa.launch
call_template_engine ${ORG_TEMPLATE} ${OUT_FILE}

ORG_TEMPLATE=project_template/ros_ws/src/virtual_ecu_proxy/CMakeLists_txt.mo
OUT_FILE=projects/${NODE_NAME}/ros_ws/src/virtual_ecu_proxy/CMakeLists.txt
call_template_engine ${ORG_TEMPLATE} ${OUT_FILE}

ORG_TEMPLATE=project_template/library/generator/template_csv.mo
OUT_FILE=projects/${NODE_NAME}/library/generator/template.csv
call_template_engine ${ORG_TEMPLATE} ${OUT_FILE}

ORG_TEMPLATE=project_template/library/generator/generate_bash.mo
OUT_FILE=projects/${NODE_NAME}/library/generator/generate.bash
call_template_engine ${ORG_TEMPLATE} ${OUT_FILE}
