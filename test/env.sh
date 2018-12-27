#!/bin/bash

export ATHRILL_HOME=$(cd $(dirname $BASH_SOURCE);cd ..; pwd)
export PATH=${PATH}:${ATHRILL_HOME}/bin/linux
export CONFIG_MEMORY=${ATHRILL_HOME}/test/config/memory.txt
export CONFIG_DEBUG=${ATHRILL_HOME}/test/config/device_config.txt
export TEST_LOG=${ATHRILL_HOME}/test/scripts/log

CMD_NAME=
function util_set_cmdname() {
    CMD_NAME=$( echo ${1} | awk -F\. '{print $1}' | awk -F\/ '{print $NF}' )
    rm -f ${TEST_LOG}/${CMD_NAME}.log
}

TEST_TIMEOUT=
TEST_BIN=
function util_do_test() {
    athrill2 -c1 -m ${CONFIG_MEMORY} -d ${CONFIG_DEBUG} ${TEST_BIN} -t ${TEST_TIMEOUT} | tee -a ${TEST_LOG}/${CMD_NAME}.log
}

function util_do_log_check() {
    egrep "Assert|failed|error" ${TEST_LOG}/${CMD_NAME}.log 
    if [ $? -ne 0 ]
    then
        echo "PASSED: $0"
        exit 0
    else
        echo "FAILED: $0"
        exit 1
    fi
}
