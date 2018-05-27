#!/bin/sh

function athrill_cont() {
    athrill_remote c ${1}
}

CPU_ELAPS=""
function athrill_elaps() {
    RET=`athrill_remote e`
    CPU_ELAPS=`echo ${RET} | awk '{print $1}'`
}

MEMORY_VALUE=""
function athrill_print_memory() {
    ADDR=${1}
    SIZE=${2}

    if [ ${SIZE} -gt 4 ]
    then
        echo "ERROR: can not print memory data size over 4bytes"
    else 
        RET=`athrill_remote p ${ADDR} ${SIZE}`
        MEMORY_VALUE=`echo ${RET} | awk '{print $1}'`
    fi
}

VARIABLE_VALUE=""
function athrill_print_variable() {
    RET=`athrill_remote p ${1}`
    ADDR=`echo ${RET} | awk '{print $1}'`
    SIZE=`echo ${RET} | awk '{print $2}'`
    athrill_print_memory ${ADDR} ${SIZE}
    VARIABLE_VALUE=${MEMORY_VALUE}
}


#SAMPLE USAGE
#athrill_cont 10
#athrill_elaps
#echo ${CPU_ELAPS}
#athrill_print_variable gl_variable1
#echo ${VARIABLE_VALUE}
#athrill_print_memory 0x6ff7408 2
#echo ${VARIABLE_VALUE}
