#!/bin/sh

. test/env.sh

TIMEOUT=4000000
rm -f log
athrill2 -c1 -m ${CONFIG_MEMORY} -d ${CONFIG_DEBUG} test/test_os/asp/ttsp/obj/check_library/exception/asp -t ${TIMEOUT} | tee ${TEST_LOG}/log.txt

egrep "Assert|failed|error" ${TEST_LOG}/log.txt 
if [ $? -ne 0 ]
then
    echo "PASSED: $0"
    exit 0
else
    echo "FAILED: $0"
    exit 1
fi
