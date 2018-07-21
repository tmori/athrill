#!/bin/sh

. test/env.sh

TIMEOUT=2000000
rm -f log
athrill2 -c1 -m ${CONFIG_MEMORY} -d ${CONFIG_DEBUG} test/test_os/asp/ttsp/obj/check_library/interrupt/asp -t ${TIMEOUT} | tee ${TEST_LOG}/log.txt

diff ${TEST_LOG}/log.txt ${EXPECT_LOG}/do_test_asp_target_interrupt.log > /dev/null
if [ $? -eq 0 ]
then
    echo "PASSED: $0"
    exit 0
else
    echo "FAILED: $0"
    exit 1
fi
