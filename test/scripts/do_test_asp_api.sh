#!/bin/sh

. test/env.sh

util_set_cmdname $0

echo "##AUTO_CODE_1"
TEST_TIMEOUT=60000000
TEST_BIN=test/test_os/asp/ttsp/obj/api_test/auto_code_1/asp
util_do_test 

echo "##AUTO_CODE_2"
TEST_TIMEOUT=60000000
TEST_BIN=test/test_os/asp/ttsp/obj/api_test/auto_code_2/asp
util_do_test 

echo "##AUTO_CODE_3"
TEST_TIMEOUT=80000000
TEST_BIN=test/test_os/asp/ttsp/obj/api_test/auto_code_3/asp
util_do_test 

util_do_log_check