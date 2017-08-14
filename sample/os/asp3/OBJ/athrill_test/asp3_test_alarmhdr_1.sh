#!/bin/sh

. ./asp3_test_common.sh

activate_athrill_quick

#start test

echo "## start test"

athrill_remote c

wait_athrill 1

athrill_remote "S 0 b"
athrill_remote c

echo "sleep 120sec"
wait_athrill 120

echo "## end test"
deactivate_athrill