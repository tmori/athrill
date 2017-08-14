#!/bin/sh

. ./asp3_test_common.sh

activate_athrill_quick

#start test

echo "## start test"

athrill_remote c

wait_athrill 1

#wait TASK1
athrill_remote "S 0 1>d"
athrill_remote c

#wait 2min
echo "sleep 2min"
wait_athrill 120

echo "## end test"
deactivate_athrill


