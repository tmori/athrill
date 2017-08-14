#!/bin/sh

. ./asp3_test_common.sh

activate_athrill

#start test

echo "## start test"

athrill_remote c

wait_athrill 1

#wait TASK1
athrill_remote "S 0 1>s"
athrill_remote c

#wait 3sec
wait_athrill 3

#wakeup TASK1
athrill_remote "S 0 1w"
athrill_remote c

#wait 3sec
wait_athrill 3

echo "## end test"
deactivate_athrill


