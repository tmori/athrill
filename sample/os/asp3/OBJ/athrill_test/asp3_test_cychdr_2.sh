#!/bin/sh

. ./asp3_test_common.sh

activate_athrill_quick

#start test

echo "## start test"

athrill_remote c

wait_athrill 1

athrill_remote "S 0 c"
athrill_remote c

#wait 60sec
echo "sleep 60sec"
wait_athrill 60

athrill_remote "S 0 C"
athrill_remote c

#wait 60sec
echo "sleep 60sec"
wait_athrill 60

echo "## end test"
deactivate_athrill