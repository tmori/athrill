#!/bin/sh

. ./asp3_test_common.sh

activate_athrill

#start test

echo "## start test"

athrill_remote c

wait_athrill 1

#terminate all TASKs
athrill_remote "S 0 1e"
athrill_remote "S 0 2e"
athrill_remote "S 0 3e"
athrill_remote c

#wait 3sec
wait_athrill 3

#activate TASK1
athrill_remote "S 0 1a"
athrill_remote c

#wait 3sec
wait_athrill 3

echo "## end test"
deactivate_athrill


