#!/bin/sh

. ./asp3_test_common.sh

activate_athrill

#start test

echo "## start test"

athrill_remote c
sleep 1
athrill_remote q

#terminate all TASKs
athrill_remote "S 0 1e"
athrill_remote "S 0 2e"
athrill_remote "S 0 3e"
athrill_remote c

#wait 3sec
sleep 3
athrill_remote q

#activate TASK1
athrill_remote "S 0 1a"
athrill_remote c

#wait 3sec
sleep 3
athrill_remote q

echo "## end test"
deactivate_athrill


