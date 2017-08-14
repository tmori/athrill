#!/bin/sh

sh -x asp3_test_dispatcher_1.sh 2>&1 | tee dispatch1.log
sh -x asp3_test_dispatcher_2.sh 2>&1| tee dispatch2.log
sh -x asp3_test_dispatcher_3.sh 2>&1| tee dispatch3.log

sh -x asp3_test_alarmhdr_1.sh 2>&1 | tee alarm1.log
sh -x asp3_test_alarmhdr_2.sh 2>&1 | tee alarm2.log

sh -x asp3_test_cychdr_1.sh 2>&1 | tee cyc1.log
sh -x asp3_test_cychdr_2.sh 2>&1 | tee cyc2.log

