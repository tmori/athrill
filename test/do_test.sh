#!/bin/sh

. ./test/env.sh

echo "#START TEST ASP TARGET TIMER"
sh test/scripts/do_test_asp_target_timer.sh
if [ $? -ne 0 ]
then
    exit 1
fi

echo "#START TEST ASP TARGET INTERRUPT"
sh test/scripts/do_test_asp_target_interrupt.sh
if [ $? -ne 0 ]
then
    exit 1
fi

echo "#START TEST ASP TARGET EXCEPTION"
sh test/scripts/do_test_asp_target_exception.sh
if [ $? -ne 0 ]
then
    exit 1
fi

exit 0