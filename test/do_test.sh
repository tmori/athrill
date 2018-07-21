#!/bin/bash

echo "#START TEST ASP TARGET TIMER"
bash test/scripts/do_test_asp_target_timer.sh
if [ $? -ne 0 ]
then
    exit 1
fi

echo "#START TEST ASP TARGET INTERRUPT"
bash test/scripts/do_test_asp_target_interrupt.sh
if [ $? -ne 0 ]
then
    exit 1
fi

echo "#START TEST ASP TARGET EXCEPTION"
bash test/scripts/do_test_asp_target_exception.sh
if [ $? -ne 0 ]
then
    exit 1
fi

echo "#START TEST ASP SIL"
bash test/scripts/do_test_asp_sil.sh
if [ $? -ne 0 ]
then
    exit 1
fi

echo "#START TEST ASP API"
bash test/scripts/do_test_asp_api.sh
if [ $? -ne 0 ]
then
    exit 1
fi

exit 0