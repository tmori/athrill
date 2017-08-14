#!/bin/sh

#PARAMETERS
WAIT_TIME=2
ASP_BIN=../asp

#activate asp
activate_athrill()
{
	if [ -f ${ASP_BIN} ]
	then
		athrill -r -i -d device_config.txt ../asp &
		
		echo "wait for start ${WAIT_TIME} sec.."
		sleep ${WAIT_TIME}
	else
		echo "Not found:${ASP_BIN}"
		exit 1
	fi
}
activate_athrill_quick()
{
	if [ -f ${ASP_BIN} ]
	then
		athrill -r -i -d device_quick_config.txt ../asp &
		
		echo "wait for start ${WAIT_TIME} sec.."
		sleep ${WAIT_TIME}
	else
		echo "Not found:${ASP_BIN}"
		exit 1
	fi
}

wait_athrill()
{
	sleep ${1}
	athrill_remote q
}

deactivate_athrill()
{
	athrill_remote "exit"
}