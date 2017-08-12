#!/bin/sh

if [ $# -ne 2 ]
then
	echo "Usage: $0 test_item template.csv"
	exit 1
fi

if [ -z ${ATHRILL_PATH} ]
then
	echo "Please set env ATHRILL_PATH"
	exit 1
fi

T_PATH=${ATHRILL_PATH}/test/tool

IFLAGS="-I ${T_PATH}/TestObject -I ${T_PATH}/TestObject/target -I ${T_PATH}/TestObject/item -I ${T_PATH}/TestObjectFactory -I ${T_PATH}/TestCodeGenerator -I ${T_PATH}/util"


ruby ${IFLAGS} ${T_PATH}/TestCodeGenerator/TestCodeGenerator.rb ${1} ${2} ${T_PATH}

