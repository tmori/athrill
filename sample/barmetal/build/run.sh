#!/bin/sh

make clean > /dev/null
make > /dev/null

if [ -f test_main.elf ]
then
	athrill -i -m ../build/v850esfk3/memory.txt test_main.elf
else
	echo "build error"
fi
