#!/bin/sh

SAKURA_FILE="./arg_sakura.txt"

sakura -R `cat ${SAKURA_FILE}` &
