#!/bin/sh

SAKURA_FILE="./arg_sakura.txt"

#geany -r `cat ${SAKURA_FILE}` &
FILE=$(cat ./arg_sakura.txt | awk '{print $3}')
LINE=$(cat ./arg_sakura.txt | awk '{print $2}')
code -r  -g ${FILE}:${LINE}