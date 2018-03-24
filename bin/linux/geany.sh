#!/bin/sh

SAKURA_FILE="./arg_sakura.txt"

which code > /dev/null
if [ $? -eq 0 ]
then
    FILE=$(cat ./arg_sakura.txt | awk '{print $3}')
    LINE=$(cat ./arg_sakura.txt | awk '{print $2}')
    code -r  -g ${FILE}:${LINE}
else
    which geany > /dev/null
    if [ $? -eq 0 ]
    then
        geany -r `cat ${SAKURA_FILE}` &
    else
        echo "ERROR not found editor: please install vscode or geany!"
        exit 1
    fi
fi

exit 0