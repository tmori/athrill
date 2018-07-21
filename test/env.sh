#!/bin/sh

export ATHRILL_HOME=${HOME}/build/tmori/athrill
export PATH=${PATH}:${ATHRILL_HOME}/bin/linux
export CONFIG_MEMORY=${HOME}/build/tmori/athrill/test/config/memory.txt
export CONFIG_DEBUG=${HOME}/build/tmori/athrill/test/config/device_config.txt
export TEST_LOG=${HOME}/build/tmori/athrill/test/scripts/log
export EXPECT_LOG=${HOME}/build/tmori/athrill/test/scripts/expect_log

