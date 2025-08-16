#!/bin/bash

mkdir -p $1 
cd $1
export PICO_SDK_PATH=../pico-sdk
if [ "$#" -eq 4 ]; then
	command_to_run="cmake .. -DN_ITER=$2 -DDO_HANN=$3 -DINCLUDE_WINDOW_IN_TIMING=$4"
	eval "$command_to_run"
	make -j
	exit 0
fi

if [ "$#" -eq 1 ]; then
	command_to_run="cmake .. -DN_ITER=500 -DDO_HANN=0 -DINCLUDE_WINDOW_IN_TIMING=0"
	eval "$command_to_run"
	make -j
	exit 0
fi

