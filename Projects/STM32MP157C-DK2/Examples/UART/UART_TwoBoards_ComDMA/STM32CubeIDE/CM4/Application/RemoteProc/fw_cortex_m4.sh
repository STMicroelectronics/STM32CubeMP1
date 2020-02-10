#!/bin/sh

rproc_class_dir="/sys/class/remoteproc/remoteproc0"
fmw_dir="/lib/firmware"
fmw_name="UART_TwoBoards_ComDMA_CM4.elf"


if [ $1 == "start" ]
then
	# Start the firmware
	cp lib/firmware/$fmw_name $fmw_dir
	echo -n "$fmw_name" > $rproc_class_dir/firmware
	echo -n start > $rproc_class_dir/state
fi

if [ $1 == "stop" ]
then
	# Stop the firmware
	echo -n stop > $rproc_class_dir/state
fi
