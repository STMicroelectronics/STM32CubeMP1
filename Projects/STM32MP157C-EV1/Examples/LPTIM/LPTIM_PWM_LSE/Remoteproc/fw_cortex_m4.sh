#!/bin/sh

rproc_class_dir="/sys/class/remoteproc/remoteproc0/"
fmw_dir="/lib/firmware"
rproc_ta="optee-ta-80a4c275-0a47-4905-8285-1486a9771a08"
project_name=$(basename $(pwd))
fmw_name="${project_name}.elf"
if [ -e /sys/bus/tee/devices/${rproc_ta} ]; then
    #The firmware is managed by OP-TEE, it must be signed.
    if [ ! -e lib/firmware/${project_name}.sig ] && [ $1 == "start" ]; then
        echo  "Error: only signed firmware can be loaded"
        exit 1
    fi
    fmw_name="${project_name}.sig"
fi
echo "`basename ${0}`: fmw_name=${fmw_name}"
rproc_state=`tr -d '\0' < $rproc_class_dir/state`

error() {
	echo -e "$1"
	exit 0
}


case $1 in
	start) ;;
	stop) ;;
	*) echo "`basename ${0}`:usage: start | stop"
	   exit 1
	   ;;
esac

#################
# Start example #
#################
if [ $1 == "start" ]
then

if [ $rproc_state == "running" ]
then
echo "Stopping running fw ..."
echo stop > $rproc_class_dir/state
fi

# Create /lib/firmware directory if not exist
if [ ! -d $fmw_dir ]
then
echo "Create $fmw_dir directory"
mkdir $fmw_dir
fi
# Copy firmware in /lib/firmware
cp lib/firmware/$fmw_name $fmw_dir/

# load and start firmware
echo $fmw_name > $rproc_class_dir/firmware
echo start > $rproc_class_dir/state

fi


################
# Stop example #
################
if [ $1 == "stop" ]
then

if [ $rproc_state == "offline" ]
then
echo "Nothing to do, no M4 fw is running"

else
echo stop > $rproc_class_dir/state

fi
fi