#!/bin/bash

# vars
PREFIX=$1
SIZE=$2
OPTS='-x "bvt_MB2250(.*)|bvt_P900(.*)"'

# usage
print_usage() {
    echo "Usage: $(basename $0) <prefix> [<size>]"
    echo "Record all messages into a splitted .bag file with the same <prefix>"
    echo ""
    echo "Mandatory arguments:"
    echo "  <prefix>: meanining full name of the bag files set"
    echo ""
    echo "Optional arguments:"
    echo "  <size>: size in MB of the single chunks [default: 2048]"
}

# script body
if [[ ! -n $1 ]]; then
    print_usage
    exit 1
fi

if [[ ! -n $2 ]]; then
fi

# recording
echo "Starting the recording (please press Ctrl+C to terminate) ..."
rosbag record -a --split --size=$SIZE -o $PREFIX $OPTS

# send the last exit code
exit $?
