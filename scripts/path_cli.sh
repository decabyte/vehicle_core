#!/bin/bash

# usage
print_usage() {
    echo "Usage: $(basename $0) <cmd>"
    echo "Send the <cmd> command to the path controller node."
    echo ""
    echo "Mandatory arguments:"
    echo "  <cmd>: either [start | pause | reset]"
    echo ""
    #echo "Optional arguments:"
    #echo "  <size>: size in MB of the single chunks [default: 2048]"
}

# script body
if [[ ! -n $1 ]]; then
    print_usage
    exit 1
fi

# vars
CMD=$1

case $CMD in
    start)
        echo "Sending start command (please press Ctrl+C to interrupt) ..."
    ;;

    reset)
        echo "Sending reset command (please press Ctrl+C to interrupt) ..."
    ;;

    pause)
        echo "Sending pause command (please press Ctrl+C to interrupt) ..."
    ;;

    *)
        echo "Wrong input, dying!"
        exit 1
    ;;
esac

# wait and send command
sleep 1

rosservice call /path/control """header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
command: '${CMD}'
"""

# send the last exit code
exit $?
