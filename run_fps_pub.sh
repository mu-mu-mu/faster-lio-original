#!/bin/bash
set -xe
source /opt/ros/noetic/setup.bash

if [ $# -lt 1 ];
then
  echo "Failed: ./run_fps_pc.sh <mode> [addr]"
  exit
else
  MODE=$1
fi

PORT=9091
if [ $# -ge 2 ];
then
  ADDR=$2
else
  ADDR="127.0.0.1" #`/sbin/ip route | awk '/default/ { print $3 }'`
fi

echo Addr: $ADDR
echo Port: $PORT

TRAPINT () {
  set +eux
  trap '' INT TERM
  kill -TERM $pstart
}

echo "Run pse manager"

./build/devel/lib/faster_lio/publisher \
                   --bag_file ~/work/20130110.bag \
                   --config_file ./config/velodyne.yaml \
                   --rcb_mode $MODE \
                   --port $PORT \
                   --addr $ADDR &
pstart=$!
trap TRAPINT INT TERM
wait
