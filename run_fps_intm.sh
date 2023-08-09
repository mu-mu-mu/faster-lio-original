#!/bin/bash
set -xe
PS_PATH=./pubsub_ext
MANAGER=psext_manager

source /opt/ros/noetic/setup.bash

set -eu

if [ $# -lt 1 ];
then
  echo "Failed: ./run_fps_pc.sh <mode> [addr]"
  exit
else
  MODE=$1
fi

if [ "$MODE" = "none" ]
then
  RCB_PATH=none
elif [ "$MODE" = "pc" ]
then
  RCB_PATH=./build/devel/lib/libpcpp_native.so
elif [ "$MODE" = "pc2" ]
then
  RCB_PATH=./build/devel/lib/libpcpp_native_direct.so
else
  echo "Failed: mode (node/pc/pc2)"
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
  kill -TERM $pintm
  pkill ${MANAGER}
}

echo "Run pse manager"
set +e
pkill ${MANAGER}
set -e
${PS_PATH}/${MANAGER} -p ${PORT} &
sleep 1


./build/devel/lib/faster_lio/run_mapping_fps \
                   --config_file ./config/velodyne.yaml \
                   --rcb_mode $MODE \
                   --port $PORT \
                   --addr $ADDR \
                   --rcb_path $RCB_PATH &
pintm=$!
trap TRAPINT INT TERM
wait
