#!/bin/bash
set -e
PS_PATH=./pubsub_ext
MANAGER=psext_manager

ADDR=`/sbin/ip route | awk '/default/ { print $3 }'`
PORT=9091

TRAPINT () {
  set +eux
  trap '' INT TERM
  pkill ${MANAGER}
}

echo "Run pse manager:"
echo Address: $ADDR
echo Port: $PORT

${PS_PATH}/${MANAGER} -p ${PORT} &

trap TRAPINT INT TERM
wait
