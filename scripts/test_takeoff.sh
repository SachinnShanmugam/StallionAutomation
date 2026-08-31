#!/bin/bash
set -e

pkill -f arduplane 2>/dev/null || true
sleep 1

# Instance 94 → SERIAL0 TCP port = 5760 + 94*10 = 6700
INSTANCE=94
PORT=$((5760 + INSTANCE * 10))

echo "Starting SITL (instance=$INSTANCE, port=$PORT)..."
cd ~/ardupilot
~/ardupilot/build/sitl/bin/arduplane \
  --model quadplane-tilttri \
  --defaults Tools/autotest/default_params/quadplane.parm,Tools/autotest/default_params/quadplane-tilttri.parm,/mnt/c/Users/SACHIN/Stallion/params/stallion_vtol_sitl.parm \
  --home 13.0827,80.2707,10,90 \
  -I${INSTANCE} > /tmp/sitl${INSTANCE}.log 2>&1 &

SITL_PID=$!
echo "SITL started, PID=$SITL_PID, port=$PORT"
echo "Waiting 10s for boot..."
sleep 10

echo "Launching mission on port $PORT..."
sed -i "s/^port = .*/port = ${PORT}/" /tmp/start_mission.py
python3 /tmp/start_mission.py
RESULT=$?

kill -9 $SITL_PID 2>/dev/null || true
exit $RESULT
