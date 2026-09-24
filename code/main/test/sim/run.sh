#!/bin/sh
# Build the closed-loop simulator from the real firmware sources and sweep it
# over N randomised robots (see the header of sim.c for what is randomised).
#
#   test/sim/run.sh [N] [first_seed]        default: 40 robots from seed 1
#
# SIM_SCALE_PCT sets the gyro sensitivity error range (default 1 = +/-1%,
# a roughly calibrated GYRO_LSB_MS_PER_DEGREE; the MPU-6050 datasheet worst
# case uncalibrated is 3). Per-robot logs: $SIM_OUT/seed<N>.log.
set -e
cd "$(dirname "$0")/../.."
N=${1:-40}; S=${2:-1}
OUT=${SIM_OUT:-/tmp/agv_sim}
export SIM_SCALE_PCT=${SIM_SCALE_PCT:-1}
mkdir -p "$OUT"
cc -std=gnu99 -O2 -w -Itest/sim -I. -Dmain=fw_main -c main.c -o "$OUT/main.o"
cc -std=gnu99 -O2 -Wall -Wextra -Wno-unused-parameter -Itest/sim -I. -o "$OUT/sim" \
   test/sim/sim.c "$OUT/main.o" solver.c wallmem.c drive.c turn.c heading.c sonar.c \
   telemetry.c panel.c -lm
pass=0; fail=0; i=$S
while [ $i -lt $((S + N)) ]; do
  if "$OUT/sim" $i "$OUT/seed$i.log" q > "$OUT/seed$i.out" 2>&1; then
    pass=$((pass+1))
  else
    fail=$((fail+1)); grep '^seed' "$OUT/seed$i.out" || echo "seed $i: crashed"
  fi
  i=$((i+1))
done
echo "sim: $pass passed, $fail failed (gyro +/-$SIM_SCALE_PCT%, logs in $OUT)"
[ $fail -eq 0 ]
