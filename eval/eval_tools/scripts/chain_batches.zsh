#!/usr/bin/env zsh
# Run matrix configs back to back, unattended.
#
# A batch must not start while another is alive: two run_matrix processes put
# two simulators on the same topics, which has silently corrupted runs before.
# So each step waits for the runner to clear, then settles before launching.
#
# Usage: chain_batches.zsh CONFIG [CONFIG ...]
set -u
cd ~/delivery/MasterProject/ros_ws/src/ActiveSlam || exit 1
source /opt/ros/jazzy/setup.zsh
source ~/delivery/MasterProject/ros_ws/install/setup.zsh

LOG=/tmp/chain_batches.log
echo "chain started $(date +%H:%M:%S) for: $*" >> $LOG

wait_for_clear() {
  while pgrep -f "run_matrix[.]py" >/dev/null 2>&1; do sleep 30; done
  sleep 45                      # let Stonefish and the recorder finish teardown
  python3 tools/stop_all.py >> $LOG 2>&1
  sleep 10
}

for cfg in "$@"; do
  wait_for_clear
  if [[ ! -f "$cfg" ]]; then
    echo "$(date +%H:%M:%S) MISSING CONFIG $cfg -- skipping" >> $LOG
    continue
  fi
  echo "$(date +%H:%M:%S) launching $cfg" >> $LOG
  python3 eval/eval_tools/scripts/run_matrix.py "$cfg" >> $LOG 2>&1
  echo "$(date +%H:%M:%S) finished $cfg (exit $?)" >> $LOG
done
echo "chain complete $(date +%H:%M:%S)" >> $LOG
