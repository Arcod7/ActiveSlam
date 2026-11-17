#!/usr/bin/env zsh
# Wait for the batch chain to finish, then run every analysis and write one
# report. Each step is independent so a failure in one still leaves the others.
set -u
cd ~/delivery/MasterProject/ros_ws/src/ActiveSlam || exit 1
source /opt/ros/jazzy/setup.zsh
source ~/delivery/MasterProject/ros_ws/install/setup.zsh

OUT=~/delivery/MasterProject/ActiveSlam-Resources/dissertation/data/night1_results.md
FIG=eval/figures/revisit_campaign_night1

# The chain script is the thing to wait on; run_matrix goes quiet between batches.
while pgrep -f "chain_batches" >/dev/null 2>&1; do sleep 60; done
while pgrep -f "run_matrix[.]py" >/dev/null 2>&1; do sleep 60; done
sleep 60

# (N) is the null_glob qualifier: an unmatched pattern expands to nothing
# instead of aborting the script, so a batch that never ran costs its own
# section rather than the whole report.
A=(eval/runs/tail_currentcfg_*(N))
B=(eval/runs/revisit_complete_*(N))
C=(eval/runs/rtm_control_*(N))
echo "resolved batches: A=${#A} B=${#B} C=${#C}" >> /tmp/chain_batches.log

{
  echo "# Night 1 results — generated $(date '+%Y-%m-%d %H:%M')"
  echo
  echo "Batches: A=${A:t} B=${B:t} C=${C:t}"
  echo "Run counts: A=$(ls -d $A/*_s*/manifest.json 2>/dev/null | wc -l)"\
       "B=$(ls -d $B/*_s*/manifest.json 2>/dev/null | wc -l)"\
       "C=$(ls -d $C/*_s*/manifest.json 2>/dev/null | wc -l)"
  echo
  echo '## Run-level comparison (secondary — see the power analysis)'
  echo '```'
  python3 eval/eval_tools/scripts/analyse_tail.py $A $B $C \
      --alias lc_repeat=lc \
      --contrast lc:lc_revisit \
      --contrast lc:lc_revisit_complete \
      --contrast lc_revisit:lc_revisit_complete 2>&1
  echo '```'
  echo
  echo '## Episode-level comparison (PRIMARY endpoint)'
  echo '```'
  python3 eval/eval_tools/scripts/analyse_episodes.py $A $B --control-arm lc 2>&1
  echo '```'
  echo
  echo '## Did the vehicle reach its target? (F4 / geodesic test)'
  echo '```'
  python3 eval/eval_tools/scripts/revisit_arrival.py $A $B 2>&1 | grep -v "^\["
  echo '```'
  echo
  echo '## Noise floor from byte-identical pairs (batch A lc vs batch C lc_repeat)'
  echo '```'
  python3 - "$A" "$C" <<'PY' 2>&1
import csv, glob, json, os, sys
import numpy as np
def drift(d):
    m=json.load(open(f'{d}/manifest.json'))
    a=[float(r['ate']) for r in csv.DictReader(open(f'{d}/metrics.csv')) if r.get('ate') not in (None,'','nan')]
    return 100*a[-1]/m['gt_path_m'] if a and m.get('gt_path_m') else None
A,C=sys.argv[1],sys.argv[2]
pairs=[]
for d in sorted(glob.glob(f'{A}/lc_s*')):
    s=os.path.basename(d).rsplit('_s',1)[1]
    c=f'{C}/lc_repeat_s{s}'
    if os.path.exists(f'{c}/manifest.json') and os.path.exists(f'{d}/manifest.json'):
        x,y=drift(d),drift(c)
        if x and y: pairs.append((s,x,y,100*abs(x-y)/np.mean([x,y])))
if not pairs:
    print('no matched pairs yet')
else:
    print(f'{"seed":>6}{"lc":>9}{"lc_repeat":>11}{"spread":>9}')
    for s,x,y,sp in pairs: print(f'{s:>6}{x:9.3f}{y:11.3f}{sp:8.1f}%')
    sp=np.array([p[3] for p in pairs])
    print(f'\nn={len(sp)} identical pairs: mean {sp.mean():.0f}%, median {np.median(sp):.0f}%, max {sp.max():.0f}%')
    print('Any claimed effect below this is indistinguishable from rerunning the same config.')
PY
  echo '```'
} > $OUT 2>&1

python3 eval/eval_tools/scripts/plot_tail.py $A $B $C --out $FIG --alias lc_repeat=lc >> $OUT 2>&1
echo "" >> $OUT
echo "Figures: $FIG" >> $OUT
echo "night1_report done $(date +%H:%M)" >> /tmp/chain_batches.log
