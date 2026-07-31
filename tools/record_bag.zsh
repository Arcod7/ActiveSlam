#!/usr/bin/env zsh
# Record a run and always leave a bag that opens.
#
# rosbag2 writes metadata.yaml only on a clean shutdown, and a recorder that is
# signalled mid-write can leave the directory without one — `ros2 bag info` then
# reports "Could not find metadata in bag directory" even though every message
# is on disk. MCAP is self-describing, so `ros2 bag reindex` reconstructs the
# metadata in full; this wrapper just makes that automatic instead of something
# the operator has to know after the fact.
#
# Usage: record_bag.zsh <output_dir> <topic> [topic ...]

set -u
out=$1
shift

ros2 bag record -o "$out" "$@" &
recorder=$!

# The launcher stops a group by signalling its whole process group, so the
# recorder is signalled directly and this shell does not need to forward
# anything. It does need to survive: a forwarding trap still let zsh exit, and
# the reindex below never ran. Ignoring the signal outright is what keeps this
# shell alive long enough to repair the bag the recorder just abandoned.
trap '' INT TERM
wait $recorder

if [[ ! -f "$out/metadata.yaml" ]]; then
    echo "record_bag: no metadata.yaml, reindexing $out"
    ros2 bag reindex "$out" || echo "record_bag: reindex failed for $out"
fi
