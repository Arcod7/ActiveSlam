#!/usr/bin/env python3
"""Stop every ActiveSlam process running on this machine.

The launcher's K key runs the same sweep. This exists for when no launcher is
left to press it in: a crashed session, a closed terminal, a killed launcher.
Its nodes carry on publishing on the same topics, and the next run then fights a
stack nothing on screen admits to.

    tools/stop_all.py          stop them
    tools/stop_all.py --list   name them and exit
"""

import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

import launcher_core as core  # noqa: E402


def main(argv):
    list_only = "--list" in argv[1:] or "-l" in argv[1:]
    ws_root = core.find_workspace_root(os.path.join(REPO_ROOT, "launcher.py"))
    if ws_root is None:
        print("no built workspace above the repository — nothing to sweep",
              file=sys.stderr)
        return 1

    # Our own process group is spared, or the sweep stops the shell it runs in.
    own = (os.getpgid(0),)
    strays = core.find_stray_processes(ws_root, exclude_pgids=own)
    if not strays:
        print("nothing running")
        return 0

    for pid, pgid, cmd in sorted(strays):
        print(f"  {pid:>8}  pgid {pgid:<8}  {core.describe_argv(cmd)}")
    if list_only:
        return 0

    stopped, survived = core.stop_stray_processes(
        ws_root, exclude_pgids=own, on_event=lambda m: print(m), strays=strays)
    print(f"stopped {stopped}" + (f", {survived} survived" if survived else ""))
    return 1 if survived else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
