"""Where evaluation output goes, resolved the same way by every producer."""

import os

ENV_VAR = 'ACTIVESLAM_EVAL_RUNS'


def runs_root():
    """Root directory for evaluation runs.

    $ACTIVESLAM_EVAL_RUNS wins, so a machine that wants its output elsewhere
    says so in its own environment rather than in a tracked file. Otherwise the
    repository's own eval/runs, located by walking up from this module: with
    --symlink-install the installed copy points back into the source tree,
    which is where run_matrix.py and the rest of the tooling look. Failing
    both, the user's home — never a path that merely happens to exist on
    somebody else's machine.
    """
    override = os.environ.get(ENV_VAR)
    if override:
        return os.path.expanduser(override)
    d = os.path.dirname(os.path.realpath(__file__))
    for _ in range(6):
        if os.path.isfile(os.path.join(d, 'dependencies.conf')) \
                and os.path.isdir(os.path.join(d, 'eval')):
            return os.path.join(d, 'eval', 'runs')
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return os.path.expanduser('~/.activeslam/eval/runs')


def new_run_dir(stamp):
    """Timestamped run directory under runs_root()."""
    return os.path.join(runs_root(), stamp)
