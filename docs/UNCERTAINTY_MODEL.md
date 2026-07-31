# The odometry uncertainty model

What the pose graph believes about its own dead-reckoning error, why each term
takes the form it does, and how the belief was checked against the error the
simulator actually produces.

This matters because the revisit trigger is a comparison between a reported
uncertainty and an allowable one. If the reported number is not a description
of the real error, a revisit fires for no reason or fails to fire when the
vehicle is genuinely lost, and the active-SLAM claim is untestable either way.

## The model

One translation sigma per dead-reckoned keyframe edge, in
`slam_backend/odom_noise.py`, as the quadrature sum of five terms.

| term | form | arm it accumulates against |
|---|---|---|
| DVL white velocity noise | `sigma_v * sqrt(dt / rate)` | none, independent per edge |
| DVL scale error | `eps * D` | displacement from the anchor |
| DVL velocity bias | `b * A` | `\|\|integral R dt\|\|` |
| heading noise, cross-track | `sigma_psi * sqrt(2 * tau * d * v)` | none, independent per edge |
| heading bias, cross-track | `sigma_bias(t) * D` | displacement from the anchor |

`d` is the edge length, `v` its mean speed, `D` the vehicle's displacement from
the first keyframe and `A` the norm of the integrated rotation.

### Why heading is in here at all

`dead_reckoning.py` rotates the DVL's body-frame velocity into the world using
the **estimated** attitude before integrating it. Every radian of heading error
is therefore converted directly into cross-track position error. A model built
only from the DVL datasheet omits this, and it is not a small omission: on the
degraded profile it is the largest single contributor.

`fused_yaw_stats()` returns the heading error's standard deviation and
correlation time by solving the `YawKalmanFilter` recursion at its fixed point.
Both come from the profile; neither is fitted. The correlation time decides the
form: short tau means the cross-track error is a random walk in distance
(additive per edge, no cumulative arm), long tau means it behaves like a bias
and takes the displacement arm instead.

### Why each systematic term has a different arm

The three systematic errors are all fixed in the **body** frame, and the
vehicle turns. Their contribution in the world is not their magnitude times
elapsed time or path length:

- Scaling body velocity by `(1+eps)` and integrating gives
  `(1+eps) * integral(R v dt)`, so the position error is `eps * displacement`.
  Not `eps * arc length`.
- A constant velocity bias `b` contributes `b * ||integral R dt||`. Not
  `b * elapsed time`.
- A heading bias `psi` contributes `psi * displacement`, by the same argument.

On the shipwreck survey these are not small corrections. The path covers
155 m of arc inside a 26 m displacement, and 56.5 s of rotation-integrated arm
inside 295 s elapsed: factors of 5.9 and 5.2 in sigma.

### The one approximation, stated

Displacement and the rotation-integrated arm both **shrink** when the vehicle
turns back toward its anchor, and a chain of independent Gaussian factors can
only ever grow a marginal. The negative increment is unrepresentable. Clamping
it to zero would ratchet the total up past the truth by 3.2x and 4.2x in
variance, so both arms are instead passed as **running maxima** -- the tightest
monotone upper bound available in this formulation. The residual overestimate,
measured against the true end value, is 1.17x in sigma for the scale term and
1.57x for the bias term.

Removing that residual requires carrying the scale and bias as estimated
**states** in the graph rather than as noise, so the marginal can follow the
geometry down as well as up. That is the principled fix and it is not
implemented here. Note that it would not change the pre-first-closure regime
that drives the trigger: those states are only observable once something else
constrains position, so before a closure they sit at their prior and the
marginal grows identically.

## Validation

Two independent checks, because a sim run gives three seeds and three seeds
cannot separate five terms.

### Monte Carlo of the chain

`eval_tools/scripts/odom_error_mc.py` replays a recorded ground-truth
trajectory through the same arithmetic the sensor sims and `dead_reckoning`
perform, for N independent trials, with any subset of terms enabled. Its
zero-noise floor is 0.039 m against signals of 0.3-1.6 m, so the harness itself
is not what is being measured.

Per-term attribution, 300 trials, degraded profile, recorded 155 m path:

| source | RMS error at 155 m |
|---|---|
| attitude, total | 0.790 m |
| — fused-yaw noise | 0.668 m |
| — compass bias walk | 0.446 m |
| — roll/pitch | 0.029 m |
| DVL velocity bias | 0.324 m |
| DVL scale | 0.276 m |
| DVL white | 0.131 m |
| **everything** | **0.798 m** |

The closed form for the dominant term, `sigma_psi * sqrt(2 * tau * L * v)`,
predicts 0.670 m where the Monte Carlo measures 0.668 m.
`fused_yaw_stats()` predicts sigma 0.147 rad and tau 0.130 s against a measured
0.133 rad and 0.140 s -- 1.10x conservative, spec-derived, not tuned.

Model against truth, across three recorded paths and two profiles:

| configuration | truth | model | ratio |
|---|---|---|---|
| s301, degraded | 0.892 m | 1.048 m | 1.18 |
| s302, degraded | 1.031 m | 1.024 m | 0.99 |
| s302, realistic | 0.308 m | 0.213 m | 0.69 |
| s602, degraded | 1.576 m | 1.850 m | 1.17 |
| s602, realistic | 0.304 m | 0.324 m | 1.07 |

Median 1.07, range 0.69-1.18. The previous per-edge model sat 6-10x under the
truth on the same five cases. The 0.69 case is left in rather than corrected
away: the model is spec-derived and the residual is reported, not absorbed.

**Rejected change, recorded so it is not retried.** The 0.69 case is systematic
rather than sampling noise -- three independent Monte Carlo seeds at 500 trials
give 0.302, 0.308 and 0.312 against a model value of 0.213 every time -- and it
appears only at the final milestone, the model tracking at 0.92 through the
first three quarters. Per-term attribution shows the last-quarter growth is
entirely heading-driven (attitude 0.174 -> 0.278, compass bias 0.135 -> 0.248)
while every DVL term is flat.

That suggested moving the heading-bias term from the displacement arm to the
arc arm, on the argument that `compass_sim` injects an unbounded random walk
rather than a fixed offset, so psi changes between outbound and return legs and
the contributions stop cancelling the way displacement does. Implemented and
measured, it is **worse**: model/truth moves from 0.69-1.18 to 1.07-3.13,
because on the degraded profile the displacement arm is already almost exactly
right (predicted 0.452 m against a measured 0.446 m for the bias term alone)
and the arc arm overshoots it by the tortuosity. Reverted.

The lesson for anyone revisiting this: the correct lever for a walking heading
bias is neither arm but a path-geometry double integral,
`Var = drift^2 * integral integral min(t,s) * (v_perp(t) . v_perp(s)) dt ds`,
whose value depends on how correlated the direction of travel is between two
times. Displacement is the right limit for a bias that holds still over the
mission and arc for one that decorrelates within it; this simulator sits
between the two and closer to displacement.

### Estimator consistency in the full pipeline

`eval_tools/scripts/consistency_report.py` scores recorded runs, reading the
per-keyframe error vector and full XYH marginal that `benchmark.py` now writes
to `keyframe_consistency.csv`.

Its sampling design is deliberate. Keyframes within one run are **not**
independent samples of consistency: the scale and bias are one draw each for
the whole run, so a single realisation dominates an entire trace. Pooling every
keyframe and testing against a chi-square band for n = total keyframes claims
an effective sample size it does not have. It is also why per-run ANEES on a
fixed configuration scatters over an order of magnitude between seeds --
2.99, 36.62 and 21.97 on three seeds of one configuration.

The report therefore takes **one sample per run per milestone**, milestones
being fixed distances along the ground-truth path, and pools those across
seeds. Samples at a given milestone are independent by construction, so the
chi-square band for n = number of seeds is legitimate, and the output is a
calibration curve against distance rather than a single number.

### What the closure-free runs show (n = 3, `odom_calibration_20260731_0935`)

Pooled ANEES over XY, one sample per seed per milestone:

| GT arc | ANEES (2 DoF) | 95% band | verdict |
|---|---|---|---|
| 20 m | 0.40 | [0.41, 4.82] | conservative 0.45x |
| 40 m | 2.22 | [0.41, 4.82] | consistent |
| 60 m | 3.37 | [0.41, 4.82] | consistent |
| 80 m | 4.60 | [0.41, 4.82] | consistent, at the edge |

The marginal is statistically consistent over 20-80 m, which the previous model
was not at any distance. Two honest qualifications: with three seeds the
chi-square band is very wide, so "consistent" is weak evidence rather than a
result; and the sequence climbs monotonically toward the upper bound, so more
seeds would likely fail the later milestones.

The residual is a **growth-rate** gap. In the full pipeline the true error grows
as `s^1.24` while the marginal grows as `s^0.33`. In the open-loop Monte Carlo
the two exponents match (0.43 against 0.44), so the gap is not in the model's
form. Two causes, neither addressed here:

- The controller runs on the SLAM estimate, so estimate error changes where the
  vehicle physically goes and compounds. The Monte Carlo replays a recorded
  path open-loop and cannot show this by construction.
- The displacement arm saturates. On a survey orbiting one object it reaches
  ~31 m and stops, so the coherent terms stop contributing while the true error
  keeps accumulating. This is the documented cost of the monotone running-max
  bound, and it is the strongest argument for the estimated-state formulation.

### Per-keyframe detail on one seed

Loop closure, revisit and map rebuild are all off in
`config/matrix_odom_calibration.yaml`, because a closure resets both the error
and the marginal and you would be measuring the closure model instead.

Exact 3-DoF NEES on the first seed: median 3.43 against a target of 3, i.e.
1.07x in sigma. That is the headline -- the marginal is close to honest, where
the previous model was 6-10x overconfident.

The residual is a **growth-rate** mismatch rather than a scale error. The ratio
of true error to the marginal's major axis runs 0.41, 1.09, 1.52, 1.96 across
one run: well calibrated in the middle, increasingly overconfident by the end.
The cause is the monotone bound documented above. On a survey that orbits a
single object the displacement arm saturates early -- 31.1 m on this run -- so
the coherent terms stop contributing while the true error keeps accumulating.
This is the predicted cost of the running maximum, appearing exactly where the
approximation says it should, and it is the strongest argument for carrying
scale and bias as estimated states instead.

Two cautions for anyone reading these numbers. `sigma_xy` is
`det(Sigma_xy)^(1/4)`, a geometric mean of the principal axes, so it is not
comparable to a predicted total error magnitude unless the covariance is
near-isotropic -- it happens to be here, major/minor 1.1, but that is a
property of this trajectory and not a licence. And per-run summaries are
dominated by the single coherent draw, so read the pooled milestone table, not
one seed's final number.

## Open problem: the reported marginal is not monotone

In a closure-free run the marginal of the newest keyframe should only grow --
every edge adds variance and nothing removes it. It does not. Across the three
calibration seeds `sigma_xy` decreases on roughly 46% of keyframe steps, with
excursions as large as 0.553 -> 0.288 m inside one run.

**Cause identified: iSAM2's incremental marginal recovery.** Rebuilding the
same chain offline over a recorded trajectory -- one dead-reckoning
BetweenFactor and one ZPR prior per node, no closures -- and recovering each
node's marginal two ways settles it:

| recovery | sigma_xy at 113 m | decreasing steps | max drop |
|---|---|---|---|
| `isam.marginalCovariance()` after each update | 0.8231 | 28% | -0.0172 |
| one batch `Marginals()` over the final graph | 0.8231 | **0%** | +0.0000 |

Identical noise model, identical final value, and the batch recovery is exactly
monotone as theory requires. `pose_graph.py` caches each keyframe's covariance
at the moment that keyframe is newest, so the published sequence is N marginals
drawn from N different, progressively relinearised Bayes trees rather than one
graph's covariance over time. Nothing about the noise model is implicated.

Three earlier suspects were tested and excluded, not merely doubted: no
pose-graph restart (`PoseGraph started` appears exactly once per run), no
closures (`lc_count` 0 throughout), and the attitude+depth prior leaves x and y
at sigma 1e3. The live yaw-prior variance was the leading hypothesis and is
**wrong** -- the correlation between per-step changes in `sigma_yaw` and in
`sigma_xy` is +0.02 to +0.11 across the three seeds. Body-frame re-expression
is also excluded: the trace of the XY block is invariant under a yaw rotation
and it falls on 39-49% of steps too.

This matters for the trigger, not just for tidiness. `revisit_planner` compares
a **single live D-opt sample** against its threshold. If that sample oscillates
by up to 2x for reasons unrelated to actual drift, the trigger fires on noise,
and the spread in where U_r first crosses 1 -- 26.7, 34.2 and 51.4 m on three
seeds of an identical configuration -- is partly this rather than genuine
seed-to-seed variation. Now that the cause is known the mitigation is clear and
cheap: feed the trigger a short median of D-opt rather than the instantaneous
sample, since the oscillation is a recovery artefact and the underlying
quantity really is monotone between closures. `dopt_median_window` does this,
defaulting to 1 (an exact no-op) so earlier comparisons stay comparable.

**How much it actually buys, measured rather than asserted.** Replaying the
three closure-free runs' recorded marginals through the filter and counting
threshold crossings:

| run | crossings, window 1 | window 3 | window 5 | flip-flops at window 1 |
|---|---|---|---|---|
| s701 | 3 | 3 | 3 | 0 |
| s702 | 3 | 2 | 2 | 1 |
| s703 | 2 | 2 | 2 | 0 |

One spurious crossing removed out of eight, and the one flip-flop (a crossing
that reverts within three keyframes). So the artefact is real and frequent in
the marginal -- ~30% of steps -- but usually small relative to the distance
between D-opt and the threshold, and it corrupts the trigger less often than
the raw dip statistics suggest. Worth having, not worth overstating.

### Still unresolved: the level, as opposed to the shape

The offline chain above reaches `sigma_xy` 0.823 m at 113 m, while the real run
over the same trajectory reports 0.476 m and ends with a true error of 0.974 m.
Taken at face value the noise model alone would be close to calibrated (0.974 /
0.823 = 1.18x) and something in the live graph is halving the reported figure,
with the sequential scan-matching BetweenFactor -- a hard-coded
`scan_sigma_trans` of 0.12 m on every consecutive pair, regardless of overlap
or inlier count -- the obvious candidate.

**Measured, and it is mostly not the scan factor.** `marginal_monotonicity.py
--scan` adds the sequential BetweenFactor at pose_graph's constant sigmas to an
otherwise identical chain, which holds keyframe count and spacing fixed and
isolates what the extra factor does to the marginal:

| seed | DR only | DR + sequential scan factor | ratio |
|---|---|---|---|
| s701 | 0.8231 | 0.6252 | 0.76 |
| s702 | 0.6187 | 0.5441 | 0.88 |

So the scan factor deflates the reported marginal by 12-24%, not the ~50%
needed to close the gap to the live run's 0.476 m. The remainder is the
confound: the real run made 277 keyframes where the offline reconstruction
makes 225, and the offline arms come from ground truth rather than drifted
odometry.

The conclusion is therefore weaker than it first looked, and the earlier
suspicion that sequential matching "claims information it does not deliver"
is only weakly supported. A constant 0.12 m sigma that ignores overlap and
inlier count is still the wrong shape for a registration noise model, and
tying it to the error-per-inlier the matcher already computes remains worth
doing -- but it is a 12-24% effect on the marginal, not the explanation for
the level.

## Choosing the allowable uncertainty

`U_r = D(Sigma) / D(Sigma_allow)` is only meaningful if `Sigma_allow` says
something. Setting it from the observed `sigma_xy` curves so the trigger fires
a convenient number of times is tuning to a conclusion, and an earlier config
did exactly that -- raising it from 0.2 to 0.45 with the comment "roughly the
mid-growth point seen in [the previous batch]".

Both thresholds are instead derived from the mission the vehicle is flying,
fixed before any trigger outcome was inspected:

    sigma_allow_xy_m  = voxel_size = 0.2

The deliverable is a 0.2 m TSDF reconstruction. Once pose uncertainty exceeds
one voxel, a measurement can be integrated into the wrong voxel and the
resolution the map claims is no longer supported.

    sigma_allow_yaw_rad = voxel_size / mapping_range = 0.2 / 4.0 = 0.05

Heading uncertainty smears a surface laterally by `psi * range`. At the typical
mapping range for this survey -- `wall_standoff` 1.5 m plus
`tsdf_frontier_standoff_m` 2.0 m, so roughly 4 m to the far side of a sweep --
one voxel of lateral smear is 0.05 rad.

## Alignment with Suresh et al. (2020)

Matching, and deliberately so:

- The factor-graph structure. Their Fig. 4 is a 3-DoF XYH relative odometry
  factor plus a 3-DoF ZPR absolute unary constraint plus ICP closures plus a
  prior at the first node. `pose_graph.py` is the same decomposition.
- `D(Sigma) = det(Sigma)^(1/3)` over a 3x3 XYH marginal expressed relative to
  the initial vehicle pose (their Eq. 4), and `U_r = D(Sigma_r)/D(Sigma_allow)`
  with a revisit at `U_r > 1` (Eq. 5, Alg. 1). Identical here, and the graph is
  anchored at keyframe 0 so the marginal is relative to the same reference.

Differing, and worth stating explicitly:

- **They apply no absolute yaw prior.** Their text is explicit that Z, roll and
  pitch are directly observed while "X, Y and yaw (XYH) is indirectly estimated
  through the accumulation of IMU/DVL odometry". Yaw drifts freely for them,
  bounded only by relative odometry and loop closures. This implementation
  priors yaw absolutely from a simulated magnetometer at every keyframe, which
  structurally caps how much the yaw half of D-opt can grow -- the reported
  `sigma_yaw` sits near 0.023 rad for an entire run and carries almost no
  information, while `sigma_xy` does the work. This is a platform difference
  (a Bluefin HAUV integrates a gyro AHRS; a BlueROV2 has a magnetometer that
  genuinely measures absolute heading) rather than an error, but it means the
  two D-opt traces are not measuring quite the same thing.
- Their odometry covariance is a **constant** per submap edge,
  `Psi = (4.14, 4.14, 0.027) x 10^-3` (Table III), i.e. sigma_xy ~ 0.064 m and
  sigma_yaw ~ 0.005 rad. They scale noise with distance only when
  forward-propagating virtual odometry to a candidate revisit pose. The
  per-edge model here is finer-grained; the difference is defensible because a
  virtual propagation along a hypothetical path cannot know that path's
  tortuosity, whereas an accumulated marginal along a flown path can.
- Their revisit utility balances propagated uncertainty against sensor
  information gain with a weight `alpha = 0.6` (their Eq. 6). The revisit
  scoring here is keyframe density minus travel, which is a weaker proxy.

## Known limitation in the hardware path

On the real vehicle, `mavlink_odometry.py` publishes both
`/slam/sensors/imu_orientation` and `/slam/sensors/compass_heading` from the
same MAVLink `ATTITUDE` yaw. In the 2026-07-28 log the difference between the
two streams is identically zero. `dead_reckoning` therefore predicts with a
value and corrects with the same value, violating the Kalman filter's
independence assumption, and its reported yaw variance is not meaningful on
hardware. The simulation path is unaffected -- `imu_sim` and `compass_sim` draw
independently -- but no real-world heading-error claim can rest on that log.

## Does a calibrated trigger beat a clock? (n = 1, indicative only)

`trigger_calibrated_20260731_0958`, one paired seed, both arms identical apart
from what starts a revisit.

| | uncertainty (U_r) | scheduled (every 34 m) |
|---|---|---|
| revisits | 3 | 3 |
| final ATE | **0.468 m** | 0.710 m |
| final absolute error | **0.411 m** | 1.306 m |
| coverage | **0.903** | 0.794 |
| chamfer | **0.741** | 0.782 |
| loop closures | 93 | 57 |

Two things worth taking from this and one thing not to.

The trigger now fires **organically on a mission-derived threshold** -- U_r
peaked at 1.20 and the state machine cycled exploring -> revisiting -> cooldown
three times. Before the model was corrected, the trigger either never fired or
needed `sigma_allow_xy_m` raised to 0.45 to fire at all. That much is a
property of the calibration, not of this seed.

The matched cadence held: both arms revisited exactly three times, so the
comparison isolates what triggered the revisit rather than how often one
happened. That is the design working.

What must **not** be taken from this is the performance gap. This is a single
seed. Seed-to-seed variance in this system is large by construction -- the
run-long DVL scale and bias draws dominate a whole trace, and per-run ANEES on
one fixed configuration has been observed at 2.99, 36.62 and 21.97. A 34% ATE
difference on n=1 is inside that noise. Five or more paired seeds are needed
before any of these numbers is quotable, and `matrix_trigger_calibrated.yaml`
is set up to do exactly that -- it was trimmed to one seed only to fit a time
budget.
