# Pre-registration: does an uncertainty trigger beat a distance schedule?

Written and committed **before** the confirmatory data exists. The commit that
adds this file precedes the commit that adds `trigger_confirm_*` results, and
that ordering is the point: the analysis below is fixed in advance so the
result needs no multiple-comparison correction and cannot be shaped after the
fact.

## Why a confirmatory run is needed

`trigger_rematched_20260731_1407` (13 paired seeds, matched cadence) found
final absolute position error of 0.253 m under the U_r trigger against 0.391 m
under a matched distance schedule -- a 35% reduction, 11/13 seeds, t p=0.019,
Wilcoxon p=0.027.

That run reported five metrics with no primary endpoint declared in advance.
The Bonferroni threshold over five is 0.010 and the Holm-adjusted p-values are
0.095, 0.279, 0.320, 0.454, 0.717, so nothing survives correction. It is a
discovery sample and is treated as one.

## Hypothesis

**H1.** With revisit cadence held equal, triggering revisits on calibrated
pose uncertainty produces a lower final absolute position error than triggering
them on distance travelled.

**H0.** No difference.

Direction is stated in advance because the discovery sample and the mechanism
both point the same way: the trigger is supposed to send the vehicle back
*when* it is uncertain, so it should most affect where the estimate ends up.
The test remains two-sided.

## Primary endpoint

**`final_abs_error`** -- the final absolute position error, one value per run,
paired by seed across the two arms.

Exactly one primary endpoint. No correction is applied to it and none is
needed.

## Primary analysis, fixed now

- Two-sided **paired t-test** on `uncertainty - scheduled`, alpha = 0.05.
- **Wilcoxon signed-rank** reported alongside as a distribution-free
  robustness check. It is not a second chance at significance: if the two
  disagree, that disagreement is reported as the finding.
- Effect size as the mean paired difference with a 95% confidence interval.
- Analysis run by `eval/eval_tools/scripts/trigger_confirm.py`, committed with
  this document, executed unmodified.

## Sample size

**20 paired seeds** (40 runs, ~4 h).

The discovery sample gave a paired sd of 0.184 against an effect of 0.138,
which needs 14 pairs for 80% power. 20 is deliberate over-provision: a
discovery estimate is biased upward by selection (winner's curse), so the true
effect is probably smaller than 0.138. Twenty pairs retain 80% power down to an
effect of 0.118.

## Seeds

800-819. Disjoint from every seed used so far (601-604, 611-623, 701-708) so
no noise realisation is reused between discovery and confirmation.

## Everything else is secondary and descriptive

`final_ate`, `final_coverage`, `final_chamfer`, `lc_count`, `revisit_count`,
`gt_path_m` are reported as descriptive statistics **with no inferential
claim** and no p-value interpreted as evidence. They exist to characterise the
runs, not to test anything.

## Validity gates, declared in advance

The comparison is void, not reinterpreted, if any of these fail:

1. **Matched cadence.** Mean `revisit_count` in the two arms must agree within
   15%. This is the control's entire purpose; the previous attempt failed it
   (3.62 against 2.31) and its apparent effect was confounded with revisit
   frequency.
2. **All runs valid.** Every run `status == ok` with `gt_path_m > 5`, so no
   deadlocked or motionless run enters the sample.
3. **Comparable travel.** Mean `gt_path_m` within 10% across arms.

If a gate fails the run is reported as void and the reason stated. A failed
gate is not grounds for switching endpoint or re-deriving the control and
re-testing on the same seeds.

## Configuration

Identical to `trigger_rematched`, both arms in one batch:

- `cooldown_s` 30, `revisit_min_closures` 1, `ratio_trigger` 1.0,
  `ratio_resume` 0.5, `dopt_median_window` 1
- `sigma_allow_xy_m` 0.2, `sigma_allow_yaw_rad` 0.05 (mission-derived; see
  `UNCERTAINTY_MODEL.md`)
- uncertainty arm `revisit_schedule_every_m: 0.0`
- scheduled arm `revisit_schedule_every_m: 22.0`
- degraded DVL, realistic IMU/compass/pressure, shipwreck scene from
  `config.yaml`, 300 s per run

## What each outcome means

- **p < 0.05, negative difference** -- H1 supported on a pre-declared endpoint
  with cadence, closures and travel controlled. Reportable as a result.
- **p >= 0.05** -- the discovery finding did not replicate. Reported as such.
  The confidence interval then bounds how large an effect remains possible.
- **Direction reverses** -- reported. No reinterpretation.
