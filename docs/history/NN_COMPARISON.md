# NN comparison workflow

## 2026-10-23 latest: isolated conventional-controller experiments

The new first section of MOTOR_OPTIMIZATION.md supersedes the deployment below.
NN models, PI gains, estimator and position controller remain unchanged.
60 real-board reset-vs-keep integral segments show keeping the integral worsens
equal-transition mean whole RMSE 7.10% and first-second RMSE 12.05%.
Do not describe R-integral reset as a proven defect. Default remains reset.
Data: data/transition_integral_ab_20261023*; 26553 continuous raw T/L frames.

Current image: nnref1-uartirq1, z0=3005, local ELF SHA256
`9AFB2FAD04698F12C5B20F9F814A2AD0A3416E7C925C61F0EFD89F1F8C215A26`.
It additionally supports a first-order setpoint filter on the P path ONLY.
I, speed feedback and feedforward are unfiltered; position mode is unchanged.
`transition reset/keep` selects the RAM integral policy; `pref 0..200` sets
the P-reference time constant in ms, stopped only. Defaults remain reset/pref0.
This is not bumpless voltage transfer or a new neural-network result.

One-repeat 0/25/50ms screening selects 25ms for a separate frozen confirmation:
whole RMSE -5.94%, first-second -15.77%, first-250ms -22.15%.
Data: data/reference_screen_20261023*. Screening is not confirmation evidence.

Independent confirmation is now complete: data/reference25_confirm_20261023*,
60 additional segments, seed691, three repeats, frozen25ms, same -888mrad
calibration, no extra flash. Equal-transition means: whole RMSE
3.211 -> 3.024 RPM (-5.83%), first-second 4.505 -> 3.975 (-11.77%),
first250ms 6.784 -> 5.398 (-20.42%), first250ms directed peak overshoot
16.952 -> 5.785 RPM (-65.88%). Each of ten transition means improves these
four metrics. Tail100ms RMSE mean improves2.69%, but 0->+5 and -15->+10
worsen1.03%/5.61%; reversal crossing is not faster. Steady ripple remains.
This is a conventional control improvement, NOT NN benefit or full dominance.
The old dynamic NN's incremental effect with the new P path is untested.

26289 raw frames continuous; all60 segment-leading frames retained in CSV;
voltage parity passes, no faults/clipping. Maximum period2.0332ms,
max computation0.3951ms, late/over-budget0 (aggregate, not WCET).
125 host tests pass. Final state stopped/healthy/calibrated/no fault/Uq0;
G0/K0/P8/Q284/j0/r0, COM32 closed. Defaults unchanged; CLI pref25 is opt-in.
Future learning should target the remaining angle-dependent/steady ripple
after separating estimator artifacts, rather than imitate the old PI harder.

## 2026-10-23 latest: continuous reversal / dynamic lookahead comparison

This section supersedes older deployment and "reversals not done" statements.
Full details and signed-transition tables are in the first section of
MOTOR_OPTIMIZATION.md. Position control was NOT changed or retested.

One verified flash deployed `nndyn1-uartirq1`. Local ELF SHA256:
`1BFB008E78C4984092788D90427BA091286E857804C098DB269B1C4F5A130EBC`.
The old model remains intact; the new 3-8-1 ReLU model has a separate header.
G7 applies static compensation and only observes dynamic NN output;
G8 adds NN dynamics; G9 adds an explicit linear predictor.
All three apply identical static compensation fully. K/alpha scales only
the dynamic term, bounded +/-30 mV, with speed targets limited to +/-15 RPM.
Default remains NN off.

The teacher is PI integral(t+250 ms)-integral(t), NOT optimal plant voltage.
Inputs are target speed, measured speed and opposed-direction state.
Repeat-level train/selection/holdout splits give held-out voltage RMSE
3.669/3.991/4.349 mV for NN/linear/zero; this is prediction evidence only.
Archive: data/dynamic_lookahead_linear_20261023.npz/.json.

Real-board comparison: 90 segments, three arms, three repeats, ten continuous
target transitions, 4 s each; P150/Q100/v0/1 V, one calibration -883 mrad.
R resets the integral at EVERY transition, unchanged in this study.
NN whole-record RMSE improves six of ten transitions versus static-only and
three of ten versus linear. Equal-weight mean transition RMSE is
3.327/3.291/3.238 RPM (static/NN/linear). NN improves this mean 1.07% and
100 ms position-window tail RMSE 4.26%, but does not improve mean first-second
RMSE. The +15 -> -15 transition worsens 5.54%. No consistent reversal-time
benefit. Three repetitions do not establish statistical superiority.
The encoder-derived metrics are not independent external speed ground truth.

39271 raw T/L frames have continuous sequence. Each arm's predicted and applied
voltage passes reference parity (maximum prediction error <1.001 mV, applied
error <2.002 mV). No faults/clipping; capture and cleanup succeeded.
Timing snapshot over 196245 iterations: max period 2.0322 ms, max computation
0.3957 ms, late/over-budget counters zero. This aggregate is not per-mode
timing or a WCET guarantee. 119 host tests pass, distinct from board validation.

Data/plot/report: data/dynamic_reversal_ab_20261023*.
NN stays experimental and OFF; final recorded state stopped, healthy,
calibrated, no fault, Uq0, G0/K0/P8/Q284, COM32 closed.
Next isolate integral-reset effects before adding NN complexity; retain the
linear comparator. Do not promote an offline-fit gain as a control gain.

## 2026-10-23 current result: matched PI / friction / learned residual

The latest completed experiment is the first section of MOTOR_OPTIMIZATION.md.
Using the UART IRQ repair image, one calibration, same P150/Q100, and the SAME
G3/alpha=1 path for all arms (zero coefficients for PI), 54 real-board trials
covered +/-5/15/30 RPM. The simple two-parameter voltage correction reduced
whole-record RMSE by 9.1..35.0%, but +15 RPM steady-state metrics worsened.
The frozen learned residual adds at most 0.252 mV over the simple correction;
there is no consistent extra NN benefit. None of the three comparisons passes
strict all-metric dominance.

Data/plot/report: data/friction_matched_confirm_20261023*.
27685 raw T/L frames have continuous +1 sequence; all three arms pass MCU
prediction/application parity at telemetry precision. 112 host tests pass.
Final state stopped/healthy/calibrated/no fault, G0/K0/P8/Q284; UART released.
No firmware flash in this experiment. Reversal and position tests are NOT done.
The deployment and state descriptions below are historical.

## Current board: nnphase1, dynamic-phase capability plus corrections

The latest round (first section of MOTOR_OPTIMIZATION.md) changed three things
that affect every earlier NN number:

1. `app/angle_shadow.c` multiplied the compiled hidden weights by the input
   std even though the generated header already stores weight*std, while the
   prediction path divides by std. Every angle-model output until now was
   scaled by 1/std per input (measured ratio 1.687/1.0/1.414 on neuron 0).
   The old "+20 mV clamp" checks still passed because they compared against
   the value the MCU reported rather than the trained model. Treat earlier G6
   gain figures as "measured on a mis-scaled model".
2. The host reference is now float32 and mirrors the firmware's evaluation
   order and milli-unit truncation; board-vs-reference agreement is within the
   1 mV telemetry quantization (max 1.2 mV, p95 1.1 mV over 476 frames).
3. Phase is uploadable per speed (`M/u/p/g/i/n/q/m`, RAM only, `z0` reports
   build 3001). The static global phase for this calibration is +0.25 deg;
   per-speed static phases are worse than a zero-phase control at held-out
   speeds, and a constant-delay model collapses to the 2 ms sampling delay.
   Closed-loop A/B with a zero-phase control shows no phase effect above
   run-to-run noise (max 5.67%, random sign, CV 1-3%).

G5 remains non-actuating. G6 stays experimental: the +/-15 RPM results do not
hold at +30 RPM once the startup transient is excluded.

## Latest deployment: nnest1 estimator/timing study

See the first section of MOTOR_OPTIMIZATION.md. nnest1 is now flashed/verified,
with the existing NN modes/models retained and default G0/v0 stopped.
SHA256: BA60F763D2C034D09402D086A3DCDB899513D29CA5E368531367052588DEA8D9.
The reset/calibration invalidates the old -2667mrad angle registration below;
new calibration travel=-882mrad, phase not yet registered. Do not reuse it.
Estimator study is NN-off evidence, not a new NN improvement claim.
Uniform 20/40/100ms position-derived metrics and limitations are in
data/estimator30_holdout_20261021_comparison_windows.json.

## Current board: nnangle2 bounded application experiment

nnangle2 is programmed and verified. G5 remains non-actuating; G6 / CLI
`nn angletrial` permits speed-only tests at 15..30 RPM magnitude, with
prediction clamped to +/-80 mV and alpha capped at .25 (max +/-20 mV applied).
RAM `a` sets calibration-specific phase in mrad, only stopped.
Current calibration travel is -0.885 rad; fitted phase -2667 mrad.
Repeat0 fits phase/DC, repeat1/2 frozen: test voltage RMSE constant 61.37 mV,
unregistered 57.39 mV, registered 46.85 mV. Physical control still requires A/B.

Two-repeat application screening (same P150/Q100, alpha .25): at +/-15 RPM
all five mean metrics improved; whole RMSE fell 5.0/6.6%, raw-position tail
RMSE fell 15.2/18.5%. At +30 RPM whole RMSE worsened 12.8%; -30 worsened
2.6%, despite lower raw-tail RMSE. This is not full-range superiority.
6462 G6 frames pass prediction/application intervals; max actual delta 20 mV.
All trials healthy/unclipped, max telemetry gap 10 ms; worst-case 2 ms loop
timing is not measured. 69 host tests pass.
Data: `data/angle_apply_screen_20261021*`. Independent +/-15 confirmation:
`data/angle_apply15_holdout_20261021*`. Old shadow-only sections are historical.

Independent confirmation completed: three repetitions, 6 s per step, seed463,
frozen phase -2667 mrad/alpha=.25/P150Q100. At +15/-15 RPM respectively:
whole RMSE -5.71/-8.25%, IAE -6.96/-9.25%, tail MAE -9.18/-16.40%,
tail std -10.12/-13.60%, raw-position tail RMSE -15.08/-16.88%.
All five mean metrics improved at both tested targets, and no clipping;
the descriptive dominance report is true for this bounded comparison only.
3630 G6 frames pass prediction/application intervals, max applied 20 mV.
This does not erase +/-30 regressions or prove position/load/statistical
superiority. Final state stopped/healthy/calibrated/no fault; baseline gains,
NN off and telemetry off restored, UART released. Phase is calibration-specific.

## Deployed nnangle1: real-board shadow validation

J-Link 602710971 programmed and verified `debug/test1.elf` on MCXA346 over
SWD 100 kHz. Post-reset G5 was accepted; one calibration succeeded
(travel -0.894 rad) and is retained for subsequent runs. Startup banner was
not captured; programming verification plus live G5 acceptance are the
deployment evidence, not an asserted banner observation.

Two-repeat NN-off/G5 comparisons at +/-15/30 RPM produced 3233 shadow frames.
Prediction interval violations: 0; unclipped Uq-baseUq: exactly 0 mV.
Point error using truncated inputs reached 6.22 mV; conservative interval
width reached 67.06 mV. This is quantized interval agreement, not full-precision
point parity. All recorded frames were healthy/unclipped with max telemetry
gap 10 ms. No 2 ms control execution timing has been measured. At +30 RPM,
shadow raw-tail error mean was ~11% above NN-off; do not claim zero runtime
impact or interpret random shadow differences as NN control benefit.

Source: `data/angle_shadow_board_20261021*`, including `_angle_parity.json`.
The prior not-programmed section below is historical. 67 host tests pass.

## Angle shadow implementation: built, not programmed

The new `nnangle1` image compiles in the existing `debug` directory. G5 / CLI
`nn angleshadow` executes the 12-input candidate but cannot apply its output.
The centralized application gate allows only the pre-existing G2/G3 modes.
Original classical/NN/speed-feedforward paths remain available.

Actual host-compiled C execution on 3003 vectors agrees with float32 Python
within 1.34e-7 V; shadow output remains baseline even with alpha=1.
This is WSL GCC host execution, not MCU timing or board validation.
66 host tests pass. `verify_angle_shadow.py` supports future UART comparison
with interval propagation for truncated angle/speed inputs and checks
non-actuation independently. Local archive/header hashes are checked by the
collector, not read back from MCU.

No probe session, programming or reset occurred. Live read-only UART confirms
stopped, sensor healthy, calibrated and no fault. Programming must use a safe
driver-power-disconnected session. Recalibration changes the angle origin;
phase registration or new data is needed before physical compensation, even
after numerical shadow parity passes. See MOTOR_OPTIMIZATION.md for commands.

## New angle-correlated candidate (offline, not deployed)

Real-board NN-off P150/Q100 records at +/-15/30 RPM now contain three
10-second repeats plus a separate frozen-model evaluation recording.
Startup exclusion is 2 s, with at least a full mechanical revolution per
steady segment. Whole repeat0/1/2 are train/validation/test respectively.

`train_angle_residual.py` trains a 12-input, 8-ReLU residual predictor using
speed, direction and sin/cos of mechanical harmonics 1/2/7/14/21.
Labels are recorded voltage residuals, not a PD teacher or identified
optimal voltage. A speed/direction-interacting explicit harmonic model
provides a cheap comparison using the same training data.

Held-out voltage RMSE: constant 61.59 mV, explicit harmonics 56.28 mV,
NN 44.84 mV. Independently collected recording: 61.67/56.35/45.60 mV.
The frozen NN improves prediction at each of the four signed speed points,
but no closed-loop control improvement or universal classical-controller
advantage is established. Labels include controller dynamics and estimator
effects; they are not pure cogging torque measurements.

Use `data/angle_residual_candidate_v2.npz/.json/.png` and
`data/angle_learning_independent_20261021_angle_evaluation.json`.
Candidate v1 used a weaker shared-angle harmonic comparator; v2 corrects
that baseline without changing the NN. Neither is compatible with ff1
upload or deployed. Calibration-relative phase and 10 ms telemetry versus
2 ms control timing require attention before deployment. Existing firmware
and calibration were preserved; 59 host tests pass.

## Latest expanded comparison

The new opt-in `profile lowfast` sets P150/Q100 for speed commands with
magnitude 15..40 RPM only. It does not enable NN or alter position settings.
Original profiles are retained. Matching-PI comparisons isolate the NN
increment; they do not establish superiority over all tuned classical designs.

`motor_feedforward_v3_low.json` retrains the same tiny network from measured
low-speed traditional-controller voltage residuals. An independently captured
three-repeat validation at +/-15/30/40 RPM reduced whole-record speed RMSE
6.7..24.4%. Five targets improved all five mean metrics, but +15 RPM tail MAE,
standard deviation and position-derived tail RMSE worsened 11.9%, 12.5%, 6.1%.
Thus strict all-metric measured dominance remains false. The low-only model
is not promoted for higher speeds or position mode. Source and mean/std plots:
`data/speed_low_v3_validation_20261021*`. 7268 actual MCU frames passed NN
prediction/application parity at integer telemetry precision.

For +/-20-degree steps, unchanged inner P8/Q650 and outer gain 16 with v2 NN
reduced RMSE 20.5/21.3%, overshoot 96.0/93.0%, and mean settling time
20.7/50.8%. Tail MAE improved but tail standard deviation increased in both
directions. Source: `data/position_20deg_gain16_alpha_20261021*`.
These are same-load/same-calibration results, not universal superiority.

`controller_dominance.py` checks all preselected metrics, not just best-looking
RMSE. Collectors now record model path/hash in trial JSON. 52 host tests pass;
no firmware rebuild/flash/reset/recalibration was needed for these trials.
See the first section of `MOTOR_OPTIMIZATION.md` for full numbers and commands.

An additional two-repeat position check at a 45-degree shifted origin covers
outer gains 16/20 and +/-10/20-degree steps. Gain16+NN improved all four
whole-record RMSE means by 21.1..33.2%, but +10 settling slowed from
0.210 to 0.725 s and -20 overshoot increased from 1.003 to 1.346 degrees.
Gain20+NN reached mean -20 overshoot 2.263 degrees; it is not promoted.
Source: `data/position_gain16_20_offset45_20261021*`; 6467 frames passed
prediction/application parity. Final fresh state confirms stopped, healthy
sensor, valid calibration, no fault; baseline gains restored and UART closed.

## Current deployed model and real motor evidence, 2026-10-21

SWD has been restored and `debug/test1.elf` (`nnff1`) successfully programmed
and verified. The current candidate is `data/motor_feedforward_v2.json`,
a one-input/eight-fixed-ReLU voltage-residual network with learned output
weights. It is **not** the incompatible six-input offline candidate described
in the historical sections below.

With unchanged P8/Q284, three independent trials at each target show captured
speed RMSE reductions of 12.52%, 9.52%, 18.13%, and 6.85% at +30, -30, +60,
and -60 RPM. Steady-state ripple is not consistently improved. Source:
`data/ff_v2_holdout_20261021.csv`; mean/std curves:
`data/ff_v2_holdout_20261021_ff100_summary.png`.

For relative +/-10-degree position steps, unchanged P8/Q650, outer gain 8,
and speed limit 60 RPM, the first three-repeat comparison gave RMSE
2.318 -> 1.682 degrees (+10) and 2.775 -> 2.320 degrees (-10).
Tail MAE improved in both directions, but settling was not uniformly faster.
Source: `data/position_ff_v2_ki650_20261021.csv/.json/.png`.
Physical starting angles/residual motion are not perfectly matched; no
statistical significance or arbitrary-load claim is made.

A second three-repeat comparison (seed 107, same calibration) reproduced
both-direction RMSE improvement: +10 degrees 2.584 -> 1.814, -10 degrees
1.635 -> 1.349. Tail MAE also decreased in both directions. Positive settling
improved 1.947 -> 1.017 seconds, but negative settling slowed 1.200 -> 1.547
seconds. All trials settled within +/-0.5 degrees. This supports a modest
demo benefit with an explicit settling-time tradeoff, not universal superiority.
Source/curves: `data/position_ff_v2_ki650_confirm_20261021.csv/.json/.png`.

Actual MCU NN telemetry matches float32 predictions and applied output within
integer telemetry precision. Verification checks both network output and
`uq - base_uq`, preventing a calculated-but-not-applied result from passing.

Reproduce without flash/reset/recalibration (new output filenames required):

```powershell
E:\miniconda\envs\py310\python.exe tools\compare_position_ff.py --output data\position_repeat.csv --ff-model data\motor_feedforward_v2.json --repeats 3 --alpha 1 --speed-ki 650 --seed 107
E:\miniconda\envs\py310\python.exe tools\report_position_ff.py data\position_repeat.csv
E:\miniconda\envs\py310\python.exe tools\verify_nn_telemetry.py data\position_repeat.csv --model data\motor_feedforward_v2.json
```

The collector restores P8/Q284, NN off, learning telemetry off, and stop at
exit while retaining calibration. Weights are RAM-only. See
`MOTOR_OPTIMIZATION.md` for CLI commands and scope limitations.

### Low-speed follow-up

The opt-in CLI `smooth` profile uses Kp=0.050, Ki=0.100. Three independent
repetitions at +/-30 RPM reduced total RMSE 21.7% / 20.5% relative to the
previous tuned+NN profile; raw-position-derived tail RMSE fell 30.8% / 27.2%.
Most steady-state improvement comes from PI retuning. With the same new PI,
NN adds 17.9% / 15.8% whole-record RMSE improvement but does not consistently
improve the steady tail. No model retraining or firmware flash was involved.
The CLI limits smooth to +/-40 RPM; only +/-30 RPM has been measured.
Data: `data/speed_low_damping2_holdout_20261021.csv`.
Position Ki=0.900 candidates had inconsistent settling and are not promoted.

### Outer position gain follow-up

With inner P8/Q650 and the same v2 feedforward, increasing the outer gain from
8 to 12 was repeatably better in the small-step demo. In a three-repeat run,
NN+12 averaged 1.40°/+10 and 1.15°/-10 RMSE; all six NN+12 trials settled
within 0.5°, while NN+8 retained a larger negative-direction tail. A second
two-repeat run after a 45° origin shift gave NN+12 RMSE 1.26°/+10 and
1.39°/-10, with mean settling 0.72/0.74 s. The current demo preset is
therefore `poskp=12`, speed limit 60 RPM, inner P8/Q650, NN alpha=1.

Data: `data/position_outer_gain_20261021.csv` and
`data/position_outer_gain_offset45_20261021.csv`; telemetry verification passed
for both. This remains a small ±10° same-load result, not a general stability
or bandwidth guarantee.

## Historical workflow: original six-input network

Use the existing powered-on calibration. This tool never flashes, resets or
calibrates the MCU. Close other serial clients before running it.

```powershell
E:\miniconda\envs\py310\python.exe tools\compare_nn.py --kind speed --seconds 4 --alpha 0.1
E:\miniconda\envs\py310\python.exe tools\compare_nn.py --kind speed --seconds 4 --alpha 0.1 --reverse-order
```

The adapter is selected by USB serial number, with a unique CH343 fallback.
`--port COM32` overrides discovery. CSV samples are flushed after each trial;
JSON metrics and a PNG plot are generated even when a later trial fails.
Cleanup sends stop, disables learning telemetry and disables NN.

Default `--kind all` also tests relative 10-degree position steps. The current
firmware X command accepts absolute unwrapped targets within +/-100 rad.
Position trials are skipped above 99 rad to preserve the existing calibration.

## Hardware observations, 2026-10-21

Two short 60 RPM comparisons used the same calibration and alpha=0.1.
First run: baseline RMSE 6.76 RPM, shadow 6.66, blended 7.63.
Reverse-order run: baseline RMSE 7.35 RPM, shadow 6.83, blended 7.09.
The reverse-order steady means were 59.82, 60.01 and 60.28 RPM respectively;
maximum telemetry sample gap was 10 ms. No recorded trial voltage clipping.
These runs do not demonstrate a repeatable improvement from the current NN.

The first position comparison had baseline final-quarter MAE 6.41 degrees,
shadow 0.0023 degrees and blended 0.188 degrees. Baseline versus shadow
variation means starting conditions need better control before attributing
position differences to NN. The reverse-order run reached the X target range
limit before position measurement, so it provides speed-only evidence.

The current model imitates a hand-selected speed-error residual teacher.
Training MSE measures teacher imitation, not improvement in motor tracking.
Keep NN off by default until repeated comparisons support a useful setting.

## Offline candidate v2, 2026-10-21

`tools/train_nn_candidate.py` creates a separate candidate; the deployed
`app/nn_model.h` and running MCU are unchanged. Reproduce in conda py310:

```powershell
E:\miniconda\envs\py310\python.exe tools\train_nn_candidate.py data\nn_demo_20261014_run4.csv --output data\nn_candidate_v2.npz --evaluate data\nn_compare_20261021_152008.csv data\nn_compare_20261021_152216.csv
```

The six inputs are target speed, measured speed, speed error, causal filtered
error derivative, position error (zero in speed mode), and baseline voltage.
Absolute accumulated positions are removed. The derivative has a 50 ms time
constant and resets at command/mode/trial transitions or sample gaps >50 ms.
It clips the raw derivative to +/-100 rad/s² before filtering. Position-mode
segments use position target changes, since their inner speed target changes
continuously. The teacher remains an explicitly chosen PD residual, not an
optimal voltage measured from the plant.

Ten complete command segments are split into six training, two validation
and two test segments. Normalization uses training rows only. Targets are
scaled during optimization; validation selects among 200/800/2000 epochs
(200 selected). Neither the test split nor independent recordings select
the model. Hidden layer remains eight ReLU units.

| Evaluation | NN teacher RMSE | Zero predictor RMSE | R² |
| --- | ---: | ---: | ---: |
| Held-out command segments | 2.425 mV | 12.216 mV | 0.9606 |
| Independent speed + position recording 152008 | 2.094 mV | 10.460 mV | 0.9599 |
| Independent speed recording 152216 | 2.050 mV | 10.797 mV | 0.9640 |

No predicted voltage saturation or normalized input clipping occurred on the
two independent recordings. The teacher changed (filtered, segmented
derivative), so these numbers are not a like-for-like comparison to the old
model's training MSE. The independent recordings also include shadow/blended
trials; they establish imitation on recorded trajectories only. They do not
establish closed-loop stability, better speed tracking or load robustness.
The derivative is computed at recorded telemetry cadence; firmware must
match that preprocessing cadence before a meaningful deployment comparison.

Outputs: `data/nn_candidate_v2.npz`, `.json` metrics and `.png` figure.
The exporter rejects v2 archives to prevent silently loading incompatible
features into v1 firmware. Export now emits valid C literals (`0.0f`, `1.0f`)
and rejects nonfinite coefficients and inconsistent dimensions.

Next deployment work is matching MCU feature preprocessing, followed by
repeated baseline/blended comparisons using the same calibration. The
candidate's present success is learning its PD teacher; NN superiority over
that cheap explicit PD expression has not been demonstrated.
