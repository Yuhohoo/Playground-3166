Validation: `JAVA_HOME=~/jdk17 bash ./gradlew build test` — **56 tests, 0 failures, 0 errors, 0 skipped**.
Separate sim rerun: `JAVA_HOME=~/jdk17 bash ./gradlew test --tests frc.robot.sim.AlignSimTest --rerun-tasks` — **4 tests passed; 20 scenarios**.

| Scenario | Converge s | Final xy m | Theta deg | Overshoot m | Sign changes | False aligned | End | Cap violations |
|---|---:|---:|---:|---:|---:|---:|---|---:|
| auto->teleop short disable | 1.28 | 0.000 | 0.01 | 0.005 | 0 | 0 | TIMED_OUT | 0 |
| moved while disabled | — | 0.650 | 0.00 | 0.000 | 0 | 0 | POSE_UNTRUSTED | 0 |
| long disable | — | 0.650 | 0.00 | 0.000 | 0 | 0 | POSE_UNTRUSTED | 0 |
| trust drop mid-align | — | 0.823 | 0.01 | 0.000 | 0 | 0 | POSE_UNTRUSTED | 0 |
| near | 1.28 | 0.000 | 0.01 | 0.005 | 0 | 0 | TIMED_OUT | 0 |
| far | 3.34 | 0.000 | 0.01 | 0.005 | 1 | 0 | TIMED_OUT | 0 |
| heading 45deg | 2.04 | 0.000 | 0.01 | 0.005 | 1 | 0 | TIMED_OUT | 0 |
| FOV edge | 2.04 | 0.000 | 0.01 | 0.005 | 1 | 0 | TIMED_OUT | 0 |
| red alliance | 2.92 | 0.000 | 0.01 | 0.000 | 0 | 0 | TIMED_OUT | 0 |
| untrusted pose | — | 1.803 | 0.00 | 0.000 | 0 | 0 | POSE_UNTRUSTED | 0 |
| unknown alliance | — | 1.803 | 0.00 | 0.000 | 0 | 0 | REFUSED | 0 |
| side | 2.92 | 0.000 | 0.01 | 0.000 | 0 | 0 | TIMED_OUT | 0 |
| lost halfway | — | 1.363 | 0.04 | 0.000 | 0 | 0 | LOST_TARGET | 0 |
| release/restart | 3.52 | 0.000 | 0.00 | 0.005 | 1 | 0 | RUNNING | 0 |
| non-peg + unknown | 2.14 | 0.000 | 0.01 | 0.005 | 1 | 0 | TIMED_OUT | 0 |
| forced pose reset | — | 0.863 | 0.01 | 0.000 | 0 | 0 | POSE_RESET | 0 |
| pose jump | 2.14 | 0.000 | 0.01 | 0.005 | 1 | 0 | TIMED_OUT | 0 |
| timeout | — | 3.301 | 0.01 | 0.000 | 0 | 0 | TIMED_OUT | 0 |
| hypothetical high | 2.06 | 0.000 | 0.01 | 0.000 | 0 | 0 | TIMED_OUT | 0 |
| hypothetical unhomed | 2.12 | 0.000 | 0.01 | 0.000 | 0 | 0 | TIMED_OUT | 0 |

Changes vs bb67c3f: all 15 shared scenario metrics are unchanged. Added **untrusted pose**: startup refuses with POSE_UNTRUSTED, every output remains zero, and alignment stays false. The original 16 scenarios have zero false-aligned events and zero cap violations. TIMED_OUT after convergence is expected when held through total timeout. Existing controller scenarios explicitly bootstrap a pure trust policy before starting; the untrusted scenario supplies false throughout.

Single-tag bootstrap now requires five consecutive validated same-ID exposures whose maximum pairwise translation distance is <= 0.05 m, in addition to distance < 2.0 m and ambiguity < 0.2. These are starting guesses, not measured, tune on robot. Pending candidates are withheld from fusion until stable. Windows are keyed by camera name: ID changes and missing/rejected fresh frames (duplicates are skipped) clear only that camera's window. Disable/enable, scheduled align, and successful bootstrap reset clear all windows. Each exposure is consumed once, including pending candidates. An unstable rolling window cannot reset; five subsequent consistent readings can. Two distinct known tags may still bootstrap immediately while align is not scheduled. Scheduled alignment never resets or bypasses the normal jump gate.

Trust is false at boot, after more than 5.0 s disabled, on a jump-gate rejection, and when align ends with POSE_RESET. Disabled duration uses injected timestamps and is checked while disabled and on re-enable. Short disables preserve trust; only a successful bootstrap reset grants trust. Gate-passing fusion before bootstrap does not grant trust. The first enabled periodic still defers processing for trigger scheduling. AlignController uses an injected trust supplier and refuses startup with POSE_UNTRUSTED; VisionSubsystem publishes Align/PoseTrusted. Tests cover stable/unstable readings, ID/rejection/align window clearing, bootstrap-dependent alignment, and injected re-enable transitions, alongside existing jump-gate and held-trigger scheduler coverage.

The camera sim uses production pure filtering with ideal odometry/latency compensation; it does not model CTRE estimator/reset dynamics or real camera ambiguity. Single-tag stability and enable transitions are tested separately in pure regression tests. Release/restart models controller stop/restart, not physical joystick dynamics. Robot validation remains required; switches stay false and deployment tags stay empty.

Per-camera window regression validation: camera A completes its five-frame single-tag bootstrap while camera B has no estimate each loop. A rejected frame from A restarts only A's streak; B's streak survives. Scheduled align clears both cameras' windows. Full build/test: **46 tests passed**, no failures/errors/skips. Separate sim rerun: **4 tests, 16 scenarios passed**, unchanged metrics with zero false alignment and cap violations. All switches remain false and deployment tags empty.


Current validation: **49 tests passed** in the full build/test, plus a separate sim rerun (**4 tests, 18 scenarios**). Added auto->teleop short disable: auto reaches aligned and ends trusted, a 3 s disable retains trust, and teleop converges without re-bootstrap. Long disable (6 s) refuses POSE_UNTRUSTED with zero output. Existing 16 scenario metrics are unchanged; all 18 scenarios have zero false alignment and cap violations. Unit tests also cover the exact 5 s boundary, expiry while disabled and on re-enable, jump rejection, and POSE_RESET end notification.


## Duplicate exposures and movement during disable
Current validation: **51 tests passed, 0 failures/errors/skips** with
`JAVA_HOME=~/jdk17 bash ./gradlew build test`. Separate sim rerun:
`JAVA_HOME=~/jdk17 bash ./gradlew test --tests frc.robot.sim.AlignSimTest --rerun-tasks`
— **4 tests, 19 scenarios passed**. Earlier validation counts above describe prior revisions.

Duplicate timestamps are skipped before frame validation and never call rejectFrame or advance/reset
the originating camera's stability window. The pure bootstrap/filter regression alternates fresh
and duplicate frames and reaches single-tag reset eligibility after exactly five fresh stable frames.

Moved-while-disabled regression and sim: auto finishes aligned and trusted, a 3 s disable preserves
trust, then a moved-pose vision frame fails the normal jump gate before teleop alignment starts.
Trust clears and alignment refuses POSE_UNTRUSTED with zero output. The short-disable scenario
still converges without re-bootstrap; long disable still refuses. All 19 scenarios have zero false
alignment and cap violations. POSE_TRUST_MAX_DISABLE_S remains 5.0, commented as a starting guess,
not measured, tune on robot. No drive/control changes; all three switches remain false and deploy
tags remain empty. No robot validation or deployment.


## Trust loss during alignment
Current revision: **53 tests passed**, no failures/errors/skips; separate sim rerun **4 tests / 20 scenarios passed**. The new **trust drop mid-align** scenario injects a jump-gate rejection at 0.80 s through the production VisionBootstrap trust tracker while alignment is running. That cycle and every later cycle return zero, end POSE_UNTRUSTED and never report aligned; Align/TrustDrops increments once. The remaining 19 scenario metrics are unchanged. The older `pose jump` scenario isolates filter rejection/recovery without mutating the trust tracker; the new scenario checks the integrated trust-loss behavior.

Unit regression also drops trust after reaching aligned, checks isAligned before the next update, verifies arbitrary trust clearing, repeat-clear counting and no resurrection after trust recovery. A scheduler regression holds the trigger through self-completion with off-center sticks: the single DriverHandoff command owns drive and commands zero, default drive never executes until neutral, and releasing the held trigger does not restart alignment or schedule another handoff. Software zero output is not a measured physical stopping distance.


## Per-camera jump rejection threshold (current revision)

Full build/test: **56 tests passed, 0 failures/errors/skips**. Separate sim rerun:
**4 tests / 20 scenarios passed**. VISION_TRUST_DROP_FRAMES = 3 is a starting guess,
not measured, tune on robot. Trust now drops after three fresh jump-rejected exposures
from the same camera. One or two retain trust and running alignment. Other cameras'
frames do not increment that count; a fresh fused frame resets only its own camera.
Duplicate timestamps neither increment nor reset counts. Jump-rejected frames never fuse.
Boot, long-disable and POSE_RESET clearing and true-to-false TrustDrops counting are unchanged.
This supersedes the single-rejection trust-loss descriptions in earlier revision notes.

The moved-while-disabled scenario supplies three rejected exposures before teleop starts;
its metrics are unchanged. The running trust-drop scenario supplies rejections at 0.80,
0.82 and 0.84 s; it ends POSE_UNTRUSTED with zero output on the third rejection.
Only changed row: `trust drop mid-align`, final xy **0.863 -> 0.823 m**; all other columns
and the other 19 rows are unchanged. All scenarios retain zero false alignment and cap violations.
