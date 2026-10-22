# Hold-to-align feature — HANDOFF

**Status: NOT tested on robot. Unit and headless closed-loop tests pass. Not deployed. All three enable flags remain false.**

## What was built
- `vision/VisionFilter` – pure accept/reject (no tags, spin rate, stale, duplicate timestamp, NaN) and std devs `base*(1+d²/k)`.
- `vision/VisionSubsystem` – per loop: `SetRobotOrientation` from drivetrain pose heading + yaw rate, reads MegaTag2 (`getBotPoseEstimate_wpiBlue_MegaTag2`) for up to 2 cameras, feeds `drivetrain.addVisionMeasurement(pose, fpgaTimestamp, stdDevs)`. The drivetrain override already applies `Utils.fpgaToCurrentTime`, so the raw FPGA timestamp is passed (do not convert twice). Exposes `getLastAcceptedTimestamp()`.
- `align/AlignMath` – target pose (tag pose + distance/lateral offset, rotated 180° to face tag), nearest allowed tag, tolerance check.
- `align/AlignStateMachine` – N-consecutive-loop aligned counter, total timeout, lost-target (no fresh vision > N s). Timestamps injected.
- `commands/AlignToPegCommand` – ProfiledPID x/y/theta (continuous), speed caps, `FieldCentric` with `ForwardPerspectiveValue.BlueAlliance`. Ends NOT aligned on timeout / lost vision / no tag. `isAligned()` only true while scheduled, in tolerance N loops, vision fresh.
- Tests: `src/test/java/frc/robot/AlignVisionTest.java` (JUnit 5; build.gradle already had junit + `useJUnitPlatform()`).

## Buttons (driver controller, port 0)
- **Align: hold Right Trigger** (only bound if `ALIGN_ENABLED`).
- **Score: Left Trigger** — currently NOT bound (see below).
- Existing bindings/default commands/autos untouched.

## Field layout (Stow and Seal is a custom local field)
- The WPILib `k2026RebuiltWelded` guess was removed. Tags load from `src/main/deploy/stow_and_seal_tags.json` (name in `AlignConstants.FIELD_LAYOUT_FILE`) via `align/FieldLayoutLoader` (never throws; warns via DriverStation/stderr).
- **The shipped JSON is a PLACEHOLDER**: empty `tags` list and made-up `field` length/width (16.0 x 8.0 m). With zero tags (or missing/bad file) the loader returns empty, so **align binding, AlignToPegCommand and vision fusion stay disabled at runtime even if `ALIGN_ENABLED`/`FUSION_ENABLED` are true**.
- To fill it: measure each tag's center pose in the WPILib blue-origin field frame (meters, z = height) and add entries to `tags`:
  `{"ID": 1, "pose": {"translation": {"x": 1.0, "y": 2.0, "z": 0.5}, "rotation": {"quaternion": {"W": 1.0, "X": 0.0, "Y": 0.0, "Z": 0.0}}}}`
  (rotation is the tag's orientation; the tag normal faces out of the tag). Set real `field.length`/`field.width`. Use the same layout in the Limelight UI field map. Then redeploy.

## Safety gating (added)
- **Score** (`AlignGating.gatedWhile`): `aligned && pieceHeld` is re-checked every loop for the whole shot; if it drops, the shoot group is cancelled and shooter + indexer are stopped.
- **Left bumper seed** is ignored while the align command is scheduled.
- **Vision freshness is per target tag**: `VisionSubsystem.getLastAcceptedTimestampForTag(id)`; only accepted frames whose fiducials include the target tag count. Fusion accepts fiducials from the entire loaded layout (including non-pegs); any unknown ID rejects the whole frame. `PEG_TAG_IDS` selects align targets and which tag freshness is recorded. Empty peg IDs allow fusion but make alignment finish immediately without a target or nonzero output.
- Tests: `GatingAndFreshnessTest`, `FieldLayoutLoaderTest`, `AlignVisionTest`.

## How to enable (all in `Constants`)
1. `AlignConstants.PEG_TAG_IDS` – currently {4, 8}, INFERRED from CAD part names, confirm with Limelight on field, and fill the JSON above.
3. `VisionConstants.CAMERA_1/CAMERA_2` – match Limelight hostnames; set camera pose in Limelight UI.
4. `VisionConstants.FUSION_ENABLED = true` (default false so odometry/autos are unchanged) and `AlignConstants.ALIGN_ENABLED = true`.
5. Scoring: there is **no beam break sensor** in the code. `RobotContainer` has a TODO with `pieceHeld = () -> false`; add a sensor, wire it, then set `SCORE_BINDING_ENABLED = true`. Score fires `shootCommandGroup()` only if `align.isAligned()` and piece held at the moment of press. No auto-score.

## Tuning list (all PLACEHOLDER - TUNE ON ROBOT)
Vision: `MAX_YAW_RATE_DEG_PER_SEC`, `MAX_FRAME_AGE_SEC`, `BASE_XY_STD_DEV_M`, `DISTANCE_SCALE_K`.
Align: offsets (`OFFSET_DISTANCE_M`, `OFFSET_LATERAL_M` — verify sign of lateral), XY/THETA PID gains, speed/accel caps, `XY_TOLERANCE_M`, `THETA_TOLERANCE_RAD`, `ALIGNED_LOOPS`, `TIMEOUT_SEC`, `LOST_TARGET_SEC`.

## Safety notes
- First-test driver rule: **release the stick before releasing the align button**; the end/release handoff holds zero with the drivetrain requirement until left-stick magnitude and right-stick X are inside the existing 10% manual deadbands, then default drive resumes.
- Risk 5: time the 0.5 s freshness grace and physical stopping distance on the real robot.
- Risks 1 (unknown alliance), 2 (diagonal speed cap), and pose-jump rejection are fixed in align/vision code only; drivetrain, default drive and drive constants remain untouched. The added align binding block alone also gains the end/release handoff. Known red/blue alliances retain blue-origin coordinates; unknown alliance refuses startup with zero output and not aligned. The 0.5 m pose-jump threshold is a starting guess, not measured, tune on robot.
- First real run: on blocks with an E-stop operator, blue then red, at low speed; then test slowly on the floor.
- Limelight computes pose using the field map uploaded to the camera. Upload the same measured stow_and_seal layout to each Limelight and check they match the robot layout: **NOT VERIFIED**.
- On each enable, vision defers its first enabled periodic to let held triggers schedule (subsystem periodic precedes trigger polling). The first otherwise accepted known-layout frame may bootstrap translation only while align is NOT scheduled: two distinct layout tags, or five consecutive same-ID single-tag readings with maximum pairwise translation spread <= 0.05 m, RawFiducial distToCamera < 2.0 m and ambiguity < 0.2 (both starting guesses, not measured, tune on robot). Gyro heading is retained. While align is scheduled, eligible frames use the normal 0.5 m jump gate and fusion, never reset; bootstrap remains pending. After bootstrap all frames use the normal gate. Disabled fusion remains inactive. Capture translation is not latency propagated for reset.
- Align compares consecutive estimated translation and wrapped heading deltas with previous command speed * elapsed time plus 0.5 m / 20-degree margins. Excess or invalid deltas latch POSE_RESET, immediately output zero and end NOT aligned; a new press is required to restart. Small resets inside the margins and changes between samples cannot be distinguished without a drivetrain reset counter. Margins are starting guesses, not measured, tune on robot.
- Release handoff publishes Align/HandoffHoldingZero=true and Align/HandoffReason="Release sticks to drive" while sticks exceed deadband; neutral, finish or interruption clears both.
- Tag pose / field-layout / offset sign errors will drive the robot at the wrong spot or into a wall.
- Timeout is total time since button press (including after alignment) — the command ends and alignment goes false.
- Heading for MegaTag2 comes from the fused drivetrain pose rotation; make sure gyro is seeded correctly (left bumper / button 7) before trusting vision.
- Align/vision dashboard keys: `Align/*`, `Vision/*`.

## Closed-loop validation and changes
- `align/AlignController` contains the exact profiled PID, vector translation cap, target selection and state machine used by `AlignToPegCommand` and the headless simulation. Start resets controllers using field-relative measured velocity. End/release/loss/timeout explicitly return or send zero; no prior nonzero request is left latched by alignment.
- `src/test/resources/placeholder_tags.json` contains synthetic 4/8/1 poses; its adjacent README marks PLACEHOLDER / NOT MEASURED. It is test-only. Deployment JSON remains empty.
- `src/test/java/frc/robot/sim/AlignSimTest.java` runs 50 Hz with deterministic Gaussian noise, 60 ms latency, 8% dropouts, 100-degree FOV, ideal odometry latency compensation and acceleration-limited kinematics. Drive maximums/deadband are read from source without starting CAN. There is no drive slew limiter and align does not inherit manual deadband.
- Scenarios cover near/far, heading, FOV edge, lateral offset, target loss, release/restart, mixed non-peg and unknown IDs, transient pose jump, total timeout, and hypothetical elevator caps. Scoring before alignment and after target freshness expires is denied through `AlignGating`.
- No elevator exists in 3166 code. Test-only high/unhomed translation caps do not implement robot functionality.
- Metrics print to test stdout and `build/sim-results.md`; a checked-in snapshot is in `docs/SIM_RESULTS.md`. The command intentionally reaches total timeout even after converging if the button remains held.
- Risks and proposed fixes with source citations: `docs/REAL_ROBOT_RISKS.md`. This sim cannot validate traction, CTRE estimation, measured field geometry, gyro mounting or actual camera calibration.
- Reproduce: `JAVA_HOME=~/jdk17 bash ./gradlew build test`.

## This revision vs bb67c3f
Single-tag reset requires VISION_SINGLE_TAG_STABLE_FRAMES = 5 and maximum pairwise translation spread <= VISION_SINGLE_TAG_MAX_SPREAD_M = 0.05. Both are starting guesses, not measured, tune on robot. Existing distance/ambiguity limits remain. Pending candidates do not fuse; missing/rejected fresh frames (duplicates are skipped) and ID changes clear only the originating camera's rolling window, keyed by camera name. Disable/enable, scheduled align and successful reset clear all windows. Each exposure is consumed once. Single-tag readings never reset or bypass the jump gate during align.

VisionBootstrap holds testable pose trust, false at boot and after a disabled duration longer than POSE_TRUST_MAX_DISABLE_S (5.0 s), measured with injected timestamps. Only a successful bootstrap reset grants trust; ordinary accepted fusion before bootstrap cannot. VisionSubsystem publishes Align/PoseTrusted. AlignController/AlignToPegCommand take an injected trust supplier and refuse startup with zero output, NOT aligned and POSE_UNTRUSTED. Held-at-enable align may refuse; release/repress after bootstrap. Existing enable-edge deferral remains.

Validation: 46 tests pass, plus separate sim rerun (4 tests, 16 scenarios). Added untrusted-pose refusal; all 15 shared sim scenarios retain bb67c3f metrics, zero false alignment and cap violations throughout. Added stable/unstable, ID/rejection/scheduled window reset, bootstrap-dependent align and re-enable trust tests. See docs/SIM_RESULTS.md and docs/REAL_ROBOT_RISKS.md. No robot validation or deployment; all switches false and deployment tags empty.

Per-camera window regression validation: camera A completes its five-frame single-tag bootstrap while camera B has no estimate each loop. A rejected frame from A restarts only A's streak; B's streak survives. Scheduled align clears both cameras' windows. Full build/test: **46 tests passed**, no failures/errors/skips. Separate sim rerun: **4 tests, 16 scenarios passed**, unchanged metrics with zero false alignment and cap violations. All switches remain false and deployment tags empty.


## Pose-trust lifetime revision
POSE_TRUST_MAX_DISABLE_S = 5.0 is a starting guess, not measured, tune on robot. Trust clears only at boot, after a disabled period strictly longer than this limit (checked while disabled and on re-enable using injected timestamps), on jump-gate rejection, or when align ends with POSE_RESET. The align command's end/stop callback is wired to the vision trust tracker in our ALIGN_ENABLED block. Ordinary frame rejection, normal align completion, short disable and enable do not clear trust. Bootstrap windows still clear on disable/enable; first-enabled-loop deferral remains. Only successful bootstrap grants trust. A 1–3 s auto-to-teleop transition can align immediately without re-bootstrap.

Validation: **49 tests pass**, separate sim rerun **4 tests / 18 scenarios**. Short-disable scenario converges without re-bootstrap; long-disable scenario refuses POSE_UNTRUSTED. All prior scenario metrics unchanged. No robot validation or deployment. Switches false and deployment tags empty; no drive/control/TunerConstants/CommandSwerveDrivetrain changes.


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


## Running alignment trust and end handoff
AlignController checks pose trust before every running update and returns zero immediately with POSE_UNTRUSTED on loss, including a jump-rejected frame. isAligned checks the live trust supplier as well, so it is false from the drop cycle even before execute; the terminal state stays not aligned after trust recovers. AlignToPegCommand publishes Align/EndReason before clearing its controller, sends zero on end, and invokes the end handoff callback.

Our ALIGN_ENABLED block schedules one shared DriverHandoff hold-zero command from every align end (self-completion and interruption). The separate trigger-release binding is removed. With the trigger still held, alignment does not restart; drive stays owned with zero output until sticks enter the existing deadbands. Releasing the trigger after self-completion does not schedule a second handoff. No default-command or drivetrain changes.

VisionSubsystem publishes Align/TrustDrops from its testable VisionBootstrap tracker. Each true-to-false transition increments once, including jump rejection, long disable and explicit trust clearing; repeated clears while already false do not increment. Only bootstrap restores trust.

Validation: **53 tests passed, 0 failures/errors/skips** using `JAVA_HOME=~/jdk17 bash ./gradlew build test`. Separate sim rerun: **4 tests / 20 scenarios passed**. New `trust drop mid-align` row ends POSE_UNTRUSTED, zero false alignment/cap violations, final xy 0.863 m; all prior scenario metrics unchanged. Regressions include trust loss after aligned, arbitrary clearing, trust-drop counter transitions, and held-trigger scheduler handoff before default drive. Not robot validated or deployed. All three enable flags remain false, deployment tags empty, and placeholders test-only.


## Per-camera trust-drop threshold (current revision)

Supersedes earlier single-jump trust-loss descriptions: VisionConstants.VISION_TRUST_DROP_FRAMES
is 3, commented "starting guess, not measured, tune on robot". Each camera counts its own
fresh jump-rejected frames; three drop trust, while one or two preserve running alignment.
A fresh accepted/fused frame resets only that camera's count. Duplicate timestamps count
neither way; rejected poses still never fuse. Alternating A,B,A,B leaves each at two;
a third rejection from either camera drops trust. Ordinary rejections do not advance counts.
Successful bootstrap and trust clearing clear counters. Boot, long disable and POSE_RESET
trust clearing remain unchanged. Running align still ends POSE_UNTRUSTED with zero output
on trust loss, preserving a810226 behavior; Align/TrustDrops still counts true-to-false transitions.

Validation: **56 tests passed, 0 failures/errors/skips** with the documented build command;
separate sim rerun **4 tests / 20 scenarios passed**. Moved-while-disabled and mid-align
regressions now require three rejected frames. Only changed sim metric: `trust drop mid-align`
final xy **0.863 -> 0.823 m**, with trust dropping at 0.84 s instead of 0.80 s.
All other rows/columns unchanged, zero false alignment/cap violations. No drive/control changes;
all three switches false, deploy tags empty, synthetic placeholders test-only. Not pushed,
not deployed, not robot validated.

## Real WPILib scheduler handoff and jump diagnostics (current revision)
`HandoffSchedulerTest` uses HAL.initialize(500, 0), an enabled blue-alliance
DriverStationSim, the real CommandScheduler, a boolean Trigger, a stand-in subsystem
and a default command recording each execution loop. The stand-in align inherits the
same AlignEndCommand end-handler path as AlignToPegCommand; both use
DriverHandoff.wireEnd. Each test cancels commands, unregisters subsystems, clears
button loops and resets DriverStationSim.

Direct end()-scheduling FAILED button release: default drive executed during the
handoff. Self-completion and disable passed that initial experiment. Production now
uses an end callback to arm a Trigger gated by !align.isScheduled() and
DriverStation.isEnabled(). Only an actual enabled align end arms it; disabling
clears it. On the next scheduler loop the trigger takes the drivetrain before default
execution. Final cases PASS: release with off-center sticks, self-completion while
held, and disable with no handoff or drive execution. Both enabled cases verify no
default execution before neutral sticks and default resumption afterward.

VisionSubsystem publishes lifetime running totals at exactly
`Align/JumpRejects/<camera>` using each configured Limelight name, initialized to zero.
JumpRejectCounter is pure and tested: fresh jump rejections increment per camera;
duplicates do not increment; accepted frames do not reset totals. Existing duplicate
filtering precedes counting. These totals do not change pose trust or consecutive
rejection thresholds.

Validation: `JAVA_HOME=~/jdk17 bash ./gradlew build test`: **59 tests passed,
0 failures/errors/skips**, including all three HAL scheduler cases and all 20 closed-loop
scenarios. Trust logic unchanged. All switches false, deployment tags empty; no
drive/control/default-command changes. Not robot validated, deployed or pushed.


## Heading-reset detection (current revision)
`HeadingResetDetector` compares wrapped consecutive fused Pose heading and RawHeading deltas
from one Phoenix state snapshot per VisionSubsystem loop, including disabled, enable-deferral
and fusion-inactive loops. First observation only baselines. An absolute delta difference
strictly greater than HEADING_RESET_THRESHOLD_DEG = 5.0 clears pose trust immediately
and increments Align/TrustDrops once per true-to-false transition. The threshold is a
starting guess, not measured, tune on robot. The existing per-cycle align trust check
then returns zero and ends POSE_UNTRUSTED; a later bootstrap and new press are needed.
Loops performing our VisionBootstrap reset skip detection entirely and re-baseline from
post-reset state. Only bootstrap grants trust.

Assumption: THETA_STD_DEV_RAD = 9999999.0 is effectively infinite, so ordinary vision
fusion cannot materially change heading. Revisit the detector if heading fusion is enabled.
Small offset changes <=5 degrees per sample, canceling resets between samples, and changes
before the first baseline can evade detection. Validate actual Phoenix sampling/reset timing
on robot. No drive/control or RobotContainer binding changes; all three switches false,
deployment tags empty. Not robot validated, deployed or pushed.

Validation: `JAVA_HOME=~/jdk17 bash ./gradlew build test` — **64 tests passed,
0 failures/errors/skips**. Separate sim rerun with `--tests frc.robot.sim.AlignSimTest
--rerun-tasks` — **4 tests / 21 scenarios passed**. Five new tests cover seed at 90 degrees
before and during alignment, small drift/physical turns, bootstrap heading reset skip and
re-baseline, independent +/-180 crossings, TrustDrops counting and later bootstrap recovery.
New `seedFieldCentric mid-match` sim starts at 90 degrees, injects pose-heading zero at 0.20 s
without changing raw physical yaw, and ends POSE_UNTRUSTED with zero output on that cycle.
Final physical xy 1.454 m, theta 77.85 degrees; all 21 scenarios have zero false alignment
and cap violations. All 20 prior scenario metrics unchanged. This injects Phoenix's documented
reset effect; it does not execute JNI or model CTRE estimator dynamics.
