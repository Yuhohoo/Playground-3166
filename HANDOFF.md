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
- First-test driver rule: **release the stick before releasing the align button**; the release handoff holds zero with the drivetrain requirement until left-stick magnitude and right-stick X are inside the existing 10% manual deadbands, then default drive resumes.
- Risk 5: time the 0.5 s freshness grace and physical stopping distance on the real robot.
- Risks 1 (unknown alliance), 2 (diagonal speed cap), and pose-jump rejection are fixed in align/vision code only; drivetrain, default drive and drive constants remain untouched. The added align binding block alone also gains the release handoff. Known red/blue alliances retain blue-origin coordinates; unknown alliance refuses startup with zero output and not aligned. The 0.5 m pose-jump threshold is a starting guess, not measured, tune on robot.
- First real run: on blocks with an E-stop operator, blue then red, at low speed; then test slowly on the floor.
- Limelight computes pose using the field map uploaded to the camera. Upload the same measured stow_and_seal layout to each Limelight and check they match the robot layout: **NOT VERIFIED**.
- On each robot enable, the first otherwise accepted frame seeing at least two distinct tags, all in the layout, bypasses the 0.5 m gate and calls `resetPose` with measured translation and current gyro heading. This deliberately trusts that frame to recover arbitrary startup odometry error; validate map/extrinsics first. Single-tag readings before bootstrap still use the 0.5 m gate, and accepting them does not complete bootstrap. After bootstrap all frames use the gate. Fusion is inactive while disabled. Reset uses capture translation without latency propagation; validate reset timing on blocks.
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
