# Hold-to-align feature — HANDOFF

**Status: NOT tested on robot. Only unit-tested (`bash ./gradlew build test` passes). Not deployed.**

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

## How to enable (all in `Constants`)
1. `AlignConstants.PEG_TAG_IDS` – fill with real tag IDs (currently empty placeholder; unknown).
2. `AlignConstants.FIELD_LAYOUT` – verify welded vs AndyMark field.
3. `VisionConstants.CAMERA_1/CAMERA_2` – match Limelight hostnames; set camera pose in Limelight UI.
4. `VisionConstants.FUSION_ENABLED = true` (default false so odometry/autos are unchanged) and `AlignConstants.ALIGN_ENABLED = true`.
5. Scoring: there is **no beam break sensor** in the code. `RobotContainer` has a TODO with `pieceHeld = () -> false`; add a sensor, wire it, then set `SCORE_BINDING_ENABLED = true`. Score fires `shootCommandGroup()` only if `align.isAligned()` and piece held at the moment of press. No auto-score.

## Tuning list (all PLACEHOLDER - TUNE ON ROBOT)
Vision: `MAX_YAW_RATE_DEG_PER_SEC`, `MAX_FRAME_AGE_SEC`, `BASE_XY_STD_DEV_M`, `DISTANCE_SCALE_K`.
Align: offsets (`OFFSET_DISTANCE_M`, `OFFSET_LATERAL_M` — verify sign of lateral), XY/THETA PID gains, speed/accel caps, `XY_TOLERANCE_M`, `THETA_TOLERANCE_RAD`, `ALIGNED_LOOPS`, `TIMEOUT_SEC`, `LOST_TARGET_SEC`.

## Safety notes
- Test first on blocks, then slow with caps lowered and someone on e-stop.
- Tag pose / field-layout / offset sign errors will drive the robot at the wrong spot or into a wall.
- Timeout is total time since button press (including after alignment) — the command ends and alignment goes false.
- Heading for MegaTag2 comes from the fused drivetrain pose rotation; make sure gyro is seeded correctly (left bumper / button 7) before trusting vision.
- Align/vision dashboard keys: `Align/*`, `Vision/*`.
