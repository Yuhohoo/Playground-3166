Validation: `JAVA_HOME=~/jdk17 bash ./gradlew build test` — **39 tests, 0 failures, 0 errors, 0 skipped**.
Separate sim rerun: `JAVA_HOME=~/jdk17 bash ./gradlew test --tests frc.robot.sim.AlignSimTest --rerun-tasks` — **4 tests passed; 15 scenarios**.

| Scenario | Converge s | Final xy m | Theta deg | Overshoot m | Sign changes | False aligned | End | Cap violations |
|---|---:|---:|---:|---:|---:|---:|---|---:|
| near | 1.28 | 0.000 | 0.01 | 0.005 | 0 | 0 | TIMED_OUT | 0 |
| far | 3.34 | 0.000 | 0.01 | 0.005 | 1 | 0 | TIMED_OUT | 0 |
| heading 45deg | 2.04 | 0.000 | 0.01 | 0.005 | 1 | 0 | TIMED_OUT | 0 |
| FOV edge | 2.04 | 0.000 | 0.01 | 0.005 | 1 | 0 | TIMED_OUT | 0 |
| red alliance | 2.92 | 0.000 | 0.01 | 0.000 | 0 | 0 | TIMED_OUT | 0 |
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

Changes vs 3f15fcf: all 14 shared scenario metrics are unchanged. Added forced pose reset at 0.80 s: estimation shifts +2 m without moving the physical robot; controller immediately returns zero on every axis, latches POSE_RESET, finishes NOT aligned and never chases the shifted estimate. Physical stopping leaves 0.863 m error in this idealized model. All 15 scenarios have zero false-aligned events and zero cap violations. TIMED_OUT after convergence is expected when held through the total timeout.

Bootstrap now requires align not scheduled and all IDs in the layout. Two distinct tags qualify, or one tag with RawFiducial distToCamera < 2.0 m and ambiguity < 0.2. During alignment eligible frames use the normal jump gate/fusion without reset and leave bootstrap pending. Regression coverage includes close/confident acceptance, distance/ambiguity boundary rejection, unknown IDs and scheduled inhibition. A real CommandScheduler test exercises subsystem periodic before trigger polling with trigger held through enable: first enabled periodic is deferred; no reset occurs while scheduled; pending bootstrap becomes eligible after release.

Controller tests cover translation and heading-only resets during motion and after alignment, persistent zero output, cleared alignment and unchanged target. Release handoff tests verify drivetrain ownership and dashboard hold/reason clearing. The camera sim uses production pure filtering and idealized odometry/latency compensation; it does not model CTRE estimation/reset dynamics or real camera ambiguity. The release/restart scenario models controller stop/restart, not physical joystick dynamics. All tuning margins are starting guesses, not measured, tune on robot. Robot validation remains required; flags stay false and deployment tags stay empty.
