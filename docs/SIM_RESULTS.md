Validation: `JAVA_HOME=~/jdk17 bash ./gradlew build test` — **43 tests, 0 failures, 0 errors, 0 skipped**.
Separate sim rerun: `JAVA_HOME=~/jdk17 bash ./gradlew test --tests frc.robot.sim.AlignSimTest --rerun-tasks` — **4 tests passed; 16 scenarios**.

| Scenario | Converge s | Final xy m | Theta deg | Overshoot m | Sign changes | False aligned | End | Cap violations |
|---|---:|---:|---:|---:|---:|---:|---|---:|
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

Changes vs bb67c3f: all 15 shared scenario metrics are unchanged. Added **untrusted pose**: startup refuses with POSE_UNTRUSTED, every output remains zero, and alignment stays false. All 16 scenarios have zero false-aligned events and zero cap violations. TIMED_OUT after convergence is expected when held through total timeout. Existing controller scenarios explicitly bootstrap a pure trust policy before starting; the untrusted scenario supplies false throughout.

Single-tag bootstrap now requires five consecutive validated same-ID exposures whose maximum pairwise translation distance is <= 0.05 m, in addition to distance < 2.0 m and ambiguity < 0.2. These are starting guesses, not measured, tune on robot. Pending candidates are withheld from fusion until stable. ID changes, rejected frames (including duplicates), disable/enable, and scheduled align clear the window. Each exposure is consumed once, including pending candidates. An unstable rolling window cannot reset; five subsequent consistent readings can. Two distinct known tags may still bootstrap immediately while align is not scheduled. Scheduled alignment never resets or bypasses the normal jump gate.

Trust is false at boot and on each disabled-to-enabled transition, and becomes true only after successful reset. Gate-passing fusion before bootstrap does not grant trust. The first enabled periodic still defers processing for trigger scheduling. AlignController uses an injected trust supplier and refuses startup with POSE_UNTRUSTED; VisionSubsystem publishes Align/PoseTrusted. Tests cover stable/unstable readings, ID/rejection/align window clearing, bootstrap-dependent alignment, and injected re-enable transitions, alongside existing jump-gate and held-trigger scheduler coverage.

The camera sim uses production pure filtering with ideal odometry/latency compensation; it does not model CTRE estimator/reset dynamics or real camera ambiguity. Single-tag stability and enable transitions are tested separately in pure regression tests. Release/restart models controller stop/restart, not physical joystick dynamics. Robot validation remains required; switches stay false and deployment tags stay empty.
