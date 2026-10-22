Validation: `JAVA_HOME=~/jdk17 bash ./gradlew build test` — **33 tests, 0 failures, 0 errors, 0 skipped**.

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
| pose jump | 2.14 | 0.000 | 0.01 | 0.005 | 1 | 0 | TIMED_OUT | 0 |
| timeout | — | 3.301 | 0.01 | 0.000 | 0 | 0 | TIMED_OUT | 0 |
| hypothetical high | 2.06 | 0.000 | 0.01 | 0.000 | 0 | 0 | TIMED_OUT | 0 |
| hypothetical unhomed | 2.12 | 0.000 | 0.01 | 0.000 | 0 | 0 | TIMED_OUT | 0 |

Changes vs 73566bd: side convergence slows from 2.14 s to 2.92 s with the direction-preserving 1 m/s vector cap; overshoot drops from 0.005 m to 0.000 m and sign changes from 1 to 0. Red uses the same blue-origin diagonal scenario and matches blue; unknown alliance refuses startup with zero motion (1.803 m initial error remains). The injected 0.65 m pose jump is now rejected before estimation/freshness; convergence remains 2.14 s, with sign changes 0 → 1, matching the no-jump trajectory. Other shared scenario metrics are unchanged. All scenarios have zero false-aligned events and zero vector-cap violations. TIMED_OUT after convergence is expected when held through the total timeout.

Bootstrap unit coverage starts odometry 3 m off, accepts the first valid two-distinct-tag reading as a translation reset, then rejects jumps above 0.5 m. Single-tag readings beyond the gate are rejected before bootstrap. Headless camera scenarios use the production pure filter and an idealized estimator, not CTRE reset/latency dynamics. Release handoff policy is unit-tested independently; the release/restart scenario models controller stop/restart, not physical joystick or scheduler timing. Robot validation remains required.
