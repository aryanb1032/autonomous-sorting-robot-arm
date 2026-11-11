# Verification records

This page indexes existing project evidence. Documentation updated 11 November 2026; no new build, simulation, firmware upload or physical motion was performed for this update.

| Record | Recorded scope | Result / boundary |
|---|---|---|
| [TEST_RESULTS.md](TEST_RESULTS.md), 26 October | arm_sorter build, 13 Python tests, dry run, mock trajectory integration, syntax and CPU inference | Recorded passes; several simulation and hardware checks were still pending at that checkpoint |
| [autonomous/VALIDATION.md](autonomous/VALIDATION.md), 2 November | 19 tests, two-cube and six-cube simulation episodes, home return and active APPROACH STOP | Recorded passes; simulated object poses and detachable-joint grasps |
| [hardware/VALIDATION.md](hardware/VALIDATION.md) | Hardware preparation and commissioning record | Consult the dated source for the exact configuration and observations |
| [docs/PROJECT_REPORT.md](docs/PROJECT_REPORT.md) | Project history and evidence | Later physical autonomy was owner-reported on a different laptop and is outside this snapshot |

The 26 October checklist is historical; later records describe subsequent simulation work. Counts belong to their own dated runs and must not be added together as a new test suite result.

Physical camera calibration, joint tracking, grasp force and end-to-end autonomous performance require checks on the current hardware. Gazebo uses object ground truth and simulated attachments. The policy experiment concerns sorting order and does not demonstrate learned grasp control or policy convergence.

The startup and grasp requirements remain documented in [AGENTS.md](AGENTS.md) and [hardware/HOME90_POLICY.md](hardware/HOME90_POLICY.md). No physical homing sensor or servo-power rail sensor is claimed.
