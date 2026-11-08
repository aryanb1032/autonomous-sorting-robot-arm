# Local validation — 2026-10-26

- Installed Stable Baselines3 2.9.2 and Gymnasium 1.4.0. pip check passed.
- Three action tests passed: bounds/wrist compensation, zero action, invalid values.
- First integration attempt correctly stopped on missing joint feedback after the
  Gazebo GUI/server had been closed. Restarted simulation before continuing.
- Short integration run saved and reloaded policy plus replay buffer. Resumed run
  increased cumulative timesteps and optimizer updates from 2 to 4.
- Tightened target selection to require a starting separation of at least 30 mm,
  preventing an already-nearby target from satisfying the 15 mm success threshold.
- Final training batch: 2 episodes, 10 actions each, 20 optimizer updates.
  Final distances: 0.0248345 m and 0.0209110 m. Both timed out without success.
- Checkpoints and experience are in runs/reach. No physical-arm commands issued.

These are implementation checks, not evidence of convergence or improvement.
Frozen-policy versus baseline evaluation is provided but has not been run.
Grasping, contact, visual perception, and sorting are not part of this RL task.
