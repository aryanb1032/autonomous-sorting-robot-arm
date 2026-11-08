#!/usr/bin/env python3
"""Evaluate a frozen policy or random baseline on reproducible targets; no updates."""
import argparse
import json
from pathlib import Path
import fcntl
import rclpy
import numpy as np
from stable_baselines3 import SAC
from reach_env import ReachEnv,REVISION,TASK_REVISION

p = argparse.ArgumentParser()
p.add_argument('--run', type=Path, default=Path(__file__).parent / 'runs' / ('reach_'+REVISION+'_'+TASK_REVISION))
p.add_argument('--random-baseline', action='store_true')
p.add_argument('--episodes', type=int, default=10)
p.add_argument('--max-steps', type=int, default=30)
p.add_argument('--seed', type=int, default=1000)
a = p.parse_args()
if a.episodes < 1 or a.max_steps < 1:
    p.error('episodes and max-steps must be positive')
lock = (Path(__file__).parent / '.training.lock').open('w')
fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
model = None
if not a.random_baseline:
    state = json.loads((a.run / 'latest.json').read_text())
    if (state.get('geometry_revision')!=REVISION or state.get('task_revision')!=TASK_REVISION):raise ValueError('Checkpoint uses obsolete robot geometry; retrain first')
    model = SAC.load(a.run / state['model'], device='cpu')
rclpy.init()
env = None
try:
    env = ReachEnv(a.max_steps)
    rows = []
    for i in range(a.episodes):
        obs, info = env.reset(seed=a.seed + i)
        rng = np.random.default_rng(a.seed + i)
        reward_sum = 0.
        while True:
            action = rng.uniform(-1, 1, 3) if model is None else model.predict(obs, deterministic=True)[0]
            obs, reward, terminated, truncated, info = env.step(action)
            reward_sum += reward
            if terminated or truncated:
                break
        row = dict(seed=a.seed + i, reward=reward_sum, **info)
        rows.append(row)
        print(json.dumps(row), flush=True)
    report = {'geometry_revision':REVISION,'task_revision':TASK_REVISION,'policy': 'random' if model is None else str(a.run), 'episodes': rows,
              'success_rate': float(np.mean([r['is_success'] for r in rows])),
              'mean_distance_m': float(np.mean([r['distance_m'] for r in rows]))}
    a.run.mkdir(parents=True, exist_ok=True)
    name = 'evaluation_random.json' if model is None else 'evaluation_policy.json'
    (a.run / name).write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'episodes'}))
finally:
    if env is not None:
        env.close()
    rclpy.shutdown()
