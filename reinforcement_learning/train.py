#!/usr/bin/env python3
"""Train one complete episode, update SAC, checkpoint, repeat."""
import argparse
import json
from pathlib import Path
import fcntl
import rclpy
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.monitor import Monitor
from reach_env import ReachEnv,REVISION,TASK_REVISION


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--episodes', type=int, default=20)
    p.add_argument('--max-steps', type=int, default=30)
    p.add_argument('--seed', type=int, default=7)
    p.add_argument('--output', type=Path, default=Path(__file__).parent / 'runs' / ('reach_'+REVISION+'_'+TASK_REVISION))
    p.add_argument('--resume', action='store_true')
    args = p.parse_args()
    if args.episodes < 1 or args.max_steps < 1:
        p.error('episodes and max-steps must be positive')
    args.output.mkdir(parents=True, exist_ok=True)
    # One training client at a time, even when output directories differ.
    lock = (Path(__file__).parent / '.training.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    pointer = args.output / 'latest.json'
    if pointer.exists() and not args.resume:
        p.error('Output already has progress; use --resume or a different --output')
    if args.resume and not pointer.exists():
        p.error('No saved progress to resume')
    torch.set_num_threads(2)
    rclpy.init()
    env = None
    try:
        env = Monitor(ReachEnv(args.max_steps), str(args.output / 'episodes'),
                      info_keywords=('is_success', 'distance_m'), override_existing=not args.resume)
        completed = 0
        if args.resume:
            state = json.loads(pointer.read_text())
            if (state.get('geometry_revision')!=REVISION or state.get('task_revision')!=TASK_REVISION):raise ValueError('Checkpoint uses obsolete robot geometry; start a new run')
            model = SAC.load(args.output / state['model'], env=env, device='cpu')
            model.load_replay_buffer(args.output / state['replay'])
            completed = state['episodes']
        else:
            model = SAC('MlpPolicy', env, learning_starts=0, buffer_size=50000,
                        batch_size=32, train_freq=(1, 'episode'), gradient_steps=-1,
                        policy_kwargs={'net_arch': [64, 64]}, device='cpu',
                        seed=args.seed, verbose=1)
        for episode in range(completed + 1, completed + args.episodes + 1):
            # Episode-based collection finishes an entire attempt even with total_timesteps=1.
            model.learn(total_timesteps=1, reset_num_timesteps=False)
            stem = f'episode_{episode:05d}'
            model.save(args.output / stem)
            model.save_replay_buffer(args.output / f'{stem}.pkl')
            state = {'episodes': episode, 'timesteps': model.num_timesteps,
                     'updates': model._n_updates, 'model': f'{stem}.zip',
                     'replay': f'{stem}.pkl', 'task': 'gazebo_tool_reaching_v1',
                     'max_steps': args.max_steps, 'geometry_revision':REVISION,'task_revision':TASK_REVISION}
            temp = pointer.with_suffix('.tmp')
            temp.write_text(json.dumps(state, indent=2) + '\n')
            temp.replace(pointer)
            # Keep latest and preceding checkpoints; bounded disk use.
            for old in args.output.glob('episode_*.*'):
                if int(old.stem.split('_')[1]) < episode - 1:
                    old.unlink()
            print(f'SAVED episode={episode} steps={model.num_timesteps} updates={model._n_updates}', flush=True)
    finally:
        if env is not None:
            env.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
