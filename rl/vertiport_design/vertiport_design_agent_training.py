"""
vertiport_design_agent_training.py
====================================
PPO + CustomGNN training scaffold for VertiportDesignEnv.

Ported from the user's prior validated `vp_design_sb3_training.py`:
- RewardLoggerCallback tracks per-episode reward / final distance / length.
- train_vertiport_design(...) constructs env + model + callback, runs
  model.learn(...), saves the model and a 4-panel training-results plot.
- evaluate_policy(...) runs the trained policy for N episodes and prints
  mean reward / mean final distance.

Run directly:
    python -m rl.vertiport_design.vertiport_design_agent_training
"""

from __future__ import annotations

import os
from datetime import datetime
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
from stable_baselines3 import A2C, PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor

from rl.vertiport_design.policies.custom_gnn import CustomGNN
from rl.vertiport_design.vertiport_design_env import VertiportDesignEnv


class RewardLoggerCallback(BaseCallback):
    """Track per-episode reward, length, and final distance for plotting."""

    def __init__(self, log_freq: int = 10, verbose: int = 0):
        super().__init__(verbose)
        self.log_freq = log_freq
        self.episode_rewards: list = []
        self.episode_distances: list = []
        self.episode_lengths: list = []
        self.timesteps: list = []

        self.current_episode_reward = 0.0
        self.current_episode_length = 0
        self.current_episode_final_distance: Optional[float] = None

    def _on_training_start(self) -> None:
        if self.verbose > 0:
            print("Training started...")

    def _on_step(self) -> bool:
        self.current_episode_reward += self.locals['rewards'][0]
        self.current_episode_length += 1

        info = self.locals['infos'][0]
        if 'current_total_distance' in info:
            self.current_episode_final_distance = info['current_total_distance']

        if self.locals['dones'][0]:
            self.episode_rewards.append(self.current_episode_reward)
            self.episode_lengths.append(self.current_episode_length)
            self.timesteps.append(self.num_timesteps)
            if self.current_episode_final_distance is not None:
                self.episode_distances.append(self.current_episode_final_distance)

            if self.verbose > 0 and len(self.episode_rewards) % self.log_freq == 0:
                print(
                    f"Episode {len(self.episode_rewards)}: "
                    f"Reward={self.current_episode_reward:.2f}, "
                    f"Length={self.current_episode_length}, "
                    f"Final Distance={self.current_episode_final_distance:.2f}"
                )

            self.current_episode_reward = 0.0
            self.current_episode_length = 0
            self.current_episode_final_distance = None

        return True

    def _on_training_end(self) -> None:
        if self.verbose > 0:
            print(f"Training finished. Total episodes: {len(self.episode_rewards)}")


def plot_training_results(callback: RewardLoggerCallback, save_path: Optional[str] = None):
    """4-panel summary: reward, final distance, cumulative reward, episode length."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    episodes = np.arange(1, len(callback.episode_rewards) + 1)

    # Reward
    ax = axes[0, 0]
    ax.plot(episodes, callback.episode_rewards, alpha=0.6, label='Episode Reward')
    if len(callback.episode_rewards) >= 10:
        window = min(50, len(callback.episode_rewards) // 5)
        if window > 1:
            moving_avg = np.convolve(callback.episode_rewards, np.ones(window) / window, mode='valid')
            ax.plot(episodes[window - 1:], moving_avg, 'r-', linewidth=2, label=f'Moving Avg (w={window})')
    ax.set_xlabel('Episode'); ax.set_ylabel('Total Reward'); ax.set_title('Training Reward')
    ax.legend(); ax.grid(True, alpha=0.3)

    # Final distance
    ax = axes[0, 1]
    if callback.episode_distances:
        ax.plot(episodes[:len(callback.episode_distances)], callback.episode_distances, alpha=0.6, label='Final Distance')
        if len(callback.episode_distances) >= 10:
            window = min(50, len(callback.episode_distances) // 5)
            if window > 1:
                moving_avg = np.convolve(callback.episode_distances, np.ones(window) / window, mode='valid')
                ax.plot(
                    episodes[window - 1:len(callback.episode_distances)], moving_avg,
                    'r-', linewidth=2, label=f'Moving Avg (w={window})'
                )
        ax.set_xlabel('Episode'); ax.set_ylabel('Final Total Distance')
        ax.set_title('Final Distance (Lower is Better)')
        ax.legend(); ax.grid(True, alpha=0.3)

    # Cumulative reward
    ax = axes[1, 0]
    ax.plot(episodes, np.cumsum(callback.episode_rewards), 'g-', linewidth=2)
    ax.set_xlabel('Episode'); ax.set_ylabel('Cumulative Reward')
    ax.set_title('Cumulative Reward'); ax.grid(True, alpha=0.3)

    # Episode length
    ax = axes[1, 1]
    ax.plot(episodes, callback.episode_lengths, alpha=0.6)
    ax.set_xlabel('Episode'); ax.set_ylabel('Length (steps)')
    ax.set_title('Episode Length'); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Training plots saved to: {save_path}")
    return fig


def train_vertiport_design(
    config_path: str,
    total_timesteps: int = 50_000,
    simulator_step: int = 2,
    episode_length: int = 160,
    region_mode: str = 'synthetic',
    region_kwargs: Optional[dict] = None,
    algorithm: str = 'PPO',
    features_dim: int = 16,
    final_dim: int = 8,
    num_gnn_layers: int = 1,
    learning_rate: float = 3e-4,
    n_steps: int = 16,
    batch_size: int = 4,
    save_model: bool = True,
    model_save_path: Optional[str] = None,
    plot_save_path: Optional[str] = None,
    verbose: int = 1,
):
    """Train PPO (or A2C) + CustomGNN on VertiportDesignEnv. Returns (model, callback)."""
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    print('Creating environment...')
    env = VertiportDesignEnv(
        config_path=config_path,
        simulator_step=simulator_step,
        total_episode_timestep=episode_length,
        region_mode=region_mode,
        region_kwargs=region_kwargs,
    )
    log_dir = f'./logs/vertiport_design_{timestamp}/'
    os.makedirs(log_dir, exist_ok=True)
    env = Monitor(env, log_dir)

    policy_kwargs = dict(
        features_extractor_class=CustomGNN,
        features_extractor_kwargs=dict(
            features_dim=features_dim,
            final_dim=final_dim,
            num_layers=num_gnn_layers,
        ),
    )

    print(f'Creating {algorithm} model with CustomGNN feature extractor...')
    if algorithm.upper() == 'PPO':
        model = PPO(
            policy='MultiInputPolicy', env=env, policy_kwargs=policy_kwargs,
            learning_rate=learning_rate, n_steps=n_steps, batch_size=batch_size,
            verbose=verbose, tensorboard_log=f'./tensorboard_logs/vertiport_{timestamp}/',
        )
    elif algorithm.upper() == 'A2C':
        model = A2C(
            policy='MultiInputPolicy', env=env, policy_kwargs=policy_kwargs,
            learning_rate=learning_rate, n_steps=n_steps,
            verbose=verbose, tensorboard_log=f'./tensorboard_logs/vertiport_{timestamp}/',
        )
    else:
        raise ValueError(f"Unknown algorithm: {algorithm}. Use 'PPO' or 'A2C'.")

    reward_callback = RewardLoggerCallback(log_freq=10, verbose=verbose)

    print(f'Starting training for {total_timesteps} timesteps...')
    model.learn(total_timesteps=total_timesteps, callback=reward_callback, progress_bar=True)

    if save_model:
        if model_save_path is None:
            model_save_path = f'./models/vertiport_design_{algorithm}_{timestamp}'
        os.makedirs(os.path.dirname(model_save_path), exist_ok=True)
        model.save(model_save_path)
        print(f'Model saved to: {model_save_path}')

    if plot_save_path is None:
        plot_save_path = f'./plots/training_results_{timestamp}.png'
    os.makedirs(os.path.dirname(plot_save_path), exist_ok=True)
    plot_training_results(reward_callback, save_path=plot_save_path)

    print('\n' + '=' * 50)
    print('TRAINING SUMMARY')
    print('=' * 50)
    print(f'Total episodes: {len(reward_callback.episode_rewards)}')
    if reward_callback.episode_rewards:
        print(f'Average episode reward: {np.mean(reward_callback.episode_rewards):.2f}')
    if reward_callback.episode_distances:
        print(f'Initial avg distance: {np.mean(reward_callback.episode_distances[:10]):.2f}')
        print(f'Final avg distance: {np.mean(reward_callback.episode_distances[-10:]):.2f}')
        improvement = np.mean(reward_callback.episode_distances[:10]) - np.mean(reward_callback.episode_distances[-10:])
        print(f'Distance improvement: {improvement:.2f}')

    return model, reward_callback


def evaluate_policy(model, env, n_episodes: int = 10, verbose: int = 1):
    """Run trained policy for n_episodes; return summary stats."""
    episode_rewards: list = []
    episode_distances: list = []

    for ep in range(n_episodes):
        obs, info = env.reset()
        done = False
        total_reward = 0.0
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            done = terminated or truncated
        episode_rewards.append(total_reward)
        if 'current_total_distance' in info:
            episode_distances.append(info['current_total_distance'])
        if verbose > 0:
            print(
                f"Eval Episode {ep + 1}: Reward={total_reward:.2f}, "
                f"Final Distance={info.get('current_total_distance', 'N/A')}"
            )

    stats = {
        'mean_reward': float(np.mean(episode_rewards)),
        'std_reward': float(np.std(episode_rewards)),
        'mean_distance': float(np.mean(episode_distances)) if episode_distances else None,
        'std_distance': float(np.std(episode_distances)) if episode_distances else None,
    }
    print('\n' + '=' * 50)
    print('EVALUATION RESULTS')
    print('=' * 50)
    print(f"Mean reward: {stats['mean_reward']:.2f} ± {stats['std_reward']:.2f}")
    if stats['mean_distance']:
        print(f"Mean final distance: {stats['mean_distance']:.2f} ± {stats['std_distance']:.2f}")
    return stats


if __name__ == '__main__':
    CONFIG = {
        'config_path': 'sample_config.yaml',
        'total_timesteps': 500 * 160,
        'simulator_step': 2,
        'episode_length': 160,
        'region_mode': 'synthetic',
        'region_kwargs': {'num_regions': 4, 'num_vertiports_per_region': 5},
        'algorithm': 'PPO',
        'features_dim': 16,
        'final_dim': 8,
        'num_gnn_layers': 1,
        'learning_rate': 3e-4,
        'n_steps': 16,
        'batch_size': 4,
        'verbose': 1,
    }

    model, callback = train_vertiport_design(**CONFIG)

    eval_env = VertiportDesignEnv(
        config_path=CONFIG['config_path'],
        simulator_step=CONFIG['simulator_step'],
        total_episode_timestep=CONFIG['episode_length'],
        region_mode=CONFIG['region_mode'],
        region_kwargs=CONFIG['region_kwargs'],
    )
    evaluate_policy(model, eval_env, n_episodes=1, verbose=1)
