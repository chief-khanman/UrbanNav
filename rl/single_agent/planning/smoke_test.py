"""Generic smoke test for any registered single-agent RL variant
(rl/single_agent/variant_registry.py).

Runs three cheap checks — not full training:
  1. env reset/step sanity check (no learning)
  2. a capped PPO.learn() call
  3. a short deterministic evaluation rollout

The point is to catch wiring bugs (registry/obs-space/reward-dispatch
mismatches), not to produce a usable policy. Reused unmodified across
plan_01/02/03 in rl/single_agent/planning/ as new variants come online.

Usage:
    python rl/single_agent/planning/smoke_test.py --variant v1_goal_only
"""
from __future__ import annotations

import argparse
import os
import tempfile

import numpy as np
import yaml
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env.base_vec_env import VecEnv

from rl.single_agent.single_agent_gym_env import UAMSimEnv
from rl.single_agent.single_agent_training import (
    build_env, build_policy_kwargs, wrap_env_for_arch,
)
from rl.single_agent.variant_registry import get_variant

PROJECT_ROOT = os.path.join(os.path.dirname(__file__), "..", "..", "..")
SAMPLE_CONFIG_PATH = os.path.join(PROJECT_ROOT, "sample_config.yaml")


def _write_smoke_config(tmp_dir: str) -> str:
    """1 STANDARD + 1 SINGLE_AGENT_LEARNING, logging/rendering disabled, short
    episode — mirrors rl/conftest.py's single_agent_config_path fixture so the
    smoke test exercises the same setup pytest already covers."""
    with open(SAMPLE_CONFIG_PATH) as f:
        cfg = yaml.safe_load(f)
    cfg["logging"]["enabled"] = False
    cfg["rendering"]["enabled"] = False
    cfg["simulator"]["total_timestep"] = 200
    cfg["fleet_composition"] = [
        {
            "type_name": "STANDARD", "count": 1, "dynamics": "PointMass",
            "controller": "PIDPointMassController", "sensor": "PartialSensor",
            "planner": "PointMass-PID",
        },
        {
            "type_name": "SINGLE_AGENT_LEARNING", "count": 1, "dynamics": "PointMass",
            "controller": "RL", "sensor": "PartialSensor", "planner": "PointMass-PID",
            "mode": "TRAIN", "policy_id": "smoke_test",
        },
    ]
    path = os.path.join(tmp_dir, "smoke_config.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(cfg, f)
    return path


def sanity_check(env) -> None:
    """Works against either a raw UAMSimEnv or a VecEnv (GNN_RNN's
    VecFrameStack-wrapped env) — the two APIs differ in reset()/step()
    arity, so branch once on type rather than threading a flag through."""
    is_vec = isinstance(env, VecEnv)
    if is_vec:
        obs = env.reset()
    else:
        obs, _info = env.reset()
    assert obs.shape[-len(env.observation_space.shape):] == env.observation_space.shape, (
        "obs shape mismatch on reset"
    )
    for _ in range(5):
        action = np.array([env.action_space.sample()]) if is_vec else env.action_space.sample()
        if is_vec:
            obs, reward, dones, infos = env.step(action)
            assert np.isfinite(reward).all(), f"non-finite reward: {reward}"
            if dones[0]:
                env.reset()
        else:
            obs, reward, terminated, truncated, info = env.step(action)
            assert np.isfinite(reward), f"non-finite reward: {reward}"
            if terminated or truncated:
                env.reset()
    print("[smoke_test] env reset/step sanity check passed")


def capped_learn(env, total_timesteps: int, seed: int, policy_kwargs: dict) -> PPO:
    model = PPO("MlpPolicy", env, policy_kwargs=policy_kwargs, seed=seed, verbose=0)
    model.learn(total_timesteps=total_timesteps)
    print(f"[smoke_test] capped learn() of {total_timesteps} steps completed")
    return model


def eval_rollout(model: PPO, env, n_steps: int) -> None:
    is_vec = isinstance(env, VecEnv)
    obs = env.reset() if is_vec else env.reset()[0]
    for _ in range(n_steps):
        action, _ = model.predict(obs, deterministic=True)
        if is_vec:
            obs, _reward, dones, infos = env.step(action)
            if dones[0]:
                print(f"[smoke_test] episode ended early: {infos[0]}")
        else:
            obs, _reward, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                print(f"[smoke_test] episode ended early: {info}")
                obs, _info = env.reset()
    print("[smoke_test] eval rollout completed")


def run(args: argparse.Namespace) -> None:
    variant = get_variant(args.variant)
    policy_kwargs = build_policy_kwargs(variant, args)
    with tempfile.TemporaryDirectory() as tmp_dir:
        config_path = _write_smoke_config(tmp_dir)
        env = wrap_env_for_arch(
            build_env(config_path, args.variant, args.n_intruder),
            args.policy_arch, args.temporal_window,
        )
        sanity_check(env)
        model = capped_learn(env, args.total_timesteps, args.seed, policy_kwargs)
        eval_rollout(model, env, n_steps=20)
    print(f"[smoke_test] variant '{args.variant}' policy_arch='{args.policy_arch}' PASSED")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", required=True)
    parser.add_argument("--total-timesteps", type=int, default=500)
    parser.add_argument("--n-intruder", type=int, default=2)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--policy-arch", choices=("MLP", "RNN", "GNN", "GNN_RNN"), default="MLP")
    parser.add_argument("--rnn-type", choices=("gru", "lstm"), default="gru")
    parser.add_argument("--temporal-window", type=int, default=4)
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
