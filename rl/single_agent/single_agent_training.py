"""CLI entry point for training a single-agent RL policy against one of the
reward/obs variants registered in rl/single_agent/variant_registry.py.

Usage:
    python -m rl.single_agent.single_agent_training --variant v1_goal_only \
        --total-timesteps 100000
"""
from __future__ import annotations

import argparse
import os

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.vec_env import DummyVecEnv, VecFrameStack

from urbannav.uam_simulator import UAMSimulator

from rl.single_agent.single_agent_gym_env import UAMSimEnv
from rl.single_agent.variant_registry import VariantConfig, get_variant
from rl.single_agent.policies.rnn_extractor import IntruderRNNExtractor
from rl.single_agent.policies.gnn_extractor import IntruderGNNExtractor
from rl.single_agent.policies.gnn_rnn_extractor import IntruderGNNRNNExtractor

CHECKPOINT_ROOT = os.path.join(os.path.dirname(__file__), "checkpoints")

# obs_types with an intruder block — the only ones a non-MLP policy_arch can target.
INTRUDER_OBS_TYPES = ('AGENT-N-INTRUDER', 'AGENT-N-INTRUDER-RA')
POLICY_ARCHES = ('MLP', 'RNN', 'GNN', 'GNN_RNN')


def build_env(config_path: str, variant_name: str, n_intruder: int) -> UAMSimEnv:
    """Construct a fresh UAMSimEnv wired to the named registry variant."""
    variant = get_variant(variant_name)
    sim = UAMSimulator(config_path=config_path)
    return UAMSimEnv(
        simulator=sim,
        obs_type=variant.obs_type,
        n_intruder=n_intruder,
        reward_type=variant.reward_type,
    )


def build_policy_kwargs(variant: VariantConfig, args: argparse.Namespace) -> dict:
    """
    Resolve --policy-arch into SB3 policy_kwargs (features_extractor_class +
    features_extractor_kwargs), or {} for the default MLP path.

    Raises ValueError if a non-MLP architecture is requested against a
    variant with no intruder block — RNN/GNN/GNN_RNN exist to model other
    agents, so they're meaningless for obs_types like AGENT/AGENT-RA.
    """
    if args.policy_arch == 'MLP':
        return {}

    if variant.obs_type not in INTRUDER_OBS_TYPES:
        raise ValueError(
            f"--policy-arch {args.policy_arch} requires an obs_type with an intruder "
            f"block ({INTRUDER_OBS_TYPES}), but variant '{variant.name}' uses "
            f"obs_type='{variant.obs_type}'."
        )
    has_ra = variant.obs_type.endswith('-RA')

    if args.policy_arch == 'RNN':
        return dict(
            features_extractor_class=IntruderRNNExtractor,
            features_extractor_kwargs=dict(
                n_intruder=args.n_intruder, has_ra=has_ra, rnn_type=args.rnn_type,
            ),
        )
    if args.policy_arch == 'GNN':
        return dict(
            features_extractor_class=IntruderGNNExtractor,
            features_extractor_kwargs=dict(n_intruder=args.n_intruder, has_ra=has_ra),
        )
    if args.policy_arch == 'GNN_RNN':
        return dict(
            features_extractor_class=IntruderGNNRNNExtractor,
            features_extractor_kwargs=dict(
                n_intruder=args.n_intruder, has_ra=has_ra, rnn_type=args.rnn_type,
                temporal_window=args.temporal_window,
            ),
        )
    raise ValueError(f"Unknown policy_arch '{args.policy_arch}'. Valid options: {POLICY_ARCHES}")


def wrap_env_for_arch(env: UAMSimEnv, policy_arch: str, temporal_window: int):
    """
    GNN_RNN needs a fixed-length frame-stack window of past observations —
    wrap in VecFrameStack via an explicit single-env DummyVecEnv. Every other
    architecture uses the raw env directly (PPO wraps it in a DummyVecEnv
    internally with no frame stacking).
    """
    if policy_arch != 'GNN_RNN':
        return env
    return VecFrameStack(DummyVecEnv([lambda: env]), n_stack=temporal_window)


def train(args: argparse.Namespace) -> PPO:
    """Train one PPO policy for args.variant and save checkpoints under
    CHECKPOINT_ROOT/<variant>/."""
    variant = get_variant(args.variant)
    policy_kwargs = build_policy_kwargs(variant, args)

    train_env = wrap_env_for_arch(
        build_env(args.config_path, args.variant, args.n_intruder),
        args.policy_arch, args.temporal_window,
    )
    eval_env = wrap_env_for_arch(
        build_env(args.config_path, args.variant, args.n_intruder),
        args.policy_arch, args.temporal_window,
    )

    checkpoint_dir = os.path.join(CHECKPOINT_ROOT, args.variant)
    os.makedirs(checkpoint_dir, exist_ok=True)

    model = PPO(
        "MlpPolicy", train_env, policy_kwargs=policy_kwargs, seed=args.seed, verbose=1,
    )

    callbacks = [
        CheckpointCallback(
            save_freq=max(args.total_timesteps // 10, 1),
            save_path=checkpoint_dir,
            name_prefix=args.variant,
        ),
        EvalCallback(
            eval_env,
            best_model_save_path=checkpoint_dir,
            log_path=checkpoint_dir,
            eval_freq=max(args.total_timesteps // 5, 1),
            n_eval_episodes=3,
        ),
    ]

    model.learn(total_timesteps=args.total_timesteps, callback=callbacks)
    model.save(os.path.join(checkpoint_dir, f"{args.variant}_final"))
    return model


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--variant", required=True, help="Name in variant_registry.VARIANT_CONFIG_MAP"
    )
    parser.add_argument("--config-path", default="sample_config.yaml")
    parser.add_argument("--total-timesteps", type=int, default=100_000)
    parser.add_argument("--n-intruder", type=int, default=3)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "--policy-arch", choices=POLICY_ARCHES, default="MLP",
        help="Other-agent feature extractor architecture. RNN/GNN/GNN_RNN require an "
             "obs_type with an intruder block.",
    )
    parser.add_argument(
        "--rnn-type", choices=("gru", "lstm"), default="gru",
        help="Recurrent cell used by --policy-arch RNN or GNN_RNN.",
    )
    parser.add_argument(
        "--temporal-window", type=int, default=4,
        help="Frame-stack window size T, used only by --policy-arch GNN_RNN.",
    )
    return parser.parse_args()


if __name__ == "__main__":
    train(parse_args())
