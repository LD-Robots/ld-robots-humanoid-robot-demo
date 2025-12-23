#!/usr/bin/env python3
"""
PPO training script for WBC parameter tuning.
Optimized for RTX 5070 Ti with parallel environments.
"""

import os
import argparse
from pathlib import Path
from datetime import datetime
import torch

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize, DummyVecEnv
from stable_baselines3.common.callbacks import (
    CheckpointCallback,
    EvalCallback,
    CallbackList,
)
from stable_baselines3.common.monitor import Monitor

from wbc_tuning_env import WbcTuningEnv


def make_env(config_path: str, rank: int):
    """Create a single environment with unique config per process."""
    def _init():
        # Each parallel env needs its own config file
        config_dir = Path(config_path).parent
        env_config = config_dir / f"wbc_controller_env{rank}.yaml"

        # Copy base config for this env
        import shutil
        shutil.copy(config_path, env_config)

        env = WbcTuningEnv(config_path=str(env_config), max_steps=1000)
        env = Monitor(env)
        return env
    return _init


def train_ppo(
    total_timesteps: int = 1_000_000,
    n_envs: int = 4,
    save_dir: str = "models/wbc_ppo",
    eval_freq: int = 10_000,
    learning_rate: float = 3e-4,
    device: str = "cuda",
):
    """Train PPO agent with parallel environments."""

    print("=" * 60)
    print("WBC PPO Training - RTX 5070 Ti Optimized")
    print("=" * 60)

    # Check GPU availability
    if device == "cuda" and not torch.cuda.is_available():
        print("WARNING: CUDA not available, falling back to CPU")
        device = "cpu"
    elif device == "cuda":
        gpu_name = torch.cuda.get_device_name(0)
        gpu_mem = torch.cuda.get_device_properties(0).total_memory / 1e9
        print(f"GPU: {gpu_name}")
        print(f"VRAM: {gpu_mem:.1f} GB")
        print(f"CUDA Version: {torch.version.cuda}")

    print(f"Parallel Environments: {n_envs}")
    print(f"Total Timesteps: {total_timesteps:,}")
    print(f"Learning Rate: {learning_rate}")
    print("=" * 60)

    # Create directories
    save_dir = Path(save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    log_dir = save_dir / "logs"
    log_dir.mkdir(exist_ok=True)

    # Base config path
    pkg_path = Path(__file__).parent.parent
    config_path = pkg_path / "config" / "wbc_controller.yaml"

    # Create parallel environments
    print("\nCreating parallel environments...")
    env_fns = [make_env(str(config_path), i) for i in range(n_envs)]
    vec_env = SubprocVecEnv(env_fns)

    # Normalize observations and rewards for faster training
    vec_env = VecNormalize(
        vec_env,
        norm_obs=True,
        norm_reward=True,
        clip_obs=10.0,
        clip_reward=10.0,
    )

    # Create evaluation environment (must be wrapped same as training env)
    eval_env_fn = make_env(str(config_path), rank=999)
    eval_env = DummyVecEnv([eval_env_fn])

    # Wrap with VecNormalize like training env (but don't train normalization)
    eval_env = VecNormalize(
        eval_env,
        norm_obs=True,
        norm_reward=False,  # Don't normalize reward for eval
        clip_obs=10.0,
        training=False,  # Don't update normalization stats during eval
    )

    # PPO hyperparameters optimized for continuous control
    model = PPO(
        "MlpPolicy",
        vec_env,
        learning_rate=learning_rate,
        n_steps=2048 // n_envs,  # Adjust for parallel envs
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,  # Encourage exploration
        vf_coef=0.5,
        max_grad_norm=0.5,
        device=device,
        verbose=1,
        tensorboard_log=str(log_dir),
    )

    print(f"\nModel created: {model.policy}")
    print(f"Device: {model.device}")

    # Callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=10_000 // n_envs,
        save_path=str(save_dir / "checkpoints"),
        name_prefix="wbc_ppo",
        save_replay_buffer=False,
        save_vecnormalize=True,
    )

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(save_dir / "best_model"),
        log_path=str(log_dir / "eval"),
        eval_freq=eval_freq // n_envs,
        n_eval_episodes=5,
        deterministic=True,
        render=False,
        verbose=1,
    )

    callback = CallbackList([checkpoint_callback, eval_callback])

    # Training
    print("\n" + "=" * 60)
    print("Starting Training...")
    print("=" * 60)
    print(f"Tensorboard: tensorboard --logdir {log_dir}")
    print("=" * 60 + "\n")

    start_time = datetime.now()

    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=callback,
            progress_bar=True,
        )
    except KeyboardInterrupt:
        print("\nTraining interrupted by user")

    training_time = (datetime.now() - start_time).total_seconds() / 3600

    # Save final model
    final_model_path = save_dir / "wbc_ppo_final"
    model.save(final_model_path)
    vec_env.save(save_dir / "vecnormalize_final.pkl")

    print("\n" + "=" * 60)
    print("Training Complete!")
    print("=" * 60)
    print(f"Training time: {training_time:.2f} hours")
    print(f"Final model saved: {final_model_path}")
    print(f"Best model saved: {save_dir / 'best_model' / 'best_model.zip'}")
    print("=" * 60)

    # Cleanup
    vec_env.close()
    eval_env.close()

    return model


def main():
    parser = argparse.ArgumentParser(description="Train PPO for WBC parameter tuning")
    parser.add_argument(
        "--timesteps",
        type=int,
        default=500_000,
        help="Total training timesteps (default: 500k, ~4-6 hours on RTX 5070 Ti)",
    )
    parser.add_argument(
        "--envs",
        type=int,
        default=4,
        help="Number of parallel environments (default: 4)",
    )
    parser.add_argument(
        "--lr",
        type=float,
        default=3e-4,
        help="Learning rate (default: 3e-4)",
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        default="models/wbc_ppo",
        help="Directory to save models",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda",
        choices=["cuda", "cpu"],
        help="Device to use for training (default: cuda)",
    )

    args = parser.parse_args()

    train_ppo(
        total_timesteps=args.timesteps,
        n_envs=args.envs,
        save_dir=args.save_dir,
        learning_rate=args.lr,
        device=args.device,
    )


if __name__ == "__main__":
    main()
