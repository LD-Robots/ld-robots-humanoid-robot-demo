#!/usr/bin/env python3
"""
Robust PPO training with ROS2 environment.
Trains in small batches with cleanup to avoid blocking issues.
Uses REAL WBC controller so learned parameters actually work!
"""

import os
import sys
import argparse
import subprocess
import time
from pathlib import Path
from datetime import datetime
import torch
import signal

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList
from stable_baselines3.common.monitor import Monitor

from wbc_tuning_env import WbcTuningEnv


def cleanup_processes():
    """Kill all ROS2/MuJoCo processes to prevent blocking."""
    print("\n🧹 Cleaning up processes...")

    processes_to_kill = [
        "ros2",
        "mujoco_simulator",
        "wbc_controller",
        "python3.*wbc_tuning_env",
    ]

    for proc_pattern in processes_to_kill:
        try:
            subprocess.run(
                f"pkill -9 -f '{proc_pattern}'",
                shell=True,
                stderr=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
            )
        except:
            pass

    time.sleep(2)
    print("✓ Cleanup complete")


def train_batch(
    batch_steps: int,
    model_path: Path = None,
    save_dir: Path = None,
    learning_rate: float = 3e-4,
    device: str = "cuda",
    total_timesteps_so_far: int = 0,
):
    """Train for a batch of steps, then save and cleanup."""

    print("\n" + "=" * 60)
    print(f"Training Batch: {batch_steps:,} steps")
    print(f"Total timesteps so far: {total_timesteps_so_far:,}")
    if model_path:
        print(f"Resuming from: {model_path}")
    print("=" * 60)

    # Create environment
    config_path = Path(__file__).parent.parent / "config" / "wbc_controller.yaml"
    env = WbcTuningEnv(config_path=str(config_path), max_steps=1000)
    env = Monitor(env)
    env = DummyVecEnv([lambda: env])

    # Load or create normalization
    vec_normalize_path = save_dir / "vecnormalize_latest.pkl"
    if vec_normalize_path.exists():
        print(f"Loading VecNormalize from: {vec_normalize_path}")
        env = VecNormalize.load(vec_normalize_path, env)
        env.training = True
        env.norm_reward = True
    else:
        env = VecNormalize(
            env,
            norm_obs=True,
            norm_reward=True,
            clip_obs=10.0,
            clip_reward=10.0,
        )

    # Load or create model
    if model_path and model_path.exists():
        print(f"Loading model from: {model_path}")
        model = PPO.load(
            model_path,
            env=env,
            device=device,
            force_reset=False,  # Keep training stats
        )
        # Manually set num_timesteps to continue from where we left off
        model.num_timesteps = total_timesteps_so_far
        model._total_timesteps = total_timesteps_so_far
        print(f"✓ Continuing from timestep {total_timesteps_so_far:,}")
    else:
        print("Creating new model...")
        # Use consistent tensorboard run name
        tb_log_name = "PPO_robust"  # Same name for all batches
        model = PPO(
            "MlpPolicy",
            env,
            learning_rate=learning_rate,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.01,
            vf_coef=0.5,
            max_grad_norm=0.5,
            device=device,
            verbose=1,
            tensorboard_log=str(save_dir / "logs"),
        )
        model.num_timesteps = 0
        model._total_timesteps = 0

    # Set consistent tensorboard log name for continuing runs
    if hasattr(model, '_logger'):
        model._logger = None  # Reset logger to use consistent naming

    # Callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=2048,  # Every rollout
        save_path=str(save_dir / "checkpoints"),
        name_prefix="wbc_ppo",
        save_replay_buffer=False,
        save_vecnormalize=True,
    )

    callback = CallbackList([checkpoint_callback])

    # Train
    print(f"\n🚀 Starting training for {batch_steps:,} steps...")
    start_time = time.time()

    try:
        model.learn(
            total_timesteps=batch_steps,
            callback=callback,
            progress_bar=True,
            reset_num_timesteps=False,  # Continue timestep count
        )
    except KeyboardInterrupt:
        print("\n⚠️  Training interrupted by user")
    except Exception as e:
        print(f"\n❌ Training error: {e}")

    elapsed = time.time() - start_time

    # Save final state
    latest_model = save_dir / "wbc_ppo_latest"
    model.save(latest_model)
    env.save(vec_normalize_path)

    print(f"\n✓ Batch complete in {elapsed/60:.1f} minutes")
    print(f"✓ Model saved: {latest_model}.zip")

    # Cleanup
    env.close()
    del model
    del env

    return latest_model


def train_robust(
    total_timesteps: int = 50_000,
    batch_size: int = 5_000,
    save_dir: str = "models/wbc_ppo_robust",
    learning_rate: float = 3e-4,
    device: str = "cuda",
):
    """Train robustly with automatic batching and cleanup."""

    print("=" * 60)
    print("🛡️  WBC PPO Robust Training - ROS2 Real Environment")
    print("=" * 60)

    # Check GPU
    if device == "cuda" and not torch.cuda.is_available():
        print("⚠️  WARNING: CUDA not available, falling back to CPU")
        device = "cpu"
    elif device == "cuda":
        gpu_name = torch.cuda.get_device_name(0)
        print(f"GPU: {gpu_name}")

    print(f"Total timesteps: {total_timesteps:,}")
    print(f"Batch size: {batch_size:,} steps")
    print(f"Number of batches: {total_timesteps // batch_size}")
    print(f"Environment: REAL WBC + ROS2 + MuJoCo")
    print("=" * 60)

    save_dir = Path(save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    (save_dir / "logs").mkdir(exist_ok=True)
    (save_dir / "checkpoints").mkdir(exist_ok=True)

    # Initial cleanup
    cleanup_processes()

    # Track progress
    current_model = None
    total_trained = 0
    batch_num = 0

    while total_trained < total_timesteps:
        batch_num += 1
        remaining = total_timesteps - total_trained
        batch_steps = min(batch_size, remaining)

        print(f"\n{'='*60}")
        print(f"📦 Batch {batch_num}/{(total_timesteps + batch_size - 1) // batch_size}")
        print(f"Progress: {total_trained:,}/{total_timesteps:,} ({100*total_trained/total_timesteps:.1f}%)")
        print(f"{'='*60}")

        try:
            # Train this batch
            current_model = train_batch(
                batch_steps=batch_steps,
                model_path=current_model,
                save_dir=save_dir,
                learning_rate=learning_rate,
                device=device,
                total_timesteps_so_far=total_trained,
            )

            total_trained += batch_steps

            # Cleanup between batches
            cleanup_processes()

            # Brief pause before next batch
            if total_trained < total_timesteps:
                print("\n⏸️  Pausing 5s before next batch...")
                time.sleep(5)

        except KeyboardInterrupt:
            print("\n\n⚠️  Training stopped by user")
            break
        except Exception as e:
            print(f"\n❌ Batch failed: {e}")
            print("Cleaning up and continuing...")
            cleanup_processes()
            time.sleep(5)

    # Final cleanup
    cleanup_processes()

    print("\n" + "=" * 60)
    print("✅ Training Complete!")
    print("=" * 60)
    print(f"Total trained: {total_trained:,} steps")
    print(f"Final model: {save_dir / 'wbc_ppo_latest.zip'}")
    print(f"\nTo export config:")
    print(f"  python3 export_config_direct.py {save_dir / 'wbc_ppo_latest'}")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description="Robust WBC PPO training with real ROS2 environment"
    )
    parser.add_argument(
        "--timesteps",
        type=int,
        default=50_000,
        help="Total training timesteps (default: 50k)",
    )
    parser.add_argument(
        "--batch",
        type=int,
        default=5_000,
        help="Steps per batch before cleanup (default: 5k)",
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
        default="models/wbc_ppo_robust",
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

    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\n\n⚠️  Received interrupt signal, cleaning up...")
        cleanup_processes()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    train_robust(
        total_timesteps=args.timesteps,
        batch_size=args.batch,
        save_dir=args.save_dir,
        learning_rate=args.lr,
        device=args.device,
    )


if __name__ == "__main__":
    main()
