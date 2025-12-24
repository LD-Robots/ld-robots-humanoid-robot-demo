#!/usr/bin/env python3
"""
Parallel PPO training for WBC gains with multiple robots in same simulation.
Runs 32 robots simultaneously, each with unique ROS2 namespaces.
Goal: Maximize walking distance without falling.
"""

import os
import sys
import argparse
import subprocess
import time
from pathlib import Path
import torch
import signal
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize
from stable_baselines3.common.callbacks import CallbackList, BaseCallback
from stable_baselines3.common.monitor import Monitor

from wbc_tuning_env import WbcGainsTuningEnv


class ParallelEpisodeInfoCallback(BaseCallback):
    """Log episode info from parallel environments."""

    def __init__(self, verbose: int = 0):
        super().__init__(verbose)
        self._reset_counters()

    def _reset_counters(self):
        self.episode_count = 0
        self.fell_count = 0
        self.timeout_count = 0
        self.distance_sum = 0.0
        self.reward_sum = 0.0
        self.step_count = 0
        self.max_distance = 0.0
        self.warmup_retries_sum = 0

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        dones = self.locals.get("dones", [])
        rewards = self.locals.get("rewards", [])

        if rewards is not None:
            self.reward_sum += float(sum(rewards))
            self.step_count += len(rewards)

        for info, done in zip(infos, dones):
            # Track warmup retries (logged at episode start/reset)
            if "warmup_retries" in info:
                self.warmup_retries_sum += info["warmup_retries"]

            if not done:
                continue

            self.episode_count += 1
            if info.get("fell"):
                self.fell_count += 1
            if info.get("reason") == "timeout":
                self.timeout_count += 1

            if "distance" in info:
                distance = float(info["distance"])
                self.distance_sum += distance
                if distance > self.max_distance:
                    self.max_distance = distance
                    if self.verbose:
                        print(f"🏆 New max distance: {distance:.3f}m")

        return True

    def _on_rollout_end(self) -> None:
        if self.step_count > 0:
            self.logger.record("custom/reward_per_step", self.reward_sum / self.step_count)
        if self.episode_count > 0:
            self.logger.record("custom/fell_rate", self.fell_count / self.episode_count)
            self.logger.record("custom/timeout_rate", self.timeout_count / self.episode_count)
            self.logger.record("custom/mean_distance", self.distance_sum / self.episode_count)
            self.logger.record("custom/max_distance", self.max_distance)
            self.logger.record("custom/mean_warmup_retries", self.warmup_retries_sum / self.episode_count)
        self._reset_counters()


class BestDistanceCallback(BaseCallback):
    """Save best model based on maximum distance traveled across all parallel envs."""

    def __init__(self, save_dir: Path, verbose: int = 0):
        super().__init__(verbose)
        self.save_dir = save_dir
        self.best_distance = 0.0
        self.best_survived = False
        self.best_timestep = 0

        # Try to load previous best
        self.stats_file = save_dir / "best_model_stats.txt"
        self._load_best_stats()

    def _load_best_stats(self):
        """Load previous best model statistics."""
        if self.stats_file.exists():
            try:
                with open(self.stats_file, "r") as f:
                    lines = f.readlines()
                    for line in lines:
                        if "distance=" in line:
                            self.best_distance = float(line.split("distance=")[1].split(",")[0])
                        if "survived=" in line:
                            self.best_survived = "True" in line
                        if "timestep=" in line:
                            self.best_timestep = int(line.split("timestep=")[1].split(",")[0])
                if self.verbose:
                    print(f"📊 Loaded previous best: distance={self.best_distance:.3f}m, "
                          f"survived={self.best_survived}, timestep={self.best_timestep}")
            except Exception as e:
                if self.verbose:
                    print(f"⚠️  Could not load previous best stats: {e}")

    def _save_best_stats(self, distance: float, survived: bool, timestep: int):
        """Save best model statistics to file."""
        with open(self.stats_file, "w") as f:
            f.write(f"distance={distance:.4f}, survived={survived}, timestep={timestep}\n")
            f.write(f"timestamp={time.strftime('%Y-%m-%d %H:%M:%S')}\n")

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        dones = self.locals.get("dones", [])

        for info, done in zip(infos, dones):
            if not done:
                continue

            distance = float(info.get("distance", 0.0))
            fell = info.get("fell", False)
            survived = not fell

            # Save if: first to survive OR (survived and further than previous best) OR (both fell but further)
            should_update = False
            reason = ""

            if survived and not self.best_survived:
                should_update = True
                reason = "first survival"
            elif survived and self.best_survived and distance > self.best_distance:
                should_update = True
                reason = f"new distance record: {distance:.3f}m > {self.best_distance:.3f}m"
            elif not survived and not self.best_survived and distance > self.best_distance:
                should_update = True
                reason = f"furthest among fallen: {distance:.3f}m"

            if should_update:
                self.best_distance = distance
                self.best_survived = survived
                self.best_timestep = self.model.num_timesteps

                best_path = self.save_dir / "best_model"
                self.model.save(best_path)

                # Save statistics
                self._save_best_stats(distance, survived, self.best_timestep)

                # Save VecNormalize for best model
                if hasattr(self.model.get_env(), "save"):
                    vecnorm_path = self.save_dir / "best_model_vecnormalize.pkl"
                    self.model.get_env().save(vecnorm_path)

                if self.verbose:
                    status = "✅ survived" if survived else "❌ fell"
                    print(f"\n🏆 Best model updated! ({reason})")
                    print(f"   Status: {status}")
                    print(f"   Distance: {distance:.3f}m")
                    print(f"   At global timestep: {self.best_timestep:,}\n")

        return True


def make_env(rank: int, config_path: str, max_steps: int = 1000):
    """
    Create a single environment instance with unique ROS2 namespace.

    Args:
        rank: Environment index (0-31)
        config_path: Path to base WBC config
        max_steps: Maximum steps per episode
    """
    def _init():
        # Create unique namespace for this robot
        namespace = f"robot_{rank}"

        # Create environment with unique namespace
        env = WbcGainsTuningEnv(
            config_path=config_path,
            max_steps=max_steps,
            namespace=namespace,  # This will be used for ROS2 topics
        )

        # Wrap with Monitor for episode statistics
        env = Monitor(env)
        return env

    return _init


def train_parallel(
    total_timesteps: int = 100_000,
    num_envs: int = 32,
    save_dir: str = "models/wbc_ppo_parallel",
    learning_rate: float = 3e-4,
    device: str = "cuda",
    resume: str = None,
):
    """
    Train with multiple parallel environments (32 robots simultaneously).

    Args:
        total_timesteps: Total training timesteps
        num_envs: Number of parallel environments (default: 32)
        save_dir: Directory to save models
        learning_rate: Learning rate for PPO
        device: Device to use (cuda/cpu)
        resume: Path to model to resume from
    """

    print("=" * 70)
    print("🚀 WBC PPO Parallel Training - 32 Robots")
    print("=" * 70)

    # Check GPU
    if device == "cuda" and not torch.cuda.is_available():
        print("⚠️  WARNING: CUDA not available, falling back to CPU")
        device = "cpu"
    elif device == "cuda":
        gpu_name = torch.cuda.get_device_name(0)
        print(f"🎮 GPU: {gpu_name}")

    print(f"🤖 Number of parallel robots: {num_envs}")
    print(f"📊 Total timesteps: {total_timesteps:,}")
    print(f"🎯 Goal: Maximize walking distance without falling")
    print(f"🔧 Tuning: 8 balance/IMU gains only")
    print("=" * 70)

    save_dir = Path(save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)
    (save_dir / "logs").mkdir(exist_ok=True)

    # Base config path
    config_path = Path(__file__).parent.parent / "config" / "wbc_controller.yaml"

    # Create parallel environments
    print(f"\n🔨 Creating {num_envs} parallel environments...")
    env_fns = [make_env(i, str(config_path), max_steps=1000) for i in range(num_envs)]

    # Use SubprocVecEnv for true parallelism
    print(f"🚀 Launching {num_envs} subprocesses...")
    env = SubprocVecEnv(env_fns, start_method='spawn')

    # Load or create VecNormalize
    vec_normalize_path = save_dir / "vecnormalize_latest.pkl"
    if vec_normalize_path.exists() and resume:
        print(f"📥 Loading VecNormalize from: {vec_normalize_path}")
        env = VecNormalize.load(vec_normalize_path, env)
        env.training = True
        env.norm_reward = True
    else:
        print("🆕 Creating new VecNormalize wrapper")
        env = VecNormalize(
            env,
            norm_obs=True,
            norm_reward=True,
            clip_obs=10.0,
            clip_reward=50.0,
            gamma=0.99,
        )

    # Load or create model
    if resume and Path(resume).exists():
        print(f"📥 Loading model from: {resume}")
        model = PPO.load(
            resume,
            env=env,
            device=device,
            tensorboard_log=str(save_dir / "logs"),
        )
    else:
        print("🆕 Creating new PPO model")
        model = PPO(
            "MlpPolicy",
            env,
            learning_rate=learning_rate,
            n_steps=2048 // num_envs,  # Adjust for parallel envs
            batch_size=256,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.01,  # Exploration
            vf_coef=0.5,
            max_grad_norm=0.5,
            device=device,
            verbose=1,
            tensorboard_log=str(save_dir / "logs"),
        )

    # Callbacks
    callbacks = CallbackList([
        ParallelEpisodeInfoCallback(verbose=1),
        BestDistanceCallback(save_dir=save_dir, verbose=1),
    ])

    # Train
    print(f"\n{'='*70}")
    print(f"🏃 Starting training with {num_envs} parallel robots...")
    print(f"{'='*70}\n")

    start_time = time.time()

    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=callbacks,
            progress_bar=True,
            tb_log_name="PPO_parallel_gains",
        )
    except KeyboardInterrupt:
        print("\n⚠️  Training interrupted by user")
    except Exception as e:
        print(f"\n❌ Training error: {e}")
        import traceback
        traceback.print_exc()

    elapsed = time.time() - start_time

    print(f"\n{'='*70}")
    print(f"✅ TRAINING COMPLETE!")
    print(f"{'='*70}")
    print(f"⏱️  Duration: {elapsed/60:.1f} minutes ({elapsed:.0f}s)")
    print(f"📈 Total timesteps: {model.num_timesteps:,}")
    print(f"🤖 Parallel robots: {num_envs}")
    print(f"{'='*70}")

    # Save final model
    final_path = save_dir / "final_model"
    model.save(final_path)
    env.save(vec_normalize_path)
    print(f"💾 Final model saved: {final_path.with_suffix('.zip')}")

    # Show best model stats
    stats_file = save_dir / "best_model_stats.txt"
    if stats_file.exists():
        with open(stats_file, "r") as f:
            stats = f.read().strip()
            print(f"\n🏆 Best model: {stats}")

    print(f"\n📁 Saved models:")
    print(f"   Final: {save_dir / 'final_model.zip'}")
    print(f"   Best:  {save_dir / 'best_model.zip'}")

    print(f"\n🚀 To export config:")
    print(f"   python3 evaluate_wbc.py --export {save_dir / 'best_model.zip'} --episodes 20")

    print(f"\n📈 View training progress:")
    print(f"   tensorboard --logdir {save_dir / 'logs'}")
    print("=" * 70)

    # Cleanup
    env.close()


def main():
    parser = argparse.ArgumentParser(
        description="Parallel WBC PPO training with 32 robots"
    )
    parser.add_argument(
        "--timesteps",
        type=int,
        default=100_000,
        help="Total training timesteps (default: 100k)",
    )
    parser.add_argument(
        "--num-envs",
        type=int,
        default=32,
        help="Number of parallel robots (default: 32)",
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
        default="models/wbc_ppo_parallel",
        help="Directory to save models",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda",
        choices=["cuda", "cpu"],
        help="Device to use (default: cuda)",
    )
    parser.add_argument(
        "--resume",
        type=str,
        default=None,
        help="Path to model to resume training from",
    )

    args = parser.parse_args()

    # Handle Ctrl+C gracefully
    def signal_handler(_sig, _frame):
        print("\n\n⚠️  Received interrupt signal, cleaning up...")
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    train_parallel(
        total_timesteps=args.timesteps,
        num_envs=args.num_envs,
        save_dir=args.save_dir,
        learning_rate=args.lr,
        device=args.device,
        resume=args.resume,
    )


if __name__ == "__main__":
    main()
