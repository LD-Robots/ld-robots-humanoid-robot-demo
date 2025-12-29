#!/usr/bin/env python3
"""
Robust PPO training that tunes only kp/kd gains.
Trains in small batches with cleanup to avoid blocking issues.
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
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.callbacks import CallbackList, BaseCallback
from stable_baselines3.common.monitor import Monitor

from wbc_tuning_env import WbcGainsTuningEnv


class EpisodeInfoCallback(BaseCallback):
    """Log episode termination reasons and distance to TensorBoard."""

    def __init__(self, verbose: int = 0):
        super().__init__(verbose)
        self._reset_counters()

    def _reset_counters(self):
        self.episode_count = 0
        self.fell_count = 0
        self.timeout_count = 0
        self.no_data_count = 0
        self.distance_sum = 0.0
        self.reward_sum = 0.0
        self.step_count = 0
        self.obs_valid_sum = 0
        self.warmup_retries_sum = 0

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        dones = self.locals.get("dones", [])
        rewards = self.locals.get("rewards", [])

        if rewards is not None:
            self.reward_sum += float(sum(rewards))
            self.step_count += len(rewards)

        for info, done in zip(infos, dones):
            # Track warmup retries
            if "warmup_retries" in info:
                self.warmup_retries_sum += info["warmup_retries"]

            if info.get("obs_valid"):
                self.obs_valid_sum += 1
            if not done:
                continue
            self.episode_count += 1
            if info.get("fell"):
                self.fell_count += 1
            if info.get("reason") == "timeout":
                self.timeout_count += 1
            if info.get("reason") == "no_data":
                self.no_data_count += 1
            if "distance" in info:
                self.distance_sum += float(info["distance"])
        return True

    def _on_rollout_end(self) -> None:
        if self.step_count > 0:
            self.logger.record("custom/reward_per_step_mean", self.reward_sum / self.step_count)
            self.logger.record("custom/obs_valid_rate", self.obs_valid_sum / self.step_count)
        if self.episode_count > 0:
            self.logger.record("custom/term_fell_rate", self.fell_count / self.episode_count)
            self.logger.record("custom/term_timeout_rate", self.timeout_count / self.episode_count)
            self.logger.record("custom/term_no_data_rate", self.no_data_count / self.episode_count)
            self.logger.record("custom/distance_mean", self.distance_sum / self.episode_count)
            self.logger.record("custom/mean_warmup_retries", self.warmup_retries_sum / self.episode_count)
        self._reset_counters()


class CustomCheckpointCallback(BaseCallback):
    """
    Checkpoint callback that correctly handles total_timesteps across batches.
    Saves models with actual global timestep count, not batch-local count.
    """

    def __init__(
        self,
        save_freq: int,
        save_path: str,
        name_prefix: str = "model",
        save_vecnormalize: bool = True,
        verbose: int = 0,
    ):
        super().__init__(verbose)
        self.save_freq = save_freq
        self.save_path = Path(save_path)
        self.name_prefix = name_prefix
        self.save_vecnormalize = save_vecnormalize
        self.save_path.mkdir(parents=True, exist_ok=True)

        # Track when we last saved to avoid duplicates
        self.last_saved_timestep = -1

    def _on_step(self) -> bool:
        # Use model.num_timesteps which is the GLOBAL timestep count
        current_timestep = self.model.num_timesteps

        # Check if we should save (every save_freq steps, but only once per checkpoint)
        if current_timestep - self.last_saved_timestep >= self.save_freq:
            # Calculate the checkpoint number
            checkpoint_num = (current_timestep // self.save_freq) * self.save_freq

            model_path = self.save_path / f"{self.name_prefix}_{checkpoint_num}_steps"

            # Only save if this exact checkpoint doesn't already exist
            if not model_path.with_suffix(".zip").exists():
                self.model.save(model_path)
                self.last_saved_timestep = current_timestep

                if self.verbose > 0:
                    print(f"💾 Checkpoint saved: {model_path.with_suffix('.zip')}")
                    print(f"   Global timesteps: {current_timestep:,}")

                # Save VecNormalize if requested
                if self.save_vecnormalize and hasattr(self.model.get_env(), "save"):
                    vecnorm_path = self.save_path / f"vecnormalize_{checkpoint_num}_steps.pkl"
                    self.model.get_env().save(vecnorm_path)
                    if self.verbose > 0:
                        print(f"   VecNormalize saved: {vecnorm_path}")

        return True


class PerformanceMonitorCallback(BaseCallback):
    """
    Monitor training performance and detect degradation.
    Tracks mean episode reward and distance over rolling window.
    """

    def __init__(self, window_size: int = 10, verbose: int = 0):
        super().__init__(verbose)
        self.window_size = window_size
        self.episode_rewards = []
        self.episode_distances = []
        self.best_mean_reward = -float('inf')
        self.best_mean_distance = 0.0
        self.degradation_count = 0

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        dones = self.locals.get("dones", [])

        for info, done in zip(infos, dones):
            if not done:
                continue

            # Track episode metrics
            if "episode" in info:
                episode_reward = info["episode"]["r"]
                self.episode_rewards.append(episode_reward)

            if "distance" in info:
                self.episode_distances.append(info["distance"])

        # Keep only recent episodes
        if len(self.episode_rewards) > self.window_size:
            self.episode_rewards = self.episode_rewards[-self.window_size:]
        if len(self.episode_distances) > self.window_size:
            self.episode_distances = self.episode_distances[-self.window_size:]

        return True

    def _on_rollout_end(self) -> None:
        if len(self.episode_rewards) >= self.window_size:
            mean_reward = np.mean(self.episode_rewards)
            mean_distance = np.mean(self.episode_distances) if self.episode_distances else 0.0

            # Log to TensorBoard
            self.logger.record("custom/mean_episode_reward_window", mean_reward)
            self.logger.record("custom/mean_distance_window", mean_distance)
            self.logger.record("custom/best_mean_reward", self.best_mean_reward)
            self.logger.record("custom/best_mean_distance", self.best_mean_distance)

            # Check for improvement
            if mean_reward > self.best_mean_reward:
                self.best_mean_reward = mean_reward
                self.degradation_count = 0
                if self.verbose:
                    print(f"📈 New best mean reward: {mean_reward:.2f}")
            else:
                self.degradation_count += 1

            if mean_distance > self.best_mean_distance:
                self.best_mean_distance = mean_distance
                if self.verbose:
                    print(f"📏 New best mean distance: {mean_distance:.3f}m")


class BestModelCallback(BaseCallback):
    """Save best model based on maximum distance traveled."""

    def __init__(self, save_dir: Path, verbose: int = 0):
        super().__init__(verbose)
        self.save_dir = save_dir
        self.best_distance = 0.0
        self.best_survived = False
        self.best_timestep = 0
        self.best_episode_length = 0

        # Try to load previous best stats if they exist
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
                        if "episode_length=" in line:
                            self.best_episode_length = int(line.split("episode_length=")[1].split(",")[0])
                if self.verbose:
                    print(f"📊 Loaded previous best: distance={self.best_distance:.3f}m, "
                          f"survived={self.best_survived}, timestep={self.best_timestep}")
            except Exception as e:
                if self.verbose:
                    print(f"⚠️  Could not load previous best stats: {e}")

    def _save_best_stats(self, distance: float, survived: bool, timestep: int, episode_length: int):
        """Save best model statistics to file."""
        with open(self.stats_file, "w") as f:
            f.write(f"distance={distance:.4f}, survived={survived}, "
                   f"timestep={timestep}, episode_length={episode_length}\n")
            f.write(f"timestamp={time.strftime('%Y-%m-%d %H:%M:%S')}\n")

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        dones = self.locals.get("dones", [])
        for info, done in zip(infos, dones):
            if not done:
                continue

            # Get distance traveled (primary metric)
            distance = float(info.get("distance", 0.0))
            fell = info.get("fell", False)
            survived = not fell
            episode_length = int(info.get("step_count", 0))

            # Save model if:
            # 1. First to survive, OR
            # 2. Survived and went further than previous best, OR
            # 3. Both fell but this one went further
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
                self.best_episode_length = episode_length

                best_path = self.save_dir / "best_model"
                self.model.save(best_path)

                # Save statistics
                self._save_best_stats(distance, survived, self.best_timestep, episode_length)

                # Save VecNormalize for best model too
                if hasattr(self.model.get_env(), "save"):
                    vecnorm_path = self.save_dir / "best_model_vecnormalize.pkl"
                    self.model.get_env().save(vecnorm_path)

                if self.verbose:
                    status = "✅ survived" if survived else "❌ fell"
                    print(f"🏆 Best model updated! ({reason})")
                    print(f"   Status: {status}")
                    print(f"   Distance: {distance:.3f}m ({episode_length} steps)")
                    print(f"   At global timestep: {self.best_timestep:,}")
        return True


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
    progress_bar: bool = False,
):
    """Train for a batch of steps, then save and cleanup."""

    if model_path is not None:
        model_path = Path(model_path)
        if not model_path.exists() and model_path.suffix != ".zip":
            zip_path = model_path.with_suffix(".zip")
            if zip_path.exists():
                model_path = zip_path

    print("\n" + "=" * 60)
    print(f"Training Batch: {batch_steps:,} steps")
    print(f"Total timesteps so far: {total_timesteps_so_far:,}")
    if model_path:
        print(f"Resuming from: {model_path}")
    print("=" * 60)

    # Create environment
    config_path = Path(__file__).parent.parent / "config" / "wbc_controller.yaml"
    env = WbcGainsTuningEnv(config_path=str(config_path), max_steps=1000)
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
        # Optimized VecNormalize settings for better reward scaling
        env = VecNormalize(
            env,
            norm_obs=True,
            norm_reward=True,
            clip_obs=10.0,
            clip_reward=50.0,  # Increased from 10.0 to handle larger reward range
            gamma=0.99,        # Match PPO gamma for correct reward normalization
        )
    # Ensure env is reset before learning (new env each batch)
    env.reset()

    # Load or create model
    if model_path and model_path.exists():
        print(f"Loading model from: {model_path}")
        model = PPO.load(
            model_path,
            env=env,
            device=device,
            force_reset=True,  # New env per batch needs reset
            tensorboard_log=str(save_dir / "logs"),
        )
        # Manually set num_timesteps to continue from where we left off
        model.num_timesteps = total_timesteps_so_far
        model._total_timesteps = total_timesteps_so_far
        print(f"✓ Continuing from timestep {total_timesteps_so_far:,}")
    else:
        print("Creating new model...")
        # Optimized hyperparameters for better training stability and exploration
        model = PPO(
            "MlpPolicy",
            env,
            learning_rate=learning_rate,  # Will be passed as argument (default 1e-4)
            n_steps=2048,                 # Kept: good balance for 100Hz control
            batch_size=256,                # Increased from 64 for more stable gradients
            n_epochs=10,                   # Kept: good for sample efficiency
            gamma=0.99,                    # Kept: standard for continuous control
            gae_lambda=0.95,               # Kept: good advantage estimation
            clip_range=0.2,                # Kept: standard PPO clipping
            ent_coef=0.05,                 # Increased from 0.01 for more exploration
            vf_coef=0.5,                   # Kept: standard value function weight
            max_grad_norm=0.5,             # Kept: prevents gradient explosion
            device=device,
            verbose=1,
            tensorboard_log=str(save_dir / "logs"),
        )
        model.num_timesteps = 0
        model._total_timesteps = 0

    # Use a stable TensorBoard run name across batches
    tb_log_name = "PPO_robust_gains"

    # Callbacks - use custom checkpoint callback that handles global timesteps correctly
    checkpoint_callback = CustomCheckpointCallback(
        save_freq=10000,  # Save every 10k steps (globally)
        save_path=str(save_dir / "checkpoints"),
        name_prefix="wbc_ppo_gains",
        save_vecnormalize=True,
        verbose=1,
    )

    callback = CallbackList([
        checkpoint_callback,
        EpisodeInfoCallback(),
        BestModelCallback(save_dir=save_dir, verbose=1),
        PerformanceMonitorCallback(window_size=10, verbose=1),
    ])

    # Train
    print(f"\n🚀 Starting training for {batch_steps:,} steps...")
    start_time = time.time()

    try:
        model.learn(
            total_timesteps=batch_steps,
            callback=callback,
            progress_bar=progress_bar,
            reset_num_timesteps=False,  # Continue timestep count
            tb_log_name=tb_log_name,
        )
    except KeyboardInterrupt:
        print("\n⚠️  Training interrupted by user")
    except Exception as e:
        print(f"\n❌ Training error: {e}")

    elapsed = time.time() - start_time

    # Calculate final timestep count after this batch
    final_timesteps = model.num_timesteps

    print(f"\n{'='*60}")
    print(f"📊 Batch Training Summary")
    print(f"{'='*60}")
    print(f"⏱️  Duration: {elapsed/60:.1f} minutes ({elapsed:.0f}s)")
    print(f"📈 Timesteps trained this batch: {batch_steps:,}")
    print(f"🎯 Total timesteps (global): {final_timesteps:,}")
    print(f"{'='*60}")

    # Save latest model (always overwrite)
    latest_model = save_dir / "wbc_ppo_latest"
    model.save(latest_model)
    latest_model_path = latest_model.with_suffix(".zip")
    env.save(vec_normalize_path)
    print(f"💾 Latest model saved: {latest_model_path}")

    # Save timestamped backup for safety (every batch)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    backup_model = save_dir / f"backup_{final_timesteps}_steps_{timestamp}"
    model.save(backup_model)
    backup_vecnorm = save_dir / f"backup_{final_timesteps}_steps_{timestamp}_vecnormalize.pkl"
    env.save(backup_vecnorm)
    print(f"🔒 Backup saved: {backup_model.with_suffix('.zip')}")
    print(f"{'='*60}\n")

    # Cleanup
    env.close()
    del model
    del env

    return latest_model_path


def train_robust(
    total_timesteps: int = 50_000,
    batch_size: int = 5_000,
    save_dir: str = "models/wbc_ppo_robust_gains",
    learning_rate: float = 1e-4,  # Reduced from 3e-4 for more stable training
    device: str = "cuda",
    progress_bar: bool = False,
):
    """Train robustly with automatic batching and cleanup."""

    print("=" * 60)
    print("🛡️  WBC PPO Robust Training - Gains Only")
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
                progress_bar=progress_bar,
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
    print("✅ TRAINING COMPLETE!")
    print("=" * 60)
    print(f"📊 Total timesteps trained: {total_trained:,}")
    print(f"📦 Total batches completed: {batch_num}")
    print(f"\n📁 Saved models:")
    print(f"   Latest: {save_dir / 'wbc_ppo_latest.zip'}")
    print(f"   Best:   {save_dir / 'best_model.zip'}")
    print(f"   Checkpoints: {save_dir / 'checkpoints'}/*.zip")
    print(f"   Backups: {save_dir}/backup_*.zip")

    # Show best model stats if available
    stats_file = save_dir / "best_model_stats.txt"
    if stats_file.exists():
        with open(stats_file, "r") as f:
            stats = f.read().strip()
            print(f"\n🏆 Best model: {stats}")

    print(f"\n🚀 To export config:")
    print(f"   python3 export_config_direct.py {save_dir / 'best_model'}")
    print(f"\n📈 View training progress:")
    print(f"   tensorboard --logdir {save_dir / 'logs'}")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description="Robust WBC PPO training (gains only)"
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
        default=1e-4,
        help="Learning rate (default: 1e-4, optimized for stability)",
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        default="models/wbc_ppo_robust_gains",
        help="Directory to save models",
    )
    parser.add_argument(
        "--device",
        type=str,
        default="cuda",
        choices=["cuda", "cpu"],
        help="Device to use for training (default: cuda)",
    )
    parser.add_argument(
        "--progress",
        action="store_true",
        help="Enable progress bar (may conflict with rich live display)",
    )

    args = parser.parse_args()

    # Handle Ctrl+C gracefully
    def signal_handler(_sig, _frame):
        """Handle interrupt signal (Ctrl+C) gracefully."""
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
        progress_bar=args.progress,
    )


if __name__ == "__main__":
    main()
