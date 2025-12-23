#!/usr/bin/env python3
"""
Watch RL training episodes live with MuJoCo visualization.
Opens viewer window to see robot walking attempts.
"""

import argparse
import time
from pathlib import Path
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from wbc_tuning_env import WbcTuningEnv


def watch_training(
    model_path: str = None,
    n_episodes: int = 5,
    deterministic: bool = True,
):
    """
    Watch trained or training model with live MuJoCo visualization.
    """

    print("=" * 60)
    print("WBC Training Visualization")
    print("=" * 60)

    # Create environment with rendering
    env = WbcTuningEnv(render_mode="human", max_steps=2000)

    if model_path:
        # Load trained model
        print(f"Loading model from: {model_path}")
        model = PPO.load(model_path)

        # Load normalization if available
        model_dir = Path(model_path).parent
        vec_normalize_path = model_dir / "vecnormalize_final.pkl"

        if vec_normalize_path.exists():
            print(f"Loading VecNormalize from: {vec_normalize_path}")
            env = DummyVecEnv([lambda: env])
            env = VecNormalize.load(vec_normalize_path, env)
            env.training = False
            env.norm_reward = False
    else:
        # Random policy (for watching untrained behavior)
        print("No model specified - using RANDOM policy")
        model = None

    print(f"\nWatching {n_episodes} episodes with MuJoCo viewer...")
    print("=" * 60)
    print("Controls:")
    print("  - Mouse: Rotate camera")
    print("  - Scroll: Zoom")
    print("  - Double-click: Track robot")
    print("  - ESC: Close viewer")
    print("=" * 60)

    for ep in range(n_episodes):
        obs, info = env.reset()
        done = False
        total_reward = 0
        steps = 0

        print(f"\nEpisode {ep + 1}/{n_episodes} starting...")

        while not done:
            if model is not None:
                action, _states = model.predict(obs, deterministic=deterministic)
            else:
                # Random action
                action = env.action_space.sample()

            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            steps += 1
            done = terminated or truncated

            # Slow down visualization for better viewing
            time.sleep(0.01)  # 100 Hz real-time

        print(f"Episode {ep + 1} finished:")
        print(f"  Total reward: {total_reward:.2f}")
        print(f"  Steps: {steps}")
        print(f"  Distance: {info.get('distance', 0.0):.3f} m")
        print(f"  Steps taken: {info.get('steps_taken', 0)}")
        print(f"  Fell: {info.get('fell', False)}")
        print(f"  Reason: {info.get('reason', 'N/A')}")

        # Brief pause between episodes
        time.sleep(2.0)

    env.close()
    print("\n" + "=" * 60)
    print("Visualization complete!")
    print("=" * 60)


def watch_latest_checkpoint():
    """Automatically find and watch the latest training checkpoint."""

    model_dir = Path("models/wbc_ppo/checkpoints")

    if not model_dir.exists():
        print("ERROR: No checkpoints found. Train a model first!")
        print("Run: python3 train_wbc_ppo.py")
        return

    # Find latest checkpoint
    checkpoints = sorted(model_dir.glob("wbc_ppo_*.zip"))

    if not checkpoints:
        print("ERROR: No checkpoint files found in", model_dir)
        return

    latest = checkpoints[-1]
    print(f"Found latest checkpoint: {latest.name}")
    print("")

    watch_training(str(latest), n_episodes=3)


def main():
    parser = argparse.ArgumentParser(
        description="Watch WBC RL training with live visualization"
    )
    parser.add_argument(
        "--model",
        type=str,
        default=None,
        help="Path to model checkpoint (default: latest checkpoint)",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=5,
        help="Number of episodes to watch (default: 5)",
    )
    parser.add_argument(
        "--random",
        action="store_true",
        help="Watch random policy instead of trained model",
    )
    parser.add_argument(
        "--stochastic",
        action="store_true",
        help="Use stochastic policy (default: deterministic)",
    )

    args = parser.parse_args()

    if args.random:
        watch_training(None, n_episodes=args.episodes)
    elif args.model:
        watch_training(
            args.model,
            n_episodes=args.episodes,
            deterministic=not args.stochastic,
        )
    else:
        # Auto-find latest checkpoint
        watch_latest_checkpoint()


if __name__ == "__main__":
    main()
