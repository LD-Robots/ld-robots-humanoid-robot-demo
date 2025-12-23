#!/usr/bin/env python3
"""
Live monitor for RL training progress.
Automatically watches latest checkpoints as they're saved during training.
"""

import time
import argparse
from pathlib import Path
from watch_training import watch_training


def monitor_training(
    checkpoint_dir: str = "models/wbc_ppo/checkpoints",
    check_interval: int = 300,  # 5 minutes
    episodes_per_check: int = 2,
):
    """
    Monitor training by periodically watching latest checkpoint.

    Args:
        checkpoint_dir: Directory where checkpoints are saved
        check_interval: Seconds between checks (default: 5 min)
        episodes_per_check: Episodes to watch per checkpoint
    """

    checkpoint_dir = Path(checkpoint_dir)
    last_checkpoint = None

    print("=" * 60)
    print("Live Training Monitor")
    print("=" * 60)
    print(f"Checkpoint directory: {checkpoint_dir}")
    print(f"Check interval: {check_interval}s ({check_interval/60:.1f} min)")
    print(f"Episodes per check: {episodes_per_check}")
    print("")
    print("This will watch the robot's performance as training progresses.")
    print("Press Ctrl+C to stop monitoring.")
    print("=" * 60)

    iteration = 0

    try:
        while True:
            iteration += 1

            # Find latest checkpoint
            if not checkpoint_dir.exists():
                print(f"\n[{iteration}] Waiting for training to start...")
                time.sleep(check_interval)
                continue

            checkpoints = sorted(checkpoint_dir.glob("wbc_ppo_*.zip"))

            if not checkpoints:
                print(f"\n[{iteration}] No checkpoints yet, waiting...")
                time.sleep(check_interval)
                continue

            latest = checkpoints[-1]

            # Check if new checkpoint available
            if latest != last_checkpoint:
                print("\n" + "=" * 60)
                print(f"[{iteration}] NEW CHECKPOINT: {latest.name}")
                print("=" * 60)

                # Watch this checkpoint
                try:
                    watch_training(
                        str(latest),
                        n_episodes=episodes_per_check,
                        deterministic=True,
                    )
                except Exception as e:
                    print(f"ERROR watching checkpoint: {e}")

                last_checkpoint = latest
            else:
                print(f"\n[{iteration}] No new checkpoint. Latest: {latest.name}")

            # Wait before next check
            print(f"Next check in {check_interval}s...")
            time.sleep(check_interval)

    except KeyboardInterrupt:
        print("\n\nMonitoring stopped by user.")
        print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description="Live monitor for WBC RL training"
    )
    parser.add_argument(
        "--interval",
        type=int,
        default=300,
        help="Check interval in seconds (default: 300 = 5 min)",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=2,
        help="Episodes to watch per checkpoint (default: 2)",
    )
    parser.add_argument(
        "--dir",
        type=str,
        default="models/wbc_ppo/checkpoints",
        help="Checkpoint directory",
    )

    args = parser.parse_args()

    monitor_training(
        checkpoint_dir=args.dir,
        check_interval=args.interval,
        episodes_per_check=args.episodes,
    )


if __name__ == "__main__":
    main()
