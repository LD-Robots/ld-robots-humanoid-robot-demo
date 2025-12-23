#!/usr/bin/env python3
"""
Evaluate trained PPO model and export best WBC configuration.
"""

import argparse
import numpy as np
import yaml
from pathlib import Path
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv

from wbc_tuning_env import WbcTuningEnv


def evaluate_model(
    model_path: str,
    n_episodes: int = 10,
    render: bool = False,
):
    """Evaluate trained model and report performance."""

    print("=" * 60)
    print("WBC PPO Model Evaluation")
    print("=" * 60)

    # Load model
    print(f"Loading model from: {model_path}")
    model = PPO.load(model_path)

    # Load normalization stats if available
    model_dir = Path(model_path).parent
    vec_normalize_path = model_dir / "vecnormalize_final.pkl"

    # Create environment
    env = WbcTuningEnv(render_mode="human" if render else None)

    # Check if VecNormalize wrapper is needed
    is_vec_env = False
    if vec_normalize_path.exists():
        print(f"Loading VecNormalize from: {vec_normalize_path}")
        env = DummyVecEnv([lambda: env])
        env = VecNormalize.load(vec_normalize_path, env)
        env.training = False
        env.norm_reward = False
        is_vec_env = True

    print(f"\nRunning {n_episodes} evaluation episodes...")
    print("=" * 60)

    episode_rewards = []
    episode_lengths = []
    episode_distances = []
    episode_steps = []
    best_params = None
    best_reward = -np.inf

    for ep in range(n_episodes):
        # Handle different reset APIs
        if is_vec_env:
            obs = env.reset()
        else:
            obs, info = env.reset()

        done = False
        total_reward = 0
        steps = 0

        while not done:
            action, _states = model.predict(obs, deterministic=True)

            # Handle different step APIs
            if is_vec_env:
                obs, reward, done, info = env.step(action)
                # Unwrap vectorized values
                reward = float(reward[0])
                done = bool(done[0])
                info = info[0] if isinstance(info, list) else info
            else:
                obs, reward, terminated, truncated, info = env.step(action)
                done = terminated or truncated

            total_reward += reward
            steps += 1

        episode_rewards.append(total_reward)
        episode_lengths.append(steps)
        episode_distances.append(info.get('distance', 0.0))
        episode_steps.append(info.get('steps_taken', 0))

        print(f"Episode {ep + 1}/{n_episodes}:")
        print(f"  Reward: {total_reward:.2f}")
        print(f"  Length: {steps} steps")
        print(f"  Distance: {info.get('distance', 0.0):.3f} m")
        print(f"  Steps taken: {info.get('steps_taken', 0)}")
        print(f"  Fell: {info.get('fell', False)}")

        if total_reward > best_reward:
            best_reward = total_reward
            # Unwrap action if needed
            if is_vec_env and isinstance(action, np.ndarray) and action.ndim > 1:
                best_params = action[0]
            else:
                best_params = action

    print("\n" + "=" * 60)
    print("Evaluation Summary")
    print("=" * 60)
    print(f"Average reward: {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f}")
    print(f"Average length: {np.mean(episode_lengths):.1f} ± {np.std(episode_lengths):.1f}")
    print(f"Average distance: {np.mean(episode_distances):.3f} ± {np.std(episode_distances):.3f} m")
    print(f"Average steps: {np.mean(episode_steps):.1f} ± {np.std(episode_steps):.1f}")
    print(f"Success rate: {sum(1 for r in episode_rewards if r > 0) / n_episodes * 100:.1f}%")
    print("=" * 60)

    env.close()

    return best_params, episode_rewards


def export_best_config(
    model_path: str,
    output_path: str = None,
    n_eval_episodes: int = 5,
):
    """
    Evaluate model multiple times and export best configuration.
    """

    print("\n" + "=" * 60)
    print("Exporting Best WBC Configuration")
    print("=" * 60)

    best_params, _ = evaluate_model(model_path, n_episodes=n_eval_episodes, render=False)

    if best_params is None:
        print("ERROR: Could not determine best parameters")
        return

    # Map parameters to config keys
    param_names = [
        'support_hip_pitch',
        'swing_hip_pitch',
        'support_ankle_pitch',
        'swing_ankle_pitch',
        'hip_roll_shift',
        'balance_kp_pitch',
        'balance_kd_pitch',
        'imu_kp_pitch',
        'step_duration',
        'support_center_x_offset',
    ]

    best_config = {}
    print("\nOptimal Parameters:")
    print("-" * 60)
    for name, value in zip(param_names, best_params):
        best_config[name] = float(value)
        print(f"  {name:30s}: {value:.4f}")

    # Save to YAML
    if output_path is None:
        pkg_path = Path(__file__).parent.parent
        output_path = pkg_path / "config" / "wbc_controller_optimized.yaml"

    # Load base config and update with optimized params
    base_config_path = Path(__file__).parent.parent / "config" / "wbc_controller.yaml"
    with open(base_config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Update parameters
    wbc_params = config['wbc_controller']['ros__parameters']
    for key, value in best_config.items():
        wbc_params[key] = value

    # Add metadata
    config['_metadata'] = {
        'optimized_by': 'PPO RL',
        'model_path': str(model_path),
        'n_eval_episodes': n_eval_episodes,
    }

    # Write optimized config
    with open(output_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)

    print("-" * 60)
    print(f"\nOptimized config saved to: {output_path}")
    print("\nTo use optimized config:")
    print(f"  cp {output_path} {base_config_path}")
    print("  ros2 launch humanoid_mujoco mujoco_with_wbc.launch.py")
    print("=" * 60)


def compare_configs(
    model_path: str,
    baseline_config: str = None,
):
    """Compare RL-optimized config vs baseline."""

    print("\n" + "=" * 60)
    print("Configuration Comparison")
    print("=" * 60)

    # Get optimized params
    best_params, _ = evaluate_model(model_path, n_episodes=3, render=False)

    param_names = [
        'support_hip_pitch',
        'swing_hip_pitch',
        'support_ankle_pitch',
        'swing_ankle_pitch',
        'hip_roll_shift',
        'balance_kp_pitch',
        'balance_kd_pitch',
        'imu_kp_pitch',
        'step_duration',
        'support_center_x_offset',
    ]

    # Load baseline
    if baseline_config is None:
        baseline_config = Path(__file__).parent.parent / "config" / "wbc_controller.yaml"

    with open(baseline_config, 'r') as f:
        baseline = yaml.safe_load(f)
        baseline_params = baseline['wbc_controller']['ros__parameters']

    print(f"\n{'Parameter':<30s} {'Baseline':>12s} {'Optimized':>12s} {'Change':>12s}")
    print("-" * 70)

    for name, opt_value in zip(param_names, best_params):
        base_value = baseline_params.get(name, 0.0)
        change = ((opt_value - base_value) / base_value * 100) if base_value != 0 else 0
        print(f"{name:<30s} {base_value:>12.4f} {opt_value:>12.4f} {change:>11.1f}%")

    print("-" * 70)


def main():
    parser = argparse.ArgumentParser(description="Evaluate trained WBC PPO model")
    parser.add_argument(
        "model_path",
        type=str,
        help="Path to trained model (e.g., models/wbc_ppo/best_model/best_model.zip)",
    )
    parser.add_argument(
        "--export",
        action="store_true",
        help="Export best configuration to YAML",
    )
    parser.add_argument(
        "--compare",
        action="store_true",
        help="Compare with baseline configuration",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=10,
        help="Number of evaluation episodes (default: 10)",
    )
    parser.add_argument(
        "--render",
        action="store_true",
        help="Render evaluation episodes",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output path for exported config",
    )

    args = parser.parse_args()

    if args.export:
        export_best_config(
            args.model_path,
            output_path=args.output,
            n_eval_episodes=args.episodes,
        )
    elif args.compare:
        compare_configs(args.model_path)
    else:
        evaluate_model(
            args.model_path,
            n_episodes=args.episodes,
            render=args.render,
        )


if __name__ == "__main__":
    main()
