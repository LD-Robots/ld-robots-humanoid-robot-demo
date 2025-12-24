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

from wbc_tuning_env import WbcTuningEnv, WbcGainsTuningEnv

PARAM_NAMES = [
    # Timing
    "hold_enter_duration",
    "stabilize_duration",
    "step_duration",
    # Thresholds
    "hold_com_threshold",
    "hold_vel_threshold",
    "hold_release_com_threshold",
    "hold_release_vel_threshold",
    "phase_error_threshold",
    "phase_hold_max",
    "phase_hold_progress_gate",
    # Step parameters
    "step_length",
    "step_height",
    "stance_width",
    # Knee angles
    "support_knee_bend",
    "swing_knee_bend",
    # Hip pitch
    "support_hip_pitch",
    "swing_hip_pitch",
    # Ankle pitch
    "support_ankle_pitch",
    "swing_ankle_pitch",
    # Hip roll
    "hip_roll_shift",
    "swing_hip_roll_out",
    # Hip yaw
    "support_hip_yaw",
    "swing_hip_yaw",
    # Balance gains - pitch
    "balance_kp_pitch",
    "balance_kd_pitch",
    "imu_kp_pitch",
    "imu_kd_pitch",
    # Balance gains - roll
    "balance_kp_roll",
    "balance_kd_roll",
    "imu_kp_roll",
    "imu_kd_roll",
    # Limits
    "ankle_pitch_limit",
    "hip_pitch_limit",
    "ankle_roll_limit",
    "hip_roll_limit",
    # Other
    "com_deadzone",
    "prediction_time",
    "filter_alpha",
    "support_center_x_offset",
]

GAIN_KEYS = [
    "balance_kp_pitch",
    "balance_kd_pitch",
    "imu_kp_pitch",
    "imu_kd_pitch",
    "balance_kp_roll",
    "balance_kd_roll",
    "imu_kp_roll",
    "imu_kd_roll",
]
GAIN_INDICES = [23, 24, 25, 26, 27, 28, 29, 30]


def _load_baseline_params(config_path: Path) -> np.ndarray:
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    wbc_params = config["wbc_controller"]["ros__parameters"]
    baseline = np.array([
        # Timing
        wbc_params.get("hold_enter_duration", 0.8),
        wbc_params.get("stabilize_duration", 2.0),
        wbc_params.get("step_duration", 0.3),
        # Thresholds
        wbc_params.get("hold_com_threshold", 0.015),
        wbc_params.get("hold_vel_threshold", 0.2),
        wbc_params.get("hold_release_com_threshold", 0.02),
        wbc_params.get("hold_release_vel_threshold", 0.08),
        wbc_params.get("phase_error_threshold", 0.06),
        wbc_params.get("phase_hold_max", 0.001),
        wbc_params.get("phase_hold_progress_gate", 0.9),
        # Step parameters
        wbc_params.get("step_length", 0.025),
        wbc_params.get("step_height", 0.015),
        wbc_params.get("stance_width", 0.1),
        # Knee angles
        wbc_params.get("support_knee_bend", 0.2),
        wbc_params.get("swing_knee_bend", 0.4),
        # Hip pitch
        wbc_params.get("support_hip_pitch", 0.35),
        wbc_params.get("swing_hip_pitch", 0.47),
        # Ankle pitch
        wbc_params.get("support_ankle_pitch", 0.22),
        wbc_params.get("swing_ankle_pitch", 0.4),
        # Hip roll
        wbc_params.get("hip_roll_shift", 0.0),
        wbc_params.get("swing_hip_roll_out", 0.0),
        # Hip yaw
        wbc_params.get("support_hip_yaw", 0.0),
        wbc_params.get("swing_hip_yaw", 0.0),
        # Balance gains - pitch
        wbc_params.get("balance_kp_pitch", 2.5),
        wbc_params.get("balance_kd_pitch", 0.8),
        wbc_params.get("imu_kp_pitch", 0.9),
        wbc_params.get("imu_kd_pitch", 0.1),
        # Balance gains - roll
        wbc_params.get("balance_kp_roll", 3.0),
        wbc_params.get("balance_kd_roll", 0.4),
        wbc_params.get("imu_kp_roll", 0.8),
        wbc_params.get("imu_kd_roll", 0.1),
        # Limits
        wbc_params.get("ankle_pitch_limit", 0.22),
        wbc_params.get("hip_pitch_limit", 0.45),
        wbc_params.get("ankle_roll_limit", 0.02),
        wbc_params.get("hip_roll_limit", 0.02),
        # Other
        wbc_params.get("com_deadzone", 0.005),
        wbc_params.get("prediction_time", 0.04),
        wbc_params.get("filter_alpha", 0.25),
        wbc_params.get("support_center_x_offset", -0.001),
    ], dtype=np.float32)
    return baseline


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

    action_dim = int(np.prod(model.action_space.shape))
    if action_dim == len(GAIN_KEYS):
        env_cls = WbcGainsTuningEnv
    else:
        env_cls = WbcTuningEnv

    # Load normalization stats if available
    model_dir = Path(model_path).parent
    vec_normalize_path = model_dir / "vecnormalize_final.pkl"
    if not vec_normalize_path.exists():
        vec_normalize_path = model_dir / "vecnormalize_latest.pkl"

    # Create environment
    env = env_cls(render_mode="human" if render else None)

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
    best_distance = 0.0
    best_survived = False

    for ep in range(n_episodes):
        # Handle different reset APIs
        if is_vec_env:
            obs = env.reset()
        else:
            obs, info = env.reset()

        done = False
        total_reward = 0
        steps = 0

        # Action is applied at reset in this env; keep it fixed for the episode
        action, _states = model.predict(obs, deterministic=True)

        while not done:
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

        distance = info.get('distance', 0.0)
        episode_rewards.append(total_reward)
        episode_lengths.append(steps)
        episode_distances.append(distance)
        episode_steps.append(info.get('steps_taken', 0))

        print(f"Episode {ep + 1}/{n_episodes}:")
        print(f"  Reward: {total_reward:.2f}")
        print(f"  Length: {steps} steps")
        print(f"  Distance: {distance:.3f} m")
        print(f"  Steps taken: {info.get('steps_taken', 0)}")
        fell = info.get("fell", False)
        print(f"  Fell: {fell}")

        survived = not fell
        should_update = False
        reason = ""

        # FIXED: Select based on DISTANCE traveled, not episode length!
        if survived and not best_survived:
            should_update = True
            reason = "first to survive"
        elif survived and best_survived and distance > best_distance:
            should_update = True
            reason = f"new distance record: {distance:.3f}m > {best_distance:.3f}m"
        elif not survived and not best_survived and distance > best_distance:
            should_update = True
            reason = f"furthest among fallen: {distance:.3f}m > {best_distance:.3f}m"

        if should_update:
            best_reward = total_reward
            best_distance = distance
            best_survived = survived
            print(f"  ✅ NEW BEST: {reason}")
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
    print("-" * 60)
    print(f"🏆 BEST SELECTED:")
    print(f"   Distance: {best_distance:.3f} m")
    print(f"   Survived: {best_survived}")
    print(f"   Max distance: {max(episode_distances):.3f} m")
    print("=" * 60)

    env.close()

    return best_params, episode_rewards


def export_best_config(
    model_path: str,
    output_path: str = None,
    n_eval_episodes: int = 5,
    baseline_config: str = None,
):
    """
    Evaluate model multiple times and export best configuration.
    Selection is based on DISTANCE traveled (primary metric).
    """

    print("\n" + "=" * 60)
    print("Exporting Best WBC Configuration")
    print("=" * 60)

    best_params, episode_rewards = evaluate_model(model_path, n_episodes=n_eval_episodes, render=False)

    if best_params is None:
        print("ERROR: Could not determine best parameters")
        return

    # Map parameters to config keys using baseline multipliers
    if baseline_config is None:
        base_config_path = Path(__file__).parent.parent / "config" / "wbc_controller.yaml"
    else:
        base_config_path = Path(baseline_config)
    baseline = _load_baseline_params(base_config_path)

    action_dim = int(np.prod(best_params.shape))
    if action_dim == len(PARAM_NAMES):
        param_names = PARAM_NAMES
        actual_values = baseline * best_params
    elif action_dim == len(GAIN_KEYS):
        param_names = GAIN_KEYS
        actual_values = baseline[GAIN_INDICES] * best_params
    else:
        raise ValueError(
            f"Unsupported action dim {action_dim}; expected {len(PARAM_NAMES)} or {len(GAIN_KEYS)}"
        )

    best_config = {}
    print("\nOptimal Parameters:")
    print("-" * 60)
    for name, value in zip(param_names, actual_values):
        best_config[name] = float(value)
        print(f"  {name:30s}: {value:.4f}")

    # Save to YAML
    if output_path is None:
        pkg_path = Path(__file__).parent.parent
        output_path = pkg_path / "config" / "wbc_controller_optimized.yaml"

    # Load base config and update with optimized params (only selected keys)
    with open(base_config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Update parameters
    wbc_params = config['wbc_controller']['ros__parameters']
    for key, value in best_config.items():
        wbc_params[key] = value

    # Add metadata
    config['_metadata'] = {
        'optimized_by': 'PPO RL (distance-based selection)',
        'model_path': str(model_path),
        'n_eval_episodes': n_eval_episodes,
        'selection_criterion': 'maximum distance traveled',
    }

    # Write optimized config
    with open(output_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

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

    if baseline_config is None:
        baseline_path = Path(__file__).parent.parent / "config" / "wbc_controller.yaml"
    else:
        baseline_path = Path(baseline_config)
    baseline = _load_baseline_params(baseline_path)

    action_dim = int(np.prod(best_params.shape))
    if action_dim == len(PARAM_NAMES):
        param_names = PARAM_NAMES
        actual_values = baseline * best_params
    elif action_dim == len(GAIN_KEYS):
        param_names = GAIN_KEYS
        actual_values = baseline[GAIN_INDICES] * best_params
    else:
        raise ValueError(
            f"Unsupported action dim {action_dim}; expected {len(PARAM_NAMES)} or {len(GAIN_KEYS)}"
        )

    # Load baseline
    with open(baseline_path, 'r') as f:
        baseline = yaml.safe_load(f)
        baseline_params = baseline['wbc_controller']['ros__parameters']

    print(f"\n{'Parameter':<30s} {'Baseline':>12s} {'Optimized':>12s} {'Change':>12s}")
    print("-" * 70)

    for name, opt_value in zip(param_names, actual_values):
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
        "--baseline",
        type=str,
        default=None,
        help="Baseline config path to use for export/compare",
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
            baseline_config=args.baseline,
        )
    elif args.compare:
        compare_configs(args.model_path, baseline_config=args.baseline)
    else:
        evaluate_model(
            args.model_path,
            n_episodes=args.episodes,
            render=args.render,
        )


if __name__ == "__main__":
    main()
