#!/usr/bin/env python3
"""
Direct config export from trained PPO model WITHOUT running simulation.
Extracts learned parameters and saves to YAML.
"""

import argparse
import numpy as np
import yaml
from pathlib import Path
from stable_baselines3 import PPO


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


def export_config_direct(
    model_path: str,
    output_path: str = None,
):
    """
    Extract parameters from trained model and export to YAML.
    No simulation needed - just reads the policy network.
    """

    print("=" * 60)
    print("Direct WBC Config Export (No Simulation)")
    print("=" * 60)

    # Load model
    print(f"\nLoading model from: {model_path}")
    model = PPO.load(model_path)

    # Extract policy action using mean of actions from random observations
    # This is more robust than using a single zero observation
    print("Extracting parameters from policy (averaging over random observations)...")

    # Get observation space dimension from model
    obs_dim = model.observation_space.shape[0]
    print(f"Model observation space: {obs_dim} dimensions")

    # Sample multiple observations and average the actions
    # This gives a more representative "mean policy" behavior
    num_samples = 100
    print(f"Sampling {num_samples} random observations to find mean action...")

    actions = []
    for _ in range(num_samples):
        # Sample random observation from observation space
        # Use small random values around zero (robot at rest state)
        random_obs = np.random.randn(obs_dim).astype(np.float32) * 0.1
        action, _ = model.predict(random_obs, deterministic=True)
        actions.append(action)

    # Average all actions
    action = np.mean(actions, axis=0)
    print(f"Mean action computed from {num_samples} samples")

    # Load base config and baseline values
    base_config_path = Path(__file__).parent.parent / "config" / "wbc_controller.yaml"
    baseline = _load_baseline_params(base_config_path)

    action_dim = int(np.prod(model.action_space.shape))
    if action_dim == len(PARAM_NAMES):
        actual_values = baseline * action
        param_names = PARAM_NAMES
    elif action_dim == len(GAIN_KEYS):
        actual_values = baseline[GAIN_INDICES] * action
        param_names = GAIN_KEYS
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
    print("-" * 60)

    # Save to YAML
    if output_path is None:
        pkg_path = Path(__file__).parent.parent
        output_path = pkg_path / "config" / "wbc_controller_optimized.yaml"

    print(f"\nLoading base config from: {base_config_path}")
    with open(base_config_path, "r") as f:
        config = yaml.safe_load(f)

    # Update parameters
    wbc_params = config['wbc_controller']['ros__parameters']
    for key, value in best_config.items():
        old_value = wbc_params.get(key, "N/A")
        wbc_params[key] = value

        # Show what changed
        if old_value != "N/A":
            change = ((value - old_value) / old_value * 100) if old_value != 0 else 0
            print(f"  {key:30s}: {old_value:8.4f} → {value:8.4f} ({change:+6.1f}%)")

    # Add metadata
    config['_metadata'] = {
        'optimized_by': 'PPO RL (direct export)',
        'model_path': str(model_path),
        'note': 'Parameters extracted from trained policy without simulation',
    }

    # Write optimized config
    print(f"\nSaving optimized config to: {output_path}")
    with open(output_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    print("\n" + "=" * 60)
    print("Export Complete!")
    print("=" * 60)
    print("\nTo use optimized config:")
    print(f"  cp {output_path} {base_config_path}")
    print("  ros2 launch humanoid_mujoco mujoco_with_wbc.launch.py")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description="Direct export of WBC config from trained PPO model (no simulation)"
    )
    parser.add_argument(
        "model_path",
        type=str,
        help="Path to trained model (e.g., models/wbc_ppo/wbc_ppo_final.zip)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output path for exported config (default: ../config/wbc_controller_optimized.yaml)",
    )

    args = parser.parse_args()

    export_config_direct(
        args.model_path,
        output_path=args.output,
    )


if __name__ == "__main__":
    main()
