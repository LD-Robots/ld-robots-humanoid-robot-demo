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

    # Get a sample observation (zeros or mean) to extract policy action
    # The trained policy will output the optimal parameters
    print("Extracting optimal parameters from policy...")

    # Get observation space dimension from model
    obs_dim = model.observation_space.shape[0]
    print(f"Model observation space: {obs_dim} dimensions")

    # Create a dummy observation (use zeros)
    dummy_obs = np.zeros(obs_dim, dtype=np.float32)

    # Get deterministic action from policy
    action, _ = model.predict(dummy_obs, deterministic=True)

    # Map ALL parameters to config keys (39 params)
    param_names = [
        # Timing
        'hold_enter_duration',
        'stabilize_duration',
        'step_duration',
        # Thresholds
        'hold_com_threshold',
        'hold_vel_threshold',
        'hold_release_com_threshold',
        'hold_release_vel_threshold',
        'phase_error_threshold',
        'phase_hold_max',
        'phase_hold_progress_gate',
        # Step parameters
        'step_length',
        'step_height',
        'stance_width',
        # Knee angles
        'support_knee_bend',
        'swing_knee_bend',
        # Hip pitch
        'support_hip_pitch',
        'swing_hip_pitch',
        # Ankle pitch
        'support_ankle_pitch',
        'swing_ankle_pitch',
        # Hip roll
        'hip_roll_shift',
        'swing_hip_roll_out',
        # Hip yaw
        'support_hip_yaw',
        'swing_hip_yaw',
        # Balance gains - pitch
        'balance_kp_pitch',
        'balance_kd_pitch',
        'imu_kp_pitch',
        'imu_kd_pitch',
        # Balance gains - roll
        'balance_kp_roll',
        'balance_kd_roll',
        'imu_kp_roll',
        'imu_kd_roll',
        # Limits
        'ankle_pitch_limit',
        'hip_pitch_limit',
        'ankle_roll_limit',
        'hip_roll_limit',
        # Other
        'com_deadzone',
        'prediction_time',
        'filter_alpha',
        'support_center_x_offset',
    ]

    best_config = {}
    print("\nOptimal Parameters:")
    print("-" * 60)
    for name, value in zip(param_names, action):
        best_config[name] = float(value)
        print(f"  {name:30s}: {value:.4f}")
    print("-" * 60)

    # Save to YAML
    if output_path is None:
        pkg_path = Path(__file__).parent.parent
        output_path = pkg_path / "config" / "wbc_controller_optimized.yaml"

    # Load base config and update with optimized params
    base_config_path = Path(__file__).parent.parent / "config" / "wbc_controller.yaml"

    print(f"\nLoading base config from: {base_config_path}")
    with open(base_config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Update parameters
    wbc_params = config['wbc_controller']['ros__parameters']
    for key, value in best_config.items():
        old_value = wbc_params.get(key, 'N/A')
        wbc_params[key] = value

        # Show what changed
        if old_value != 'N/A':
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
