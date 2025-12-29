#!/bin/bash
# Quick training script pentru WBC PPO auto-tuning
# Optimized for RTX 5070 Ti

set -e  # Exit on error

echo "=========================================="
echo "WBC PPO Auto-Tuning - Quick Start"
echo "=========================================="
echo ""

# Check if in correct directory
if [ ! -f "train_wbc_ppo.py" ]; then
    echo "ERROR: Run this script from the scripts/ directory"
    echo "cd src/control/wbc_pinocchio_controller/scripts/"
    exit 1
fi

# Check Python dependencies
echo "Checking dependencies..."
python3 -c "import stable_baselines3" 2>/dev/null || {
    echo "Installing dependencies..."
    pip install -r requirements_rl.txt
}

# Check GPU
echo ""
echo "Checking GPU..."
python3 -c "import torch; print(f'CUDA: {torch.cuda.is_available()}'); print(f'GPU: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else \"None\"}')"

# Create models directory
mkdir -p models/wbc_ppo

echo ""
echo "=========================================="
echo "Starting PPO Training"
echo "=========================================="
echo "Estimated time: 4-6 hours on RTX 5070 Ti"
echo "Parallel envs: 4"
echo "Total timesteps: 500,000"
echo ""
echo "Monitor progress:"
echo "  tensorboard --logdir models/wbc_ppo/logs/"
echo ""
echo "Press Ctrl+C to stop training"
echo "=========================================="
echo ""

# Start training
python3 train_wbc_ppo.py \
    --timesteps 500000 \
    --envs 4 \
    --lr 3e-4 \
    --device cuda

echo ""
echo "=========================================="
echo "Training Complete!"
echo "=========================================="
echo ""
echo "Evaluating best model..."
python3 evaluate_wbc.py models/wbc_ppo/best_model/best_model.zip \
    --episodes 5

echo ""
echo "Exporting optimized configuration..."
python3 evaluate_wbc.py models/wbc_ppo/best_model/best_model.zip \
    --export

echo ""
echo "=========================================="
echo "Auto-Tuning Complete!"
echo "=========================================="
echo ""
echo "Optimized config saved to:"
echo "  ../config/wbc_controller_optimized.yaml"
echo ""
echo "To use optimized config:"
echo "  cp ../config/wbc_controller_optimized.yaml ../config/wbc_controller.yaml"
echo "  ros2 launch humanoid_mujoco mujoco_with_wbc.launch.py"
echo ""
echo "=========================================="
