# WBC Parameter Auto-Tuning cu PPO

Sistem complet de Reinforcement Learning pentru găsirea automată a parametrilor optimi pentru WBC walking controller.

## 🎯 Caracteristici

- **GPU Accelerat**: Optimizat pentru RTX 5070 Ti (12GB VRAM)
- **Parallel Training**: 4+ medii simulate simultan
- **Auto-tuning**: 10 parametri critici optimizați automat
- **Fast Convergence**: ~4-8 ore training (500k timesteps)
- **Export Direct**: Configurație optimizată salvată în YAML

## 📦 Instalare

### 1. Instalează dependențele Python

```bash
cd src/control/wbc_pinocchio_controller/scripts/
pip install -r requirements_rl.txt
```

### 2. Verifică GPU

```bash
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
python3 -c "import torch; print(f'GPU: {torch.cuda.get_device_name(0)}')"
```

Output așteptat:
```
CUDA available: True
GPU: NVIDIA GeForce RTX 5070 Ti
```

## 🚀 Utilizare

### Pas 1: Training PPO

**Quick start** (4-6 ore pe RTX 5070 Ti):
```bash
python3 train_wbc_ppo.py --timesteps 500000 --envs 4
```

**Full training** (8-12 ore, rezultate mai bune):
```bash
python3 train_wbc_ppo.py --timesteps 1000000 --envs 6
```

**Parametri disponibili:**
```bash
python3 train_wbc_ppo.py \
    --timesteps 500000 \     # Total training steps
    --envs 4 \               # Parallel environments (4-8 recommended)
    --lr 3e-4 \              # Learning rate
    --save-dir models/wbc_ppo \  # Output directory
    --device cuda            # Use GPU (cuda) or CPU
```

### Pas 2: Monitorizare Training (opțional)

În alt terminal, pornește TensorBoard:
```bash
tensorboard --logdir models/wbc_ppo/logs/
```

Deschide browser la `http://localhost:6006` pentru grafice live.

### Pas 3: Evaluare Model

După training, evaluează performanța:
```bash
python3 evaluate_wbc.py models/wbc_ppo/best_model/best_model.zip \
    --episodes 10
```

### Pas 4: Export Configurație Optimizată

```bash
python3 evaluate_wbc.py models/wbc_ppo/best_model/best_model.zip \
    --export \
    --output ../config/wbc_controller_optimized.yaml
```

### Pas 5: Folosește Configurația Optimizată

```bash
# Backup config vechi
cp ../config/wbc_controller.yaml ../config/wbc_controller_backup.yaml

# Activează config optimizat
cp ../config/wbc_controller_optimized.yaml ../config/wbc_controller.yaml

# Testează
ros2 launch humanoid_mujoco mujoco_with_wbc.launch.py
```

## 📊 Comparare Configurații

Pentru a vedea diferențele între baseline și optimizat:
```bash
python3 evaluate_wbc.py models/wbc_ppo/best_model/best_model.zip --compare
```

Output exemplu:
```
Parameter                      Baseline    Optimized       Change
----------------------------------------------------------------------
support_hip_pitch                0.3500       0.1234      -64.7%
swing_hip_pitch                  0.4700       0.1567      -66.7%
balance_kp_pitch                 2.5000       1.8234      -27.1%
...
```

## 🎮 Parametri Optimizați

RL agent învață să ajusteze acești 10 parametri:

| Parametru | Range | Descriere |
|-----------|-------|-----------|
| `support_hip_pitch` | [0.02, 0.20] | Unghi hip pentru picior suport |
| `swing_hip_pitch` | [0.03, 0.25] | Unghi hip pentru picior swing |
| `support_ankle_pitch` | [0.0, 0.15] | Unghi ankle suport |
| `swing_ankle_pitch` | [0.0, 0.20] | Unghi ankle swing |
| `hip_roll_shift` | [0.04, 0.12] | Transfer lateral greutate |
| `balance_kp_pitch` | [1.0, 3.5] | Gain proporțional balance |
| `balance_kd_pitch` | [0.3, 1.5] | Gain derivativ balance |
| `imu_kp_pitch` | [0.2, 1.5] | Gain IMU feedback |
| `step_duration` | [0.3, 0.8] | Durată pas (secunde) |
| `support_center_x_offset` | [-0.05, 0.0] | Forward bias CoM |

## 🏆 Reward Function

Agent-ul maximizează:
- ✅ **Forward progress** (+100 per meter)
- ✅ **Staying upright** (+0.5 per step at good height)
- ✅ **Stability** (penalizare pentru oscilații)
- ✅ **Survival time** (+0.1 per timestep)
- ❌ **Falling** (-100 penalty)

## ⚙️ Arhitectură

**State Space** (45 dimensions):
- Base position & orientation (7)
- Joint positions (12)
- Joint velocities (12)
- IMU accelerations (3)
- IMU angular velocities (3)
- Episode info (8)

**Policy Network**:
- MLP: [256, 256] hidden layers
- Activation: ReLU
- Output: Continuous actions (10 params)

**Algorithm**: Proximal Policy Optimization (PPO)
- Optimizat pentru continuous control
- Clip ratio: 0.2
- Entropy bonus: 0.01
- GAE λ: 0.95

## 📈 Performanță Așteptată

După 500k timesteps (~4-6 ore):
- **Success rate**: 60-80% episodes fără cădere
- **Average distance**: 2-5 metri per episod
- **Steps taken**: 20-50+ pași consecutivi
- **Improvement**: 3-5x față de tuning manual

După 1M timesteps (~8-12 ore):
- **Success rate**: 80-95%
- **Average distance**: 5-10+ metri
- **Steps taken**: 50-100+ pași
- **Walking stability**: Mult mai smooth

## 🔧 Troubleshooting

### GPU Out of Memory
Reduce numărul de medii paralele:
```bash
python3 train_wbc_ppo.py --envs 2  # Instead of 4
```

### Training prea lent (CPU)
Verifică că folosești GPU:
```bash
python3 train_wbc_ppo.py --device cuda
```

### Divergență în training
Reduce learning rate:
```bash
python3 train_wbc_ppo.py --lr 1e-4  # Instead of 3e-4
```

### Environment nu pornește
Verifică că ROS2 + MuJoCo funcționează:
```bash
ros2 launch humanoid_mujoco mujoco_with_wbc.launch.py
```

## 📚 Documentație Detaliată

- **Stable-Baselines3**: https://stable-baselines3.readthedocs.io/
- **PPO Algorithm**: https://arxiv.org/abs/1707.06347
- **Gymnasium**: https://gymnasium.farama.org/

## 🎯 Next Steps După Training

1. **Fine-tuning**: Continuă training cu learning rate mai mic
   ```bash
   python3 train_wbc_ppo.py --timesteps 200000 --lr 1e-4 \
       --save-dir models/wbc_ppo_finetune
   ```

2. **Transfer Learning**: Folosește model antrenat ca starting point
3. **Reward Shaping**: Ajustează reward function pentru obiective specifice
4. **Domain Randomization**: Adaugă variație în masă, teren, etc.

## ⚡ Tips pentru Training Mai Rapid

1. **Reduce max_steps per episod**: Din 1000 în 500 (în `wbc_tuning_env.py`)
2. **Folosește mai multe CPU cores**: `--envs 8` (dacă ai RAM suficient)
3. **Early stopping**: Oprește când reward plateaus
4. **Warmstart**: Pornește de la parametri aproape de soluție

## 📝 Citation

Dacă folosești acest sistem în research, citează:
```
PPO WBC Auto-Tuning System
Humanoid Robot Walking Controller Optimization
ROS 2 Jazzy + MuJoCo + Stable-Baselines3
```

---

**Succes cu training-ul! 🚀**

Pentru întrebări sau probleme, deschide un issue.
