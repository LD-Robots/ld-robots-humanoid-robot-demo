# 🎥 Ghid Complet de Vizualizare RL Training

Da, poți vedea **tot ce face** robotul în timpul training-ului! Aici sunt **TOATE** metodele de vizualizare.

---

## 🎯 Metoda 1: TensorBoard - Grafice Real-Time (RECOMANDAT)

### Ce vezi:
- 📈 Reward curve (performance în timp)
- 📊 Episode length (cât supraviețuiește)
- 🎯 Success rate
- 📉 Loss curves (policy & value)
- ⚡ Learning rate

### Cum folosești:

**În timpul training-ului, deschide ALT TERMINAL:**

```bash
cd src/control/wbc_pinocchio_controller/scripts/
tensorboard --logdir models/wbc_ppo/logs/
```

**Deschide browser:** http://localhost:6006

**Live updates** la fiecare 30 secunde!

### Screenshot exemplu:
```
Reward Curve:
  Episode 100:  -50.2
  Episode 500:   45.8  ← Învață!
  Episode 1000: 125.3  ← Mult mai bine!
  Episode 2000: 245.7  ← Excelent!
```

---

## 🤖 Metoda 2: Watch Latest Checkpoint (Vezi robotul ACUM)

### Ce vezi:
- Fereastra MuJoCo cu robot 3D
- Robotul încercând să meargă
- Camera urmărește robotul
- Real-time simulation

### Cum folosești:

**În alt terminal (în timpul training-ului):**

```bash
python3 watch_training.py
```

Asta va găsi **automat** ultimul checkpoint și îl va arăta live!

**Sau specific checkpoint:**
```bash
python3 watch_training.py --model models/wbc_ppo/checkpoints/wbc_ppo_10000_steps.zip
```

**Controale MuJoCo Viewer:**
- **Mouse drag**: Rotește camera
- **Scroll**: Zoom in/out
- **Double-click**: Centrează pe robot
- **Right-click drag**: Pan camera
- **ESC**: Închide viewer

### Opțiuni:
```bash
# Watch 10 episodes
python3 watch_training.py --episodes 10

# Watch specific checkpoint
python3 watch_training.py --model models/wbc_ppo/best_model/best_model.zip

# Watch random policy (cum arată fără training)
python3 watch_training.py --random
```

---

## 🔄 Metoda 3: Live Monitor - Auto-update (CEL MAI COOL!)

### Ce face:
- La fiecare 5 minute, verifică dacă e checkpoint nou
- Dacă da, **automat** arată robotul cu noul checkpoint
- Vezi **progresul în timp real** fără să faci nimic!

### Cum folosești:

**În alt terminal (lasă-l să ruleze):**

```bash
python3 live_monitor.py
```

**Exemplu output:**
```
[1] No checkpoints yet, waiting...
[2] NEW CHECKPOINT: wbc_ppo_10000_steps.zip
    → Opens MuJoCo viewer and shows 2 episodes
[3] No new checkpoint. Latest: wbc_ppo_10000_steps.zip
[4] NEW CHECKPOINT: wbc_ppo_20000_steps.zip
    → Shows improved performance!
```

**Customizare:**
```bash
# Check every 2 minutes (120s)
python3 live_monitor.py --interval 120

# Watch 5 episodes per checkpoint
python3 live_monitor.py --episodes 5

# Custom directory
python3 live_monitor.py --dir models/custom_ppo/checkpoints
```

---

## 📊 Metoda 4: Console Output - Text Progress

Training script-ul afișează progres în consolă:

```
Episode 100/5000: reward=45.2, length=234, success=True
Episode 200/5000: reward=78.5, length=456, success=True
Episode 300/5000: reward=124.3, length=789, success=True
...
```

Plus **progress bar**:
```
Training: |████████████████------------------| 45% (2250/5000 episodes)
```

---

## 🎬 Setup Complet - Vezi TOT simultan

### Terminal 1: Training
```bash
cd src/control/wbc_pinocchio_controller/scripts/
python3 train_wbc_ppo.py --timesteps 500000 --envs 4
```

### Terminal 2: TensorBoard
```bash
cd src/control/wbc_pinocchio_controller/scripts/
tensorboard --logdir models/wbc_ppo/logs/
```
→ Deschide http://localhost:6006

### Terminal 3: Live Monitor
```bash
cd src/control/wbc_pinocchio_controller/scripts/
python3 live_monitor.py --interval 300
```

### Browser: TensorBoard Dashboard
→ http://localhost:6006

**Acum vezi TOTUL:**
- ✅ Text progress în Terminal 1
- ✅ Grafice live în Browser
- ✅ Robot 3D în Terminal 3 (la fiecare 5 min)

---

## 🎯 Ce să cauți în vizualizări:

### TensorBoard:

**Episode Reward (cel mai important):**
```
Bine:  Trend crescător ↗
Rău:   Flat sau descrescător →/↘
```

**Episode Length:**
```
Bine:  Crește în timp (robotul supraviețuiește mai mult)
Rău:   Rămâne constant mic
```

**Value Loss:**
```
Bine:  Scade treptat
Rău:   Explodează sau oscilează mult
```

### MuJoCo Viewer:

**Primele 100k steps:**
- Robot cade imediat
- Mișcări haotice
- Normal! Învață ce să NU facă

**După 200k steps:**
- Face 5-10 pași înainte să cadă
- Încă instabil dar progres vizibil

**După 500k steps:**
- Face 20-50+ pași
- Mers relativ stabil
- Încă cade ocazional

**După 1M steps:**
- Walking smooth și consistent
- 50-100+ pași fără cădere
- Poate merge "la infinit"

---

## 📸 Înregistrare Video (Bonus)

Vrei să salvezi video cu progresul?

```bash
# Install screen recording
sudo apt install kazam  # sau SimpleScreenRecorder

# Record MuJoCo window
kazam
```

Sau direct din Python:

```python
# În wbc_tuning_env.py, adaugă:
import cv2

# În step():
frame = self.sim.render()  # Get MuJoCo frame
cv2.imwrite(f"frames/frame_{self.step_count:06d}.png", frame)

# Apoi creează video:
# ffmpeg -framerate 30 -i frames/frame_%06d.png output.mp4
```

---

## 🐛 Troubleshooting Vizualizare

### "Cannot open MuJoCo viewer"

**Soluție 1:** Verifică DISPLAY
```bash
echo $DISPLAY  # Should be :0 or :1
export DISPLAY=:0
```

**Soluție 2:** Dezactivează headless mode
```python
# În wbc_tuning_env.py:
env = WbcTuningEnv(render_mode="human")  # NOT None
```

### "TensorBoard not updating"

Refresh page în browser (F5) sau:
```bash
# Restart TensorBoard
pkill tensorboard
tensorboard --logdir models/wbc_ppo/logs/ --reload_interval 10
```

### "Too many MuJoCo windows open"

```bash
# Close all MuJoCo windows
pkill -9 mujoco
pkill -9 python3
```

### "Viewer is laggy"

Reduce frame rate:
```python
# În watch_training.py:
time.sleep(0.05)  # 20 Hz instead of 100 Hz
```

---

## 🎓 Best Practices

### Pentru monitoring eficient:

1. **TensorBoard**: Lasă-l deschis MEREU
   - Verifică la fiecare 30 min dacă reward crește
   - Dacă stagnează după 100k steps → oprește training

2. **Live Monitor**: Folosește-l ocazional
   - Nu la fiecare 5 min (consumă resurse)
   - La fiecare 30-60 min e suficient
   - Sau manual când vezi spike în reward

3. **Console Output**: Perfect pentru quick checks
   - Vezi imediat dacă rulează
   - Success rate în ultimele episoade

### Workflow recomandat:

**Primele 2 ore:**
- Check TensorBoard la fiecare 30 min
- Watch checkpoint la 1h și 2h
- Asigură-te că reward crește

**După primele 2 ore:**
- Lasă training să ruleze peste noapte
- TensorBoard pe al 2-lea monitor (dacă ai)
- Dimineața: watch best_model

---

## 🚀 Quick Commands

```bash
# Watch random policy (baseline)
python3 watch_training.py --random --episodes 3

# Watch latest checkpoint
python3 watch_training.py

# Watch best model
python3 watch_training.py --model models/wbc_ppo/best_model/best_model.zip

# Monitor training (auto-watch new checkpoints)
python3 live_monitor.py --interval 300

# TensorBoard
tensorboard --logdir models/wbc_ppo/logs/
```

---

**Acum ai vizualizare completă! Vezi exact ce învață agentul și cum se îmbunătățește! 🎉**
