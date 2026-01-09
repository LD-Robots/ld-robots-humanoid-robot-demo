# G1 CoM + Posture mc_rtc controller

Minimal mc_rtc controller for G1 that adds a CoM task and a Posture task.

## Build plugin

```bash
cmake -S . -B build
cmake --build build
```

The library is created in `build/` as `G1CoMPosture.so`.

## Configure mc_rtc

Add this to your mc_rtc config:

```yaml
MainRobot: G1
Enabled: [G1CoMPosture]
ControllerModulePaths: ["/home/alex/Documents/GitHub/ld-robots-humanoid-robot-demo/src/control/mc_rtc_controller/build"]
```

To keep the mc_rtc config in this repo, run:

```bash
cd ld-robots-humanoid-robot-demo/src/control/mc_rtc_controller/scripts
./install.sh
```

This script backs up any existing `~/.config/mc_rtc/mc_rtc.yaml` (timestamped)
and replaces it with a symlink to `config/mc_rtc.yaml`.

It also links `config/G1.yaml` to `/usr/local/share/mc_mujoco/G1.yaml` (may require sudo).
It also links `config/g1.json` to `~/.config/mc_rtc/robots/g1.json` and
`config/g1.rsdf` to `~/.config/mc_rtc/robots/rsdf/G1/g1.rsdf`.

## Run mc_mujoco

```bash
mc_mujoco --sync
```

If `ControllerModulePaths` is relative (e.g. `["../build"]`), either:
- run `mc_mujoco --sync` from the `scripts` folder, or
- use an absolute path in `ControllerModulePaths`.
