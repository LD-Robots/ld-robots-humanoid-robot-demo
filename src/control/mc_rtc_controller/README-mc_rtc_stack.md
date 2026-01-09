# mc_rtc + mc_mujoco stack (CMake)

This README documents a local build/install workflow for the mc_rtc stack and mc_mujoco using CMake.

## Repositories

Clone these repos (or update existing checkouts).

With submodules (use `--recursive`):
- SpaceVecAlg (https://github.com/jrl-umi3218/SpaceVecAlg)
- RBDyn (https://github.com/jrl-umi3218/RBDyn)
- sch-core (https://github.com/jrl-umi3218/sch-core)
- Tasks (https://github.com/jrl-umi3218/Tasks)
- TVM (https://github.com/jrl-umi3218/TVM)
- eigen-qld (https://github.com/jrl-umi3218/eigen-qld)
- eigen-quadprog (https://github.com/jrl-umi3218/eigen-quadprog)
- ndcurves (https://github.com/loco-3d/ndcurves)
- state-observation (https://github.com/jrl-umi3218/state-observation)
- mc_rtc (https://github.com/jrl-umi3218/mc_rtc)
- mc_mujoco (git@github.com:rohanpsingh/mc_mujoco.git)

Without submodules:
- mc_env_description (https://github.com/jrl-umi3218/mc_env_description)
- mc_int_obj_description (https://github.com/jrl-umi3218/mc_int_obj_description)
- jvrc_description (https://github.com/jrl-umi3218/jvrc_description)

Example commands:

```bash
git clone --recursive <repo_url>
# or if already cloned
git submodule update --init --recursive
```

## Build order (CMake)

Build and install in this order to satisfy dependencies:

1) SpaceVecAlg
2) RBDyn
3) sch-core
4) Tasks
5) TVM
6) eigen-qld
7) eigen-quadprog
8) ndcurves
9) state-observation
10) mc_rtc
11) mc_mujoco
12) mc_env_description
13) mc_int_obj_description
14) jvrc_description

## Build + install template

Use the same commands for each repo (adjust the path):

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j
sudo cmake --install build
```

If you rebuild a dependency, re-run the build/install for dependents.

## mc_rtc configuration

After install, mc_rtc reads:

- `~/.config/mc_rtc/mc_rtc.yaml`
- `~/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml`

In this project, we keep local configs under:

- `src/control/mc_rtc_controller/config/`

and link them using:

```bash
cd src/control/mc_rtc_controller/scripts
sudo ./install.sh
```

## Run mc_mujoco

From the controller scripts folder (to keep relative paths working):

```bash
cd src/control/mc_rtc_controller/scripts
mc_mujoco --sync
```

## Notes

- `G1.yaml` is linked into `/usr/local/share/mc_mujoco/G1.yaml` by `install.sh`.
- The MuJoCo model `g1.xml` lives under `/usr/local/share/mc_mujoco/G1/xml/` by default.
- If you change installed libraries in `/usr/local`, restart any running tools that use them.
