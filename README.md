# State Estimation

This repository is a C++/Eigen IMU+GPS Error-State Kalman Filter baseline for ego-state estimation.

## What is in this repo

- A C++ ESKF with deterministic initialization, IMU prediction, and 2D GPS updates.
- In-process GPS generation from pose data.
- A bundled pose/IMU pair so the app runs out of the box.
- Unit tests and an end-to-end integration test.

## Supported inputs and preparing your run

If you'd like to use the simulation-built data bundled with this repo, then the project can be built and run as is.

This repo supports the bundled `scene_pose.json` / `scene_ms_imu.json` pair and `scene-XXXX_pose.json` / `scene-XXXX_ms_imu.json` pairs from the nuScenes CAN bus data published February 2020 by nuScenes [which you have to download separately from this repo]. No nuScenes data is shipped here. See [the input files docs](https://jasonmreich.com/docs/reference/input-files) for more details if you plan on using your own copy of the nuScenes CAN bus data.

## Disclaimers

- This repository is an educational demo only, it is not for safety-critical use.
- This project is not affiliated with, endorsed by, or sponsored by nuScenes or Motional.
- No nuScenes/Motional data or content is included or redistributed here.
- A Python simulation was built to create synthetic data for this project. The synthetic data is included in `scenarios` and is sufficient for running all of the code in this repo.

## Build and run

Start here:

```bash
git clone https://github.com/jreich218/state-estimation.git
cd state-estimation
```

The local package-install commands below assume Debian or Ubuntu.

### 1. Build locally

```bash
sudo apt-get update
sudo apt-get install -y cmake g++ libeigen3-dev nlohmann-json3-dev
```

Run this from the repo root:

```bash
./run.sh
```

### 2. Build in a dev container

Install what this path needs on the host:

- Docker
- Visual Studio Code
- the Dev Containers extension for Visual Studio Code

Open the cloned folder in Visual Studio Code, then run `Dev Containers: Reopen in Container`.

Once the container opens, run these from the repo root inside the container.
Visual Studio Code should open the terminal there automatically.

```bash
./run.sh
```

## Output

- Then main output is `output/eskf_sim_log.csv`. Each row is one GPS update. It records the timestamp, the filter position estimate, the GPS measurement, the reference pose, the GPS innovation, the NIS, and the resulting position error so you can inspect how the filter behaved over the run.
- The app also prints a short run summary to stdout, including the number of GPS updates and the xy RMSE for raw GPS and for the ESKF.
- The app creates and uses synthetic GPS data at runtime from the selected pose stream. The generated samples are written to `output/gps.json` for optional inspection.
