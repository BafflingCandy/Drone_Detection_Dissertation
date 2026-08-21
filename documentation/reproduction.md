# Reproduction guide

The repository supports two levels of reproduction: analysis from saved CSVs, and full legacy FlyNetSim/ns-3 simulation.

## 1. Inspect or regenerate the 100-seed route suite

Requirements:

- MATLAB with table, plotting, and `exportgraphics` support.

Run `code/matlab/scenario_generation/generate_100_seed_waypoint_tables.m`. The script resolves the repository location automatically and writes regenerated tables and figures under `reproduced_output/100_seed_scenarios/`.

The pseudo-random generator intentionally mirrors the 32-bit linear-congruential generator in the ns-3 prison scenario. Seed `N` therefore recreates the route selected by `--randomSeed=N` with the default target.

## 2. Re-run analysis from saved ns-3 observations

1. Keep the submitted directory structure intact so the shared `code/matlab/corpus_paths.m` helper can locate inputs.
2. Run the desired stages:
   - localisation baselines in `code/matlab/localisation/`;
   - layout and AoA/RSSI comparisons in `code/matlab/sensor_layouts/`;
   - Simulink dataset preparation/build scripts in `code/matlab/simulink/`.
3. Compare regenerated summary CSVs and PNGs under `reproduced_output/` with the submitted evidence in `data/processed_results/` and `figures/`.

The curated scripts contain no machine-specific drive letters or user-directory paths. Inputs are resolved from the repository and regenerated outputs are kept separate from submitted evidence.

## 3. Inspect Simulink validation

Open the models in `models/` with a compatible MATLAB/Simulink release. Their saved measurement inputs, reference estimates, outputs, and summary tables are under `data/simulink/`.

The all-directional and hybrid models are representative time-domain validations. They are not a statistical substitute for the multi-path MATLAB evaluation.

## 4. Rebuild the FlyNetSim/ns-3 experiment

The historical upstream environment used Ubuntu 16.04, Python 2.7, ns-3.27, ArduPilot SITL, CZMQ/ZeroMQ, libxml2, and PyQt4. Consult `third_party/flynetsim_attribution.md`, install the upstream FlyNetSim stack, and then:

1. Place a selected directory from `code/ns3/` in the ns-3.27 `scratch/` directory.
2. Replace the upstream FlyNetSim launcher with, or apply the changes from, `code/flynetsim_integration/FlyNetSim.py`.
3. Build ns-3 with its `waf` tool.
4. Run a prison scenario, for example:

```bash
python FlyNetSim.py \
  --scenario prison \
  --prison-sensor-layout fence24 \
  --sensor-profile uniform \
  --drone-path random_waypoint \
  --random-seed 1 \
  --random-target-x 560 \
  --random-target-y 520 \
  --rf-threshold -60
```

The ns-3 target writes `prison_sensor_detections.csv`. Archive it with a filename that records the layout, profile, seed, and threshold.

## Expected limitations

Rebuilding the legacy simulator is environment-sensitive. The saved data is included so markers can inspect and reproduce the analysis even if the historical Python 2/ns-3.27/ArduPilot stack cannot be rebuilt on a current operating system.
