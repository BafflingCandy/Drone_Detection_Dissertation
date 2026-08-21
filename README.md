# RF-Based Drone Detection and Localisation for a Protected Facility

This repository is the curated corpus for an individual university project investigating passive RF sensor layouts for detecting and localising an approaching drone around a simulated prison-style protected facility.

The project combines a modified FlyNetSim/ns-3 scenario with MATLAB and Simulink analysis. It compares a 24-sensor directional prison-perimeter layout with a hybrid 24-sensor layout containing 16 directional sensors and 8 omnidirectional RSSI anchors. The principal evaluation assets are a deterministic 100-seed ingress-route suite, raw ns-3 RF observations, localisation analyses, and time-domain Simulink validation.

Open [index.html](index.html) for the required item-by-item corpus index. See [documentation/reproduction.md](documentation/reproduction.md) for reproduction instructions and [documentation/results_manifest.md](documentation/results_manifest.md) for the relationship between data, scripts, and figures.

## Research objective

The project asks whether a passive, comparatively low-cost RF sensor network can provide useful warning and localisation of an emitting drone approaching a protected site. The evaluation focuses on:

- coverage before and across the secure boundary;
- localisation availability and error;
- the trade-off between directional accuracy and omnidirectional continuity;
- repeatability across varied approach directions; and
- whether a selected MATLAB pipeline can be represented consistently in Simulink.

This is a simulation study. It is not evidence of performance in a real prison, obstructed RF environment, or against a silent/non-emitting drone.

## System overview

1. **ArduPilot SITL and FlyNetSim** provide the original UAV/network co-simulation framework.
2. **ns-3 project scenarios** add prison geometry, sensor placement, deterministic paths, Friis received-power calculation, threshold detection, and CSV logging.
3. **The 100-seed route suite** supplies identical geographically varied ingress routes for controlled sensor-layout evaluation.
4. **MATLAB** applies controlled measurement-noise experiments, RSSI ranging, angle-of-arrival (AoA) localisation, weighted and robust least-squares position estimation, sensor-layout comparisons, and Kalman filtering to smooth successive position estimates.
5. **Simulink** implements representative time-domain versions of the 24-directional-sensor layout and the hybrid 16-directional/8-omnidirectional layout with Kalman filtering.

## What was added to FlyNetSim

FlyNetSim itself is third-party software and is not claimed as original work. Project-specific additions include:

- command-line selection of the Friis and prison scenarios;
- configurable RF thresholds, layouts, sensor profiles, drone paths, and random seeds;
- a modelled 1 km × 1 km environment containing a 400 m × 300 m secure area and a 50 m clear zone;
- deterministic cardinal and diagonal random-waypoint approaches;
- passive 2.4 GHz Friis received-power observations;
- standard and tiered-threshold sensor profiles;
- confirmed-detection, zone, boundary-time, and pre-boundary-alert logging; and
- CSV exports consumed by the MATLAB/Simulink workflow.

Only the relevant project-specific source is retained in this curated corpus. Obtain the upstream simulator and its dependencies from the sources in [third_party/flynetsim_attribution.md](third_party/flynetsim_attribution.md).

## 100-seed scenario dataset

The core scenario dataset defines 100 deterministic drone routes distributed across west, east, north, south, and the four corner ingress regions. Every route contains a start position, three intermediate waypoints, a secure-wall crossing, and a common protected target. Seeds map directly to the random-waypoint implementation in the ns-3 prison scenario.

Using identical routes allows candidate sensor layouts to be compared without route geometry changing between tests. The dataset provides systematic scenario coverage; it does **not** represent every physically possible continuous trajectory and must not be described as exhaustive.

The four representations are:

- [`wide`](data/100_seed_scenarios/random_waypoint_100_seed_wide.csv): one row per complete route;
- [`long`](data/100_seed_scenarios/random_waypoint_100_seed_long.csv): one row per waypoint;
- [`path samples`](data/100_seed_scenarios/random_waypoint_100_seed_path_samples.csv): positions sampled at approximately 10 m intervals;
- [`coverage summary`](data/100_seed_scenarios/random_waypoint_100_seed_coverage_summary.csv): boundary and zone coverage per seed.

The corpus also contains five complete ns-3 runs for the standard 24-sensor prison layout and five runs for its tiered-threshold variant, where secure-wall sensors use a different detection threshold from perimeter sensors. The 100-route tables define the broader reusable route suite; they must not be mistaken for 100 completed RF runs for every layout.

## Terminology used in the files

- **`fence24`:** the internal experiment name for a layout of 24 RF sensors placed around the modelled prison's outer fence and inner secure wall. It is a filename/configuration label, not a separate technology.
- **Directional sensor:** a sensor model that observes signals only inside its configured field of view and supplies both received signal strength (RSSI) and an angle-of-arrival (AoA) bearing.
- **Omnidirectional sensor:** a sensor model that can receive from every horizontal direction but supplies RSSI-derived range information rather than an AoA bearing.
- **Tiered-threshold (`tiered_wall`) profile:** a version of the 24-sensor layout in which the inner secure-wall sensors use a more sensitive −70 dBm threshold while the outer-perimeter sensors retain the experiment's configured threshold. This tests whether stronger sensitivity near the secure wall improves observation continuity. “Tiered” therefore refers to two threshold levels, not an additional physical layer of sensors.
- **Hybrid layout:** the 24-sensor arrangement containing 16 directional RSSI/AoA sensors and 8 omnidirectional RSSI anchors.
- **Kalman filtering:** a recursive tracking method that predicts the drone's next position from its recent motion and combines that prediction with new noisy measurements.
- **Hybrid Kalman configuration:** the hybrid sensor layout followed by weighted measurement fusion, constant-velocity Kalman filtering, and rejection of implausibly large measurement updates.

The refined multi-route MATLAB tuning selected acceleration/measurement standard-deviation parameters of 0.5 and 5. The representative Simulink validation uses the same Kalman parameters, together with the same −70 dBm threshold, 0.02 omnidirectional-measurement weight, and 80 m residual gate, on its selected validation path.

## Repository structure

| Path | Contents |
|---|---|
| `code/flynetsim_integration/` | Modified launcher connecting FlyNetSim to the project scenarios |
| `code/ns3/` | Friis baseline, range baseline, and prison RF scenario source |
| `code/matlab/` | Dataset generation, localisation, layout comparison, Kalman, and Simulink preparation scripts |
| `data/100_seed_scenarios/` | Deterministic 100-route evaluation suite |
| `data/ns3_rf_measurements/` | Raw standard and tiered-threshold ns-3 sensor observations |
| `data/sensor_layouts/` | Sensor coordinates and orientation/configuration tables |
| `data/processed_results/` | Compact analysis and validation results |
| `data/simulink/` | Inputs, reference estimates, results, and summaries |
| `figures/` | Selected final scenario, layout, localisation, and Simulink figures |
| `models/` | Two selected Simulink `.slx` models |
| `documentation/` | Parameters, reproduction, results mapping, testing, and limitations |
| `third_party/` | Attribution and upstream dependency information |

## Principal findings represented by the corpus

- The all-directional RSSI/AoA arrangement produced the lowest errors when localisation was available, but outward-facing fields of view reduced continuity after perimeter crossing.
- Replacing eight directional positions with omnidirectional RSSI anchors improved continuity without adding a second directional layer.
- Weighted fusion and a tuned constant-velocity Kalman filter—which predicts motion between measurements and smooths noisy position estimates—reduced the hybrid layout's error while retaining complete tracking on the evaluated validation paths.
- The Simulink models reproduce the main accuracy-versus-availability trade-off observed in the MATLAB evaluation.

Interpret these results within the assumptions and limitations documented in [documentation/limitations.md](documentation/limitations.md).

## Authorship

This is an individual-project corpus. Project-specific source, datasets, analysis scripts, models, documentation, and figures are attributed to **Pratyush**. Files adapted from FlyNetSim/ns-3 are identified as adaptations and remain subject to their upstream licences. See [LICENSES_AND_ATTRIBUTION.md](LICENSES_AND_ATTRIBUTION.md).

## Submission snapshot

For assessment, use a fixed GitHub commit or release rather than a moving branch. Downloading the release source ZIP preserves `index.html` and all its relative links. The dissertation, abstract, logbook, and video recording are intentionally excluded from this corpus in accordance with the submission instructions.
