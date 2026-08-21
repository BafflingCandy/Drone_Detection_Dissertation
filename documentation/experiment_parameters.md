# Experiment parameters

## Site geometry

| Parameter | Value |
|---|---:|
| Simulation area | 1,000 m × 1,000 m |
| Secure area X bounds | 300–700 m |
| Secure area Y bounds | 350–650 m |
| Clear-zone width | 50 m |
| Drone altitude | 20 m |
| Drone speed | 10 m/s |
| Common target | (560 m, 520 m, 20 m) |

The site is fictional. Dimensions are modelling assumptions, not measurements from a real prison.

## RF model

| Parameter | Baseline value |
|---|---:|
| Carrier frequency | 2.4 GHz |
| Transmit power | 20 dBm |
| Transmit gain | 0 dBi |
| Sensor gain | 0 dBi |
| Standard prison threshold | Configurable; saved core runs use −60 dBm |
| Secure-wall tier threshold | −70 dBm in the tiered profile |
| Propagation | Friis free-space relationship |

## Route suite

- Seeds: 1–100
- Entry classes: four cardinal and four diagonal/corner regions
- Intermediate waypoints: three
- Route target: common protected point
- Path sampling for coverage tables: approximately every 10 m
- Generator: `code/matlab/scenario_generation/generate_100_seed_waypoint_tables.m`

## Final sensing configurations

- **24-sensor all-directional prison layout (`fence24`):** 24 outward-facing directional sensors positioned around the outer fence and secure wall. Each supplies RSSI and an AoA bearing when the drone lies within its field of view.
- **24-sensor hybrid prison layout:** the same physical placement, but with 16 directional RSSI/AoA sensors and 8 omnidirectional RSSI anchors. The omnidirectional anchors improve observation continuity after the drone passes an outward-facing sensor sector.
- **Hybrid layout with Kalman filtering:** weighted directional and omnidirectional measurements are passed to a constant-velocity Kalman filter. The filter predicts motion between measurements, smooths noise, and rejects updates whose residual exceeds the configured gate.

The exact sensor coordinates are stored in `data/sensor_layouts/` and the analysis parameters remain visible in the corresponding MATLAB scripts.
