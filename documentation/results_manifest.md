# Results manifest

This manifest maps the principal claims to source data, analysis, and output figures.

| Evaluation component | Primary inputs | Main analysis | Evidence/output |
|---|---|---|---|
| 100-route scenario coverage | Seeds 1–100 and prison geometry | `generate_100_seed_waypoint_tables.m` | `data/100_seed_scenarios/`; `figures/scenario_coverage/` |
| Standard and tiered-threshold RF detection | Ten raw ns-3 CSVs | Prison ns-3 scenario and MATLAB localisation scripts | `data/ns3_rf_measurements/` |
| Exploratory optimised directional layout | Raw runs from the 24-sensor prison scenario | `aoa_opt_kalman.m` | `aoa_opt_sensor_layout.csv`; optimised-layout and error figures |
| Fixed 24-directional-sensor prison layout | Raw RF runs | `aoa_fence24_kalman.m` | `aoa_fence24_kalman_summary.csv`; all-directional layout figure |
| WLS and robust-WLS localisation | Raw standard runs | WLS/robust-WLS scripts and helper functions | Corresponding compact summary CSVs |
| Hybrid 16-directional/8-omnidirectional layout with Kalman filtering | Raw standard runs | `tune_hybrid_kalman_fence24.m` | Refined tuning, validation result, estimate CSVs; validation figures |
| All-directional Simulink validation | Saved Simulink inputs | `prison_rf_sensor_localisation_model.slx` | Directional model result/summary CSVs and figures |
| Hybrid-layout Kalman-filter Simulink validation | Saved Simulink inputs | `prison_rf_hybrid_kalman_model.slx` | Hybrid model result/summary CSVs and figures |

The refined MATLAB tuning and representative Simulink validation both use acceleration and measurement standard deviations of 0.5 and 5, a −70 dBm hybrid threshold, 0.02 omnidirectional-measurement weight, and an 80 m residual gate. The Simulink run uses the selected seed-4 validation path rather than repeating the complete multi-route tuning experiment.

## Interpretation rule

The route tables establish repeatable geometries. The ten archived ns-3 files contain complete RF observations for five standard and five tiered-threshold seed runs. In the tiered-threshold profile, inner secure-wall sensors use a more sensitive −70 dBm threshold while outer-perimeter sensors retain the configured experiment threshold. Processed MATLAB results may contain repeated noise trials and held-out validation calculations derived from those observations. These categories are deliberately separated in the corpus to prevent a route definition from being mistaken for a completed RF run.
