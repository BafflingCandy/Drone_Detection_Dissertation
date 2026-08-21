# Testing and validation record

The saved corpus documents the following validation layers:

1. **Range baseline:** packet reception counters and distance logging were added to verify communication behaviour as separation increased.
2. **Friis baseline:** a simple passive RF model checked distance-to-received-power behaviour and thresholded detections across alternative small layouts.
3. **Prison scenario:** deterministic paths, sensor layouts, zone classification, secure-boundary crossing, and pre-boundary alerts were logged to CSV.
4. **Repeatable route coverage:** seeds 1–100 were regenerated independently in MATLAB using matching pseudo-random and waypoint logic.
5. **Noise sensitivity:** controlled RSSI/AoA noise trials compared localisation methods.
6. **Train/validation separation:** parameters for the hybrid 16-directional/8-omnidirectional layout with Kalman filtering were tuned on designated routes and evaluated on held-out seed routes.
7. **Independent implementation form:** the 24-directional-sensor and hybrid Kalman-filtered processing pipelines were represented in Simulink and compared with the MATLAB conclusions.

Saved raw data, compact summaries, models, and figures are retained so these checks can be inspected without depending exclusively on prose in the dissertation.
