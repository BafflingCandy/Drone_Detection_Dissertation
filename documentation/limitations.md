# Scope and limitations

- The protected facility and its dimensions are fictional modelling assumptions.
- Friis free-space propagation does not represent walls, reflections, multipath, vegetation, terrain, weather, receiver calibration, or ambient emitters.
- RSSI and AoA noise is synthetically controlled rather than calibrated against the proposed hardware.
- A passive RF system cannot detect a non-emitting or radio-silent drone.
- The 100 deterministic routes provide broad, repeatable scenario coverage but are not exhaustive.
- Only five complete standard and five tiered-threshold ns-3 random-seed RF outputs are archived in the core corpus. The tiered profile changes secure-wall sensitivity; it does not add sensors.
- Sensor costs, antenna patterns, legal placement constraints, and prison operations were not physically validated.
- Simulink results are representative time-domain validation, not a replacement for multi-path statistical evaluation.
- The saved Simulink hybrid model uses an earlier Kalman parameter candidate (acceleration/measurement standard deviations 0.2/3) rather than the later refined MATLAB selection (0.5/5), so their numerical results are not a like-for-like replication.
- The upstream FlyNetSim stack is based on historical dependencies and may be difficult to rebuild on a modern host.
- Reported simulation performance should not be interpreted as deployment readiness.
