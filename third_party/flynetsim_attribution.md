# FlyNetSim integration

FlyNetSim supplied the original coupling between ArduPilot SITL, ns-3, ZeroMQ, and the ground-control/UAV processes. The project used that framework as the simulation foundation and added RF sensing experiments rather than creating the complete simulator.

The curated corpus contains only the modified launcher and the scenario source needed to show the project contribution. It does not bundle ArduPilot, the full ns-3.27 tree, compiled libraries, or FlyNetSim build output.

## Upstream baseline

- FlyNetSim repository: <https://github.com/saburhb/FlyNetSim>
- FlyNetSim paper: <https://doi.org/10.1145/3242102.3242118>
- Historical FlyNetSim environment: Ubuntu 16.04, Python 2.7, ns-3.27, ArduPilot SITL, CZMQ/ZeroMQ, libxml2, and PyQt4.

Because the upstream stack is historical, exact environment recreation may require a compatible Ubuntu virtual machine or container. The saved CSV datasets permit the MATLAB and Simulink analysis to be inspected without rebuilding that legacy stack.
