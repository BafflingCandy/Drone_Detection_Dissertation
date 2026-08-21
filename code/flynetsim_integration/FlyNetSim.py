from __future__ import print_function
import os
import signal
import subprocess
import threading
import argparse
import time
import xml.etree.cElementTree as Et
import zmq
import gcs_pubsub as gcs
import uav_pubsub as uav


def create_zmq(zmq_type, con_string, prefix="", verbose=False):
    context = zmq.Context()
    if "PUB" in zmq_type:
        if verbose:
            print("[MAIN] [ZMQ] Binding publisher started " + con_string)
        sock_new = context.socket(zmq.PUB)
        sock_new.bind(con_string)
        if verbose:
            print("[MAIN] [ZMQ] Publisher bound complete " + con_string)
    elif "SUB" in zmq_type:
        if verbose:
            print("[MAIN] [ZMQ] Subscriber connect started " + con_string)
        sock_new = context.socket(zmq.SUB)
        sock_new.connect(con_string)
        sock_new.setsockopt(zmq.SUBSCRIBE, prefix)
        if verbose:
            print("[MAIN] [ZMQ] Subscriber connect complete " + con_string + " Prefix " + prefix)
    else:
        return None
    return sock_new


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="FlyNetSim: An Open Source UAV Network Simulator")
    parser.add_argument("-i", "--instance", help="Number of UAV instances (Default: 1)",
                        type=int, default=1)
    parser.add_argument("-p", "--path", help="Path to the configured ns-3.27 directory (or set FLYNETSIM_NS3_PATH)",
                        type=str, default=os.environ.get("FLYNETSIM_NS3_PATH", ""))
    parser.add_argument("-c", "--control", help="Type of Control (Default: 0): 0 - Individual, 1 - Group",
                        type=int, choices=[0, 1], default=0)
    parser.add_argument("-l", "--layout", help="Layout for multiple UAVs (Default: 0): 0 - Linear, 1 - Grid",
                        type=int, choices=[0, 1], default=0)
    parser.add_argument("-s", "--simple", help="Linear Layout: X-distance (m)",
                        type=int, default=5)
    parser.add_argument("-g", "--grid", help="Grid Layout: Width  X-distance  Y-distance",
                        type=int, nargs=3, default=[4, 5, 5])
    parser.add_argument("-n", "--network", help="Network type (Default: 0): 0 - Wifi, 1 - LTE, 2 - Multi-network",
                        type=int, choices=[0, 1, 2], default=0)
    parser.add_argument("-t", "--traffic", help="Create external traffic on the path: NodeCount TrafficRate PacketSize",
                        type=int, nargs=3, default=[0, 0, 0])
    parser.add_argument("--scenario", help="ns-3 scenario target to run: friis or prison",
                        type=str, choices=["friis", "prison"], default="friis")
    parser.add_argument("--sensor-layout", help="RF sensor layout for Friis test: corner, perimeter, gate, or random",
                        type=str, choices=["corner", "perimeter", "gate", "random"], default="corner")
    parser.add_argument("--prison-sensor-layout", help="RF sensor layout for prison scenario: perimeter8, cluster16, station24, or fence24",
                        type=str, choices=["perimeter8", "cluster16", "station24", "fence24"], default="perimeter8")
    parser.add_argument("--sensor-profile", help="Prison RF threshold profile: uniform or tiered_wall",
                        type=str, choices=["uniform", "tiered_wall"], default="uniform")
    parser.add_argument("--drone-path", help="Prison drone path for repeatable sensor experiments",
                        type=str,
                        choices=["west_midpoint", "west_offset_low", "west_offset_high", "south_midpoint",
                                 "north_midpoint", "east_midpoint", "diagonal_sw", "diagonal_nw",
                                 "random_waypoint"],
                        default="west_midpoint")
    parser.add_argument("--random-seed", help="Seed for reproducible random_waypoint prison drone path",
                        type=int, default=1)
    parser.add_argument("--random-target-x", help="Vulnerable target X coordinate for random_waypoint path",
                        type=float, default=560.0)
    parser.add_argument("--random-target-y", help="Vulnerable target Y coordinate for random_waypoint path",
                        type=float, default=520.0)
    parser.add_argument("--rf-threshold", help="RF detection threshold in dBm for prison sensor scenario",
                        type=float, default=-60.0)
    parser.add_argument("-v", "--verbose", help="Increase output verbosity. Levels: 0-Disable, 1-GCS, 2-UAVs, 3-Both",
                        type=int, choices=[0, 1, 2, 3], default=0)
    args = parser.parse_args()
    if not args.path:
        parser.error("Provide the ns-3.27 directory with --path or FLYNETSIM_NS3_PATH")
    args.path = os.path.abspath(os.path.expanduser(args.path))

    xml_root = Et.Element("simulator")
    Et.SubElement(xml_root, "instance").text = str(args.instance)
    Et.SubElement(xml_root, "network").text = str(args.network)

    xml_traffic = Et.SubElement(xml_root, "traffic")
    Et.SubElement(xml_traffic, "count").text = str(args.traffic[0])
    Et.SubElement(xml_traffic, "rate").text = str(args.traffic[1])
    Et.SubElement(xml_traffic, "size").text = str(args.traffic[2])

    xml_vehicle = Et.SubElement(xml_root, "vehicle")
    for u in range(args.instance):
        xml_uav = Et.SubElement(xml_vehicle, "uav", name=str(u))
        Et.SubElement(xml_uav, "x").text = str(args.simple + (u-1)*5)
        Et.SubElement(xml_uav, "y").text = "0"
        Et.SubElement(xml_uav, "z").text = "10"

    tree = Et.ElementTree(xml_root)
    if args.path.startswith('~'):
        home_path = os.path.abspath('../../../')
        xml_path = home_path + args.path[1:]
    else:
        xml_path = args.path
    if xml_path.endswith('/'):
        tree.write(xml_path + "config.xml")
    else:
        tree.write(xml_path + "/config.xml")


    proc_instance = []
    uav_obj = []
    uav_id = []
    gcs_obj = None

    time.sleep(0.1)
    if args.scenario == "prison":
        # PRISON SENSOR SCENARIO:
        # Run the dissertation-focused prison sensor target with configurable
        # RF threshold, sensor layout, and deterministic drone path.
        ns_run = ("uav-net-sim-prison-sensor --rfThresholdDbm=" + str(args.rf_threshold) +
                  " --sensorLayout=" + args.prison_sensor_layout +
                  " --sensorProfile=" + args.sensor_profile +
                  " --dronePath=" + args.drone_path +
                  " --randomSeed=" + str(args.random_seed) +
                  " --randomTargetX=" + str(args.random_target_x) +
                  " --randomTargetY=" + str(args.random_target_y))
    else:
        # RF SENSOR ADDITION:
        # Keep the historical Friis layout target available for comparison/debugging.
        ns_run = "uav-net-sim-friis-test --sensorLayout=" + args.sensor_layout
    log_path = os.environ.get("FLYNETSIM_LOG_PATH", "ns3_run.log")
    ns_cmd = "xterm -T Network_Simulator -e 'cd " + args.path + " && ./waf --run=\"" + ns_run + "\" | tee " + log_path + "'"
    print("[MAIN] Starting the network simulator: " + ns_cmd)
    proc_instance.append(subprocess.Popen(ns_cmd, shell=True))
    time.sleep(3)
    print("[MAIN] Started the network simulator: " + ns_cmd)

    # Common Publisher for all the UAVs
    uav_zmq_tel_connection_str = "tcp://127.0.0.1:5600"  # NS-3
    # uav_zmq_tel_connection_str = "tcp://127.0.0.1:5551"  # DIRECT
    uav_zmq_tel_socket = create_zmq("PUB", uav_zmq_tel_connection_str, verbose=True)

    for i in range(args.instance):
        p_sitl = subprocess.Popen("xterm -T SITL_" + str(i) +
                                  " -e dronekit-sitl copter --instance " + str(i), shell=True)
        proc_instance.append(p_sitl)
        time.sleep(2)
        uav_id = format(i, "03d")  # type: str

        # Seperate Subcribers with individual filters
        uav_zmq_control_connection_str = "tcp://127.0.0.1:5601"  # NS-3
        # uav_zmq_control_connection_str = "tcp://127.0.0.1:5550"  # DIRECT
        ftr = "@@@G_" + uav_id
        uav_zmq_control_socket = create_zmq("SUB", uav_zmq_control_connection_str, ftr, True)
        if args.verbose == 2 or args.verbose == 3:
            ver = True
        else:
            ver = False
        uav_thread = threading.Thread(target=uav.UAV, args=(format(i, "03d"), (5760 + i * 10),
                                                            uav_zmq_tel_socket, uav_zmq_control_socket, ver))  # DIRECT
        uav_thread.setName("UAV_" + uav_id)
        uav_thread.deamon = True
        uav_thread.start()
        uav_obj.append(uav_thread)

    if args.verbose == 1 or args.verbose == 3:
        ver = True
    else:
        ver = False

    gcs.main(5501, 5500, args.instance, ver)  # NS3
    # gcs.main(5551, 5550, args.verbose)  # DIRECT

    print("[MAIN] Terminating the SITL instances")
    for p in proc_instance:
        print("[MAIN] Killing process:", p.pid)
        os.kill(p.pid, 0)
        p.kill()
        os.killpg(os.getpgid(p.pid+1), signal.SIGTERM)
