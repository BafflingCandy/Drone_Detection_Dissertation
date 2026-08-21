
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector> 
#include <time.h>
#include <ctime>
#include <algorithm>
#include <czmq.h>
#ifdef LOG_INFO
#undef LOG_INFO
#endif

#ifdef LOG_DEBUG
#undef LOG_DEBUG
#endif

#ifdef LOG_WARNING
#undef LOG_WARNING
#endif

#ifdef LOG_ERROR
#undef LOG_ERROR
#endif

#include <libxml/parser.h>
#include <libxml/xmlIO.h>
#include <libxml/xinclude.h>
#include <libxml/tree.h>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/lte-module.h"
#include "ns3/point-to-point-module.h"
#include "myApps.h"
#include "myInput.h"
#include <cmath>


vector <Ptr<MyApp>> appVectCom;
vector <Ptr<MyApp>> appVectTel;

static uint64_t g_commandRx = 0;
static uint64_t g_telemetryRx = 0;

// PRISON SENSOR SCENARIO:
// Fictional medium/high-security rectangular correctional facility.
// Dimensions are modelling assumptions, not real prison measurements.
static const double SIM_MIN_X = 0.0;
static const double SIM_MAX_X = 1000.0;
static const double SIM_MIN_Y = 0.0;
static const double SIM_MAX_Y = 1000.0;
static const double PRISON_MIN_X = 300.0;
static const double PRISON_MAX_X = 700.0;
static const double PRISON_MIN_Y = 350.0;
static const double PRISON_MAX_Y = 650.0;
static const double PRISON_CLEAR_ZONE_M = 50.0;
static const double FENCE_MIN_X = PRISON_MIN_X - PRISON_CLEAR_ZONE_M;
static const double FENCE_MAX_X = PRISON_MAX_X + PRISON_CLEAR_ZONE_M;
static const double FENCE_MIN_Y = PRISON_MIN_Y - PRISON_CLEAR_ZONE_M;
static const double FENCE_MAX_Y = PRISON_MAX_Y + PRISON_CLEAR_ZONE_M;
static const double PRISON_SENSOR_HEIGHT_M = 15.0;
static const Vector PRISON_BASE_STATION_POS = Vector (500.0, 500.0, 20.0);
static const Vector PRISON_DRONE_START_POS = Vector (50.0, 500.0, 20.0);
static const Vector PRISON_DRONE_TARGET_POS = Vector (500.0, 500.0, 20.0);
static const double PRISON_DRONE_SPEED_MPS = 10.0;
static const double PRISON_CRITICAL_RADIUS_M = 50.0;
static const double PRISON_PERIMETER_TOLERANCE_M = 5.0;
static const double PRISON_PI = 3.14159265358979323846;

// PRISON SENSOR SCENARIO:
// RF assumptions for the passive sensor model. The threshold is configurable
// at runtime with --rfThresholdDbm. The optional tiered_wall profile keeps
// perimeter/fence sensors user-configurable while making the secure-wall
// midpoint sensors more sensitive for localisation inside the clear zone.
static const double RF_FREQUENCY_HZ = 2.4e9;
static const double RF_TX_POWER_DBM = 20.0;
static const double RF_TX_GAIN_DBI = 0.0;
static const double RF_SENSOR_GAIN_DBI = 0.0;
static const double RF_SECURE_WALL_THRESHOLD_DBM = -70.0;
static double g_rfDetectionThresholdDbm = -60.0;
static std::string g_sensorLayout = "perimeter8";
static std::string g_sensorProfile = "uniform";
static std::string g_dronePath = "west_midpoint";
static uint32_t g_randomSeed = 1;
static double g_randomTargetX = 560.0;
static double g_randomTargetY = 520.0;
static const char *RF_SENSOR_CSV = "prison_sensor_detections.csv";
static bool g_confirmedAlertRaised = false;
static bool g_useFixedPrisonPath = true;
static bool g_randomPathInitialized = false;
static std::vector<Vector> g_randomWaypoints;
static std::vector<double> g_randomCumulativeDistances;
static double g_randomTotalDistance = 0.0;
static double g_randomBoundaryDistance = 0.0;
static double g_randomLastDistanceAlongPath = 0.0;

// PRISON SENSOR SCENARIO:
// Keep CSV output one-run-at-a-time so repeated threshold runs do not mix.
static void
InitializeSensorCsv ()
{
  g_confirmedAlertRaised = false;
  std::ofstream csv;
  csv.open (RF_SENSOR_CSV, std::ios_base::trunc);
  csv << "time,sensor_layout,sensor_profile,drone_path,random_seed,random_target_x,random_target_y,uav_x,uav_y,uav_z,sensor_id,sensor_x,sensor_y,sensor_z,distance,rx_power_dbm,threshold_dbm,detected,num_sensors_detecting,confirmed_detection,inside_prison,zone,time_to_boundary,alert_before_boundary\n";
  csv.close ();
}

// PRISON SENSOR SCENARIO:
// Generic 3D Euclidean distance helper.
static double
CalculateDistanceMeters (Vector a, Vector b)
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;
  return std::sqrt (dx * dx + dy * dy + dz * dz);
}

// PRISON SENSOR SCENARIO:
// Estimate received power at a passive RF sensor using Friis path loss in dB.
static double
CalculateFriisRxPowerDbm (double distanceMeters)
{
  double safeDistance = std::max (distanceMeters, 1.0);
  double pathLossDb = 20.0 * std::log10 (safeDistance) +
                      20.0 * std::log10 (RF_FREQUENCY_HZ) -
                      147.55;
  return RF_TX_POWER_DBM + RF_TX_GAIN_DBI + RF_SENSOR_GAIN_DBI - pathLossDb;
}

// PRISON SENSOR SCENARIO:
// Add 8 sensors: four corners plus four side midpoints around the prison.
static void
AddPrisonPerimeterSensors (Ptr<ListPositionAllocator> allocator)
{
  allocator->Add (Vector (PRISON_MIN_X, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector ((PRISON_MIN_X + PRISON_MAX_X) / 2.0, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MAX_X, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MAX_X, (PRISON_MIN_Y + PRISON_MAX_Y) / 2.0, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MAX_X, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector ((PRISON_MIN_X + PRISON_MAX_X) / 2.0, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MIN_X, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MIN_X, (PRISON_MIN_Y + PRISON_MAX_Y) / 2.0, PRISON_SENSOR_HEIGHT_M));
}

// PRISON SENSOR SCENARIO:
// Add 16 sensors: four corners plus three-sensor clusters around each side
// midpoint. Cluster spacing is 30 m to encourage multi-sensor confirmation.
static void
AddPrisonClusteredSensors (Ptr<ListPositionAllocator> allocator)
{
  // Corners
  allocator->Add (Vector (PRISON_MIN_X, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MAX_X, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MAX_X, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MIN_X, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));

  // South midpoint cluster
  allocator->Add (Vector (470.0, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (500.0, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (530.0, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));

  // East midpoint cluster
  allocator->Add (Vector (PRISON_MAX_X, 470.0, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MAX_X, 500.0, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MAX_X, 530.0, PRISON_SENSOR_HEIGHT_M));

  // North midpoint cluster
  allocator->Add (Vector (470.0, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (500.0, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (530.0, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));

  // West midpoint cluster
  allocator->Add (Vector (PRISON_MIN_X, 470.0, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MIN_X, 500.0, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MIN_X, 530.0, PRISON_SENSOR_HEIGHT_M));
}

// PRISON SENSOR SCENARIO:
// Add 24 sensors as 8 monitoring stations with 3 RF receivers per station.
// Each original perimeter point is kept, with support receivers 30 m away
// along the adjacent wall direction(s). This improves confirmation and gives
// cleaner localisation evidence near both corners and side midpoints.
static void
AddPrisonStationSensors (Ptr<ListPositionAllocator> allocator)
{
  // Southwest corner station
  allocator->Add (Vector (PRISON_MIN_X, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MIN_X + 30.0, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MIN_X, PRISON_MIN_Y + 30.0, PRISON_SENSOR_HEIGHT_M));

  // South midpoint station
  allocator->Add (Vector (470.0, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (500.0, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (530.0, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));

  // Southeast corner station
  allocator->Add (Vector (PRISON_MAX_X, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MAX_X - 30.0, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MAX_X, PRISON_MIN_Y + 30.0, PRISON_SENSOR_HEIGHT_M));

  // East midpoint station
  allocator->Add (Vector (PRISON_MAX_X, 470.0, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MAX_X, 500.0, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MAX_X, 530.0, PRISON_SENSOR_HEIGHT_M));

  // Northeast corner station
  allocator->Add (Vector (PRISON_MAX_X, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MAX_X - 30.0, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MAX_X, PRISON_MAX_Y - 30.0, PRISON_SENSOR_HEIGHT_M));

  // North midpoint station
  allocator->Add (Vector (470.0, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (500.0, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (530.0, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));

  // Northwest corner station
  allocator->Add (Vector (PRISON_MIN_X, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MIN_X + 30.0, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MIN_X, PRISON_MAX_Y - 30.0, PRISON_SENSOR_HEIGHT_M));

  // West midpoint station
  allocator->Add (Vector (PRISON_MIN_X, 470.0, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MIN_X, 500.0, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MIN_X, 530.0, PRISON_SENSOR_HEIGHT_M));
}

// PRISON SENSOR SCENARIO:
// Add 24 sensors using an outer-fence plus secure-wall layout. The secure wall
// remains the prison boundary, while the outer fence creates a 50 m clear zone.
// Most sensors are mounted around the controlled fence line, with four midpoint
// sensors kept on the secure wall. This forms non-collinear triangles across
// the clear zone for localisation while keeping the sensors within prison
// controlled infrastructure.
static void
AddPrisonFenceSensors (Ptr<ListPositionAllocator> allocator)
{
  // West fence, bottom to top
  allocator->Add (Vector (FENCE_MIN_X, FENCE_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (FENCE_MIN_X, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (FENCE_MIN_X, 450.0, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (FENCE_MIN_X, 550.0, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (FENCE_MIN_X, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (FENCE_MIN_X, FENCE_MAX_Y, PRISON_SENSOR_HEIGHT_M));

  // North fence, west to east excluding the already-added northwest corner
  allocator->Add (Vector (PRISON_MIN_X, FENCE_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (400.0, FENCE_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (600.0, FENCE_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MAX_X, FENCE_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (FENCE_MAX_X, FENCE_MAX_Y, PRISON_SENSOR_HEIGHT_M));

  // East fence, top to bottom excluding the already-added northeast corner
  allocator->Add (Vector (FENCE_MAX_X, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (FENCE_MAX_X, 550.0, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (FENCE_MAX_X, 450.0, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (FENCE_MAX_X, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (FENCE_MAX_X, FENCE_MIN_Y, PRISON_SENSOR_HEIGHT_M));

  // South fence, east to west excluding the already-added southeast/southwest corners
  allocator->Add (Vector (PRISON_MAX_X, FENCE_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (600.0, FENCE_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (400.0, FENCE_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MIN_X, FENCE_MIN_Y, PRISON_SENSOR_HEIGHT_M));

  // Secure-wall midpoint sensors inside the controlled perimeter
  allocator->Add (Vector (PRISON_MIN_X, 500.0, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (500.0, PRISON_MIN_Y, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (PRISON_MAX_X, 500.0, PRISON_SENSOR_HEIGHT_M));
  allocator->Add (Vector (500.0, PRISON_MAX_Y, PRISON_SENSOR_HEIGHT_M));
}

static uint32_t
GetPrisonSensorCount (std::string sensorLayout)
{
  if (sensorLayout == "station24" ||
      sensorLayout == "fence24")
  {
    return 24;
  }
  if (sensorLayout == "cluster16")
  {
    return 16;
  }
  return 8;
}

// PRISON SENSOR SCENARIO:
// Keep the original uniform-threshold behaviour as the baseline. For the
// tiered_wall experiment, only fence24 secure-wall midpoint sensors S20-S23
// use the fixed -70 dBm sensitivity; outer-fence sensors still use the
// user-provided --rfThresholdDbm value.
static double
GetSensorThresholdDbm (uint32_t sensorId)
{
  bool isFence24WallSensor = (g_sensorLayout == "fence24" && sensorId >= 20 && sensorId <= 23);
  if (g_sensorProfile == "tiered_wall" && isFence24WallSensor)
  {
    return RF_SECURE_WALL_THRESHOLD_DBM;
  }
  return g_rfDetectionThresholdDbm;
}

static void
AddPrisonSensorsForLayout (Ptr<ListPositionAllocator> allocator, std::string sensorLayout)
{
  if (sensorLayout == "fence24")
  {
    AddPrisonFenceSensors (allocator);
  }
  else if (sensorLayout == "station24")
  {
    AddPrisonStationSensors (allocator);
  }
  else if (sensorLayout == "cluster16")
  {
    AddPrisonClusteredSensors (allocator);
  }
  else
  {
    AddPrisonPerimeterSensors (allocator);
  }
}

// PRISON SENSOR SCENARIO:
// Deterministic drone paths for fair layout comparison. Each path represents
// a plausible contraband-drone approach toward an internal prison destination.
static bool
IsRandomWaypointPath (std::string dronePath)
{
  return dronePath == "random_waypoint";
}

static double
ClampDouble (double value, double minValue, double maxValue)
{
  return std::max (minValue, std::min (value, maxValue));
}

// PRISON SENSOR SCENARIO:
// Small deterministic pseudo-random helper. We avoid non-repeatable randomness
// so --randomSeed=N always regenerates the same attack path for dissertation
// comparison.
static double
NextRandom01 (uint32_t &state)
{
  state = state * 1664525u + 1013904223u;
  return static_cast<double> (state & 0x00FFFFFFu) / static_cast<double> (0x01000000u);
}

static double
RandomBetween (uint32_t &state, double minValue, double maxValue)
{
  return minValue + (maxValue - minValue) * NextRandom01 (state);
}

static Vector
GetRandomWaypointTarget ()
{
  return Vector (g_randomTargetX, g_randomTargetY, 20.0);
}

static Vector
GenerateRandomWaypointStart (uint32_t &state)
{
  uint32_t entryCase = static_cast<uint32_t> (RandomBetween (state, 0.0, 8.0));
  double offset = RandomBetween (state, 90.0, 180.0);

  if (entryCase == 0)
  {
    return Vector (ClampDouble (FENCE_MIN_X - offset, SIM_MIN_X, SIM_MAX_X),
                   RandomBetween (state, FENCE_MIN_Y, FENCE_MAX_Y),
                   20.0);
  }
  if (entryCase == 1)
  {
    return Vector (ClampDouble (FENCE_MAX_X + offset, SIM_MIN_X, SIM_MAX_X),
                   RandomBetween (state, FENCE_MIN_Y, FENCE_MAX_Y),
                   20.0);
  }
  if (entryCase == 2)
  {
    return Vector (RandomBetween (state, FENCE_MIN_X, FENCE_MAX_X),
                   ClampDouble (FENCE_MIN_Y - offset, SIM_MIN_Y, SIM_MAX_Y),
                   20.0);
  }
  if (entryCase == 3)
  {
    return Vector (RandomBetween (state, FENCE_MIN_X, FENCE_MAX_X),
                   ClampDouble (FENCE_MAX_Y + offset, SIM_MIN_Y, SIM_MAX_Y),
                   20.0);
  }
  if (entryCase == 4)
  {
    return Vector (ClampDouble (FENCE_MIN_X - offset, SIM_MIN_X, SIM_MAX_X),
                   ClampDouble (FENCE_MIN_Y - offset, SIM_MIN_Y, SIM_MAX_Y),
                   20.0);
  }
  if (entryCase == 5)
  {
    return Vector (ClampDouble (FENCE_MIN_X - offset, SIM_MIN_X, SIM_MAX_X),
                   ClampDouble (FENCE_MAX_Y + offset, SIM_MIN_Y, SIM_MAX_Y),
                   20.0);
  }
  if (entryCase == 6)
  {
    return Vector (ClampDouble (FENCE_MAX_X + offset, SIM_MIN_X, SIM_MAX_X),
                   ClampDouble (FENCE_MIN_Y - offset, SIM_MIN_Y, SIM_MAX_Y),
                   20.0);
  }

  return Vector (ClampDouble (FENCE_MAX_X + offset, SIM_MIN_X, SIM_MAX_X),
                 ClampDouble (FENCE_MAX_Y + offset, SIM_MIN_Y, SIM_MAX_Y),
                 20.0);
}

static bool
FindSegmentPrisonEntryFraction (Vector a, Vector b, double &entryFraction)
{
  bool found = false;
  double bestT = 2.0;
  double dx = b.x - a.x;
  double dy = b.y - a.y;

  if (std::fabs (dx) > 1e-9)
  {
    double txWest = (PRISON_MIN_X - a.x) / dx;
    double yWest = a.y + dy * txWest;
    if (txWest >= 0.0 && txWest <= 1.0 && yWest >= PRISON_MIN_Y && yWest <= PRISON_MAX_Y)
    {
      found = true;
      bestT = std::min (bestT, txWest);
    }

    double txEast = (PRISON_MAX_X - a.x) / dx;
    double yEast = a.y + dy * txEast;
    if (txEast >= 0.0 && txEast <= 1.0 && yEast >= PRISON_MIN_Y && yEast <= PRISON_MAX_Y)
    {
      found = true;
      bestT = std::min (bestT, txEast);
    }
  }

  if (std::fabs (dy) > 1e-9)
  {
    double tySouth = (PRISON_MIN_Y - a.y) / dy;
    double xSouth = a.x + dx * tySouth;
    if (tySouth >= 0.0 && tySouth <= 1.0 && xSouth >= PRISON_MIN_X && xSouth <= PRISON_MAX_X)
    {
      found = true;
      bestT = std::min (bestT, tySouth);
    }

    double tyNorth = (PRISON_MAX_Y - a.y) / dy;
    double xNorth = a.x + dx * tyNorth;
    if (tyNorth >= 0.0 && tyNorth <= 1.0 && xNorth >= PRISON_MIN_X && xNorth <= PRISON_MAX_X)
    {
      found = true;
      bestT = std::min (bestT, tyNorth);
    }
  }

  entryFraction = bestT;
  return found;
}

static void
InitializeRandomWaypointPath ()
{
  if (!IsRandomWaypointPath (g_dronePath) || g_randomPathInitialized)
  {
    return;
  }

  uint32_t state = g_randomSeed;
  Vector start = GenerateRandomWaypointStart (state);
  Vector target = GetRandomWaypointTarget ();

  g_randomWaypoints.clear ();
  g_randomCumulativeDistances.clear ();
  g_randomWaypoints.push_back (start);

  // RANDOM WAYPOINT PATH:
  // Three intermediate waypoints keep the drone moving toward the vulnerable
  // roof/window target but introduce lateral deviations. This is more realistic
  // than adding more straight-line fixed paths while still being reproducible.
  const uint32_t numIntermediateWaypoints = 3;
  double dx = target.x - start.x;
  double dy = target.y - start.y;
  double length2d = std::sqrt (dx * dx + dy * dy);
  double perpX = (length2d > 1e-9) ? (-dy / length2d) : 0.0;
  double perpY = (length2d > 1e-9) ? (dx / length2d) : 0.0;

  for (uint32_t i = 1; i <= numIntermediateWaypoints; ++i)
  {
    double progress = static_cast<double> (i) / static_cast<double> (numIntermediateWaypoints + 1);
    double lateralOffset = RandomBetween (state, -80.0, 80.0) * std::sin (PRISON_PI * progress);
    double forwardJitter = RandomBetween (state, -25.0, 25.0);

    Vector waypoint = Vector (
      ClampDouble (start.x + dx * progress + perpX * lateralOffset + (dx / std::max (length2d, 1.0)) * forwardJitter,
                   SIM_MIN_X, SIM_MAX_X),
      ClampDouble (start.y + dy * progress + perpY * lateralOffset + (dy / std::max (length2d, 1.0)) * forwardJitter,
                   SIM_MIN_Y, SIM_MAX_Y),
      20.0
    );
    g_randomWaypoints.push_back (waypoint);
  }

  g_randomWaypoints.push_back (target);

  g_randomCumulativeDistances.push_back (0.0);
  g_randomTotalDistance = 0.0;
  g_randomBoundaryDistance = 0.0;
  bool boundaryFound = false;

  for (uint32_t i = 1; i < g_randomWaypoints.size (); ++i)
  {
    Vector a = g_randomWaypoints[i - 1];
    Vector b = g_randomWaypoints[i];
    double segmentDistance = CalculateDistanceMeters (a, b);

    if (!boundaryFound)
    {
      double boundaryFraction = 0.0;
      if (FindSegmentPrisonEntryFraction (a, b, boundaryFraction))
      {
        g_randomBoundaryDistance = g_randomTotalDistance + segmentDistance * boundaryFraction;
        boundaryFound = true;
      }
    }

    g_randomTotalDistance += segmentDistance;
    g_randomCumulativeDistances.push_back (g_randomTotalDistance);
  }

  if (!boundaryFound)
  {
    g_randomBoundaryDistance = g_randomTotalDistance;
  }

  g_randomLastDistanceAlongPath = 0.0;
  g_randomPathInitialized = true;

  std::cout << "[PRISON_SENSOR_RANDOM_PATH]"
            << " Seed=" << g_randomSeed
            << " Target=(" << target.x << "," << target.y << "," << target.z << ")"
            << " TotalDistance=" << g_randomTotalDistance
            << " BoundaryDistance=" << g_randomBoundaryDistance
            << std::endl;
  for (uint32_t i = 0; i < g_randomWaypoints.size (); ++i)
  {
    Vector waypoint = g_randomWaypoints[i];
    std::cout << "[PRISON_SENSOR_RANDOM_PATH]"
              << " Waypoint" << i << "=("
              << waypoint.x << "," << waypoint.y << "," << waypoint.z << ")"
              << std::endl;
  }
}

static Vector
GetRandomWaypointPositionAtDistance (double distanceAlongPath)
{
  InitializeRandomWaypointPath ();

  if (g_randomWaypoints.empty ())
  {
    return PRISON_DRONE_START_POS;
  }

  if (distanceAlongPath <= 0.0)
  {
    return g_randomWaypoints.front ();
  }
  if (distanceAlongPath >= g_randomTotalDistance)
  {
    return g_randomWaypoints.back ();
  }

  for (uint32_t i = 1; i < g_randomWaypoints.size (); ++i)
  {
    if (distanceAlongPath <= g_randomCumulativeDistances[i])
    {
      double segmentStartDistance = g_randomCumulativeDistances[i - 1];
      double segmentLength = g_randomCumulativeDistances[i] - segmentStartDistance;
      double segmentProgress = (distanceAlongPath - segmentStartDistance) / std::max (segmentLength, 1e-9);
      Vector a = g_randomWaypoints[i - 1];
      Vector b = g_randomWaypoints[i];
      return Vector (
        a.x + (b.x - a.x) * segmentProgress,
        a.y + (b.y - a.y) * segmentProgress,
        a.z + (b.z - a.z) * segmentProgress
      );
    }
  }

  return g_randomWaypoints.back ();
}

static Vector
GetDronePathStart (std::string dronePath)
{
  if (IsRandomWaypointPath (dronePath))
  {
    InitializeRandomWaypointPath ();
    return g_randomWaypoints.empty () ? PRISON_DRONE_START_POS : g_randomWaypoints.front ();
  }

  if (dronePath == "west_offset_low")
  {
    return Vector (50.0, 430.0, 20.0);
  }
  if (dronePath == "west_offset_high")
  {
    return Vector (50.0, 575.0, 20.0);
  }
  if (dronePath == "south_midpoint")
  {
    return Vector (500.0, 50.0, 20.0);
  }
  if (dronePath == "north_midpoint")
  {
    return Vector (500.0, 950.0, 20.0);
  }
  if (dronePath == "east_midpoint")
  {
    return Vector (950.0, 500.0, 20.0);
  }
  if (dronePath == "diagonal_sw")
  {
    return Vector (50.0, 100.0, 20.0);
  }
  if (dronePath == "diagonal_nw")
  {
    return Vector (50.0, 900.0, 20.0);
  }
  return PRISON_DRONE_START_POS;
}

static Vector
GetDronePathTarget (std::string dronePath)
{
  if (IsRandomWaypointPath (dronePath))
  {
    return GetRandomWaypointTarget ();
  }

  if (dronePath == "west_offset_low")
  {
    return Vector (500.0, 430.0, 20.0);
  }
  if (dronePath == "west_offset_high")
  {
    return Vector (500.0, 575.0, 20.0);
  }
  if (dronePath == "south_midpoint" ||
      dronePath == "north_midpoint" ||
      dronePath == "east_midpoint" ||
      dronePath == "diagonal_sw" ||
      dronePath == "diagonal_nw")
  {
    return PRISON_DRONE_TARGET_POS;
  }
  return PRISON_DRONE_TARGET_POS;
}

// PRISON SENSOR SCENARIO:
// First point where the selected drone path crosses the prison boundary.
// This lets the CSV report "time available before perimeter crossing" for
// every deterministic approach direction, not only the original west path.
static Vector
GetDronePathBoundaryPoint (std::string dronePath)
{
  if (IsRandomWaypointPath (dronePath))
  {
    InitializeRandomWaypointPath ();
    return GetRandomWaypointPositionAtDistance (g_randomBoundaryDistance);
  }

  if (dronePath == "west_offset_low")
  {
    return Vector (PRISON_MIN_X, 430.0, 20.0);
  }
  if (dronePath == "west_offset_high")
  {
    return Vector (PRISON_MIN_X, 575.0, 20.0);
  }
  if (dronePath == "south_midpoint")
  {
    return Vector (500.0, PRISON_MIN_Y, 20.0);
  }
  if (dronePath == "north_midpoint")
  {
    return Vector (500.0, PRISON_MAX_Y, 20.0);
  }
  if (dronePath == "east_midpoint")
  {
    return Vector (PRISON_MAX_X, 500.0, 20.0);
  }
  if (dronePath == "diagonal_sw")
  {
    return Vector (331.25, PRISON_MIN_Y, 20.0);
  }
  if (dronePath == "diagonal_nw")
  {
    return Vector (331.25, PRISON_MAX_Y, 20.0);
  }
  return Vector (PRISON_MIN_X, 500.0, 20.0);
}

static bool
IsValidDronePath (std::string dronePath)
{
  return (dronePath == "west_midpoint" ||
          dronePath == "west_offset_low" ||
          dronePath == "west_offset_high" ||
          dronePath == "south_midpoint" ||
          dronePath == "north_midpoint" ||
          dronePath == "east_midpoint" ||
          dronePath == "diagonal_sw" ||
          dronePath == "diagonal_nw" ||
          IsRandomWaypointPath (dronePath));
}

static bool
IsInsidePrison (Vector pos)
{
  return (pos.x >= PRISON_MIN_X &&
          pos.x <= PRISON_MAX_X &&
          pos.y >= PRISON_MIN_Y &&
          pos.y <= PRISON_MAX_Y);
}

static std::string
GetPrisonZone (Vector pos)
{
  bool nearWestPerimeter = std::fabs (pos.x - PRISON_MIN_X) <= PRISON_PERIMETER_TOLERANCE_M &&
                           pos.y >= PRISON_MIN_Y &&
                           pos.y <= PRISON_MAX_Y;
  bool nearEastPerimeter = std::fabs (pos.x - PRISON_MAX_X) <= PRISON_PERIMETER_TOLERANCE_M &&
                           pos.y >= PRISON_MIN_Y &&
                           pos.y <= PRISON_MAX_Y;
  bool nearSouthPerimeter = std::fabs (pos.y - PRISON_MIN_Y) <= PRISON_PERIMETER_TOLERANCE_M &&
                            pos.x >= PRISON_MIN_X &&
                            pos.x <= PRISON_MAX_X;
  bool nearNorthPerimeter = std::fabs (pos.y - PRISON_MAX_Y) <= PRISON_PERIMETER_TOLERANCE_M &&
                            pos.x >= PRISON_MIN_X &&
                            pos.x <= PRISON_MAX_X;
  if (nearWestPerimeter ||
      nearEastPerimeter ||
      nearSouthPerimeter ||
      nearNorthPerimeter)
  {
    return "perimeter_crossing";
  }

  if (IsInsidePrison (pos))
  {
    Vector currentTarget = GetDronePathTarget (g_dronePath);
    if (CalculateDistanceMeters (pos, currentTarget) <= PRISON_CRITICAL_RADIUS_M)
    {
      return "critical_zone";
    }
    return "internal_airspace";
  }

  if (pos.x >= PRISON_MIN_X - 100.0 &&
      pos.x <= PRISON_MAX_X + 100.0 &&
      pos.y >= PRISON_MIN_Y - 100.0 &&
      pos.y <= PRISON_MAX_Y + 100.0)
  {
    return "approach_zone";
  }

  if (pos.x >= SIM_MIN_X && pos.x <= SIM_MAX_X && pos.y >= SIM_MIN_Y && pos.y <= SIM_MAX_Y)
  {
    return "early_warning_zone";
  }

  return "outside_simulation_region";
}

static double
CalculateTimeToBoundarySeconds (Vector pos)
{
  if (IsInsidePrison (pos))
  {
    return 0.0;
  }

  if (IsRandomWaypointPath (g_dronePath))
  {
    InitializeRandomWaypointPath ();
    if (g_randomLastDistanceAlongPath >= g_randomBoundaryDistance)
    {
      return 0.0;
    }
    return (g_randomBoundaryDistance - g_randomLastDistanceAlongPath) / PRISON_DRONE_SPEED_MPS;
  }

  Vector start = GetDronePathStart (g_dronePath);
  Vector boundary = GetDronePathBoundaryPoint (g_dronePath);
  double startToBoundary = CalculateDistanceMeters (start, boundary);
  double startToCurrent = CalculateDistanceMeters (start, pos);

  if (startToCurrent >= startToBoundary)
  {
    return 0.0;
  }

  return CalculateDistanceMeters (pos, boundary) / PRISON_DRONE_SPEED_MPS;
}

// PRISON SENSOR SCENARIO:
// Move the ns-3 UAV along the selected deterministic contraband path. This
// avoids manual GUI variability and makes layout/threshold/path comparisons
// repeatable.
static void
UpdatePrisonDronePath (Ptr<Node> uav)
{
  Ptr<ConstantPositionMobilityModel> mmUAV = uav->GetObject<ConstantPositionMobilityModel> ();
  double elapsed = Simulator::Now ().GetSeconds ();

  if (IsRandomWaypointPath (g_dronePath))
  {
    InitializeRandomWaypointPath ();
    g_randomLastDistanceAlongPath = std::min (PRISON_DRONE_SPEED_MPS * elapsed, g_randomTotalDistance);
    mmUAV->SetPosition (GetRandomWaypointPositionAtDistance (g_randomLastDistanceAlongPath));
    Simulator::Schedule (Seconds (1.0), &UpdatePrisonDronePath, uav);
    return;
  }

  Vector start = GetDronePathStart (g_dronePath);
  Vector target = GetDronePathTarget (g_dronePath);
  double pathDistance = CalculateDistanceMeters (start, target);
  double progress = std::min ((PRISON_DRONE_SPEED_MPS * elapsed) / pathDistance, 1.0);

  Vector pos = Vector (
    start.x + (target.x - start.x) * progress,
    start.y + (target.y - start.y) * progress,
    start.z + (target.z - start.z) * progress
  );

  mmUAV->SetPosition (pos);
  Simulator::Schedule (Seconds (1.0), &UpdatePrisonDronePath, uav);
}

// PRISON SENSOR SCENARIO:
// Every second, calculate whether each fixed RF sensor detects the UAV and
// append dissertation metrics to prison_sensor_detections.csv.
static void
LogSensorDetections (Ptr<Node> uav, NodeContainer sensors)
{
  Ptr<MobilityModel> uavMob = uav->GetObject<MobilityModel> ();
  Vector uavPos = uavMob->GetPosition ();
  double now = Simulator::Now ().GetSeconds ();
  bool insidePrison = IsInsidePrison (uavPos);
  std::string zone = GetPrisonZone (uavPos);
  double timeToBoundary = CalculateTimeToBoundarySeconds (uavPos);

  std::vector<double> distances;
  std::vector<double> rxPowers;
  std::vector<double> thresholds;
  std::vector<int> detections;
  uint32_t numSensorsDetecting = 0;

  for (uint32_t i = 0; i < sensors.GetN (); ++i)
  {
    Ptr<MobilityModel> sensorMob = sensors.Get (i)->GetObject<MobilityModel> ();
    Vector sensorPos = sensorMob->GetPosition ();
    double distance = CalculateDistanceMeters (uavPos, sensorPos);
    double rxPowerDbm = CalculateFriisRxPowerDbm (distance);
    double thresholdDbm = GetSensorThresholdDbm (i);
    int detected = (rxPowerDbm >= thresholdDbm) ? 1 : 0;

    distances.push_back (distance);
    rxPowers.push_back (rxPowerDbm);
    thresholds.push_back (thresholdDbm);
    detections.push_back (detected);
    if (detected)
    {
      numSensorsDetecting++;
    }
  }

  int confirmedDetection = (numSensorsDetecting >= 2) ? 1 : 0;
  if (confirmedDetection && !insidePrison)
  {
    g_confirmedAlertRaised = true;
  }
  int alertBeforeBoundary = g_confirmedAlertRaised ? 1 : 0;

  std::ofstream csv;
  csv.open (RF_SENSOR_CSV, std::ios_base::app);

  for (uint32_t i = 0; i < sensors.GetN (); ++i)
  {
    Ptr<MobilityModel> sensorMob = sensors.Get (i)->GetObject<MobilityModel> ();
    Vector sensorPos = sensorMob->GetPosition ();

    csv << now << ","
        << g_sensorLayout << ","
        << g_sensorProfile << ","
        << g_dronePath << ","
        << g_randomSeed << ","
        << g_randomTargetX << ","
        << g_randomTargetY << ","
        << uavPos.x << "," << uavPos.y << "," << uavPos.z << ","
        << i << ","
        << sensorPos.x << "," << sensorPos.y << "," << sensorPos.z << ","
        << distances[i] << ","
        << rxPowers[i] << ","
        << thresholds[i] << ","
        << detections[i] << ","
        << numSensorsDetecting << ","
        << confirmedDetection << ","
        << (insidePrison ? 1 : 0) << ","
        << zone << ","
        << timeToBoundary << ","
        << alertBeforeBoundary << "\n";
  }

  csv.close ();

  std::cout << "[PRISON_SENSOR]"
            << " Time=" << now
            << "s Layout=" << g_sensorLayout
            << " Profile=" << g_sensorProfile
            << " Path=" << g_dronePath
            << " UAV=(" << uavPos.x << "," << uavPos.y << "," << uavPos.z << ")"
            << " Zone=" << zone
            << " SensorsDetecting=" << numSensorsDetecting
            << " Confirmed=" << confirmedDetection
            << " AlertBeforeBoundary=" << alertBeforeBoundary
            << std::endl;

  Simulator::Schedule (Seconds (1.0), &LogSensorDetections, uav, sensors);
}


//code given by chatgpt for helper function
static void
PrintDistance (Ptr<Node> uav, Ptr<Node> ap)
{
  Ptr<MobilityModel> uavMob = uav->GetObject<MobilityModel> ();
  Ptr<MobilityModel> apMob = ap->GetObject<MobilityModel> ();

  Vector uavPos = uavMob->GetPosition ();
  Vector apPos = apMob->GetPosition ();

  double dx = uavPos.x - apPos.x;
  double dy = uavPos.y - apPos.y;
  double dz = uavPos.z - apPos.z;

  double distance = std::sqrt (dx*dx + dy*dy + dz*dz);

  std::cout << "[DISTANCE] Time=" << Simulator::Now ().GetSeconds ()
            << "s UAV=(" << uavPos.x << "," << uavPos.y << "," << uavPos.z << ")"
            << " AP=(" << apPos.x << "," << apPos.y << "," << apPos.z << ")"
            << " Distance=" << distance << "m";

  std::cout << " FRIIS_MODEL";

  std::cout << " CommandRx=" << g_commandRx
            << " TelemetryRx=" << g_telemetryRx;

  std::cout << std::endl;

  Simulator::Schedule (Seconds (1.0), &PrintDistance, uav, ap);
}


/************ GLOBAL CONSTANTS ************/
long g_cmd_start = 0;
long g_cmd_end = 0;
long g_cmd_tot_delay = 0;
long g_cmd_num = 0;


pthread_t tid_gcs[2];
pthread_t tid_uav[2];

Ptr<MyApp> uavApp;
Ptr<MyApp> gcsApp;


// Prepare our context and publisher for Commands to UAVs from GCS
void *contextCm = zmq_ctx_new (); 
void *publisherCm = zmq_socket (contextCm, ZMQ_PUB);

// Prepare our context and publisher for Telemetry from UAVs
void *contextTm = zmq_ctx_new (); 
void *publisherTm = zmq_socket (contextTm, ZMQ_PUB);

// Prepare our context and publisher for video sensor data
void *contextVid = zmq_ctx_new (); 
void *publisherVid = zmq_socket (contextVid, ZMQ_PUB);


//global subscriber for use in the rcvpacket callback
int rc;
void *context = zmq_ctx_new ();
void *subscriber = zmq_socket (context, ZMQ_SUB);



using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("uav-net-sim");

void RcvPacket(Ptr<Packet> p, Address &addr)
{
        uint8_t *buffer = new uint8_t[p->GetSize() + 32];
        memset(buffer, '\0', p->GetSize() + 32);
        p->CopyData(buffer, p->GetSize());
        char *string =  (char *)buffer;
        char *sTime = (char *)malloc(11);
        memset(sTime, '\0', 11);
        sprintf(sTime, "%10ld***", ns3::Simulator::Now().GetMilliSeconds());
        strcat(string, sTime);

	char *s = (char *) malloc(1512);
        memset(s, '\0', 1512);
	strcpy(s, string);
  
        if(string[3] == 'G')   // Packet from GCS to UAV with Control Commands
        {
	            g_commandRx++;

          std::cout << "[NS3 RX][COMMAND]"
                    << " Time=" << Simulator::Now().GetSeconds()
                    << "s Size=" << p->GetSize()
                    << " TotalCommandRx=" << g_commandRx
                    << std::endl; //new function added here
          zmq_msg_t message;
          zmq_msg_init_size (&message, strlen (string));
          memcpy (zmq_msg_data (&message), string, strlen (string));
          printf(" Publish MSG : %s \n", string);
          zmq_sendmsg (publisherCm, &message, 0);
          zmq_msg_close (&message);

	  // Tokenize to parse packet to get packet ingress and egress time : do it in every one sec interval	 
	  g_cmd_end = ns3::Simulator::Now().GetMilliSeconds();
	  g_cmd_num++;
	  
          char* token = strtok(s, "***");
	  int count = 0;
          long ingress_time = 0;
          long egress_time = 0;

	  while(token)
	  {
	      if(count == 4)
              {
             	ingress_time = (atol)(token);
              }
              else if(count == 5)
              {
               	egress_time = (atol)(token);
              }
              count++;
              //printf("count = %d   token: %s \n", count, token);
              token = strtok(NULL, "***");
	  }

	  long net_delay = egress_time - ingress_time + 1;
	  g_cmd_tot_delay += net_delay;

	  if((g_cmd_end - g_cmd_start) > 1000)   // print average network packet delay for command packets, in every sec
	  {
	      float g_cmd_avg_delay = (float)(g_cmd_tot_delay)/g_cmd_num;
              printf(">>>>>> Average Network Delay for command packets: %f MilliSec.\n", g_cmd_avg_delay);

	      //reset variables
	      g_cmd_tot_delay = 0;
	      g_cmd_num = 0; 
	      g_cmd_start = g_cmd_end;
	   }

        }
        else if (string[3] == 'U')  // Telemetry Packets from UAV to GCS
        {
	  
	  g_telemetryRx++;

          std::cout << "[NS3 RX][TELEMETRY]"
                    << " Time=" << Simulator::Now().GetSeconds()
                    << "s Size=" << p->GetSize()
                    << " TotalTelemetryRx=" << g_telemetryRx
                    << std::endl;
         
          zmq_msg_t message;
          zmq_msg_init_size (&message, strlen (string));
          memcpy (zmq_msg_data (&message), string, strlen (string));
          printf(" Publish MSG : %s \n", string);
          zmq_sendmsg (publisherTm, &message, 0); //Telemetry Information
          std::cout << "Send DATA to EDGE >>>>>>>> " << std::endl;
          zmq_msg_close (&message);

        }
        else if(string[3] == 'S')   // for SENSOR  // This section may need revision
        {
          printf("RECEIVED SENSOR DATA FROM NS3\n");

          while(true)
          {
            zmq_msg_t msg;
            zmq_msg_init (&msg);
            rc = (zmq_msg_recv (&msg, subscriber, 0));
            int size = zmq_msg_size (&msg);
            printf("ZMQ SENSOR DATA SIZE: %d \n", size);
            char *strVZ = (char*)malloc (size + 1);
            memcpy (strVZ, zmq_msg_data (&msg), size);
            strVZ[size] = '\0';
            printf("POPPED MSG from ZMQ: %s \n", strVZ);

            if(strncmp(string+9,strVZ+9, 6) > 0)
            {
              //pop more from ZMQ and discard the current one
              //continue;
            }
            else
            {
              printf(" Publish MSG from ZMQ: %s \n", strVZ);
              zmq_sendmsg (publisherTm, &msg, 0); //Telemetry Information
              break;
            } 
            zmq_msg_close (&msg);
            free(strVZ);
          }

        }
	else
	{

	}

        free(sTime);
        free(s);

        delete[] buffer;

}


void RcvPktTrace(){
        Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx", MakeCallback(&RcvPacket));
}

/*****************************************************/
/**** START rcvCommands(): commands from GCS-ZMQ  ****/ 
/**** send command to specific UAV node over socket***/
/*****************************************************/

void* rcvCommands(void *arg)
{

  std::cout << "STRUCT VOID : " << arg << std::endl; 
  vector <Ptr<MyApp>> *vectCom = (vector <Ptr<MyApp>> *) arg;
  int numUav = vectCom->size();
  std::cout << "Number of UAV APPs receiving Commands: " << numUav << std::endl; 
  int rc;
  void *context = zmq_ctx_new ();
  void *subscriber = zmq_socket (context, ZMQ_SUB);  

  rc = zmq_connect (subscriber, "tcp://localhost:5500");
  assert (rc == 0);
  zmq_setsockopt( subscriber, ZMQ_SUBSCRIBE, "", 0);
  
  sleep(2);

  while(1)
  {
    zmq_msg_t message;
    zmq_msg_init (&message);
    rc = (zmq_msg_recv (&message, subscriber, 0));
    int size = zmq_msg_size (&message);
    char *string = (char*)malloc (size + 1 + 15);
    memcpy (string, zmq_msg_data (&message), size);
    zmq_msg_close (&message);
    string [size] = '\0';
    char sTime[15] = {"\0"};
    sprintf(sTime, "%11ld***", ns3::Simulator::Now().GetMilliSeconds());
    strcat(string, sTime);

    char uav_index[4]; // expects index starts from 0 from the external APP
    memcpy(uav_index, &string[5], 3);
    uav_index[3] = '\0';
    int sock_index = atoi(uav_index);
    Ptr<MyApp> app = (*vectCom)[sock_index]; //Pick specific UAV App 
 
    Ptr<Socket> send_socket = app->m_socket;
    //  Ptr<Socket> send_socket = gcsApp->m_socket;
    last_schedule_time += 0.001; // Maintains Causality, although Scheduling is sequential
    Time tNext (Seconds (last_schedule_time));
    ns3::Simulator::Schedule(tNext, &MyApp::SendMsg, app, send_socket, string);
    //ns3::Simulator::Schedule(tNext, &MyApp::SendMsg, gcsApp, send_socket, string);
  }

  zmq_close (subscriber);
  zmq_term (context);
  return 0;
}
/************** END rcvCommands() ***************/


/*****************************************************/
/**** START rcvTelemetry(): commands from UAV-ZMQ ****/ 
/****   send telemetry to  GCS node over socket    ***/
/*****************************************************/

void* rcvTelemetry(void *arg)
{
  zmq_bind (publisherVid, "tcp://127.0.0.1:5000");

  std::cout << "STRUCT VOID : " << arg << std::endl; 
  vector <Ptr<MyApp>> *vectTel = (vector <Ptr<MyApp>> *) arg;
  int numUav = vectTel->size();
  std::cout << "Number of UAV APPs sending telemetry: " << numUav << std::endl;

  int rc;
  void *context = zmq_ctx_new ();
  void *subscriber = zmq_socket (context, ZMQ_SUB);  

  rc = zmq_connect (subscriber, "tcp://localhost:5600");
  assert (rc == 0);
  zmq_setsockopt( subscriber, ZMQ_SUBSCRIBE, "", 0);
  
  sleep(2);

  while(1)
  {
    zmq_msg_t message;
    zmq_msg_init (&message);
    rc = (zmq_msg_recv (&message, subscriber, 0));
    int size = zmq_msg_size (&message);
    char *string = (char*)malloc (size + 1 + 23);
    memcpy (string, zmq_msg_data (&message), size);
    string [size] = '\0';

    printf(" TELEMETRY Recvd: Size %ld \n", strlen(string));
    printf(" TELEMETRY Recvd: %s \n", string);

    //check if SENSOR data received, then publish in video ZMQ
    char *prefix = (char*)malloc(7);
    memcpy (prefix, string, 6);
    prefix[6] = '\0';

    if(!strcmp(prefix, "SENSOR"))
    {
        zmq_sendmsg (publisherVid, &message, 0);
        zmq_msg_close(&message);
    }
    free(prefix);

    //Concatenate time at the end for telemetry
    char *sTime = (char *) malloc(15);
    memset(sTime, '\0', 15);
    sprintf(sTime, "%11ld***", ns3::Simulator::Now().GetMilliSeconds());
    strcat(string, sTime);

    //check the UAV ID that sent the telemetry
    char uav_index[4];
    memcpy(uav_index, &string[5], 3); 
    uav_index[3] = '\0';  //saved the UAV index


    /********** Now Update the location of UAV from the telemetry *****/
    /******** Intercept telemetry to get location info ********/
    char *loc_pattern = (char *) malloc(12);
    memcpy(loc_pattern, string+11, 11);
    loc_pattern[11] = '\0';

    std::cout << " *********** LOC PATTERN :::::: " << loc_pattern << " ***********" << std::endl;

    bool isDistanceUpdate = (strncmp(loc_pattern, "DISTANCE***", 11) == 0);

    if(isDistanceUpdate && !g_useFixedPrisonPath)  //If the message has the word 'DISTANCE', update the location of UAV
    { 
	//location update
        std::cout << "DISTANCE VALUE found " << std::endl;
        
        char *curX = (char *)malloc(15);
        char *curY = (char *)malloc(15);

        int size = 0;
        for( char *c = &string[22]; *c!='*'; c++){
            curX[size++] = *c;
        }
        curX[size] = '\0';
        size = size + 3; //increment to cover the '***'

        int size2 = 0;
        for( char *c = &string[22+size]; *c!='*'; c++){
            curY[size2++] = *c;
        }
        curY[size2] = '\0';

        printf(">>>>>> X-Value:  %s  ;  Y-Value:  %s <<<<<<\n", curX, curY);

        float cur_x = atof(curX);
        float cur_y = atof(curY);

        //Update mobility based on current position
        printf("X \t %f \t Y \t %f\n", cur_x, cur_y);
        Ptr<Node> uavNode = ns3::NodeList::GetNode(0); // Need to change for multi-UAV
        Ptr<ConstantPositionMobilityModel> mmUAV = uavNode->GetObject<ConstantPositionMobilityModel>();
        ns3::Simulator::ScheduleNow(&ConstantPositionMobilityModel::SetPosition, mmUAV, Vector(cur_x, cur_y, 10));

        free(curX);
        free(curY);
     
    }
    // position update done for uav_index

    if(!isDistanceUpdate)  // The UAV message is a normal Telemetry and not a location update
    {
      /******** Send the telemetry to the GCS TCP App ********/
      int sock_index = atoi(uav_index);
      Ptr<MyApp> app = (*vectTel)[sock_index];  
      Ptr<Socket> send_socket = app->m_socket;
      std::cout << "SELECTED APP ------- " << app << std::endl;
    
      //std::cout << "UAV APP ------- " << uavApp << std::endl;
      //Ptr<Socket> send_socket = uavApp->m_socket;

      last_schedule_time += 0.001; // Maintains Causality, although Scheduling is sequential
      Time tNext (Seconds (last_schedule_time));
      ns3::Simulator::Schedule(tNext, &MyApp::SendMsg, app, send_socket, string);
      //ns3::Simulator::Schedule(tNext, &MyApp::SendMsg, uavApp, send_socket, string);
    }
    free(loc_pattern);
    zmq_msg_close (&message);

    free(sTime);
  }

  zmq_close (subscriber);
  zmq_term (context);
  return 0;
}
/************** END rcvTelemetry() ***************/





/************* MAIN() FUNCTION ***************/

int main (int argc, char *argv[])
{
  int nUAV = 1;
  int network_type = 0;
  int nCong = 10;
  float inputCongRate = 0.0;
  int congPktSize = 100;
  std::string congRate = "2Kbps"; // This rate will not have any effect, actual rate is taken from inputCongRate

  CommandLine cmd;
  cmd.AddValue ("rfThresholdDbm", "RF detection threshold in dBm for passive prison sensors", g_rfDetectionThresholdDbm);
  cmd.AddValue ("sensorLayout", "Prison RF sensor layout: perimeter8, cluster16, station24, or fence24", g_sensorLayout);
  cmd.AddValue ("sensorProfile", "Prison RF threshold profile: uniform or tiered_wall", g_sensorProfile);
  cmd.AddValue ("dronePath", "Prison drone path: fixed path name or random_waypoint", g_dronePath);
  cmd.AddValue ("randomSeed", "Seed for reproducible random_waypoint drone path", g_randomSeed);
  cmd.AddValue ("randomTargetX", "Fixed vulnerable target X coordinate for random_waypoint path", g_randomTargetX);
  cmd.AddValue ("randomTargetY", "Fixed vulnerable target Y coordinate for random_waypoint path", g_randomTargetY);
  cmd.Parse (argc, argv);

  if (g_sensorLayout != "perimeter8" &&
      g_sensorLayout != "cluster16" &&
      g_sensorLayout != "station24" &&
      g_sensorLayout != "fence24")
  {
    std::cout << "[PRISON_SENSOR] Unknown sensorLayout='" << g_sensorLayout
              << "'. Falling back to perimeter8." << std::endl;
    g_sensorLayout = "perimeter8";
  }

  if (g_sensorProfile != "uniform" &&
      g_sensorProfile != "tiered_wall")
  {
    std::cout << "[PRISON_SENSOR] Unknown sensorProfile='" << g_sensorProfile
              << "'. Falling back to uniform." << std::endl;
    g_sensorProfile = "uniform";
  }

  if (g_sensorProfile == "tiered_wall" && g_sensorLayout != "fence24")
  {
    std::cout << "[PRISON_SENSOR] sensorProfile=tiered_wall only changes "
              << "fence24 secure-wall sensors. Current layout will run with "
              << "the uniform threshold behaviour." << std::endl;
  }

  if (!IsValidDronePath (g_dronePath))
  {
    std::cout << "[PRISON_SENSOR] Unknown dronePath='" << g_dronePath
              << "'. Falling back to west_midpoint." << std::endl;
    g_dronePath = "west_midpoint";
  }

  if (IsRandomWaypointPath (g_dronePath))
  {
    g_randomTargetX = ClampDouble (g_randomTargetX, PRISON_MIN_X, PRISON_MAX_X);
    g_randomTargetY = ClampDouble (g_randomTargetY, PRISON_MIN_Y, PRISON_MAX_Y);
    InitializeRandomWaypointPath ();
  }

  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(1448));
  InitializeSensorCsv ();

  /*********** Bind Publisher socket ***********/
  zmq_bind (publisherCm, "tcp://127.0.0.1:5601");
  zmq_bind (publisherTm, "tcp://127.0.0.1:5501");


  /*********** Load input from XML file ***********/
  MyInput *input = new MyInput();
  input->loadInput();
  nUAV = input->m_num_uav;
  network_type = input->m_network;
  nCong = input->m_num_traffic;
  inputCongRate = input->m_traf_rate;
  congPktSize = input->m_traf_size; 

  std::cout << " INPUT FROM XML : " << std::endl;  
  std::cout << " ----------------------- : " << std::endl;  
  std::cout << " NUMBER OF UAV(s) : " << nUAV << std::endl;  
  std::cout << " NUMBER OF CONTENDING NODES : " << nCong << std::endl;  
  std::cout << " TRAFFIC RATE OF EACH CONTENDING NODES : " << inputCongRate << std::endl;  
  std::cout << " PACKET SIZE OF EACH OF CONTENDING NODES : " << congPktSize << std::endl;  

  if(network_type == 0)
    std::cout << "----------- CREATE WIFI NETWORK ------------" << std::endl;
  else
    std::cout << "----------- CREATE LTE NETWORK ------------" << std::endl;


  /***************** Define the Network Physical Layer *******************/

  NS_LOG_INFO ("Create UAV nodes.");
  NodeContainer uavNode;
  uavNode.Create (nUAV); // Number of UAVs read from the config.xml file
  NodeContainer congNode;
  congNode.Create (nCong); // Number of contending traffic nodes read from xml file

  /********* Set Position and Mobility for WiFi Sta nodes *******/
  // For all the UAVs
  Ptr<ListPositionAllocator> positionAllocUav = CreateObject<ListPositionAllocator> ();
  for (uint32_t i = 0; i < uavNode.GetN(); ++i){
    float xVal = input->m_x_values[i];
    float yVal = input->m_y_values[i];
    float zVal = input->m_z_values[i];
    positionAllocUav->Add (Vector (xVal, yVal, zVal));
  }
  MobilityHelper mobilityUav;
  mobilityUav.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityUav.SetPositionAllocator(positionAllocUav);
  mobilityUav.Install (uavNode);

  // PRISON SENSOR SCENARIO:
  // Override the XML/manual starting position with the fixed contraband-drone
  // approach path selected for repeatable prison sensor experiments.
  Ptr<ConstantPositionMobilityModel> prisonUavMob = uavNode.Get(0)->GetObject<ConstantPositionMobilityModel>();
  prisonUavMob->SetPosition (GetDronePathStart (g_dronePath));
  Simulator::Schedule (Seconds (1.0), &UpdatePrisonDronePath, uavNode.Get (0));

  // For all the external traffic nodes
  MobilityHelper mobilityCong;
  mobilityCong.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityCong.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
					"rho", DoubleValue(10),
					"X", DoubleValue(25.0),
					"Y", DoubleValue(25.0));
  mobilityCong.Install (congNode);


  /*********** Install IP stack on all WiFi Station nodes *********/ 
  InternetStackHelper internetUav;
  internetUav.Install (uavNode);
  internetUav.Install (congNode);


  /********************* Configure Default LTE Parameters ****************************/
  Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
  Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::PfFfMacScheduler::HarqEnabled", BooleanValue (false));
  Config::SetDefault ("ns3::PfFfMacScheduler::CqiTimerThreshold", UintegerValue (10));
  Config::SetDefault ("ns3::LteEnbRrc::EpsBearerToRlcMapping",EnumValue(LteEnbRrc::RLC_AM_ALWAYS));
  Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue(100));
  Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(100));
  Config::SetDefault ("ns3::LteUePhy::EnableUplinkPowerControl", BooleanValue (false));

  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);
  NS_LOG_INFO("Created the LTE Helper");

  //This creates the sgw/pgw node
  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);  // This remote host will be in GCS
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  /*********************** INTERNET stack in EPC *********************************/
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.00001)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // Explicitly create the nodes required by the topology
  NodeContainer enbNodes;
  enbNodes.Create(1);

  Ptr<ListPositionAllocator> positionAllocEnb = CreateObject<ListPositionAllocator> ();
  positionAllocEnb->Add (Vector (PRISON_BASE_STATION_POS.x, PRISON_BASE_STATION_POS.y, 30.0));

  MobilityHelper mobilityLte;
  mobilityLte.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityLte.SetPositionAllocator(positionAllocEnb);
  mobilityLte.Install (enbNodes);
  //mobilityLte.Install (uavNode);  //Mobility already defined for UAV node(s)


  /**************** Scheduler, Propagation and Fading *********************/
  lteHelper->SetHandoverAlgorithmType ("ns3::NoOpHandoverAlgorithm"); // disable automatic handover
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FriisPropagationLossModel"));

  /*************** Create Devices **************************/
  NetDeviceContainer enbDevices;
  enbDevices = lteHelper->InstallEnbDevice(enbNodes);

  NetDeviceContainer ueDevices;
  //ueDevices = lteHelper->InstallUeDevice (ueNodes);
  for (uint32_t u = 0; u < uavNode.GetN (); ++u)
  {
    ueDevices.Add(lteHelper->InstallUeDevice (uavNode.Get(u)));
  }

  
  NetDeviceContainer congueDevices;
  for (uint32_t u = 0; u < congNode.GetN (); ++u)
  {
    congueDevices.Add(lteHelper->InstallUeDevice (congNode.Get(u)));
  }


  /******************* INTERNET Stack in LTE ***********************/
  Ipv4InterfaceContainer ueIpIfaceList;
  // assign IP address to UEs
  for (uint32_t u = 0; u < uavNode.GetN (); ++u)
  {
    Ipv4AddressHelper ipv4;
    Ptr<Node> ue = uavNode.Get (u);
    Ptr<NetDevice> ueLteDevice = ueDevices.Get (u);
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevice));
    ueIpIfaceList.Add(ueIpIface);

    // set the default gateway for the UE
    Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
    ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

    lteHelper->Attach(ueLteDevice, enbDevices.Get(0));
    lteHelper->ActivateDedicatedEpsBearer (ueLteDevice, EpsBearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT), EpcTft::Default ());
  }

  Ipv4InterfaceContainer congueIpIfaceList;
  // assign IP address to UEs
  for (uint32_t u = 0; u < congNode.GetN (); ++u)
  {
    Ipv4AddressHelper ipv4;
    Ptr<Node> congue = congNode.Get (u);
    Ptr<NetDevice> congueLteDevice = congueDevices.Get (u);
    Ipv4InterfaceContainer congueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (congueLteDevice));
    congueIpIfaceList.Add(congueIpIface);

    // set the default gateway for the UE
    Ptr<Ipv4StaticRouting> congueStaticRouting = ipv4RoutingHelper.GetStaticRouting (congue->GetObject<Ipv4> ());
    congueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

    lteHelper->Attach(congueLteDevice, enbDevices.Get(0));
    lteHelper->ActivateDedicatedEpsBearer (congueLteDevice, EpsBearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT), EpcTft::Default ());
  }

  /********************** Finished LTE Networks ********************/



  /******************** Define WiFi stack *****************/
  NodeContainer nodesWifiAp;
  nodesWifiAp.Create (1);

  // PRISON SENSOR SCENARIO:
  // Passive RF sensors are created separately from the WiFi AP/GCS node.
  // perimeter8 uses 8 sensors; cluster16 uses 16 sensors.
  // station24 uses 24 sensors: 8 stations with 3 receivers each.
  // fence24 uses 20 outer-fence sensors plus 4 secure-wall midpoint sensors.
  // They do not send control packets; they only log detection metrics.
  NodeContainer sensorNodes;
  sensorNodes.Create (GetPrisonSensorCount (g_sensorLayout));

  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();

  // PRISON SENSOR SCENARIO:
  // Keep the simulated UAV/AP WiFi transmit power aligned with the received
  // power calculation used by the passive sensor logger.
  wifiPhy.Set ("TxPowerStart", DoubleValue (RF_TX_POWER_DBM));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (RF_TX_POWER_DBM));

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay (
    "ns3::ConstantSpeedPropagationDelayModel"
);
 
  wifiChannel.AddPropagationLoss (
    "ns3::FriisPropagationLossModel",
    "Frequency", DoubleValue (2.4e9)
);

  wifiPhy.SetChannel (wifiChannel.Create ()); //wifi with Friis propogation model


  /*************** Create and configure MAC layer **************/
  Ssid ssid; 
  WifiHelper wifi;
  wifi.SetStandard(WIFI_PHY_STANDARD_80211g);
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager ("ns3::ArfWifiManager");

  NetDeviceContainer devicesWifiAp;
  NetDeviceContainer devicesWifiSta;
  NetDeviceContainer devicesWifiCong;

  char ssidString[10] = {'\0'};
  sprintf(ssidString, "AP_1");
  ssid = Ssid (ssidString);

  wifiMac.SetType ("ns3::ApWifiMac",
                   "Ssid", SsidValue (ssid));
  devicesWifiAp.Add(wifi.Install (wifiPhy, wifiMac, nodesWifiAp.Get (0)));

  for (uint32_t i = 0; i < uavNode.GetN(); ++i){
    wifiMac.SetType ("ns3::StaWifiMac",
                   "Ssid", SsidValue (ssid),
                   "ActiveProbing", BooleanValue (false));
    devicesWifiSta.Add (wifi.Install (wifiPhy, wifiMac, NodeContainer (uavNode.Get (i)))); //for all UAVs
  }

  for (uint32_t k = 0; k < congNode.GetN(); ++k){
    wifiMac.SetType ("ns3::StaWifiMac",
                   "Ssid", SsidValue (ssid),
                   "ActiveProbing", BooleanValue (false));
    devicesWifiCong.Add (wifi.Install (wifiPhy, wifiMac, NodeContainer (congNode.Get (k))));
  }

  std::cout << " AP MAC ADDRESS : "  << devicesWifiAp.Get(0)->GetAddress() << std::endl;
  std::cout << " UAV MAC ADDRESS : "  << devicesWifiSta.Get(0)->GetAddress() << std::endl;  // Use this MAC address to track the SINR for specific UAV


  /********** Define Initial Position and Mobility of WiFi AP ************/
  /***********************************************************************/
  // For the AP or base station
  Ptr<ListPositionAllocator> positionAllocAp = CreateObject<ListPositionAllocator> ();
  positionAllocAp->Add (PRISON_BASE_STATION_POS);
  MobilityHelper mobilityAp;
  mobilityAp.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityAp.SetPositionAllocator(positionAllocAp);
  mobilityAp.Install (nodesWifiAp);
  Simulator::Schedule (Seconds (1.0), &PrintDistance, uavNode.Get (0), nodesWifiAp.Get (0));

  // PRISON SENSOR SCENARIO:
  // Baseline and comparison layouts:
  // perimeter8 uses corners plus side midpoints.
  // cluster16 uses corners plus three-sensor clusters around side midpoints.
  // station24 uses three-receiver stations at all 8 perimeter reference points.
  // fence24 uses an outer-fence clear-zone layout for localisation triangles.
  /********** Define Initial Positions of Passive RF Sensors ************/
  Ptr<ListPositionAllocator> positionAllocSensors = CreateObject<ListPositionAllocator> ();
  AddPrisonSensorsForLayout (positionAllocSensors, g_sensorLayout);

  MobilityHelper mobilitySensors;
  mobilitySensors.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilitySensors.SetPositionAllocator(positionAllocSensors);
  mobilitySensors.Install (sensorNodes);

  // PRISON SENSOR SCENARIO:
  // Start one-second periodic logging after both UAV and sensor mobility exist.
  Simulator::Schedule (Seconds (1.0), &LogSensorDetections, uavNode.Get (0), sensorNodes);
 
  /************** Define IP stack for WiFi AP *************/
  InternetStackHelper internetWifi;
  internetWifi.Install (nodesWifiAp);

  Ipv4AddressHelper ipv4Wifi;
  Ipv4InterfaceContainer interfacesWifiAp;
  Ipv4InterfaceContainer interfacesWifiSta;
  Ipv4InterfaceContainer interfacesWifiCong;
    char ipString[30] = {'\0'};
    sprintf(ipString, "10.10.1.0");
    ipv4Wifi.SetBase (ipString, "255.255.255.0");
    interfacesWifiAp.Add(ipv4Wifi.Assign (devicesWifiAp.Get(0)));
    for (uint32_t i = 0; i < uavNode.GetN(); ++i){
      interfacesWifiSta.Add(ipv4Wifi.Assign (devicesWifiSta.Get(i)));
    }
  interfacesWifiCong.Add(ipv4Wifi.Assign (devicesWifiCong));
   
  /******************************************************************/




  /************ Write Application *****************/
  /************************************************/


  if(network_type == 0)
  {
  /************** Uplink Data transfer for Telemetry *************/
  for (uint32_t i = 0; i < uavNode.GetN(); ++i){
    Ptr<Node> remoteWifiHost = nodesWifiAp.Get (0);
    Ipv4Address remoteWifiHostAddr = interfacesWifiAp.GetAddress (0);
    uint16_t sinkport = 100+i;
    Address sinkAddress(InetSocketAddress (remoteWifiHostAddr, sinkport));
    ApplicationContainer sinkApp;
    PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), sinkport));
    sinkApp = packetSinkHelper.Install(remoteWifiHost);
    sinkApp.Start(Seconds(0.0));

    Ptr<Socket> ns3TcpSocket = Socket::CreateSocket(uavNode.Get(i), TcpSocketFactory::GetTypeId());
    Ptr<MyApp> app = CreateObject<MyApp>();
    app->Setup(ns3TcpSocket, sinkAddress, 1400, 50000, DataRate("1Mbps"), (2*i), 1);
    uavNode.Get(i)->AddApplication(app);
    app->SetStartTime(Seconds(1.0));
    std::cout << "APP : " << app << std::endl; 
    uavApp = app;
    appVectTel.push_back(app);
  }
 }
 else
 {
  /************** LTE Uplink Data transfer for Telemetry *************/
  for (uint32_t i = 0; i < uavNode.GetN(); ++i){
    uint16_t sinkport = 110+i;
    Address sinkAddress(InetSocketAddress (remoteHostAddr, sinkport));
    ApplicationContainer sinkApp;
    PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), sinkport));
    sinkApp = packetSinkHelper.Install(remoteHost);
    sinkApp.Start(Seconds(0.1));

    Ptr<Socket> ns3TcpSocket = Socket::CreateSocket(uavNode.Get(i), TcpSocketFactory::GetTypeId());
    Ptr<MyApp> app = CreateObject<MyApp>();
    app->Setup(ns3TcpSocket, sinkAddress, 1400, 50000, DataRate("1Mbps"), (2*i), 1);
    uavNode.Get(i)->AddApplication(app);
    app->SetStartTime(Seconds(1.0));
    uavApp = app;
    appVectTel.push_back(app);

  }

 }
 

 
  if(network_type == 1)
  {
  /************** LTE Downlink Data transfer for Control *************/
   for (uint32_t i = 0; i < uavNode.GetN(); ++i){
    Ptr<Node> remoteLteHost = uavNode.Get (i);
    Ipv4Address remoteLteHostAddr = ueIpIfaceList.GetAddress (i);
    //Ipv4Address remoteHostAddrWifi = interfacesWifiSta.GetAddress (i);

    uint16_t sinkport = 2000+i;
    Address sinkAddress(InetSocketAddress (remoteLteHostAddr, sinkport));
    ApplicationContainer sinkApp;
    PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), sinkport));
    sinkApp = packetSinkHelper.Install(remoteLteHost);
    sinkApp.Start(Seconds(0.0));

    Ptr<Socket> ns3TcpSocket = Socket::CreateSocket(remoteHost, TcpSocketFactory::GetTypeId());
    Ptr<MyApp> app = CreateObject<MyApp>();
    app->Setup(ns3TcpSocket, sinkAddress, 1400, 50000, DataRate("1Mbps"), (2*i), 1);
    remoteHost->AddApplication(app);
    app->SetStartTime(Seconds(1.0));
    std::cout << "Sending LTE packet on Downlink" << std::endl;
    gcsApp = app;
    appVectCom.push_back(app);
  }
 }
 else
 { 
  /************** Downlink Data transfer for Control commands *************/
  for (uint32_t i = 0; i < uavNode.GetN(); ++i){
    Ptr<Node> remoteWifiHost = uavNode.Get (i);
    Ipv4Address remoteWifiHostAddr = interfacesWifiSta.GetAddress (i);
    uint16_t sinkport = 1000+i;
    Address sinkAddress(InetSocketAddress (remoteWifiHostAddr, sinkport));
    ApplicationContainer sinkApp;
    PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), sinkport));
    sinkApp = packetSinkHelper.Install(remoteWifiHost);
    sinkApp.Start(Seconds(0.0));

    Ptr<Socket> ns3TcpSocket = Socket::CreateSocket(nodesWifiAp.Get(0), TcpSocketFactory::GetTypeId());
    Ptr<MyApp> app = CreateObject<MyApp>();
    app->Setup(ns3TcpSocket, sinkAddress, 1400, 50000, DataRate("1Mbps"), (2*i), 0);
    nodesWifiAp.Get(0)->AddApplication(app);
    app->SetStartTime(Seconds(1.0));
    std::cout << "APP : " << app << std::endl; 
    gcsApp = app;
    appVectCom.push_back(app);
  }
 }



 if(network_type == 0)
 {
   /************** WiFi Uplink Data transfer for Congestion *************/
  if (DataRate(congRate) > 0)
  {
   for (uint32_t i = 0; i < congNode.GetN(); ++i){
    Ptr<Node> remoteHost = nodesWifiAp.Get (0);
    Ipv4Address remoteHostAddr = interfacesWifiAp.GetAddress (0);
    uint16_t sinkport = 1234+ i;
    Address sinkAddress(InetSocketAddress (remoteHostAddr, sinkport));
    ApplicationContainer sinkApp;
    PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), sinkport));
    sinkApp = packetSinkHelper.Install(remoteHost);
    sinkApp.Start(Seconds(0.1));

    Ptr<Socket> ns3TcpSocket = Socket::CreateSocket(congNode.Get(i), TcpSocketFactory::GetTypeId());
    Ptr<MyApp> app = CreateObject<MyApp>();
    app->Setup(ns3TcpSocket, sinkAddress, congPktSize, 500000000, DataRate(congRate), (101+i), 1);
    app->m_congId = i;
    app->m_rate = inputCongRate;
    congNode.Get(i)->AddApplication(app);
    app->SetStartTime(Seconds(0.1));
    std::cout << "Sending Congestion Traffic" <<std::endl;
    app->SendPacket();
   }
  }
  else
  {
    std::cout << " Congestion Traffic rate = 0" <<std::endl;
  }
 }
 else
 {
  if (DataRate(congRate) > 0)
  {
   /************** LTE Uplink Data transfer for Congestion *************/
   for (uint32_t i = 0; i < congNode.GetN(); ++i){
    uint16_t sinkport = 500+i;
    Address sinkAddress(InetSocketAddress (remoteHostAddr, sinkport));
    ApplicationContainer sinkApp;
    PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), sinkport));
    sinkApp = packetSinkHelper.Install(remoteHost);
    sinkApp.Start(Seconds(0.1));

    Ptr<Socket> ns3TcpSocket = Socket::CreateSocket(congNode.Get(i), TcpSocketFactory::GetTypeId());
    Ptr<MyApp> app = CreateObject<MyApp>();
    app->Setup(ns3TcpSocket, sinkAddress, congPktSize, 500000000, DataRate(congRate), (101+i), 1);
    app->m_congId = i;
    app->m_rate = inputCongRate;
    congNode.Get(i)->AddApplication(app);
    app->SetStartTime(Seconds(0.1));
    std::cout << "Sending Congestion Traffic" <<std::endl;
    app->SendPacket();

   }
  }
  else
  {
    std::cout << " Congestion Traffic rate = 0" <<std::endl;
  }
 }
 

  /************* CREATE PUB SUB THREADS ******************/
  int err;
  err = pthread_create(&(tid_gcs[0]), NULL, &rcvCommands, (void *)(&appVectCom));
  if(err != 0)
            printf("\n can't create thread : [%s]", strerror(err));
    else
            printf("\n Command Thread created successfully \n");

  err = pthread_create(&(tid_uav[0]), NULL, &rcvTelemetry, (void *)(&appVectTel));
  if(err != 0)
            printf("\n can't create thread : [%s]", strerror(err));
    else
            printf("\n Telemetry Thread created successfully \n");


  sleep(2);
 
  rc = zmq_connect (subscriber, "tcp://localhost:5000");
  assert (rc == 0);
  zmq_setsockopt( subscriber, ZMQ_SUBSCRIBE, "", 0);

  /*****************************************************/


  std::cout << "Current Simulation Time :" << Simulator::Now().GetMilliSeconds () << std::endl;


  Simulator::Schedule(Seconds(1.001), RcvPktTrace);
  Simulator::Stop (Seconds (100000.));
  Simulator::Run ();
  Simulator::Destroy ();
}
