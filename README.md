# Godot UAV Simulator

A comprehensive multi-drone simulation system built with Godot Engine 4.3, featuring real-time pathfinding, collision detection, and 3D visualization for Urban Air Mobility (UAM) operations.

## Overview

The Godot UAV Simulator is designed to simulate multiple autonomous drones operating in an urban airspace environment. It combines a Godot-based 3D visualization engine with a Python-based pathfinding server using Conflict-Based Search (CBS) algorithms to plan conflict-free routes for multiple drones simultaneously.

### Key Features

- **Multi-Drone Simulation**: Supports concurrent operation of multiple drones with different models and capabilities
- **Conflict-Based Pathfinding**: Python server uses CBS algorithm to plan collision-free routes
- **Real-Time Visualization**: 3D terrain rendering with interactive camera controls
- **Active Drones Display**: Left-side GUI panel showing active drones with their first waypoint times (simulation time)
- **Collision Detection**: Automatic detection and logging of drone proximity events
- **Pre-Request Route System**: Routes are requested 10 minutes before ETD for efficient planning
- **Multiple Drone Models**: Three distinct drone types with varying performance characteristics
- **Terrain System**: GridMap-based terrain visualization with altitude data
- **WebSocket Communication**: Real-time bidirectional communication between Godot and Python

## Project Structure

```
Godot-UAV-Simulator - v2/
├── scripts/
│   ├── core/              # Core simulation systems
│   │   ├── simulation_engine.gd      # Main simulation loop and coordination
│   │   ├── drone_manager.gd           # Drone lifecycle management
│   │   ├── flight_plan_manager.gd    # CSV flight plan loading and queue management
│   │   ├── route_pre_request_manager.gd  # Pre-request route system (10 min before ETD)
│   │   ├── visualization_system.gd    # 3D rendering and camera controls
│   │   ├── WebSocketManager.gd        # WebSocket client for Python communication
│   │   ├── gridmap_manager.gd         # Terrain GridMap management
│   │   ├── simple_logger.gd          # CSV logging system
│   │   └── DebugLogger.gd             # Advanced logging with categories
│   ├── drone/
│   │   └── drone.gd                   # Individual drone behavior and movement
│   ├── Python/
│   │   └── Route Gen Basic CBS/
│   │       ├── WebSocketServer.py     # Python WebSocket server
│   │       ├── cbs_pathfinder.py      # Conflict-Based Search pathfinding
│   │       ├── graph_loader.py        # Graph data loading
│   │       ├── sim_logger.py          # Structured logger + Python CSV route outputs
│   │       └── coordinate_constants.py # Coordinate system constants
│   └── ui/
│       └── simple_ui.gd                # User interface controls
├── scenes/
│   ├── main/
│   │   └── main.tscn                  # Main scene entry point
│   └── GridMap/
│       └── terrain_gridmap.tscn       # Terrain visualization scene
├── data/
│   ├── Regular_Lattice_Manhattan_200 FP_2DP_2Hrs_Ordered.csv  # Flight plan data
│   └── Filtered_FAA_UAS_FacilityMap_Data_LGA.csv  # Terrain altitude data
├── resources/                         # 3D models and meshes
├── drone_models_specifications.txt    # Detailed drone model documentation
└── project.godot                      # Godot project configuration
```

## System Requirements

### Godot Engine
- **Version**: 4.3 (GL Compatibility renderer)
- **Platform**: Windows, Linux, macOS
- **Display**: 1920x1080 recommended

### Python Server
- **Python**: 3.8 or higher
- **Dependencies**:
  - `websockets` - WebSocket server implementation
  - `networkx` - Graph data structures
  - `numpy` - Numerical computations
- **Graph Data**: `regular_lattice_graph.pkl` must be present in Python script directory

### Hardware
- **RAM**: 4GB minimum, 8GB recommended
- **GPU**: OpenGL 3.3 compatible graphics card
- **Storage**: ~500MB for project files

## Quick Start

### 1. Start the Python WebSocket Server

```bash
cd scripts/Python/Route\ Gen\ Basic\ CBS
python WebSocketServer.py
```

The server will start on `ws://localhost:8765` and wait for connections from Godot.

### 2. Open Project in Godot

1. Open Godot Engine 4.3
2. Click "Import" and select the `project.godot` file
3. Click "Import & Edit"
4. Press `F5` or click "Play" to run the simulation

### 3. Control the Simulation

- **Start/Pause**: Use the UI controls in the top-left corner
- **Speed Multiplier**: Adjust simulation speed via slider (0.5x to 5.0x)
- **Headless Mode**: Disable visualization for faster performance
- **Drone Port Selection**: Select different drone ports to view their locations
- **Active Drones Panel**: Left side of screen displays all active (flying) drones with their first waypoint times (simulation time)

## Drone Models

The simulator supports three drone models with distinct performance characteristics:

### 1. Long Range FWVTOL (Fixed-Wing VTOL)
- **Max Speed**: 55.0 m/s (~200 km/h)
- **Max Range**: 150 km
- **Battery**: 2,000 Wh
- **Payload**: 5.0 kg
- **Usage**: ~90% of flight plans (primary workhorse)

### 2. Light Quadcopter
- **Max Speed**: 18.0 m/s (~65 km/h)
- **Max Range**: 8 km
- **Battery**: 250 Wh
- **Payload**: 0.5 kg
- **Usage**: ~1-2% of flight plans (short-range deliveries)

### 3. Heavy Quadcopter
- **Max Speed**: 25.0 m/s (~90 km/h)
- **Max Range**: 15 km
- **Battery**: 800 Wh
- **Payload**: 8.0 kg
- **Usage**: ~5-8% of flight plans (heavy cargo)

For detailed specifications, see `drone_models_specifications.txt`.

## Flight Plan Data

Flight plans are loaded from CSV files in the `data/` directory. The default file is:
- `Regular_Lattice_Manhattan_200 FP_2DP_2Hrs_Ordered.csv`

### CSV Format

Each row contains:
- `FlightPlanID`: Unique identifier (e.g., "FP000001")
- `DronePortID`: Origin port (e.g., "DP1", "DP2")
- `ETD`: Estimated Time of Departure (formatted time)
- `ETD_Seconds`: ETD in seconds (float)
- `OriginLat`, `OriginLon`: Origin coordinates (decimal degrees)
- `OriginNodeID`: Origin graph node ID (e.g., "L0_X0_Y0")
- `DestinationLat`, `DestinationLon`: Destination coordinates
- `DestinationNodeID`: Destination graph node ID
- `DroneModel`: Model type (String)
- `EstimatedFlightTime`: Duration in minutes (float)
- `Ceiling`: Maximum altitude in meters (float)

## Architecture Overview

The simulator uses a hybrid architecture:

1. **Godot Client**: Handles visualization, drone movement, collision detection, and simulation timing
2. **Python Server**: Performs pathfinding using CBS algorithm and maintains drone registry
3. **WebSocket Bridge**: Real-time bidirectional communication between systems

### Key Systems

- **SimulationEngine**: Main coordinator, manages simulation loop and system integration
- **DroneManager**: Creates, updates, and removes drone instances
- **FlightPlanManager**: Loads and queues flight plans from CSV
- **RoutePreRequestManager**: Sends route requests 10 minutes before ETD
- **VisualizationSystem**: 3D rendering, camera controls, terrain visualization
- **WebSocketManager**: Autoload singleton for Python communication

For detailed architecture documentation, see [ARCHITECTURE.md](ARCHITECTURE.md).

## WebSocket Protocol

### Route Request (Godot → Python)

```json
{
  "type": "request_route",
  "drone_id": "FP000001",
  "model": "Long Range FWVTOL",
  "etd_seconds": 1234.5,
  "start_node_id": "L0_X0_Y0",
  "end_node_id": "L0_X6_Y2",
  "start_position": {"lon": 0.0, "lat": 0.0, "alt": 0.0},
  "end_position": {"lon": 1000.0, "lat": 1000.0, "alt": 50.0},
  "max_speed": 55.0,
  "simulation_time": 1234.5,
  "client_request_sent_time": 1698765432.123
}
```

### Route Response (Python → Godot)

```json
{
  "type": "route_response",
  "drone_id": "FP000001",
  "status": "success",
  "route": [
    {
      "lat": 40.55417343,
      "lon": -73.99583928,
      "altitude": 50.0,
      "speed": 55.0,
      "description": "Origin (waypoint 1)"
    }
  ],
  "server_request_received_time": 1698765432.125,
  "server_response_sent_time": 1698765432.456,
  "pathfinding_duration": 0.331
}
```

CBS timing and returned waypoint speeds now use the same `max_speed` provided by Godot (no speed scaling factor applied in Python). For pre-requested routes, CBS temporal planning starts from `etd_seconds`, while `simulation_time` is still used for housekeeping (e.g., registry cleanup). The CBS node conflict time threshold is configured at 20 seconds.

## Collision Detection

- **Detection Radius**: 5.0 meters per drone (10m diameter safety zone)
- **Arrival Threshold**: 5.0 meters for waypoint arrival
- **Method**: Area3D-based automatic collision detection via Godot physics engine
- **Logging**: All collision events logged to CSV via SimpleLogger
- **Visualization**: Persistent collision markers are automatically created at collision locations
  - Orange/yellow sphere markers (10m radius) mark where collisions occurred
  - Markers display drone IDs and distance information
  - Markers remain visible for the entire simulation duration
  - Markers can be toggled on/off via `VisualizationSystem.set_show_collision_markers()`
  - All markers can be cleared via `VisualizationSystem.clear_collision_markers()`

## Coordinate Systems

### Geographic to World Conversion

The simulator uses a reference point for coordinate conversion:
- **Origin Latitude**: 40.55417343° (decimal degrees)
- **Origin Longitude**: -73.99583928° (decimal degrees)

Conversion formula:
- `meters_per_deg_lat = 111320.0`
- `meters_per_deg_lon = 111320.0 * cos(latitude_radians)`
- `x = (lon - origin_lon) * meters_per_deg_lon` (East/West)
- `z = (origin_lat - lat) * meters_per_deg_lat` (North/South, inverted for Godot)

### Graph Node IDs

Nodes use format: `L{level}_X{x}_Y{y}` (e.g., `L0_X0_Y0`)
- Enables O(1) graph lookup instead of O(n) coordinate matching
- Significantly improves pathfinding performance

## Performance Characteristics

- **Physics Rate**: 100 Hz (100 physics ticks per second)
- **Route Pre-Request**: Routes requested 10 minutes (600 seconds) before ETD
- **Route Timeout**: 
  - 5 seconds for pre-requests (system clock time)
  - 90 seconds for individual drone requests (simulation time)
  - 3 seconds for CBS pathfinding (system clock time)
- **Memory Optimization**: Heap-based route storage with maximum 1000 routes
- **Logging Interval**: CSV logs written every 10 seconds of simulation time

## Logging and Output

### CSV Logging (SimpleLogger)
- **Drone States**: `logs/simple_log.csv` - Position, speed, target, completion status
- **Collision Events**: `logs/collision_log.csv` - Collision start/end events with distances
- **Logging Interval**: Every 10 seconds of simulation time

### Python Route/Time CSV Logging
- **Received Routes**: `logs/python_routes_received.csv` - One row per waypoint per planned drone route from Python (includes overfly timing)
- **Startup behavior**: `python_routes_received.csv` is cleared when the Python WebSocket server starts, then rows are appended during that run
- **Notebook waypoint labels**: `logs/uav_route_visualization.ipynb` can annotate each plotted waypoint with `overfly_time_sim_s` from `python_routes_received.csv` (3D and 2D plots)
- **Overlap handling in notebook**: waypoint time labels use seeded random positional jitter (plus a light bbox in 2D) to keep labels readable when drones share the same route/waypoints; current jitter defaults are `label_jitter_xy=0.15`, `label_jitter_z=0.06`, `label_jitter_2d=0.15`
- **Route-color labels in notebook**: each waypoint time label uses the same color as its corresponding plotted route line for easier route association
- **Interactive route controls**: notebook now uses Plotly for interactive 3D/2D route views; click legend items to toggle per-route visibility and use `Labels ON/OFF` buttons to control time-label traces

### Debug Logging (DebugLogger)
- **Categories**: ROUTE, WEBSOCKET, DRONE, SIMULATION, TERRAIN, VISUALIZATION, HEAP, FLIGHT_PLAN, GENERAL
- **Log Levels**: DEBUG, INFO, WARNING, ERROR
- **Verbosity Levels**: SILENT, MINIMAL, NORMAL, VERBOSE
- **Table Format**: All messages use fixed-width columns for unified readable output with Python
- **Note**: DebugLogger is optional and must be configured as autoload singleton in Project Settings

### Cross-Language Log Standard (Fixed-Width Table)

Godot (`DebugLogger`) and Python (`sim_logger.py`) emit logs in a unified fixed-width table format:

| Column | Width | Description |
|--------|-------|-------------|
| ts | 12 | Timestamp (sim time in Godot, Unix time in Python) |
| level | 8 | DEBUG, INFO, WARNING, ERROR |
| category | 14 | Subsystem (ROUTE, WEBSOCKET, DRONE, etc.) |
| source | 6 | `godot` or `python` |
| event | 32 | Short event name |
| data | 150 | Key=value pairs |

Example output (both Godot and Python):

```
ts            level    category       source event                           data
12.34s        INFO     ROUTE          godot  connection_established         {url=ws://localhost:8765}
1730556789.12 INFO     WEBSOCKET      python client_connected                {client_address=127.0.0.1 status=ready}
```

Python logger runtime controls:
- `SIM_LOG_LEVEL`: `DEBUG`, `INFO`, `WARNING`, `ERROR` (default: `INFO`)
- `SIM_LOG_FORMAT`: `table` (default), `json`, or `pretty`

Examples:
- `SIM_LOG_FORMAT=table` for aligned fixed-width output (default)
- `SIM_LOG_FORMAT=json` for one JSON object per line (dashboards/parsing)
- `SIM_LOG_FORMAT=pretty` for compact `[LEVEL][CATEGORY][python] event | key=value` format

## Development

### Key Files to Modify

- **Drone Behavior**: `scripts/drone/drone.gd`
- **Pathfinding Algorithm**: `scripts/Python/Route Gen Basic CBS/cbs_pathfinder.py`
- **Simulation Logic**: `scripts/core/simulation_engine.gd`
- **Visualization**: `scripts/core/visualization_system.gd`
- **Route Pre-Request System**: `scripts/core/route_pre_request_manager.gd`
- **Terrain System**: `scripts/core/gridmap_manager.gd`
- **Logging**: `scripts/core/simple_logger.gd` or `scripts/core/DebugLogger.gd`

### Adding New Drone Models

1. Add model specification to `drone.gd` `_set_model_attributes()` function
2. Update `drone_models_specifications.txt` documentation
3. Ensure CSV flight plan data includes new model name

## Troubleshooting

### WebSocket Connection Failed
- Ensure Python server is running on `localhost:8765`
- Check firewall settings
- Verify no other process is using port 8765

### Drones Not Launching
- Check flight plan CSV file format
- Verify ETD times are in the future
- Check console for error messages

### Pathfinding Timeouts
- Increase CBS timeout in `WebSocketServer.py`
- Reduce number of concurrent drones
- Check graph connectivity

## References

- **Drone Specifications**: See `drone_models_specifications.txt`
- **Architecture Details**: See [ARCHITECTURE.md](ARCHITECTURE.md)
- **Godot Documentation**: https://docs.godotengine.org/
- **NetworkX Documentation**: https://networkx.org/

## License

[Add your license information here]

## Contributors

[Add contributor information here]

---

**Last Updated**: 2026-03-04 - Increased CBS node conflict threshold to 20 seconds and kept `python_routes_received.csv` startup reset behavior
**Godot Version**: 4.3 (GL Compatibility)
**Python Version**: 3.8+

