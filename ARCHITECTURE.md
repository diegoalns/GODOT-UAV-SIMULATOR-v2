# Architecture Documentation

## System Overview

The Godot UAV Simulator is a distributed simulation system consisting of two main components:

1. **Godot Client** (GDScript): 3D visualization, physics simulation, and drone management
2. **Python Server**: Pathfinding, route planning, and conflict resolution

These components communicate via WebSocket protocol for real-time bidirectional data exchange.

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    GODOT CLIENT                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         SimulationEngine (Main Coordinator)           │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌───────────┐ │  │
│  │  │DroneManager  │  │FlightPlanMgr │  │RoutePreReq│ │  │
│  │  └──────┬───────┘  └──────┬───────┘  └─────┬──────┘ │  │
│  │         │                 │                 │         │  │
│  │  ┌──────▼─────────────────▼─────────────────▼───────┐ │  │
│  │  │         WebSocketManager (Autoload)              │ │  │
│  │  └───────────────────────┬─────────────────────────┘ │  │
│  └───────────────────────────┼───────────────────────────┘  │
│                               │                               │
│  ┌───────────────────────────▼───────────────────────────┐  │
│  │         VisualizationSystem (3D Rendering)              │  │
│  │  - Terrain GridMap                                      │  │
│  │  - Drone Meshes                                         │  │
│  │  - Drone Labels (3D Text)                               │  │
│  │  - Camera Controls                                      │  │
│  └─────────────────────────────────────────────────────────┘  │
└───────────────────────────┬───────────────────────────────────┘
                            │ WebSocket (ws://localhost:8765)
                            │
┌───────────────────────────▼───────────────────────────────────┐
│                    PYTHON SERVER                               │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │         WebSocketServer (asyncio)                         │ │
│  │  ┌────────────────────────────────────────────────────┐ │ │
│  │  │         CBS Pathfinder                              │ │ │
│  │  │  - Conflict-Based Search Algorithm                  │ │ │
│  │  │  - Graph Traversal                                  │ │ │
│  │  │  - Conflict Resolution                              │ │ │
│  │  └────────────────────────────────────────────────────┘ │ │
│  │                                                          │ │
│  │  ┌────────────────────────────────────────────────────┐ │ │
│  │  │         Drone Registry                              │ │ │
│  │  │  - Active drone routes                              │ │ │
│  │  │  - Overfly times                                    │ │ │
│  │  │  - Registry cleanup                                │ │ │
│  │  └────────────────────────────────────────────────────┘ │ │
│  └──────────────────────────────────────────────────────────┘ │
│                                                               │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │         Graph Data (NetworkX)                            │ │
│  │  - Regular lattice graph                                │ │
│  │  - Node positions (lat, lon, alt)                       │ │
│  │  - Edge weights (distances)                             │ │
│  └──────────────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────┘
```

## Component Details

### Godot Client Components

#### 1. SimulationEngine (`scripts/core/simulation_engine.gd`)

**Purpose**: Main simulation coordinator and loop manager

**Responsibilities**:
- Manages simulation time and speed multiplier
- Coordinates all subsystems (drone manager, flight plans, visualization)
- Executes three-phase simulation loop:
  1. Route pre-requests (10 min before ETD)
  2. Drone creation from heap (at ETD)
  3. Fallback queue-based drone launching
- Updates all drones each physics frame
- Handles UI interactions

**Key Variables**:
- `simulation_time: float` - Current simulation time in seconds
- `speed_multiplier: float` - Time acceleration factor (1x, 2x, 5x, 10x)
- `running: bool` - Simulation pause/play state
- `headless_mode: bool` - Visualization on/off toggle

**Dependencies**:
- `DroneManager` - Drone lifecycle
- `FlightPlanManager` - Flight plan loading
- `RoutePreRequestManager` - Route pre-request system
- `VisualizationSystem` - 3D rendering
- `SimpleUI` - User interface (includes ActiveDronesPanel)
- `SimpleLogger` - CSV logging
- `DebugLogger` - Advanced logging (if autoload configured)

**UI Integration**:
- Connects ActiveDronesPanel to DroneManager via `ui.set_drone_manager(drone_manager)`
- ActiveDronesPanel displays active drones with first waypoint times on left side of screen

#### 2. DroneManager (`scripts/core/drone_manager.gd`)

**Purpose**: Manages all active drone instances

**Responsibilities**:
- Creates drone instances from flight plans
- Updates all drones each frame
- Removes completed drones from memory (including visualization cleanup)
- Coordinates with visualization system

**Key Data Structures**:
- `drones: Dictionary` - Active drones keyed by ID (String → Drone)
- `visualization_system: VisualizationSystem` - Reference to visualization system for cleanup

**Key Functions**:
- `create_test_drone()` - Creates and initializes a drone
- `update_all()` - Updates all drones with delta time
- `remove_completed_drones()` - Removes completed drones and cleans up their visual representations (meshes and labels)

#### 3. FlightPlanManager (`scripts/core/flight_plan_manager.gd`)

**Purpose**: Loads and manages flight plan queue

**Responsibilities**:
- Loads flight plans from CSV file
- Maintains sorted queue by ETD
- Provides plans needing route requests (10 min before ETD)
- Provides plans ready to launch (at ETD)
- Converts lat/lon to world coordinates

**Key Data Structures**:
- `flight_plan_queue: Array` - Sorted queue of flight plans (Dictionary objects)

**Key Functions**:
- `load_flight_plans()` - Loads CSV and sorts by ETD
- `get_plans_needing_route_requests()` - Returns plans needing pre-requests
- `get_next_pending_plans()` - Returns plans ready to launch
- `latlon_to_position()` - Geographic to world coordinate conversion

**CSV Format**:
- 13 columns: FlightPlanID, DronePortID, ETD, ETD_Seconds, OriginLat, OriginLon, OriginNodeID, DestinationLat, DestinationLon, DestinationNodeID, DroneModel, EstimatedFlightTime, Ceiling

#### 4. RoutePreRequestManager (`scripts/core/route_pre_request_manager.gd`)

**Purpose**: Manages route pre-request system (10 minutes before ETD)

**Responsibilities**:
- Sends route requests 10 minutes before ETD
- Stores successful routes in min-heap ordered by ETD
- Tracks pending requests and handles timeouts
- Provides routes from heap when ETD is reached

**Key Data Structures**:
- `successful_routes_heap: Array` - Min-heap of route entries (Dictionary with etd, plan_id, received_time)
- `route_storage: Dictionary` - Full route data keyed by plan_id
- `pending_route_requests: Dictionary` - Tracking pending requests

**Key Functions**:
- `send_route_request()` - Sends WebSocket route request
- `peek_earliest_route()` - Gets earliest route without removing
- `pop_earliest_route()` - Removes and returns earliest route
- `check_timeouts()` - Handles timed-out requests (5 second timeout)
- `cleanup_stale_routes()` - Removes routes with ETD far in the past
- `get_heap_stats()` - Returns heap and storage statistics
- `heap_insert()` - Inserts entry into min-heap
- `_bubble_up()` / `_bubble_down()` - Heap maintenance functions

**Memory Optimization**:
- Heap stores minimal metadata (~50 bytes per entry)
- Full route arrays stored separately in `route_storage`
- Maximum heap size: 1000 routes
- Automatic cleanup of stale routes (default: 5 minutes old)

**Constants**:
- `ROUTE_REQUEST_TIMEOUT: float = 5.0` - Timeout for pre-requests (system clock time)
- `MAX_HEAP_SIZE: int = 1000` - Maximum routes in heap

#### 5. WebSocketManager (`scripts/core/WebSocketManager.gd`)

**Purpose**: WebSocket client for Python server communication

**Type**: Autoload singleton (accessible globally)

**Responsibilities**:
- Manages WebSocket connection to Python server
- Handles connection, disconnection, and reconnection
- Sends messages to server
- Emits signals for received data

**Key Signals**:
- `connected` - Emitted when connection established
- `disconnected` - Emitted when connection lost
- `data_received(data)` - Emitted when data received from server

**Key Functions**:
- `connect_to_server(url)` - Initiates connection
- `send_message(message)` - Sends JSON string to server
- `_physics_process()` - Polls WebSocket each physics frame (100Hz)

**Configuration**:
- Default URL: `ws://localhost:8765`
- Reconnect delay: 3 seconds
- Polling rate: 100 Hz (physics frame rate)

#### 6. VisualizationSystem (`scripts/core/visualization_system.gd`)

**Purpose**: 3D rendering and camera controls

**Responsibilities**:
- Renders terrain using GridMap
- Displays drone meshes (LRVTOL_UAV.glb model)
- Displays 3D labels above drones showing ID, model, speed, and status
- Manages camera (balloon-style free camera)
- Handles user input for camera movement
- Manages lighting and environment

**Key Components**:
- `terrain_gridmap: GridMap` - Terrain visualization
- `gridmap_manager: GridMapManager` - Terrain data management
- `drone_meshes: Dictionary` - Visual drone representations (String → Node3D)
- `drone_labels: Dictionary` - 3D text labels above drones (String → Label3D)
- `balloon_ref: CharacterBody3D` - Camera controller

**Label System**:
- `show_drone_labels: bool` - Enable/disable label display (default: true)
- `label_offset_height: float` - Vertical offset above drone in meters (default: 200.0)
- `label_font_size: int` - Font size in pixels (default: 24)
- `label_billboard_mode: Label3D.BillboardMode` - Billboard mode for camera-facing labels
- Labels display: drone ID, model type, current speed (if moving), and status (waiting/completed)
- Labels automatically update position and text as drones move

**Key Functions**:
- `add_drone()` - Adds visual representation and label for drone
- `update_drone_position()` - Updates drone mesh position and label text
- `remove_drone()` - Removes drone visualization and label
- `setup_terrain()` - Initializes terrain GridMap

**Camera Controls**:
- WASD: Move camera
- Mouse: Rotate camera
- Mouse wheel: Adjust speed
- Escape: Toggle mouse capture

#### 7. GridMapManager (`scripts/core/gridmap_manager.gd`)

**Purpose**: Manages terrain GridMap data loading and population

**Responsibilities**:
- Loads terrain altitude data from CSV file
- Maps CSV coordinates directly to grid indices
- Populates GridMap with terrain tiles
- Provides terrain altitude queries

**Key Data Structures**:
- `terrain_data: Dictionary` - CSV data storage
- `lat_to_grid_z: Dictionary` - Latitude to grid Z index mapping
- `lon_to_grid_x: Dictionary` - Longitude to grid X index mapping
- `grid_to_altitude: Dictionary` - Grid coordinates to altitude mapping

**Key Functions**:
- `initialize_gridmap()` - Initializes GridMap node
- `load_terrain_data()` - Loads CSV terrain data
- `populate_gridmap()` - Populates GridMap with terrain tiles
- `get_terrain_altitude_at_position()` - Queries altitude at world position

**CSV File**:
- `Filtered_FAA_UAS_FacilityMap_Data_LGA.csv` - FAA UAS facility map data
- Format: CEILING (feet), LATITUDE, LONGITUDE

**Grid Properties**:
- `tile_width: float = 705.11` - Tile width in meters (X axis, longitude)
- `tile_height: float = 927.67` - Tile height in meters (Z axis, latitude)
- Grid spacing: 1/120 degrees (~0.008333 degrees)

#### 8. SimpleLogger (`scripts/core/simple_logger.gd`)

**Purpose**: CSV logging system for simulation data

**Type**: Singleton (via static `instance` variable)

**Responsibilities**:
- Logs drone states to CSV files
- Logs collision events
- Logs mean distance calculations
- Manages log file creation and writing

**Key Log Files**:
- `logs/simple_log.csv` - Main drone states (Time, DroneID, Position, etc.)
- `logs/mean_distances.csv` - Mean distance between drones over time
- `logs/collision_log.csv` - Collision events (start/end, distances, positions)

**Key Functions**:
- `create_log_file()` - Creates log files and directories
- `update()` - Called each frame to log data
- `log_drone_states()` - Logs current drone positions and states
- `log_mean_distance()` - Logs mean distance between all drones
- `log_collision_event()` - Logs collision start/end events

**Logging Interval**:
- `log_interval: float = 10.0` - Logs every 10 seconds of simulation time

#### 9. DebugLogger (`scripts/core/DebugLogger.gd`)

**Purpose**: Advanced logging system with categories and verbosity levels

**Type**: Autoload singleton (when configured in Project Settings)

**Responsibilities**:
- Categorized logging (ROUTE, WEBSOCKET, DRONE, etc.)
- Verbosity level control (SILENT, MINIMAL, NORMAL, VERBOSE)
- Log level filtering (DEBUG, INFO, WARNING, ERROR)
- Timestamp and formatting support

**Key Enums**:
- `LogLevel`: DEBUG, INFO, WARNING, ERROR
- `VerbosityLevel`: SILENT, MINIMAL, NORMAL, VERBOSE
- `Category`: ROUTE, WEBSOCKET, DRONE, SIMULATION, TERRAIN, VISUALIZATION, HEAP, FLIGHT_PLAN, GENERAL

**Key Functions**:
- `log_debug()` / `log_info()` / `log_warning()` / `log_error()` - Category-based logging
- `should_show_verbose()` - Checks if verbose output is enabled
- `should_show_tables()` - Checks if table formatting is enabled
- `get_current_simulation_time()` - Gets simulation time for timestamps

**Configuration**:
- `current_log_level: LogLevel = LogLevel.INFO` - Minimum log level
- `current_verbosity: VerbosityLevel = VerbosityLevel.NORMAL` - Detail level
- `category_enabled: Dictionary` - Per-category enable/disable flags

#### 10. SimpleUI (`scripts/ui/simple_ui.gd`)

**Purpose**: User interface controls for simulation

**Responsibilities**:
- Start/Pause button control
- Speed multiplier slider (0.5x to 5.0x)
- Headless mode toggle
- Drone port selector
- Time display (simulation time and real runtime)
- Active drones panel integration

**Key Signals**:
- `start_requested` - Emitted when start button pressed
- `pause_requested` - Emitted when pause button pressed
- `speed_changed(multiplier)` - Emitted when speed slider changes
- `headless_mode_changed(enable)` - Emitted when headless toggle changes
- `port_selected(port_id)` - Emitted when port selector changes

**Key Components**:
- `active_drones_panel: ActiveDronesPanel` - Panel displaying active drones with first waypoint times

**Key Functions**:
- `setup_ui()` - Creates UI elements and active drones panel
- `set_drone_ports()` - Populates port selector dropdown
- `set_drone_manager()` - Connects active drones panel to DroneManager
- `update_time()` - Updates time display label
- `update_status()` - Updates status label text

#### 10a. ActiveDronesPanel (`scripts/ui/active_drones_panel.gd`)

**Purpose**: Displays active (flying) drones with their first waypoint times

**Responsibilities**:
- Shows list of active drones on left side of screen
- Displays drone ID and first waypoint time (simulation time)
- Updates in real-time as drones become active or complete
- Sorts drones by first waypoint time (earliest first)

**Key Variables**:
- `drone_manager: DroneManager` - Reference to DroneManager for accessing drone data
- `panel_width: int` - Width of panel in pixels (default: 250)
- `title_font_size: int` - Font size for title label (default: 18)
- `drone_entry_font_size: int` - Font size for drone entries (default: 14)

**Key Functions**:
- `setup_panel()` - Creates and configures panel UI elements
- `set_drone_manager()` - Sets DroneManager reference
- `update_drone_list()` - Updates displayed list of active drones
- `create_drone_entry()` - Creates UI entry for a single drone

**Display Criteria**:
- Drone must not be completed (`completed = false`)
- Drone must not be waiting for route response (`waiting_for_route_response = false`)
- Drone must have valid first waypoint time (`first_waypoint_time >= 0.0`)

**Update Frequency**:
- Updates every frame via `_process()` to reflect current active drones

#### 11. Drone (`scripts/drone/drone.gd`)

**Purpose**: Individual drone behavior and movement

**Type**: Extends `Area3D` (for collision detection)

**Responsibilities**:
- Holonomic movement along waypoint route
- Route request/response handling
- Collision detection via Area3D signals
- Waypoint arrival detection
- Wait period handling (e.g., 60s at destination)

**Key State Variables**:
- `current_position: Vector3` - Current world position
- `route: Array` - Array of waypoint dictionaries
- `current_waypoint_index: int` - Current waypoint index
- `waiting_for_route_response: bool` - Route request pending flag
- `first_waypoint_time: float` - Simulation time when first waypoint should be reached (seconds, -1.0 if not set)
- `is_colliding: bool` - Collision state
- `collision_partners: Array` - IDs of colliding drones

**Key Functions**:
- `initialize()` - Sets up drone with start/end positions and model
- `update()` - Updates position and checks waypoint arrival
- `_process_server_route()` - Converts Python route to Godot waypoints
- `_on_route_response_received()` - Handles WebSocket route response
- `_on_area_entered()` / `_on_area_exited()` - Collision detection handlers

**Movement Model**:
- Holonomic (direct waypoint following, no physics constraints)
- Speed varies by flight phase (takeoff: 60%, cruise: 100%, approach: 70%, landing: 40%)
- Waypoint arrival threshold: 5.0 meters

**Collision Detection**:
- Radius: 15.0 meters per drone (30m diameter safety zone)
- Automatic via Area3D signals
- Logged to CSV via SimpleLogger

### Python Server Components

#### 1. WebSocketServer (`scripts/Python/Route Gen Basic CBS/WebSocketServer.py`)

**Purpose**: WebSocket server for route planning requests

**Responsibilities**:
- Listens for route requests from Godot
- Coordinates pathfinding via CBS algorithm
- Maintains active drone registry
- Sends route responses back to Godot

**Key Functions**:
- `websocket_handler()` - Main message handler
- `cleanup_registry()` - Removes completed drones from registry
- `calculate_overfly_times()` - Computes when drone will pass each node
- `find_closest_node()` - Finds nearest graph node to coordinates

**Message Types**:
- `request_route` - Route planning request from Godot
- `drone_completed` - Notification that drone finished route

**Registry Structure**:
```python
{
  "drone_id": {
    "route_nodes": [node1, node2, ...],  # List of str
    "overfly_times": [t1, t2, ...],      # List of float (seconds)
    "start_time": float                    # Simulation time
  }
}
```

#### 2. CBS Pathfinder (`scripts/Python/Route Gen Basic CBS/cbs_pathfinder.py`)

**Purpose**: Conflict-Based Search pathfinding algorithm

**Algorithm**: Conflict-Based Search (CBS) for multi-agent pathfinding

**Key Features**:
- Plans complete round-trip routes (origin → destination → origin)
- Resolves conflicts between drones using temporal constraints
- Supports wait periods at destination (default: 60 seconds)
- Timeout: 3 seconds maximum pathfinding time

**Key Parameters**:
- `conflict_threshold: float` - Time window for conflict detection (10 seconds)
- `max_cbs_iterations: int` - Maximum algorithm iterations (10)
- `round_trip: bool` - Whether to plan return journey (True)
- `wait_time_at_destination: float` - Wait duration in seconds (60.0)

**Returns**:
- `(path_nodes, overfly_times)` - Tuple of node list and time list, or None if no path found

**Note**: The algorithm plans complete round-trip routes including:
- Outbound path: origin → destination
- Wait period: 60 seconds at destination
- Return path: destination → origin

#### 3. Graph Loader (`scripts/Python/Route Gen Basic CBS/graph_loader.py`)

**Purpose**: Loads graph data from pickle file with comprehensive validation

**Graph Format**:
- NetworkX graph structure (Graph, DiGraph, MultiGraph, or MultiDiGraph)
- Nodes have `pos` attribute: `(lat, lon, alt)` tuple
- Edges have `weight` attribute: distance in meters
- Node IDs: Format `L{level}_X{x}_Y{y}` (e.g., `L0_X0_Y0`)

**Key Function**:
- `load_graph_from_pickle()` - Loads graph from `regular_lattice_graph.pkl`

**Validation Performed**:
- Checks graph type validity
- Validates graph is not empty
- Ensures all nodes have `pos` attribute (creates if missing from lat/lon/altitude)
- Validates edges have `weight` attribute
- Checks graph connectivity (connected/weakly connected/strongly connected)

#### 4. Coordinate Constants (`scripts/Python/Route Gen Basic CBS/coordinate_constants.py`)

**Purpose**: Shared coordinate conversion constants

**Key Constants**:
- `ORIGIN_LAT_DEGREES = 40.55417343` - Reference latitude
- `ORIGIN_LON_DEGREES = -73.99583928` - Reference longitude
- `METERS_PER_DEGREE_LAT = 111320` - Meters per degree latitude
- `METERS_PER_DEGREE_LON` - Calculated based on latitude (~84,613 m/deg at 40.5°N)

**Key Functions**:
- `get_coordinate_bounds_meters()` - Returns coordinate bounds in meters
- `degrees_to_meters()` - Converts lat/lon degrees to meters relative to origin

## Data Flow

### Route Pre-Request Flow (10 minutes before ETD)

```
SimulationEngine (simulation_time)
    │
    ├─> FlightPlanManager.get_plans_needing_route_requests()
    │   └─> Returns plans where (ETD - 600s) <= simulation_time
    │
    ├─> RoutePreRequestManager.send_route_request(plan)
    │   ├─> Creates WebSocket message
    │   ├─> WebSocketManager.send_message(message)
    │   └─> Tracks request in pending_route_requests
    │
    └─> Python Server receives request
        ├─> CBS Pathfinder finds route
        ├─> Stores route in registry
        └─> Sends response back
            │
            └─> RoutePreRequestManager receives response
                ├─> Stores route in heap (ordered by ETD)
                └─> Stores full route in route_storage
```

### Drone Launch Flow (at ETD)

```
SimulationEngine (simulation_time >= ETD)
    │
    ├─> RoutePreRequestManager.peek_earliest_route()
    │   └─> Checks if earliest route ETD <= simulation_time
    │
    ├─> RoutePreRequestManager.pop_earliest_route()
    │   └─> Removes route from heap and returns it
    │
    ├─> DroneManager.create_test_drone()
    │   ├─> Creates Drone instance
    │   ├─> Initializes with precomputed route
    │   └─> Adds to visualization system
    │
    └─> Drone.update() (each physics frame)
        ├─> Moves toward current waypoint
        ├─> Checks waypoint arrival
        └─> Updates collision detection
```

### Fallback Route Request Flow (if no pre-request)

```
Drone.initialize() (no precomputed route)
    │
    ├─> Creates route request message
    ├─> WebSocketManager.send_message(message)
    ├─> Starts timeout timer (90 seconds)
    │
    └─> Waits for response
        │
        ├─> Response received
        │   ├─> Drone._on_route_response_received()
        │   ├─> Processes route waypoints
        │   └─> Starts movement
        │
        └─> Timeout (90 seconds)
            └─> Cancel flight (no default route fallback)
```

## Coordinate Systems

### Geographic Coordinates (Lat/Lon)

- **Format**: Decimal degrees
- **Reference Point**: 
  - Latitude: 40.55417343°
  - Longitude: -73.99583928°
- **Usage**: Flight plan CSV, Python graph nodes, WebSocket messages

### World Coordinates (Godot)

- **Format**: Vector3 (x, y, z) in meters
- **Axes**:
  - X: East/West (positive = East)
  - Y: Up/Down (positive = Up, altitude)
  - Z: North/South (positive = South, inverted from lat)
- **Conversion**: `latlon_to_position()` function in FlightPlanManager

### Graph Node IDs

- **Format**: `L{level}_X{x}_Y{y}` (e.g., `L0_X0_Y0`)
- **Purpose**: O(1) graph lookup instead of O(n) coordinate matching
- **Usage**: Flight plan CSV, WebSocket route requests

## Timing Systems

### Simulation Time

- **Type**: `float` (seconds)
- **Updates**: Every physics frame (100 Hz)
- **Formula**: `simulation_time += time_step * speed_multiplier`
- **Usage**: ETD comparison, route timing, collision logging

### System Clock Time

- **Type**: `float` (seconds since Unix epoch)
- **Purpose**: Network latency measurement, timeout calculation
- **Usage**: WebSocket message timestamps, pathfinding duration

### Hybrid Timing

Both simulation time and system clock time are tracked for:
- **Simulation Logic**: Uses simulation time (affected by pause/speed)
- **Network Metrics**: Uses system clock time (real-world seconds)

## Memory Management

### Route Pre-Request System

- **Heap Storage**: Minimal metadata only (~50 bytes per entry)
- **Full Route Storage**: Separate dictionary for large route arrays
- **Maximum Heap Size**: 1000 routes (prevents unbounded growth)
- **Cleanup**: Routes removed from heap when drone launches

### Drone Registry (Python)

- **Structure**: Dictionary mapping drone_id to route data
- **Cleanup**: Removed when drone completes route (last overfly time < current time)
- **Cleanup Trigger**: On each route request (before pathfinding)

### Drone Instances (Godot)

- **Storage**: Dictionary in DroneManager
- **Cleanup**: Removed when `completed = true` (includes visualization cleanup: meshes and labels)
- **Cleanup Trigger**: `remove_completed_drones()` called each frame
- **Cleanup Order**: 
  1. Remove from visualization system (meshes and labels)
  2. Free drone node from scene tree
  3. Remove from dictionary

## Performance Characteristics

### Physics Rate

- **Rate**: 100 Hz (100 physics ticks per second)
- **Purpose**: Smooth drone movement and collision detection
- **Configuration**: `Engine.physics_ticks_per_second = 100` in SimulationEngine

### Route Pre-Request Timing

- **Pre-Request Window**: 10 minutes (600 seconds) before ETD
- **Purpose**: Allows pathfinding to complete before launch
- **Timeout**: 5 seconds for pre-requests (system clock time, not simulation time)
- **Note**: Timeout uses system clock because network communication happens in real-time

### Pathfinding Timeout

- **CBS Timeout**: 3 seconds (system clock time)
- **Purpose**: Prevents hanging on complex pathfinding problems
- **Fallback**: Returns timeout status, drone flight cancelled

### Route Request Timeout (Individual)

- **Timeout**: 90 seconds (simulation time)
- **Purpose**: Prevents drones waiting indefinitely for routes
- **Behavior**: Always cancels flight (no default route fallback)

## Error Handling

### WebSocket Connection

- **Reconnection**: Automatic with 3-second delay
- **Failed Sends**: Logged as warnings, requests retried on reconnect
- **State Tracking**: `is_connected` flag prevents sending when disconnected

### Pathfinding Failures

- **No Path Found**: Returns `status: "no_path"`, flight cancelled
- **Timeout**: Returns `status: "timeout"`, flight cancelled
- **Graph Node Not Found**: Returns `status: "error"`, flight cancelled

### Route Response Handling

- **Invalid JSON**: Flight cancelled with debug message
- **Missing Route Data**: Flight cancelled with debug message
- **No Path Found** (`status: "no_path"`): Flight cancelled with debug message
- **Pathfinding Error** (`status: "error"`): Flight cancelled with debug message
- **Pathfinding Timeout** (`status: "timeout"`): Flight cancelled with debug message
- **Client Timeout** (90s): Flight cancelled with debug message
- **Wrong Drone ID**: Ignored (signal disconnection prevents processing)
- **Note**: Drones only fly if they receive a valid route from Python server (`status: "success"` with route array)

## Extension Points

### Adding New Drone Models

1. Add model case to `Drone._set_model_attributes()`
2. Update `drone_models_specifications.txt`
3. Ensure CSV includes model name

### Modifying Pathfinding Algorithm

1. Replace `cbs_pathfinder.py` with new algorithm
2. Maintain interface: `(graph, start_node, goal_node, start_time, speed, registry, ...)`
3. Return format: `(path_nodes, overfly_times)` or `None`

### Adding New Message Types

1. Add message type to WebSocket handler in Python
2. Add corresponding signal handler in Godot
3. Update protocol documentation

### Custom Visualization

1. Modify `VisualizationSystem` rendering functions
2. Add new mesh types to `add_drone()`
3. Customize camera controls in `_process()` and `_input()`

---

## Additional Components

### Autoload Singletons

**WebSocketManager** (`scripts/core/WebSocketManager.gd`):
- Configured in `project.godot` as autoload singleton
- Accessible globally via `WebSocketManager`
- Manages WebSocket connection to Python server

**DebugLogger** (`scripts/core/DebugLogger.gd`):
- Optional autoload singleton (not currently in `project.godot`)
- Can be added via Project Settings → Autoload
- Provides categorized logging with verbosity control

### Data Files

**Flight Plans**:
- `data/Regular_Lattice_Manhattan_200 FP_2DP_2Hrs_Fixed.csv` - Main flight plan data
- Format: 13 columns with flight plan details

**Terrain Data**:
- `data/Filtered_FAA_UAS_FacilityMap_Data_LGA.csv` - FAA UAS facility map data
- Format: CEILING (feet), LATITUDE, LONGITUDE
- Used by GridMapManager for terrain visualization

**Graph Data**:
- `scripts/Python/Route Gen Basic CBS/regular_lattice_graph.pkl` - NetworkX graph pickle file
- Contains airspace graph with node positions and edge weights

---

**Last Updated**: 2025-01-27 - Added ActiveDronesPanel component and first_waypoint_time tracking
**Documentation Version**: 1.2

