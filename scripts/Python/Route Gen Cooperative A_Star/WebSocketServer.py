import asyncio
import websockets
import json
import sys
import time  # Import time module for measuring pathfinding duration
from pathlib import Path
import numpy as np
import networkx as nx
from coordinate_constants import *
from graph_loader import load_graph_from_pickle
from cbs_pathfinder import cbs_find_path  # Import CBS pathfinding function
sys.path.append(str(Path(__file__).parent))

# Define helper functions for position mapping
def slant_range(p1, p2):
    """Calculates the 3D distance between two points in meters."""
    dx = p2[0] - p1[0]  # X distance in meters
    dy = p2[1] - p1[1]  # Y distance in meters
    dz = p2[2] - p1[2]  # Z distance in meters
    return np.sqrt(dx**2 + dy**2 + dz**2)

def find_closest_node(graph, target_position):
    """
    Find the closest available node in the graph to the given 3D position.
    
    Args:
        graph: NetworkX graph with nodes having 'pos' attributes
        target_position: dict with lat, lon, alt coordinates
    
    Returns:
        tuple: The closest node identifier or None if no available nodes
    """
    if isinstance(target_position, dict):
        # Handle coordinate format: lon, lat, alt → lat, lon, alt for graph coordinates
        if 'lon' in target_position and 'lat' in target_position and 'alt' in target_position:
            target_pos = (target_position['lat'], target_position['lon'], target_position['alt'])
        else:
            target_pos = (target_position['x'], target_position['y'], target_position['z'])
    else:
        target_pos = tuple(target_position)
    
    min_distance = float('inf')
    closest_node = None
    
    for node in graph.nodes():
        # Only consider available nodes
        if not graph.nodes[node].get('available', True):
            continue
            
        node_pos = graph.nodes[node]['pos']
        distance = slant_range(target_pos, node_pos)
        
        if distance < min_distance:
            min_distance = distance
            closest_node = node
    
    return closest_node


# Load the graph from pickle file using the graph loader module
airspace_graph = load_graph_from_pickle()

# ============================================================================
# DRONE REGISTRY - Tracks active drones with their routes and overfly times
# ============================================================================
# Dictionary structure: {drone_id: {"route_nodes": [node1, node2, ...], 
#                                    "overfly_times": [t1, t2, ...], 
#                                    "start_time": float}}
# Each entry stores the sequence of graph nodes the drone will traverse and
# the simulation time when it will overfly each node
active_drones_registry = {}  # Dictionary mapping drone_id (str) to route data (dict)

def cleanup_registry(current_simulation_time: float):
    """
    Remove drones from registry whose last node overfly time is less than current simulation time.
    This indicates the drone has completed its route.
    
    Args:
        current_simulation_time: float - Current simulation time in seconds from Godot
    """
    drones_to_remove = []  # List of drone IDs to remove from registry (list of str)
    
    # Iterate through all registered drones to check completion status
    for drone_id, route_data in active_drones_registry.items():
        overfly_times = route_data.get("overfly_times", [])  # List of overfly times (list of float)
        
        # Check if drone has completed route (last overfly time < current time)
        if len(overfly_times) > 0:
            last_overfly_time = overfly_times[-1]  # Last overfly time (float, seconds)
            if last_overfly_time < current_simulation_time:
                drones_to_remove.append(drone_id)  # Mark for removal (str)
    
    # Remove completed drones from registry
    for drone_id in drones_to_remove:
        removed_data = active_drones_registry.pop(drone_id, None)  # Remove and get data (dict or None)
        if removed_data:
            print(f"│ {'Registry Cleanup':<25} │ ✓ Removed {drone_id:<48} │")
            print(f"│ {'  Last Overfly':<25} │ {removed_data['overfly_times'][-1]:.2f}s < {current_simulation_time:.2f}s{'':<30} │")
    
    return len(drones_to_remove)  # Return count of removed drones (int)

def calculate_overfly_times(path_nodes: list, start_time: float, speed: float, graph):
    """
    Calculate simulation times when drone will overfly each node in the path.
    Uses edge weights (distances) and drone speed to compute arrival times.
    
    Args:
        path_nodes: List of node IDs representing the route (list of str)
        start_time: Simulation time when route starts (float, seconds)
        speed: Drone speed in m/s (float)
        graph: NetworkX graph with edge weights (NetworkX Graph)
    
    Returns:
        List of overfly times corresponding to each node (list of float, seconds)
    """
    overfly_times = [start_time]  # Initialize with start time (list of float)
    current_time = start_time  # Track cumulative time (float, seconds)
    
    # Calculate time to traverse each edge in the path
    for i in range(len(path_nodes) - 1):
        current_node = path_nodes[i]  # Current node ID (str)
        next_node = path_nodes[i + 1]  # Next node ID (str)
        
        # Get edge weight (distance in meters) from graph
        if graph.has_edge(current_node, next_node):
            edge_weight = graph[current_node][next_node].get('weight', 0.0)  # Edge weight in meters (float)
        else:
            # Fallback: calculate distance from node positions if edge doesn't exist
            pos1 = graph.nodes[current_node]['pos']  # Position tuple (lat, lon, alt) (tuple of float)
            pos2 = graph.nodes[next_node]['pos']  # Position tuple (lat, lon, alt) (tuple of float)
            edge_weight = slant_range(pos1, pos2)  # 3D distance in meters (float)
        
        # Calculate traversal time: distance / speed (time in seconds)
        traversal_time = edge_weight / speed if speed > 0 else 0.0  # Time to traverse edge (float, seconds)
        current_time += traversal_time  # Add to cumulative time (float, seconds)
        overfly_times.append(current_time)  # Store overfly time for next node (float, seconds)
    
    return overfly_times  # Return list of overfly times (list of float)

def print_registry_status(current_simulation_time: float):
    """
    Print debug information about current registry state.
    Shows all active drones, their route nodes, and overfly times.
    
    Args:
        current_simulation_time: Current simulation time for reference (float, seconds)
    """
    registry_size = len(active_drones_registry)  # Number of active drones (int)
    
    print("├" + "─"*25 + "┼" + "─"*50 + "┤")
    print(f"│ {'Registry Status':<25} │ Active Drones: {registry_size:<40} │")
    print(f"│ {'  Simulation Time':<25} │ {current_simulation_time:.2f} seconds{'':<35} │")
    
    # Print details for each active drone
    if registry_size > 0:
        print("├" + "─"*25 + "┼" + "─"*50 + "┤")
        print(f"│ {'Active Drones':<25} │ {'Details':<50} │")
        print("├" + "─"*25 + "┼" + "─"*50 + "┤")
        
        for drone_id, route_data in active_drones_registry.items():
            route_nodes = route_data.get("route_nodes", [])  # List of node IDs (list of str)
            overfly_times = route_data.get("overfly_times", [])  # List of overfly times (list of float)
            start_time = route_data.get("start_time", 0.0)  # Route start time (float, seconds)
            
            # Format route info: show first 3 nodes, "...", last node
            if len(route_nodes) > 4:
                route_str = f"{route_nodes[0]} → {route_nodes[1]} → ... → {route_nodes[-1]}"  # Truncated route string (str)
            else:
                route_str = " → ".join(route_nodes)  # Full route string (str)
            
            # Show first and last overfly times
            first_time = overfly_times[0] if len(overfly_times) > 0 else 0.0  # First overfly time (float, seconds)
            last_time = overfly_times[-1] if len(overfly_times) > 0 else 0.0  # Last overfly time (float, seconds)
            
            print(f"│ {'  ' + drone_id:<23} │ Nodes: {len(route_nodes)}, Times: {first_time:.1f}s → {last_time:.1f}s{'':<15} │")
            print(f"│ {'    Route':<25} │ {route_str[:48]:<48} │")
    else:
        print("├" + "─"*25 + "┼" + "─"*50 + "┤")
        print(f"│ {'  Status':<25} │ No active drones in registry{'':<25} │")

# WebSocket server to handle drone creation messages and respond accordingly
async def websocket_handler(websocket):
    print("\n" + "="*80)
    print("│ WEBSOCKET CLIENT CONNECTED")
    print("="*80)
    try:
        async for message in websocket:
            
            try:
                # Parse the JSON message
                data = json.loads(message)
                
                # Check for drone creation messages
                if data.get("type") == "request_route":
                    drone_id = data.get("drone_id")
                    model = data.get("model")
                    start_pos = data.get("start_position")
                    end_pos = data.get("end_position")
                    max_speed = data.get("max_speed")
                    
                    # Get simulation time from Godot for registry cleanup (float, seconds)
                    simulation_time = data.get("simulation_time", 0.0)
                    
                    # Record when route request message was received (simulation time from Godot)
                    print(f"│ {'Request Received':<25} │ Simulation time: {simulation_time:.3f} seconds{'':<25} │")
                    
                    # NEW: Get node IDs if provided (efficient O(1) lookup)
                    start_node_id = data.get("start_node_id")
                    end_node_id = data.get("end_node_id")
                    
                    # ========================================================================
                    # REGISTRY CLEANUP: Remove completed drones before processing new request
                    # ========================================================================
                    removed_count = cleanup_registry(simulation_time)  # Count of removed drones (int)
                    if removed_count > 0:
                        print(f"│ {'Cleanup Summary':<25} │ Removed {removed_count} completed drone(s){'':<28} │")
                    
                    # Print table header for route request
                    print("\n" + "─"*80)
                    print(f"│ 🚁 ROUTE REQUEST: {drone_id}")
                    print("─"*80)
                    print(f"│ {'Parameter':<25} │ {'Value':<50} │")
                    print("├" + "─"*25 + "┼" + "─"*50 + "┤")
                    print(f"│ {'Drone Model':<25} │ {model:<50} │")
                    print(f"│ {'Max Speed':<25} │ {max_speed:.1f} m/s{'':<42} │")
                    
                    # Determine start and end nodes - prioritize Node IDs for speed
                    start_node = None
                    end_node = None
                    
                    print("├" + "─"*25 + "┼" + "─"*50 + "┤")
                    
                    # METHOD 1: Use Node IDs if provided (FAST - O(1) hash lookup)
                    if start_node_id and end_node_id:
                        print(f"│ {'Node Resolution':<25} │ {'Using Node IDs (O(1) lookup)':<50} │")
                        print(f"│ {'  Start Node ID':<25} │ {start_node_id:<50} │")
                        print(f"│ {'  End Node ID':<25} │ {end_node_id:<50} │")
                        
                        # Direct graph lookup - instant O(1) operation
                        if airspace_graph.has_node(start_node_id):
                            start_node = start_node_id
                            print(f"│ {'  Start Status':<25} │ {'✓ Found in graph':<50} │")
                        else:
                            print(f"│ {'  Start Status':<25} │ {'✗ NOT in graph':<50} │")
                        
                        if airspace_graph.has_node(end_node_id):
                            end_node = end_node_id
                            print(f"│ {'  End Status':<25} │ {'✓ Found in graph':<50} │")
                        else:
                            print(f"│ {'  End Status':<25} │ {'✗ NOT in graph':<50} │")
                    
                    # METHOD 2: Fallback to coordinate mapping if Node IDs not available (SLOW - O(n) search)
                    if start_node is None or end_node is None:
                        print(f"│ {'Node Resolution':<25} │ {'Fallback: Coordinate mapping (O(n) search)':<50} │")
                        print(f"│ {'  Start Coords':<25} │ Lon:{start_pos['lon']:.6f}, Lat:{start_pos['lat']:.6f}, Alt:{start_pos['alt']:.0f}m{' ':<1} │")
                        print(f"│ {'  End Coords':<25} │ Lon:{end_pos['lon']:.6f}, Lat:{end_pos['lat']:.6f}, Alt:{end_pos['alt']:.0f}m{' ':<1} │")
                        
                        if start_node is None:
                            start_node = find_closest_node(airspace_graph, start_pos)
                            if start_node:
                                print(f"│ {'  Start Mapping':<25} │ ✓ Mapped to {str(start_node)[:46]:<46} │")
                        
                        if end_node is None:
                            end_node = find_closest_node(airspace_graph, end_pos)
                            if end_node:
                                print(f"│ {'  End Mapping':<25} │ ✓ Mapped to {str(end_node)[:46]:<46} │")
                    
                    # Check if we successfully found both nodes
                    if start_node is None or end_node is None:
                        print("├" + "─"*25 + "┼" + "─"*50 + "┤")
                        print(f"│ {'STATUS':<25} │ {'❌ ERROR: Could not find valid graph nodes':<50} │")
                        print("└" + "─"*25 + "┴" + "─"*50 + "┘")
                        response = {
                            "type": "route_response",
                            "drone_id": drone_id,
                            "status": "error",
                            "message": "Could not find valid graph nodes for start or end position"
                        }
                        await websocket.send(json.dumps(response))
                        continue
                    
                    print("├" + "─"*25 + "┼" + "─"*50 + "┤")
                    print(f"│ {'Pathfinding':<25} │ Computing conflict-free path (CBS)...{'':<20} │")

                    try:
                        # ========================================================================
                        # CBS PATHFINDING: Find conflict-free path using Conflict-Based Search
                        # ========================================================================
                        # Record start time for pathfinding process (real-world time in seconds)
                        pathfinding_start_time = time.time()  # float: Real-world time when pathfinding starts (seconds)
                        
                        # Use 80% of max speed (same as waypoint speed) for consistency
                        waypoint_speed = max_speed * 0.8  # Waypoint speed in m/s (float)
                        
                        # Run CBS algorithm to find conflict-free round trip path
                        # CBS plans complete route: origin → destination → origin with 60s wait at destination
                        # CBS considers existing routes in registry and avoids conflicts
                        cbs_result = cbs_find_path(
                            graph=airspace_graph,  # NetworkX graph with edge weights (nx.Graph)
                            start_node=start_node,  # Starting node ID (str)
                            goal_node=end_node,  # Goal node ID (str)
                            start_time=simulation_time,  # Simulation time when route starts (float, seconds)
                            speed=waypoint_speed,  # Drone speed for temporal calculations (float, m/s)
                            registry=active_drones_registry,  # Active drones registry for constraint extraction (dict)
                            conflict_threshold=10.0,  # Conflict threshold: 10 seconds (float, seconds)
                            max_cbs_iterations=10,  # Maximum CBS iterations (int)
                            round_trip=True,  # Plan complete round trip route (bool)
                            wait_time_at_destination=60.0  # Wait 60 seconds at destination before return (float, seconds)
                        )
                        
                        # Check if CBS found a path
                        if cbs_result is None:
                            # No conflict-free path found - record timing
                            pathfinding_end_time = time.time()  # float: Real-world time when pathfinding completed with no path (seconds)
                            pathfinding_duration = pathfinding_end_time - pathfinding_start_time  # float: Pathfinding duration in seconds
                            print(f"│ {'Pathfinding Complete':<25} │ Duration: {pathfinding_duration:.3f} seconds (no path){'':<22} │")
                            print(f"│ {'  Path Result':<25} │ ❌ No conflict-free path exists{'':<30} │")
                            print("├" + "─"*25 + "┼" + "─"*50 + "┤")
                            print(f"│ {'STATUS':<25} │ {'❌ ERROR: No conflict-free path found':<50} │")
                            print("└" + "─"*25 + "┴" + "─"*50 + "┘")
                            # Send error response
                            response = {
                                "type": "route_response",
                                "drone_id": drone_id,
                                "status": "no_path",
                                "message": "No conflict-free path found in graph between start and end positions"
                            }
                            await websocket.send(json.dumps(response))
                            continue
                        
                        # Record end time for pathfinding process (real-world time in seconds)
                        pathfinding_end_time = time.time()  # float: Real-world time when pathfinding completes (seconds)
                        pathfinding_duration = pathfinding_end_time - pathfinding_start_time  # float: Pathfinding duration in seconds
                        print(f"│ {'Pathfinding Complete':<25} │ Duration: {pathfinding_duration:.3f} seconds{'':<30} │")
                        
                        # Unpack CBS result: path nodes and overfly times
                        path_nodes, overfly_times = cbs_result  # path_nodes: list of str, overfly_times: list of float
                        
                        # Convert path nodes to geographic coordinates (lat, lon, altitude)
                        # Let Godot handle the conversion to world coordinates for consistency
                        route = []  # List of waypoint dictionaries to send to Godot
                        for i, node in enumerate(path_nodes):
                            # Get node position: tuple of (lat, lon, alt) where lat/lon are in degrees, alt in meters
                            node_pos = airspace_graph.nodes[node]['pos']  # (lat, lon, alt)
                            
                            # Determine waypoint description based on position in route
                            # Check if this is the destination node (where wait occurs)
                            is_destination = (node == end_node)  # Check if this is destination node (bool)
                            
                            # Determine waypoint type for description
                            # Check if next waypoint is also destination (indicates wait period)
                            next_node = path_nodes[i + 1] if i + 1 < len(path_nodes) else None  # Next node ID (str or None)
                            is_wait_waypoint = (is_destination and next_node == end_node and i + 1 < len(overfly_times))  # Check if this is wait waypoint (bool)
                            
                            if i == 0:
                                waypoint_desc = f"Origin (waypoint {i+1})"  # Origin waypoint description (str)
                            elif is_wait_waypoint:
                                # Wait waypoint at destination (duplicate destination node)
                                # Safely calculate wait duration with bounds check
                                if i + 1 < len(overfly_times):
                                    wait_duration = overfly_times[i + 1] - overfly_times[i]  # Wait duration in seconds (float)
                                else:
                                    wait_duration = 60.0  # Fallback wait duration (float, seconds)
                                waypoint_desc = f"Destination - Wait {wait_duration:.0f}s (waypoint {i+1})"  # Wait waypoint description (str)
                            elif is_destination:
                                # First arrival at destination (before wait)
                                waypoint_desc = f"Destination (waypoint {i+1})"  # Destination waypoint description (str)
                            elif i == len(path_nodes) - 1:
                                waypoint_desc = f"Return to origin (waypoint {i+1})"  # Final waypoint description (str)
                            elif node == start_node and i > len(path_nodes) / 2:
                                # Returned to origin
                                waypoint_desc = f"Return to origin (waypoint {i+1})"  # Return origin waypoint description (str)
                            elif i < len(path_nodes) / 2:
                                waypoint_desc = f"Outbound waypoint {i+1}"  # Outbound waypoint description (str)
                            else:
                                waypoint_desc = f"Return waypoint {i+1}"  # Return waypoint description (str)
                            
                            # Create waypoint with geographic coordinates - Godot will convert to world position
                            waypoint = {
                                "lat": node_pos[0],         # Latitude in decimal degrees (float)
                                "lon": node_pos[1],         # Longitude in decimal degrees (float)
                                "altitude": node_pos[2],    # Altitude in meters (float)
                                "speed": waypoint_speed,    # Waypoint speed: 80% of drone's max speed (float, m/s)
                                "description": waypoint_desc  # Human-readable waypoint label (string)
                            }
                            route.append(waypoint)
                        
                        # Debug: Print route structure for verification
                        print(f"│ {'  Route Structure':<25} │ Total waypoints: {len(route)}{'':<32} │")
                        if len(route) > 0:
                            print(f"│ {'    First waypoint':<25} │ {route[0].get('description', 'Unknown')[:48]:<48} │")
                            print(f"│ {'    Last waypoint':<25} │ {route[-1].get('description', 'Unknown')[:48]:<48} │")
                        
                        # Calculate route statistics for display
                        # Use edge weights from graph for accurate distance calculation
                        total_distance = 0.0  # Total route distance in meters (float)
                        for i in range(len(path_nodes) - 1):
                            current_node = path_nodes[i]  # Current node ID (str)
                            next_node = path_nodes[i + 1]  # Next node ID (str)
                            
                            # Get edge weight (distance in meters) from graph
                            if airspace_graph.has_edge(current_node, next_node):
                                edge_weight = airspace_graph[current_node][next_node].get('weight', 0.0)  # Edge weight in meters (float)
                            else:
                                # Fallback: calculate distance from node positions if edge doesn't exist
                                pos1 = airspace_graph.nodes[current_node]['pos']  # Position tuple (lat, lon, alt) (tuple of float)
                                pos2 = airspace_graph.nodes[next_node]['pos']  # Position tuple (lat, lon, alt) (tuple of float)
                                edge_weight = slant_range(pos1, pos2)  # 3D distance in meters (float)
                            
                            total_distance += edge_weight  # Add edge distance to total (float, meters)
                        
                        # Calculate actual flight time from CBS-computed overfly times
                        # Flight time = last overfly time - first overfly time
                        # This accounts for all edge weights and any temporal adjustments made by CBS
                        if len(overfly_times) > 0:
                            actual_flight_time = overfly_times[-1] - overfly_times[0]  # Actual flight time in seconds (float)
                        else:
                            actual_flight_time = 0.0  # Fallback if no overfly times (float, seconds)
                        
                        print(f"│ {'  Path Found':<25} │ ✓ {len(route)} waypoints{'':<36} │")
                        print(f"│ {'  Total Distance':<25} │ {total_distance:.0f} meters{'':<37} │")
                        print(f"│ {'  Est. Flight Time':<25} │ {actual_flight_time:.1f} seconds ({actual_flight_time/60:.1f} minutes){'':<19} │")
                        
                        # ========================================================================
                        # REGISTRY STORAGE: Save route and overfly times to registry
                        # ========================================================================
                        active_drones_registry[drone_id] = {
                            "route_nodes": path_nodes,           # List of node IDs (list of str)
                            "overfly_times": overfly_times,       # List of overfly times (list of float)
                            "start_time": simulation_time         # Route start time (float, seconds)
                        }
                        print(f"│ {'  Registry Update':<25} │ ✓ Saved {drone_id} route to registry{'':<20} │")
                        print(f"│ {'  Route Start Time':<25} │ {simulation_time:.2f} seconds{'':<33} │")
                        print(f"│ {'  Route End Time':<25} │ {overfly_times[-1]:.2f} seconds{'':<33} │")
                        
                        # Print current registry status for debugging
                        print_registry_status(simulation_time)
                        
                        print("├" + "─"*25 + "┼" + "─"*50 + "┤")
                        print(f"│ {'STATUS':<25} │ {'✅ SUCCESS - Conflict-free route computed':<50} │")
                        print("└" + "─"*25 + "┴" + "─"*50 + "┘")
                        
                    except Exception as e:
                        # Handle any unexpected errors during pathfinding
                        # Record end time even if pathfinding failed (real-world time in seconds)
                        pathfinding_end_time = time.time()  # float: Real-world time when pathfinding error occurred (seconds)
                        pathfinding_duration = pathfinding_end_time - pathfinding_start_time  # float: Pathfinding duration before error (seconds)
                        print(f"│ {'Pathfinding Failed':<25} │ Duration: {pathfinding_duration:.3f} seconds (error){'':<20} │")
                        print(f"│ {'  Path Result':<25} │ ❌ Error during pathfinding{'':<28} │")
                        print(f"│ {'  Error Details':<25} │ {str(e)[:48]:<48} │")
                        print("├" + "─"*25 + "┼" + "─"*50 + "┤")
                        print(f"│ {'STATUS':<25} │ {'❌ ERROR: Pathfinding failed':<50} │")
                        print("└" + "─"*25 + "┴" + "─"*50 + "┘")
                        # Send error response
                        response = {
                            "type": "route_response",
                            "drone_id": drone_id,
                            "status": "error",
                            "message": f"Pathfinding error: {str(e)}"
                        }
                        await websocket.send(json.dumps(response))
                        continue


                    # Send acknowledgment with route
                    response = {
                        "type": "route_response",
                        "drone_id": drone_id,
                        "status": "success",
                        "route": route
                    }
                    await websocket.send(json.dumps(response))
                elif data.get("type") == "drone_completed":
                    # Handle drone completion message from Godot
                    drone_id = data.get("drone_id")
                    completion_time = data.get("simulation_time", 0.0)
                    
                    if drone_id in active_drones_registry:
                        # Drone still in registry - remove it explicitly
                        removed_data = active_drones_registry.pop(drone_id)
                        print("\n" + "─"*80)
                        print(f"│ ✓ DRONE COMPLETED: {drone_id}")
                        print("─"*80)
                        print(f"│ {'Completion Time':<25} │ {completion_time:.2f} seconds{'':<33} │")
                        print(f"│ {'Route Nodes':<25} │ {len(removed_data['route_nodes'])} nodes{'':<36} │")
                        print(f"│ {'Last Overfly Time':<25} │ {removed_data['overfly_times'][-1]:.2f} seconds{'':<30} │")
                        print("└" + "─"*80)
                    else:
                        # Drone already removed by time-based cleanup - this is expected and harmless
                        # The cleanup_registry() function already removed it based on overfly time
                        print(f"│ {'Registry Info':<25} │ Drone {drone_id} already cleaned up (time-based){'':<15} │")
                else:
                    # Echo other messages
                    await websocket.send(f"Echo: {message}")
            except json.JSONDecodeError:
                # Handle non-JSON messages - silently ignore or echo
                pass
                
    except websockets.ConnectionClosed:
        print("\n" + "="*80)
        print("│ WEBSOCKET CLIENT DISCONNECTED")
        print("="*80 + "\n")

# Start the server on localhost:8765
async def start_server():
    async with websockets.serve(websocket_handler, 'localhost', 8765):
        print("\n" + "="*80)
        print("│ 🌐 COOPERATIVE A* WEBSOCKET SERVER")
        print("="*80)
        print(f"│ Server Address: ws://localhost:8765")
        print(f"│ Status: RUNNING")
        print(f"│ Waiting for connections from Godot simulation...")
        print("="*80 + "\n")
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    asyncio.run(start_server())
    