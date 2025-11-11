import asyncio
import websockets
import json
import sys
from pathlib import Path
import numpy as np
import networkx as nx
from coordinate_constants import *
from graph_loader import load_graph_from_pickle
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
                    battery_percentage = data.get("battery_percentage")
                    max_speed = data.get("max_speed")
                    max_range = data.get("max_range")
                    
                    # NEW: Get node IDs if provided (efficient O(1) lookup)
                    start_node_id = data.get("start_node_id")
                    end_node_id = data.get("end_node_id")
                    
                    # Print table header for route request
                    print("\n" + "─"*80)
                    print(f"│ 🚁 ROUTE REQUEST: {drone_id}")
                    print("─"*80)
                    print(f"│ {'Parameter':<25} │ {'Value':<50} │")
                    print("├" + "─"*25 + "┼" + "─"*50 + "┤")
                    print(f"│ {'Drone Model':<25} │ {model:<50} │")
                    print(f"│ {'Battery Level':<25} │ {battery_percentage:.1f}%{'':<45} │")
                    print(f"│ {'Max Speed':<25} │ {max_speed:.1f} m/s{'':<42} │")
                    print(f"│ {'Max Range':<25} │ {max_range:.0f} m{'':<44} │")
                    
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
                    print(f"│ {'Pathfinding':<25} │ Computing shortest path...{'':<26} │")

                    try:
                        # Calculate shortest path between nodes
                        path_nodes = nx.shortest_path(airspace_graph, source=start_node, target=end_node, weight='weight')
                        
                        # Convert path nodes to geographic coordinates (lat, lon, altitude)
                        # Let Godot handle the conversion to world coordinates for consistency
                        route = []  # List of waypoint dictionaries to send to Godot
                        for i, node in enumerate(path_nodes):
                            # Get node position: tuple of (lat, lon, alt) where lat/lon are in degrees, alt in meters
                            node_pos = airspace_graph.nodes[node]['pos']  # (lat, lon, alt)
                            
                            # Create waypoint with geographic coordinates - Godot will convert to world position
                            waypoint = {
                                "lat": node_pos[0],         # Latitude in decimal degrees (float)
                                "lon": node_pos[1],         # Longitude in decimal degrees (float)
                                "altitude": node_pos[2],    # Altitude in meters (float)
                                "speed": max_speed * 0.8,   # Waypoint speed: 80% of drone's max speed (float, m/s)
                                "description": f"Graph waypoint {i+1}"  # Human-readable waypoint label (string)
                            }
                            route.append(waypoint)
                        
                        # Calculate route statistics for display
                        total_distance = sum(
                            slant_range(airspace_graph.nodes[path_nodes[i]]['pos'], 
                                      airspace_graph.nodes[path_nodes[i+1]]['pos'])
                            for i in range(len(path_nodes)-1)
                        )
                        est_time = total_distance / (max_speed * 0.8) if max_speed > 0 else 0
                        
                        print(f"│ {'  Path Found':<25} │ ✓ {len(route)} waypoints{'':<36} │")
                        print(f"│ {'  Total Distance':<25} │ {total_distance:.0f} meters{'':<37} │")
                        print(f"│ {'  Est. Flight Time':<25} │ {est_time:.1f} seconds ({est_time/60:.1f} minutes){'':<19} │")
                        print("├" + "─"*25 + "┼" + "─"*50 + "┤")
                        print(f"│ {'STATUS':<25} │ {'✅ SUCCESS - Route computed successfully':<50} │")
                        print("└" + "─"*25 + "┴" + "─"*50 + "┘")
                        
                    except nx.NetworkXNoPath:
                        print(f"│ {'  Path Result':<25} │ ❌ No path exists{'':<36} │")
                        print("├" + "─"*25 + "┼" + "─"*50 + "┤")
                        print(f"│ {'STATUS':<25} │ {'❌ ERROR: No path found in graph':<50} │")
                        print("└" + "─"*25 + "┴" + "─"*50 + "┘")
                        # Send error response
                        response = {
                            "type": "route_response",
                            "drone_id": drone_id,
                            "status": "no_path",
                            "message": "No path found in graph between start and end positions"
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
    