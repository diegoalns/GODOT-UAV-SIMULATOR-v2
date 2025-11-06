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

def debug_position_mapping(graph, position):
    """
    Debug function to show how a position maps to graph nodes.
    
    Args:
        graph: NetworkX graph
        position: dict with 'lat', 'lon', 'alt' coordinates
    
    Returns:
        tuple: The closest available node or None
    """
    closest_node = find_closest_node(graph, position)
    
    if isinstance(position, dict):
        if 'lon' in position and 'lat' in position and 'alt' in position:
            pos = (position['lat'], position['lon'], position['alt'])
        else:
            pos = (position['x'], position['y'], position['z'])
    else:
        pos = position
    
    print(f"Position: {pos}")
    print(f"Closest available node: {closest_node}")
    
    if closest_node:
        closest_pos = graph.nodes[closest_node]['pos']
        distance = slant_range(pos, closest_pos)
        print(f"Closest node position: {closest_pos}")
        print(f"Distance to closest: {distance:.1f}m")
        print(f"Node available: {graph.nodes[closest_node].get('available', True)}")
    
    return closest_node

# Load the graph from pickle file using the graph loader module
airspace_graph = load_graph_from_pickle()

# WebSocket server to handle drone creation messages and respond accordingly
async def websocket_handler(websocket):
    print("Client connected!")
    try:
        async for message in websocket:
            print(f"Received from client: {message}")
            
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
                    
                    print(f"Route request for drone: {drone_id} ({model})")
                    print(f"  Start: (lon:{start_pos['lon']}, lat:{start_pos['lat']}, alt:{start_pos['alt']})")
                    print(f"  End: (lon:{end_pos['lon']}, lat:{end_pos['lat']}, alt:{end_pos['alt']})")
                    print(f"  Battery: {battery_percentage}%, Max Speed: {max_speed} m/s, Range: {max_range} m")
                    print(f"  Planning route...")

                    # Debug position mapping
                    print(f"  Mapping start position...")
                    start_node = debug_position_mapping(airspace_graph, start_pos)
                    print(f"  Mapping end position...")
                    end_node = debug_position_mapping(airspace_graph, end_pos)
                    
                    if start_node is None or end_node is None:
                        print(f"  Could not find valid nodes for start or end position")
                        response = {
                            "type": "route_response",
                            "drone_id": drone_id,
                            "status": "error",
                            "message": "Could not find valid graph nodes for start or end position"
                        }
                        await websocket.send(json.dumps(response))
                        continue

                    try:
                        # Calculate shortest path between nodes
                        path_nodes = nx.shortest_path(airspace_graph, source=start_node, target=end_node, weight='weight')
                        
                        # Convert path nodes back to 3D coordinates for the drone
                        route = []
                        for i, node in enumerate(path_nodes):
                            node_pos = airspace_graph.nodes[node]['pos']  # (lat, lon, alt)
                            waypoint = {
                                "x": node_pos[1],      # Python lon (East/West) → Godot X
                                "y": node_pos[2],      # Python alt (Up/Down) → Godot Y
                                "z": node_pos[0],      # Python lat (North/South) → Godot Z
                                "altitude": node_pos[2],
                                "speed": max_speed * 0.8,  # Use 80% of max speed for waypoints
                                "description": f"Graph waypoint {i+1}"
                            }
                            route.append(waypoint)
                        
                        print(f"  Found path with {len(route)} waypoints")
                        
                    except nx.NetworkXNoPath:
                        print(f"  No path found between start and end nodes!")
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
                # Handle non-JSON messages
                await websocket.send(f"Echo: {message}")
                
    except websockets.ConnectionClosed:
        print("Client disconnected.")
        print(airspace_graph.nodes[(3,4,1)]['pos'])
        # Access edges with source and target nodes
        # Get first edge connected to the node (3,4,1)
        edges = list(airspace_graph.edges([(3,4,1)]))
        if edges:
            for i in range(len(edges)):
                print(edges[i])
                u, v = edges[i]  # Get the first edge (source, target)
                print(airspace_graph.edges[u, v]['weight'])
        else:
            print("No edges found for node (3,4,1)")

# Start the server on localhost:8765
async def start_server():
    async with websockets.serve(websocket_handler, 'localhost', 8765):
        print("WebSocket server started on ws://localhost:8765")
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    asyncio.run(start_server())
    