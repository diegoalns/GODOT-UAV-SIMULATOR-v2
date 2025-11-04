import asyncio
import websockets
import json
import sys
from pathlib import Path
import numpy as np
import networkx as nx
import GraphBuilder
from coordinate_constants import *
sys.path.append(str(Path(__file__).parent))

airspace_graph = GraphBuilder.create_graph()

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
                    start_node = GraphBuilder.debug_position_mapping(airspace_graph, start_pos)
                    print(f"  Mapping end position...")
                    end_node = GraphBuilder.debug_position_mapping(airspace_graph, end_pos)
                    
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
    