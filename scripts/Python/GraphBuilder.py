import networkx as nx
from geopy.distance import geodesic
import numpy as np
import plotly.graph_objects as go  # Import Plotly
import random
import math
import copy

# Define a function to calculate the slant range (3D distance) between two points
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
        target_position: tuple or list with (x, y, z) coordinates in meters
    
    Returns:
        tuple: The closest node identifier (i, j, k)
    """
    if isinstance(target_position, dict):
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

def position_to_node_coordinates(target_position, graph_bounds=None):
    """
    Convert a 3D position to the closest grid coordinates.
    
    Args:
        target_position: dict with 'x', 'y', 'z' keys or tuple (x, y, z)
        graph_bounds: dict with graph boundary information (optional)
    
    Returns:
        tuple: Grid coordinates (i, j, k) that correspond to the position
    """
    if isinstance(target_position, dict):
        x, y, z = target_position['x'], target_position['y'], target_position['z']
    else:
        x, y, z = target_position
    
    # Default graph parameters (should match create_graph function)
    meters_per_degree = 111320
    origin_lat_degrees = 40.55417343
    origin_lon_degrees = -73.99583928
    
    min_lat_degree, max_lat_degree = 40.55417343, 40.6125
    min_lon_degree, max_lon_degree = -73.99583928, -73.9292
    min_alt_feet, max_alt_feet = 0, 300
    
    # Convert altitude from feet to meters
    min_alt_meters = min_alt_feet * 0.3048
    max_alt_meters = max_alt_feet * 0.3048
    
    # Define the bounding box for geographic (meters coordinates)
    min_lat = (min_lat_degree - origin_lat_degrees) * meters_per_degree
    max_lat = (max_lat_degree - origin_lat_degrees) * meters_per_degree
    min_lon = (min_lon_degree - origin_lon_degrees) * meters_per_degree
    max_lon = (max_lon_degree - origin_lon_degrees) * meters_per_degree
    
    # Grid dimensions
    n_lat, n_lon, n_alt = 100, 100, 2
    
    # Calculate grid indices
    lat_idx = int(np.clip((x - min_lat) / (max_lat - min_lat) * (n_lat - 1), 0, n_lat - 1))
    lon_idx = int(np.clip((y - min_lon) / (max_lon - min_lon) * (n_lon - 1), 0, n_lon - 1))
    alt_idx = int(np.clip((z - min_alt_meters) / (max_alt_meters - min_alt_meters) * (n_alt - 1), 0, n_alt - 1))
    
    return (lat_idx, lon_idx, alt_idx)

def get_graph_bounds():
    """
    Get the bounds of the graph in 3D space.
    
    Returns:
        dict: Dictionary with min/max values for x, y, z coordinates
    """
    meters_per_degree = 111320
    origin_lat_degrees = 40.55417343
    origin_lon_degrees = -73.99583928
    
    min_lat_degree, max_lat_degree = 40.55417343, 40.8875
    min_lon_degree, max_lon_degree = -73.99583928, -73.5958
    min_alt_feet, max_alt_feet = 0, 300
    
    # Convert altitude from feet to meters
    min_alt_meters = min_alt_feet * 0.3048
    max_alt_meters = max_alt_feet * 0.3048
    
    # Define the bounding box for geographic (meters coordinates)
    min_lat = (min_lat_degree - origin_lat_degrees) * meters_per_degree
    max_lat = (max_lat_degree - origin_lat_degrees) * meters_per_degree
    min_lon = (min_lon_degree - origin_lon_degrees) * meters_per_degree
    max_lon = (max_lon_degree - origin_lon_degrees) * meters_per_degree
    
    return {
        'x_min': min_lat, 'x_max': max_lat,
        'y_min': min_lon, 'y_max': max_lon, 
        'z_min': min_alt_meters, 'z_max': max_alt_meters
    }

def debug_position_mapping(graph, position):
    """
    Debug function to show how a position maps to graph nodes.
    
    Args:
        graph: NetworkX graph
        position: dict with 'x', 'y', 'z' or tuple (x, y, z)
    """
    bounds = get_graph_bounds()
    closest_node = find_closest_node(graph, position)
    calculated_node = position_to_node_coordinates(position)
    
    if isinstance(position, dict):
        pos = (position['x'], position['y'], position['z'])
    else:
        pos = position
    
    print(f"Position: {pos}")
    print(f"Graph bounds: X[{bounds['x_min']:.1f}, {bounds['x_max']:.1f}], Y[{bounds['y_min']:.1f}, {bounds['y_max']:.1f}], Z[{bounds['z_min']:.1f}, {bounds['z_max']:.1f}]")
    print(f"Calculated node: {calculated_node}")
    print(f"Closest available node: {closest_node}")
    
    if closest_node:
        closest_pos = graph.nodes[closest_node]['pos']
        distance = slant_range(pos, closest_pos)
        print(f"Closest node position: {closest_pos}")
        print(f"Distance to closest: {distance:.1f}m")
        print(f"Node available: {graph.nodes[closest_node].get('available', True)}")
    
    return closest_node

def create_graph():
    meters_per_degree = 111320  # Approximate meters per degree
    #Define the bounding box for geographic coordinates (Degrees)
    min_lat_degree, max_lat_degree = 40.55417343, 40.8875
    min_lon_degree, max_lon_degree = -73.99583928, -73.5958
    origin_lat_degrees = 40.55417343
    origin_lon_degrees = -73.99583928
    min_alt_feet, max_alt_feet = 50, 350

    # Convert altitude from feet to meters
    min_alt_meters = min_alt_feet * 0.3048
    max_alt_meters = max_alt_feet * 0.3048

    # Define the bounding box for geographic (meters coordinates)
    min_lat = (min_lat_degree - origin_lat_degrees) * meters_per_degree
    max_lat = (max_lat_degree - origin_lat_degrees) * meters_per_degree
    min_lon = (min_lon_degree - origin_lon_degrees) * meters_per_degree
    max_lon = (max_lon_degree - origin_lon_degrees) * meters_per_degree

    # Define grid dimensions
    n_lat, n_lon, n_alt = 100, 100, 3

    # Conversion to position meters ()

    # Generate grid points using NumPy's linspace
    lats = np.linspace(min_lat, max_lat, n_lat)
    lons = np.linspace(min_lon, max_lon, n_lon)
    alts = np.linspace(min_alt_meters, max_alt_meters, n_alt)

    # Create node identifiers and their geographic positions
    nodes = [(i, j, k) for i in range(n_lat) for j in range(n_lon) for k in range(n_alt)]
    pos = {(i, j, k): (lats[i], lons[j], alts[k]) for i in range(n_lat) for j in range(n_lon) for k in range(n_alt)}

    # Randomly assign availability
    availability = {node: np.random.random() < 1 for node in nodes}

    # Create an empty directed graph using NetworkX
    G = nx.DiGraph()

    # Add all nodes to the graph
    G.add_nodes_from(nodes)

    # Set node attributes for geographic coordinates
    nx.set_node_attributes(G, pos, 'pos')
    nx.set_node_attributes(G, availability, 'available')

    # Add edges to connect each node to all its neighbors, including diagonals
    for i in range(n_lat):
        for j in range(n_lon):
            for k in range(n_alt):
                if not G.nodes[(i, j, k)]['available']:
                    continue
                for di in [-1, 0, 1]:
                    for dj in [-1, 0, 1]:
                        for dk in [-1, 0, 1]:
                            if di == dj == dk == 0:
                                continue
                            ni, nj, nk = i + di, j + dj, k + dk
                            if 0 <= ni < n_lat and 0 <= nj < n_lon and 0 <= nk < n_alt:
                                if not G.nodes[(ni, nj, nk)]['available']:
                                    continue
                                u, v = (i, j, k), (ni, nj, nk)
                                dist = slant_range(pos[u], pos[v])
                                G.add_edge(u, v, weight=dist)
    return G

