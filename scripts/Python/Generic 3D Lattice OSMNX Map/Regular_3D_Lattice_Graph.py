"""
Regular 3D Lattice Graph for UAS Path Planning - Manhattan NYC

This script creates a regular 5-layer 3D lattice graph (30x15x5) for UAS (drone) path planning.
Unlike the OSM-based version, this creates a perfect grid structure positioned over Manhattan.

Key Features:
- Regular grid: 30 nodes × 15 nodes × 5 layers = 2,250 total nodes
- Grid spacing: 200 meters between adjacent nodes within layers
- 5 altitude layers: 0ft, 100ft, 200ft, 300ft, 400ft (0-4)
- Reference line: (40.762697, -73.99984) to (40.748845, -73.968621)
  - This line defines the 15-node dimension (perpendicular expansion)
  - Grid extends perpendicular to this line for 30 nodes
- Horizontal edges: 4-connected (N,S,E,W) with bidirectional arcs (200m each)
- Vertical edges: Undirected connections between layers (100ft = ~30.48m)
- Real-world constraints: FAA airspace ceilings and building obstacles
- Each edge has geographic heading (true bearing in degrees)
- Visualizes with Plotly 3D interactive graph
- Color coding: Green nodes = available, Red nodes = unavailable

Output: Interactive 3D HTML visualization + pickle file
"""

import pandas as pd
import numpy as np
import networkx as nx
import osmnx as ox
import plotly.graph_objects as go
from math import sqrt, atan2, degrees, radians, sin, cos
from geopy.distance import geodesic
import pickle
from datetime import datetime
import webbrowser

# --- Configuration ---

# 1. Grid Dimensions
GRID_X = 15  # Nodes along perpendicular to reference line
GRID_Y = 7  # Nodes along reference line
NUM_LAYERS = 5  # Layers 0-4
HORIZONTAL_SPACING_M = 400  # Spacing between nodes within layers (meters)

# 2. Reference Line (defines grid orientation)
# This line forms the 15-node dimension
REF_POINT_A = (40.762697, -73.99984)  # (lat, lon)
REF_POINT_B = (40.748845, -73.968621)  # (lat, lon)

# 3. Layer Altitudes (in feet, converted to meters)
LAYER_ALTITUDES_FT = [0, 100, 200, 300, 400]  # Layers 0-4
VERTICAL_SPACING_FT = 100  # Distance between layers
FEET_TO_METERS = 0.3048
METERS_TO_FEET = 3.28084

# 4. Real-World Constraints
FAA_CEILING_CSV = "Filtered_FAA_UAS_FacilityMap_Data_LGA.csv"
MIN_CLEARANCE_FT = 100  # Minimum clearance above obstacles
OBSTACLE_SEARCH_RADIUS_M = 50  # Radius to search for obstacles
DEFAULT_BUILDING_HEIGHT_FT = 50  # Conservative default

# 5. Visualization Parameters
LAYERS_TO_DISPLAY = [0, 1, 2, 3, 4]  # Which layers to show initially
NODE_SIZE = 5
HORIZONTAL_LINE_WIDTH = 3.5
VERTICAL_LINE_WIDTH = 2.5

# 6. Output Configuration
SAVE_GRAPH_PICKLE = True
PICKLE_FILENAME = "regular_lattice_graph.pkl"
HTML_FILENAME = "regular_lattice_graph.html"

# --- Helper Functions ---

def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the true bearing (azimuth) from point 1 to point 2.
    Returns bearing in degrees (0-360), where 0° = North, 90° = East, etc.
    
    Uses the forward azimuth formula for geodetic calculations.
    """
    # Convert to radians
    lat1_rad = radians(lat1)
    lat2_rad = radians(lat2)
    dlon_rad = radians(lon2 - lon1)
    
    # Calculate bearing
    x = sin(dlon_rad) * cos(lat2_rad)
    y = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dlon_rad)
    
    bearing_rad = atan2(x, y)
    bearing_deg = (degrees(bearing_rad) + 360) % 360
    
    return bearing_deg

def destination_point(lat, lon, bearing_deg, distance_m):
    """
    Calculate destination point given start point, bearing, and distance.
    
    Args:
        lat, lon: Start point in degrees
        bearing_deg: Bearing in degrees (0-360)
        distance_m: Distance in meters
    
    Returns:
        (lat, lon) of destination point
    """
    # Earth radius in meters
    R = 6371000
    
    # Convert to radians
    lat_rad = radians(lat)
    lon_rad = radians(lon)
    bearing_rad = radians(bearing_deg)
    
    # Angular distance
    angular_dist = distance_m / R
    
    # Calculate destination
    lat2_rad = np.arcsin(
        sin(lat_rad) * cos(angular_dist) +
        cos(lat_rad) * sin(angular_dist) * cos(bearing_rad)
    )
    
    lon2_rad = lon_rad + atan2(
        sin(bearing_rad) * sin(angular_dist) * cos(lat_rad),
        cos(angular_dist) - sin(lat_rad) * sin(lat2_rad)
    )
    
    return degrees(lat2_rad), degrees(lon2_rad)

def calculate_perpendicular_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the bearing perpendicular to the line from point 1 to point 2.
    Returns bearing 90° counter-clockwise from the original line.
    """
    original_bearing = calculate_bearing(lat1, lon1, lat2, lon2)
    perpendicular_bearing = (original_bearing - 90) % 360
    return perpendicular_bearing

# --- Grid Generation ---

def create_grid_nodes():
    """
    Creates a regular 30x15x5 grid of nodes positioned over Manhattan.
    
    Grid orientation:
    - Y-axis (15 nodes): Along reference line A→B
    - X-axis (30 nodes): Perpendicular to reference line
    - Z-axis (5 layers): Vertical altitude
    
    Returns:
        dict: {node_id: {attributes}}
    """
    print("1. Generating regular 3D lattice grid...")
    print(f"   Grid dimensions: {GRID_X} × {GRID_Y} × {NUM_LAYERS}")
    print(f"   Horizontal spacing: {HORIZONTAL_SPACING_M}m")
    print(f"   Reference line: {REF_POINT_A} → {REF_POINT_B}")
    
    nodes = {}
    
    # Calculate grid orientation
    # Y-axis: bearing along reference line A→B
    y_bearing = calculate_bearing(REF_POINT_A[0], REF_POINT_A[1], 
                                   REF_POINT_B[0], REF_POINT_B[1])
    
    # X-axis: perpendicular to reference line (90° counter-clockwise)
    x_bearing = (y_bearing - 90) % 360
    
    print(f"   Y-axis bearing (along reference line): {y_bearing:.2f}°")
    print(f"   X-axis bearing (perpendicular): {x_bearing:.2f}°")
    
    # Generate nodes for each layer
    node_count = 0
    for layer in range(NUM_LAYERS):
        altitude_ft = LAYER_ALTITUDES_FT[layer]
        altitude_m = altitude_ft * FEET_TO_METERS
        
        for y in range(GRID_Y):
            for x in range(GRID_X):
                # Calculate position
                # Start from reference point A, move along Y-axis, then X-axis
                
                # First, move along Y-axis from point A
                y_distance = y * HORIZONTAL_SPACING_M
                lat_y, lon_y = destination_point(
                    REF_POINT_A[0], REF_POINT_A[1],
                    y_bearing, y_distance
                )
                
                # Then, move along X-axis (perpendicular)
                x_distance = x * HORIZONTAL_SPACING_M
                lat_final, lon_final = destination_point(
                    lat_y, lon_y,
                    x_bearing, x_distance
                )
                
                # Create node ID
                node_id = f"L{layer}_X{x}_Y{y}"
                
                # Store node data
                nodes[node_id] = {
                    'layer': layer,
                    'grid_x': x,
                    'grid_y': y,
                    'lat': lat_final,
                    'lon': lon_final,
                    'altitude': altitude_m,
                    'altitude_ft': altitude_ft,
                    'pos_x': x * HORIZONTAL_SPACING_M,  # Meters from origin
                    'pos_y': y * HORIZONTAL_SPACING_M,
                    'available': True,  # Will be updated later
                    'label': node_id
                }
                
                node_count += 1
        
        print(f"   Layer {layer} ({altitude_ft}ft): {GRID_X * GRID_Y} nodes created")
    
    print(f"   Total nodes created: {node_count}")
    return nodes

# --- Load Constraint Data ---

def load_faa_ceiling_data(csv_path):
    """Load FAA UAS ceiling data from CSV."""
    print(f"\n2a. Loading FAA ceiling data from {csv_path}...")
    try:
        faa_df = pd.read_csv(csv_path)
        print(f"    Loaded {len(faa_df)} FAA ceiling grid points")
        ceiling_values = faa_df['CEILING'].unique()
        print(f"    Ceiling values: {sorted(ceiling_values)}")
        return faa_df
    except FileNotFoundError:
        print(f"    WARNING: FAA ceiling file not found. Using default 400 ft ceiling.")
        return None
    except Exception as e:
        print(f"    ERROR loading FAA data: {e}")
        return None

def load_building_data(bbox):
    """
    Extracts building footprints and heights from OpenStreetMap via OSMNx.
    
    Args:
        bbox: Dictionary with keys 'south', 'north', 'west', 'east' in decimal degrees
    
    Returns a GeoDataFrame with building geometries and estimated heights in feet.
    """
    print(f"2b. Downloading building data from OpenStreetMap...")
    try:
        # Get all buildings in the bounding box
        # OSMnx features_from_bbox expects bbox as (left, bottom, right, top) = (west, south, east, north)
        # SAME format as graph_from_bbox!
        buildings = ox.features_from_bbox(
            bbox=(bbox['west'], bbox['south'], bbox['east'], bbox['north']),
            tags={'building': True}
        )
        
        print(f"    Downloaded {len(buildings)} buildings")
        
        # Extract height information (convert to feet)
        heights = []
        for idx, building in buildings.iterrows():
            height_ft = None
            
            # Try different height attributes
            if 'height' in building and pd.notna(building['height']):
                try:
                    # Height might be string like "50 m" or just "50"
                    height_str = str(building['height']).replace('m', '').strip()
                    height_m = float(height_str)
                    height_ft = height_m * METERS_TO_FEET
                except:
                    pass
            
            # Fallback to building levels
            if height_ft is None and 'building:levels' in building and pd.notna(building['building:levels']):
                try:
                    levels = float(building['building:levels'])
                    # Estimate 3.5 meters (11.5 ft) per floor
                    height_ft = levels * 11.5
                except:
                    pass
            
            # Use default if no data available
            if height_ft is None:
                height_ft = DEFAULT_BUILDING_HEIGHT_FT
            
            heights.append(height_ft)
        
        buildings['height_ft'] = heights
        
        # Statistics
        with_data = sum(1 for h in heights if h != DEFAULT_BUILDING_HEIGHT_FT)
        print(f"    {with_data}/{len(buildings)} buildings have height data")
        print(f"    Height range: {min(heights):.1f} - {max(heights):.1f} ft")
        
        return buildings
        
    except Exception as e:
        print(f"    ERROR downloading building data: {e}")
        print(f"    Continuing without building data (conservative defaults will be used)")
        return None

def get_faa_ceiling_at_location(lat, lon, faa_df):
    """Get FAA ceiling at location by finding nearest grid point."""
    if faa_df is None:
        return 400  # Default ceiling
    
    # Calculate distances to all grid points
    faa_df['dist'] = np.sqrt(
        (faa_df['LATITUDE'] - lat)**2 + 
        (faa_df['LONGITUDE'] - lon)**2
    )
    
    nearest = faa_df.loc[faa_df['dist'].idxmin()]
    return nearest['CEILING']

def get_max_obstacle_height_near_node(lat, lon, buildings_gdf, radius_m=OBSTACLE_SEARCH_RADIUS_M):
    """Find maximum building height within radius of node."""
    if buildings_gdf is None or len(buildings_gdf) == 0:
        return 0.0
    
    from shapely.geometry import Point
    
    # Create point for the node (in WGS84)
    node_point = Point(lon, lat)
    
    # Convert radius from meters to degrees (approximate at this latitude)
    # At 40°N latitude: 1 degree ≈ 85 km, so 50m ≈ 0.0006 degrees
    radius_deg = radius_m / 111000
    
    # Find buildings within bounding box
    nearby = buildings_gdf[
        (buildings_gdf.geometry.bounds['minx'] >= lon - radius_deg) &
        (buildings_gdf.geometry.bounds['maxx'] <= lon + radius_deg) &
        (buildings_gdf.geometry.bounds['miny'] >= lat - radius_deg) &
        (buildings_gdf.geometry.bounds['maxy'] <= lat + radius_deg)
    ]
    
    if len(nearby) == 0:
        return 0.0
    
    return nearby['height_ft'].max()

def check_node_availability(node_data, faa_df, building_df):
    """
    Check if node is available based on FAA ceiling and obstacles.
    
    Rules:
    - Node altitude must not exceed FAA ceiling
    - Node altitude must be at least MIN_CLEARANCE_FT above highest obstacle
    
    Returns: (available, reason, faa_ceiling_ft, max_obstacle_ft)
    """
    lat = node_data['lat']
    lon = node_data['lon']
    altitude_ft = node_data['altitude_ft']
    
    # Get FAA ceiling
    faa_ceiling_ft = get_faa_ceiling_at_location(lat, lon, faa_df)
    
    # Get max obstacle height
    max_obstacle_ft = get_max_obstacle_height_near_node(lat, lon, building_df)
    
    # Check constraints
    if altitude_ft > faa_ceiling_ft:
        return False, "Above FAA ceiling", faa_ceiling_ft, max_obstacle_ft
    
    if altitude_ft < (max_obstacle_ft + MIN_CLEARANCE_FT):
        return False, "Too close to obstacle", faa_ceiling_ft, max_obstacle_ft
    
    return True, "Available", faa_ceiling_ft, max_obstacle_ft

def update_node_availability(nodes, faa_df, building_df):
    """Update availability status for all nodes."""
    print("\n3. Checking node availability based on constraints...")
    
    total_nodes = len(nodes)
    available_count = 0
    unavailable_ceiling = 0
    unavailable_obstacle = 0
    
    layer_stats = {layer: {'total': 0, 'available': 0} for layer in range(NUM_LAYERS)}
    
    for node_id, node_data in nodes.items():
        layer = node_data['layer']
        layer_stats[layer]['total'] += 1
        
        # Layer 0 is always available (takeoff/landing)
        if layer == 0:
            node_data['available'] = True
            node_data['unavailable_reason'] = 'Takeoff/Landing Zone'
            # Still get airspace info for reference
            faa_ceiling = get_faa_ceiling_at_location(node_data['lat'], node_data['lon'], faa_df)
            max_obstacle = get_max_obstacle_height_near_node(node_data['lat'], node_data['lon'], building_df)
            node_data['faa_ceiling_ft'] = faa_ceiling
            node_data['max_obstacle_ft'] = max_obstacle
            available_count += 1
            layer_stats[layer]['available'] += 1
        else:
            # Check availability for layers 1-4
            available, reason, faa_ceiling, max_obstacle = check_node_availability(
                node_data, faa_df, building_df
            )
            
            node_data['available'] = available
            node_data['unavailable_reason'] = reason
            node_data['faa_ceiling_ft'] = faa_ceiling
            node_data['max_obstacle_ft'] = max_obstacle
            
            if available:
                available_count += 1
                layer_stats[layer]['available'] += 1
            else:
                if "ceiling" in reason.lower():
                    unavailable_ceiling += 1
                else:
                    unavailable_obstacle += 1
    
    # Print statistics
    print(f"\n   Overall Node Availability:")
    print(f"   - Total nodes: {total_nodes}")
    print(f"   - Available: {available_count} ({100*available_count/total_nodes:.1f}%)")
    print(f"   - Unavailable (FAA ceiling): {unavailable_ceiling}")
    print(f"   - Unavailable (obstacle clearance): {unavailable_obstacle}")
    print(f"\n   By Layer:")
    for layer in range(NUM_LAYERS):
        stats = layer_stats[layer]
        pct = 100 * stats['available'] / stats['total'] if stats['total'] > 0 else 0
        print(f"   - Layer {layer} ({LAYER_ALTITUDES_FT[layer]}ft): {stats['available']}/{stats['total']} available ({pct:.1f}%)")

# --- Graph Construction ---

def create_graph_with_edges(nodes):
    """
    Create NetworkX DiGraph with horizontal and vertical edges.
    
    Horizontal edges (within layers):
    - 4-connected grid (N, S, E, W neighbors)
    - Bidirectional (two directed arcs)
    - Only between available nodes
    - Include geographic heading
    
    Vertical edges (between layers):
    - Connect vertically aligned nodes
    - Undirected
    - Connect all nodes (available and unavailable)
    """
    print("\n4. Creating graph structure with edges...")
    
    G = nx.DiGraph()
    
    # Add all nodes to graph
    for node_id, node_data in nodes.items():
        G.add_node(node_id, **node_data)
    
    # Track edge statistics
    horizontal_edges = 0
    vertical_edges = 0
    
    # 4a. Add horizontal edges within each layer
    print("   4a. Adding horizontal edges (4-connected grid)...")
    
    for layer in range(NUM_LAYERS):
        layer_h_edges = 0
        
        for y in range(GRID_Y):
            for x in range(GRID_X):
                node_id = f"L{layer}_X{x}_Y{y}"
                node_data = nodes[node_id]
                
                # Skip if node is unavailable (except Layer 0)
                if not node_data['available'] and layer != 0:
                    continue
                
                # Define 4 neighbors: East, West, North, South
                # East: x+1, West: x-1, North: y+1, South: y-1
                neighbors = []
                
                if x + 1 < GRID_X:  # East
                    neighbors.append(f"L{layer}_X{x+1}_Y{y}")
                if x - 1 >= 0:  # West
                    neighbors.append(f"L{layer}_X{x-1}_Y{y}")
                if y + 1 < GRID_Y:  # North
                    neighbors.append(f"L{layer}_X{x}_Y{y+1}")
                if y - 1 >= 0:  # South
                    neighbors.append(f"L{layer}_X{x}_Y{y-1}")
                
                # Add edges to available neighbors
                for neighbor_id in neighbors:
                    neighbor_data = nodes[neighbor_id]
                    
                    # Only connect to available nodes (except Layer 0)
                    if not neighbor_data['available'] and layer != 0:
                        continue
                    
                    # Calculate edge properties
                    lat1, lon1 = node_data['lat'], node_data['lon']
                    lat2, lon2 = neighbor_data['lat'], neighbor_data['lon']
                    
                    # Distance (should be ~200m, but calculate actual)
                    distance_m = geodesic((lat1, lon1), (lat2, lon2)).meters
                    
                    # Geographic heading (true bearing)
                    heading = calculate_bearing(lat1, lon1, lat2, lon2)
                    
                    # Add directed edge
                    G.add_edge(
                        node_id, neighbor_id,
                        weight=distance_m,
                        length=distance_m,
                        heading=heading,
                        layer_type='Horizontal'
                    )
                    
                    layer_h_edges += 1
                    horizontal_edges += 1
        
        if layer == 0:
            print(f"      Layer {layer}: {layer_h_edges} horizontal edges (Note: Layer 0 for takeoff/landing)")
        else:
            print(f"      Layer {layer}: {layer_h_edges} horizontal edges")
    
    # 4b. Add vertical edges between layers
    print("   4b. Adding vertical edges between layers...")
    
    for layer in range(NUM_LAYERS - 1):
        layer_v_edges = 0
        
        for y in range(GRID_Y):
            for x in range(GRID_X):
                # Current layer node
                node_id = f"L{layer}_X{x}_Y{y}"
                # Next layer node (directly above)
                neighbor_id = f"L{layer+1}_X{x}_Y{y}"
                
                # Vertical edges connect ALL nodes (available or not)
                node_data = nodes[node_id]
                neighbor_data = nodes[neighbor_id]
                
                # Calculate distance (should be ~30.48m = 100ft)
                distance_m = VERTICAL_SPACING_FT * FEET_TO_METERS
                
                # Add bidirectional edges (undirected = two directed edges)
                # Upward edge
                G.add_edge(
                    node_id, neighbor_id,
                    weight=distance_m,
                    length=distance_m,
                    heading=None,  # Vertical edges don't have horizontal heading
                    layer_type='Vertical'
                )
                
                # Downward edge
                G.add_edge(
                    neighbor_id, node_id,
                    weight=distance_m,
                    length=distance_m,
                    heading=None,
                    layer_type='Vertical'
                )
                
                layer_v_edges += 2  # Count both directions
                vertical_edges += 2
        
        print(f"      Layer {layer}↔{layer+1}: {layer_v_edges} vertical edges (bidirectional)")
    
    print(f"\n   Total edges created:")
    print(f"   - Horizontal (within layers): {horizontal_edges}")
    print(f"   - Vertical (between layers): {vertical_edges}")
    print(f"   - Total: {G.number_of_edges()}")
    
    # 4c. Remove all horizontal edges from Layer 0
    print("   4c. Removing all horizontal edges from Layer 0...")
    edges_to_remove = []
    for u, v, data in G.edges(data=True):
        if data.get('layer_type') == 'Horizontal':
            u_layer = G.nodes[u]['layer']
            if u_layer == 0:
                edges_to_remove.append((u, v))
    
    for edge in edges_to_remove:
        G.remove_edge(edge[0], edge[1])
    
    print(f"      Removed {len(edges_to_remove)} horizontal edges from Layer 0")
    print(f"   Final total edges: {G.number_of_edges()}")
    
    return G

# --- Visualization ---

def visualize_3d_graph(G):
    """Create interactive 3D Plotly visualization."""
    print("\n5. Generating interactive 3D Plotly visualization...")
    
    # Create DataFrame from nodes
    nodes_df = pd.DataFrame.from_dict(dict(G.nodes(data=True)), orient='index')
    
    print(f"   Total nodes: {len(nodes_df)}")
    print(f"   Total edges: {G.number_of_edges()}")
    
    # --- Node traces by layer ---
    node_traces = []
    nodes_df['color'] = nodes_df['available'].apply(lambda x: 'green' if x else 'red')
    
    for layer in range(NUM_LAYERS):
        layer_nodes = nodes_df[nodes_df['layer'] == layer].copy()
        altitude_ft = LAYER_ALTITUDES_FT[layer]
        
        node_trace = go.Scatter3d(
            x=layer_nodes['pos_x'],
            y=layer_nodes['pos_y'],
            z=layer_nodes['altitude'],
            mode='markers',
            name=f'L{layer} Nodes ({altitude_ft}ft)',
            legendgroup=f'nodes_layer{layer}',
            marker=dict(
                symbol='circle',
                size=NODE_SIZE,
                color=layer_nodes['color'],
                opacity=0.6
            ),
            hovertemplate='<b>Node:</b> %{text}<br>' +
                          '<b>Grid Position:</b> (X%{customdata[0]}, Y%{customdata[1]})<br>' +
                          '<b>Layer:</b> %{customdata[2]}<br>' +
                          '<b>Lat/Lon:</b> (%{customdata[3]:.6f}, %{customdata[4]:.6f})<br>' +
                          '<b>Altitude:</b> %{z:.1f}m (%{customdata[5]:.0f}ft)<br>' +
                          '<b>Available:</b> %{customdata[6]}<br>' +
                          '<b>FAA Ceiling:</b> %{customdata[7]:.0f}ft<br>' +
                          '<b>Max Obstacle:</b> %{customdata[8]:.0f}ft<extra></extra>',
            text=layer_nodes['label'],
            customdata=layer_nodes[['grid_x', 'grid_y', 'layer', 'lat', 'lon', 
                                   'altitude_ft', 'available', 'faa_ceiling_ft', 'max_obstacle_ft']],
            visible=True if layer in LAYERS_TO_DISPLAY else 'legendonly'
        )
        node_traces.append(node_trace)
        print(f"   Layer {layer}: {len(layer_nodes)} nodes")
    
    # --- Edge traces ---
    edge_traces = []
    
    # Collect cone data for direction arrows
    cone_data_by_layer = {layer: {'x': [], 'y': [], 'z': [], 'u': [], 'v': [], 'w': [], 'text': []} 
                          for layer in range(NUM_LAYERS)}
    
    # Horizontal edges by layer
    print("   Creating edge traces...")
    for layer in range(NUM_LAYERS):
        h_coords = {'x': [], 'y': [], 'z': []}
        h_info = []
        h_count = 0
        
        for u, v, data in G.edges(data=True):
            if data.get('layer_type') == 'Horizontal':
                u_layer = G.nodes[u]['layer']
                
                if u_layer == layer:
                    u_data = G.nodes[u]
                    v_data = G.nodes[v]
                    
                    h_coords['x'].extend([u_data['pos_x'], v_data['pos_x'], None])
                    h_coords['y'].extend([u_data['pos_y'], v_data['pos_y'], None])
                    h_coords['z'].extend([u_data['altitude'], v_data['altitude'], None])
                    
                    heading = data.get('heading', 0)
                    length = data.get('length', 0)
                    info = f"{u} → {v} | Heading: {heading:.1f}° | Length: {length:.1f}m"
                    h_info.extend([info, info, ''])
                    h_count += 1
                    
                    # Add cone marker at endpoint showing direction
                    # Place cone 80% along the edge towards the endpoint
                    cone_pos_x = u_data['pos_x'] + 0.8 * (v_data['pos_x'] - u_data['pos_x'])
                    cone_pos_y = u_data['pos_y'] + 0.8 * (v_data['pos_y'] - u_data['pos_y'])
                    cone_pos_z = u_data['altitude'] + 0.8 * (v_data['altitude'] - u_data['altitude'])
                    
                    # Direction vector (normalized)
                    dx = v_data['pos_x'] - u_data['pos_x']
                    dy = v_data['pos_y'] - u_data['pos_y']
                    dz = v_data['altitude'] - u_data['altitude']
                    norm = sqrt(dx**2 + dy**2 + dz**2)
                    if norm > 0:
                        dx, dy, dz = dx/norm, dy/norm, dz/norm
                    
                    cone_data_by_layer[layer]['x'].append(cone_pos_x)
                    cone_data_by_layer[layer]['y'].append(cone_pos_y)
                    cone_data_by_layer[layer]['z'].append(cone_pos_z)
                    cone_data_by_layer[layer]['u'].append(dx * 720)  # Scale for visibility (adjusted for 200m edges)
                    cone_data_by_layer[layer]['v'].append(dy * 720)
                    cone_data_by_layer[layer]['w'].append(dz * 720)
                    cone_data_by_layer[layer]['text'].append(f"{u} → {v}")
        
        if h_coords['x']:
            altitude_ft = LAYER_ALTITUDES_FT[layer]
            edge_traces.append(go.Scatter3d(
                x=h_coords['x'],
                y=h_coords['y'],
                z=h_coords['z'],
                mode='lines',
                name=f'L{layer} Edges ({altitude_ft}ft)',
                legendgroup=f'edges_layer{layer}',
                line=dict(color='blue', width=HORIZONTAL_LINE_WIDTH),
                text=h_info,
                hovertemplate='<b>Horizontal Edge</b><br>%{text}<extra></extra>',
                opacity=0.4,
                visible=True if layer in LAYERS_TO_DISPLAY else 'legendonly'
            ))
            print(f"      Layer {layer}: {h_count} horizontal edges")
            
            # Add direction cone markers for this layer
            if len(cone_data_by_layer[layer]['x']) > 0:
                edge_traces.append(go.Cone(
                    x=cone_data_by_layer[layer]['x'],
                    y=cone_data_by_layer[layer]['y'],
                    z=cone_data_by_layer[layer]['z'],
                    u=cone_data_by_layer[layer]['u'],
                    v=cone_data_by_layer[layer]['v'],
                    w=cone_data_by_layer[layer]['w'],
                    name=f'L{layer} Arrows ({altitude_ft}ft)',
                    legendgroup=f'arrows_layer{layer}',
                    showlegend=True,
                    showscale=False,  # Hide the color scale bar
                    colorscale=[[0, 'black'], [1, 'black']],  # Black arrows for direction
                    sizemode='absolute',
                    sizeref=40,  # Cone size (adjusted for 200m edges, reference uses 130 for longer streets)
                    text=cone_data_by_layer[layer]['text'],
                    hovertemplate='<b>Direction Arrow</b><br>%{text}<extra></extra>',
                    visible=True if layer in LAYERS_TO_DISPLAY else 'legendonly',
                    opacity=0.7
                ))
                print(f"      Layer {layer}: {len(cone_data_by_layer[layer]['x'])} direction arrows")
    
    # Vertical edges
    v_coords = {'x': [], 'y': [], 'z': []}
    v_info = []
    v_count = 0
    
    for u, v, data in G.edges(data=True):
        if data.get('layer_type') == 'Vertical':
            u_data = G.nodes[u]
            v_data = G.nodes[v]
            
            # Only show upward edges (to avoid duplicates in visualization)
            if u_data['layer'] < v_data['layer']:
                v_coords['x'].extend([u_data['pos_x'], v_data['pos_x'], None])
                v_coords['y'].extend([u_data['pos_y'], v_data['pos_y'], None])
                v_coords['z'].extend([u_data['altitude'], v_data['altitude'], None])
                
                length = data.get('length', 0)
                info = f"{u} ↕ {v} | Length: {length:.1f}m"
                v_info.extend([info, info, ''])
                v_count += 1
    
    if v_coords['x']:
        edge_traces.append(go.Scatter3d(
            x=v_coords['x'],
            y=v_coords['y'],
            z=v_coords['z'],
            mode='lines',
            name=f'Vertical Edges',
            line=dict(color='red', width=VERTICAL_LINE_WIDTH),
            text=v_info,
            hovertemplate='<b>Vertical Edge</b><br>%{text}<extra></extra>',
            opacity=0.5
        ))
        print(f"      Vertical: {v_count} edges (showing upward only)")
    
    # Create figure
    fig = go.Figure(data=edge_traces + node_traces)
    
    fig.update_layout(
        scene=dict(
            xaxis_title='Position X (Meters)',
            yaxis_title='Position Y (Meters)',
            zaxis_title='Altitude (Meters)',
            bgcolor='#f0f0f0',
            aspectmode='manual',
            aspectratio=dict(x=1, y=0.5, z=0.02),  # Adjusted for 30x15 grid
            camera=dict(eye=dict(x=1.5, y=1.5, z=1.2))
        ),
        title=f'Regular 3D Lattice Graph: {GRID_X}×{GRID_Y}×{NUM_LAYERS} ({G.number_of_nodes()} nodes, {G.number_of_edges()} edges)',
        height=900,
        showlegend=True
    )
    
    # Save HTML
    fig.write_html(HTML_FILENAME, auto_open=False)
    print(f"\n6. Visualization saved to {HTML_FILENAME}")
    
    # Open in browser
    try:
        webbrowser.open_new_tab(HTML_FILENAME)
        print(f"7. Opened {HTML_FILENAME} in web browser")
    except Exception as e:
        print(f"7. Could not open browser automatically: {e}")

# --- Save Graph ---

def save_graph_to_pickle(G, filename):
    """Save graph to pickle file with metadata."""
    print(f"\n8. Saving graph to pickle file...")
    
    try:
        metadata = {
            'created_at': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'grid_dimensions': f'{GRID_X}×{GRID_Y}×{NUM_LAYERS}',
            'grid_x': GRID_X,
            'grid_y': GRID_Y,
            'num_layers': NUM_LAYERS,
            'horizontal_spacing_m': HORIZONTAL_SPACING_M,
            'vertical_spacing_ft': VERTICAL_SPACING_FT,
            'layer_altitudes_ft': LAYER_ALTITUDES_FT,
            'reference_point_a': REF_POINT_A,
            'reference_point_b': REF_POINT_B,
            'num_nodes': G.number_of_nodes(),
            'num_edges': G.number_of_edges(),
            'faa_ceiling_csv': FAA_CEILING_CSV,
            'building_source': 'OpenStreetMap (OSMnx)',
            'min_clearance_ft': MIN_CLEARANCE_FT
        }
        
        graph_data = {
            'graph': G,
            'metadata': metadata
        }
        
        with open(filename, 'wb') as f:
            pickle.dump(graph_data, f, protocol=pickle.HIGHEST_PROTOCOL)
        
        import os
        file_size_mb = os.path.getsize(filename) / (1024 * 1024)
        
        print(f"   ✓ Graph saved to '{filename}'")
        print(f"   ✓ File size: {file_size_mb:.2f} MB")
        print(f"   ✓ Nodes: {metadata['num_nodes']}, Edges: {metadata['num_edges']}")
        print(f"\n   To load this graph:")
        print(f"   >>> import pickle")
        print(f"   >>> with open('{filename}', 'rb') as f:")
        print(f"   >>>     data = pickle.load(f)")
        print(f"   >>>     G = data['graph']")
        print(f"   >>>     metadata = data['metadata']")
        
    except Exception as e:
        print(f"   ✗ ERROR saving graph: {e}")

# --- Main Execution ---

if __name__ == '__main__':
    print("="*70)
    print("REGULAR 3D LATTICE GRAPH GENERATOR")
    print("="*70)
    
    # 1. Generate grid nodes
    nodes = create_grid_nodes()
    
    # 2. Calculate bounding box from grid extent
    all_lats = [node['lat'] for node in nodes.values()]
    all_lons = [node['lon'] for node in nodes.values()]
    
    # Create bbox as a dictionary (matching reference file format)
    bbox = {
        'south': min(all_lats),
        'north': max(all_lats),
        'west': min(all_lons),
        'east': max(all_lons)
    }
    print(f"\nGrid bounding box:")
    print(f"  N={bbox['north']:.6f}, S={bbox['south']:.6f}, E={bbox['east']:.6f}, W={bbox['west']:.6f}")
    
    # 3. Load constraint data
    faa_df = load_faa_ceiling_data(FAA_CEILING_CSV)
    building_df = load_building_data(bbox)
    
    # 4. Update node availability
    update_node_availability(nodes, faa_df, building_df)
    
    # 4. Create graph with edges
    G = create_graph_with_edges(nodes)
    
    # 5. Visualize
    visualize_3d_graph(G)
    
    # 6. Save to pickle
    if SAVE_GRAPH_PICKLE:
        save_graph_to_pickle(G, PICKLE_FILENAME)
    
    print("\n" + "="*70)
    print("GRAPH GENERATION COMPLETE")
    print("="*70)
    print(f"Total Nodes: {G.number_of_nodes()}")
    print(f"Total Edges: {G.number_of_edges()}")
    print(f"Grid: {GRID_X}×{GRID_Y}×{NUM_LAYERS}")
    print(f"Output files: {HTML_FILENAME}, {PICKLE_FILENAME}")
    print("="*70)
