"""
Road Network Graph with Altitude Availability - Directed Graph (DiGraph)

This script creates a 5-layer 3D lattice graph for UAS (drone) path planning in Manhattan, NYC.
It generates a directed graph (DiGraph) where street edges have specific directions.

Key Features:
- Downloads street network from OpenStreetMap using OSMnx
- Creates 5 altitude layers (0-4) spaced 30 meters apart
- Implements directed street edges (one-way navigation) with random direction assignment
- Checks node availability based on FAA airspace ceilings and building obstacles
- Layer 0: Takeoff/landing zones (no street edges, always available)
- Layers 1-4: Navigation layers with availability constraints
- Adds vertical edges (bidirectional) between layers
- Adds diagonal edges (bidirectional) between neighboring nodes in adjacent layers
- Visualizes the 3D graph with Plotly showing direction arrows (black cones) on streets
- Color coding: Green nodes = available, Red nodes = unavailable

Output: Interactive 3D HTML visualization (nyc_lattice_graph.html)
"""

import osmnx as ox
import networkx as nx
import pandas as pd
import numpy as np
import plotly.graph_objects as go
from math import sqrt
import random
from geopy.distance import geodesic
import webbrowser # Imported for manually opening the generated HTML file
import pickle
from datetime import datetime

# --- Configuration ---

# 1. Define the Bounding Box (Manhattan Island)
# Covers from Battery Park (south) to Inwood (north)
MIN_LAT = 40.700292  # Battery Park area (southern tip)
MAX_LAT = 40.878178  # Inwood/northern tip
MIN_LON = -74.018231 # West side (Hudson River)
MAX_LON = -73.907005 # East side (East River)

# Use bounding box instead of place name for precise area control
USE_BOUNDING_BOX = False
PLACE_NAME = "Manhattan, New York, NY, USA" # Fallback if bounding box fails

# 2. Lattice Parameters
NUM_LAYERS = 5         # Total layers (0 to 4)
LAYER_HEIGHT_M = 30    # Fixed vertical distance between layers in meters
ENABLE_DIAGONAL_EDGES = False  # Set to False to disable diagonal connections between layers

# 3. Visualization Parameters
LAYERS_TO_DISPLAY = [0, 1, 2, 3, 4]  # Which layers to show in visualization (0-4)
                                       # Example: [0, 2, 4] shows only layers 0, 2, and 4

# Node and edge display sizes
NODE_SIZE = 7           # Size of node markers (default: 1.5)
STREET_LINE_WIDTH = 3.5     # Width of street edges (default: 1)
VERTICAL_LINE_WIDTH = 2.5   # Width of vertical edges (default: 2)
DIAGONAL_LINE_WIDTH = 2   # Width of diagonal edges (default: 1)

# 4. Obstacle and Airspace Configuration
FAA_CEILING_CSV = "Filtered_FAA_UAS_FacilityMap_Data_LGA.csv"  # Path to FAA ceiling data
MIN_CLEARANCE_FT = 100    # Minimum clearance above obstacles (feet)
OBSTACLE_SEARCH_RADIUS_M = 50  # Radius to search for obstacles around each node (meters)
DEFAULT_BUILDING_HEIGHT_FT = 50  # Conservative default for buildings without height data (feet)
METERS_TO_FEET = 3.28084  # Conversion factor

# 5. Graph Save Configuration
SAVE_GRAPH_PICKLE = True  # Set to True to save the graph as a pickle file
PICKLE_FILENAME = "nyc_lattice_graph.pkl"  # Name of the pickle file to save

# --- Step 0: Load Obstacle and Airspace Data ---

def load_faa_ceiling_data(csv_path):
    """
    Loads FAA UAS ceiling data from CSV file.
    Returns a DataFrame with CEILING (ft), LATITUDE, LONGITUDE.
    """
    print(f"0a. Loading FAA ceiling data from {csv_path}...")
    try:
        faa_df = pd.read_csv(csv_path)
        print(f"    Loaded {len(faa_df)} FAA ceiling grid points")
        print(f"    Ceiling values: {faa_df['CEILING'].unique()}")
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
    Returns a GeoDataFrame with building geometries and estimated heights in feet.
    """
    print(f"0b. Downloading building data from OpenStreetMap...")
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
    """
    Gets the FAA ceiling (in feet) at a given lat/lon by finding nearest grid point.
    """
    if faa_df is None:
        return 400  # Default ceiling
    
    # Calculate distances to all grid points
    faa_df['dist'] = np.sqrt(
        (faa_df['LATITUDE'] - lat)**2 + 
        (faa_df['LONGITUDE'] - lon)**2
    )
    
    # Return ceiling of nearest point
    nearest = faa_df.loc[faa_df['dist'].idxmin()]
    return nearest['CEILING']

def get_max_obstacle_height_near_node(lat, lon, buildings_gdf, radius_m=OBSTACLE_SEARCH_RADIUS_M):
    """
    Finds the maximum building/obstacle height within a radius of a node.
    Returns height in feet.
    """
    if buildings_gdf is None or len(buildings_gdf) == 0:
        return 0.0  # No obstacles
    
    from shapely.geometry import Point
    
    # Create point for the node (in WGS84)
    node_point = Point(lon, lat)
    
    # Convert radius from meters to degrees (approximate at this latitude)
    # At 40°N latitude: 1 degree ≈ 85 km, so 50m ≈ 0.0006 degrees
    radius_deg = radius_m / 111000  # rough approximation
    
    # Find buildings within bounding box (faster than checking all)
    nearby = buildings_gdf[
        (buildings_gdf.geometry.bounds['minx'] >= lon - radius_deg) &
        (buildings_gdf.geometry.bounds['maxx'] <= lon + radius_deg) &
        (buildings_gdf.geometry.bounds['miny'] >= lat - radius_deg) &
        (buildings_gdf.geometry.bounds['maxy'] <= lat + radius_deg)
    ]
    
    if len(nearby) == 0:
        return 0.0
    
    # Find max height among nearby buildings
    max_height = nearby['height_ft'].max()
    return max_height

def check_node_availability(node_data, faa_df, buildings_gdf):
    """
    Determines if a node is available based on:
    1. Node altitude must be at least MIN_CLEARANCE_FT above highest obstacle
    2. Node altitude must not exceed FAA ceiling
    
    Returns: (available: bool, reason: str, faa_ceiling_ft: float, max_obstacle_ft: float)
    """
    lat = node_data['lat']
    lon = node_data['lon']
    altitude_m = node_data['altitude']
    altitude_ft = altitude_m * METERS_TO_FEET
    
    # Get FAA ceiling at this location
    faa_ceiling_ft = get_faa_ceiling_at_location(lat, lon, faa_df)
    
    # Get maximum obstacle height near this node
    max_obstacle_ft = get_max_obstacle_height_near_node(lat, lon, buildings_gdf)
    
    # Apply availability rules
    if altitude_ft > faa_ceiling_ft:
        return False, "Above FAA ceiling", faa_ceiling_ft, max_obstacle_ft
    
    if altitude_ft < (max_obstacle_ft + MIN_CLEARANCE_FT):
        return False, "Too close to obstacle", faa_ceiling_ft, max_obstacle_ft
    
    return True, "Available", faa_ceiling_ft, max_obstacle_ft

# --- Step 1 & 2: Generate Base Graph (Layer 0) and Project ---

def create_base_graph(place_name=None):
    """
    Downloads the street network for a specified bounding box or place name, projects it to a metric
    coordinate system, and calculates meter positions relative to the origin.
    """
    if USE_BOUNDING_BOX:
        print(f"1. Downloading and processing base street network for bounding box:")
        print(f"   Lat: {MIN_LAT} to {MAX_LAT}")
        print(f"   Lon: {MIN_LON} to {MAX_LON}")
        
        # Download graph using bounding box for precise area control
        # OSMnx 2.x expects bbox as (left, bottom, right, top) = (west, south, east, north)
        # left/west = MIN_LON, bottom/south = MIN_LAT, right/east = MAX_LON, top/north = MAX_LAT
        G_base = ox.graph_from_bbox(
            bbox=(MIN_LON, MIN_LAT, MAX_LON, MAX_LAT),
            network_type="drive", 
            simplify=True
        )
    else:
        print(f"1. Downloading and processing base street network for '{place_name}' (Layer 0)...")
        
        # Download graph using place name as fallback
        G_base = ox.graph_from_place(
            place_name,
            network_type="drive", 
            simplify=True
        )
    
    # Project the graph to a local meter-based coordinate system (e.g., UTM)
    G_projected = ox.project_graph(G_base)

    # Get the minimum projected coordinates to set a (0, 0) origin for meter positions
    min_x = min(data['x'] for node, data in G_projected.nodes(data=True))
    min_y = min(data['y'] for node, data in G_projected.nodes(data=True))

    # Calculate and store meter-based positions (pos_x, pos_y) and coordinates (lat, lon)
    for u, data in G_projected.nodes(data=True):
        # Coordinates in degrees (from OSMnx's original graph)
        data['lat'] = G_base.nodes[u]['y']
        data['lon'] = G_base.nodes[u]['x']
        
        # Position in meters relative to the south-west corner of the box
        data['pos_x'] = data['x'] - min_x
        data['pos_y'] = data['y'] - min_y
        data['altitude'] = 0.0
        # NEW RULE: All Layer 0 nodes are always available (takeoff/landing zones)
        data['available'] = True
        data['layer'] = 0
        data['base_id'] = u # Store the original OSM ID
        data['label'] = f"L0_ID{u}"
    
    # Recalculate edge weights (length in meters) in the projected graph
    # This ensures accurate weights for the horizontal edges
    # NEW RULE: Street edges are DIRECTED. If oneway not specified, assign random direction
    # Track processed edge pairs to avoid creating bidirectional edges
    processed_pairs = set()
    
    for u, v, data in list(G_projected.edges(data=True)):
        data['length'] = data.get('length', 0.0)
        
        # Ensure a 'length' attribute exists
        if 'length' not in data or data['length'] == 0.0:
            # Fallback distance calculation using projected coordinates
            x1, y1 = G_projected.nodes[u]['x'], G_projected.nodes[u]['y']
            x2, y2 = G_projected.nodes[v]['x'], G_projected.nodes[v]['y']
            data['length'] = sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        # Create a sorted tuple to identify the edge pair (regardless of direction)
        edge_pair = tuple(sorted([u, v]))
        
        # Skip if we've already processed this edge pair
        if edge_pair in processed_pairs:
            continue
        
        # Mark this edge pair as processed
        processed_pairs.add(edge_pair)
        
        # Check if edge has a specified direction (oneway tag from OSM)
        oneway = data.get('oneway', False)
        
        # If it's a oneway street, keep only the correct direction
        if oneway:
            # Remove reverse direction if it exists
            if G_projected.has_edge(v, u):
                G_projected.remove_edge(v, u)
        else:
            # For two-way streets, randomly choose ONE direction only
            # First, remove the reverse edge if it exists
            if G_projected.has_edge(v, u):
                G_projected.remove_edge(v, u)
            
            # Then randomly decide which direction to keep
            if random.random() < 0.5:
                # Keep current direction (u -> v) - do nothing
                pass
            else:
                # Reverse direction: remove u->v and add v->u instead
                G_projected.add_edge(v, u, **data.copy())
                G_projected.remove_edge(u, v)

    # Return as DiGraph for path planning
    return nx.DiGraph(G_projected)

# --- Step 3 & 4: Build Multi-Layer Graph and Assign Node Data ---

def create_multilayer_graph(G_base, faa_df, buildings_gdf):
    """
    Duplicates the base graph to create upper layers (1-4) and initializes the
    vertical connections. Checks availability of each node based on obstacles and FAA ceilings.
    """
    print("2. Creating 4 duplicate layers and checking node availability...")
    G = nx.DiGraph(G_base)
    base_nodes = list(G_base.nodes(data=True))
    
    # Track availability statistics
    total_nodes = 0
    available_nodes = 0
    unavailable_by_ceiling = 0
    unavailable_by_obstacle = 0
    
    for L in range(1, NUM_LAYERS):
        altitude = L * LAYER_HEIGHT_M
        layer_available = 0
        layer_total = 0
        
        # 3a. Duplicate Nodes for the New Layer L
        # First pass: Create all nodes with availability info
        for u, data in base_nodes:
            new_id = f"{u}_{L}"
            
            # Copy all data from the base node
            new_data = data.copy()
            new_data['altitude'] = altitude
            new_data['layer'] = L
            new_data['base_id'] = u
            new_data['label'] = f"L{L}_ID{u}"
            
            # Check node availability
            available, reason, faa_ceiling, max_obstacle = check_node_availability(
                new_data, faa_df, buildings_gdf
            )
            
            new_data['available'] = available
            new_data['unavailable_reason'] = reason
            new_data['faa_ceiling_ft'] = faa_ceiling
            new_data['max_obstacle_ft'] = max_obstacle
            
            # Update statistics
            total_nodes += 1
            layer_total += 1
            if available:
                available_nodes += 1
                layer_available += 1
            else:
                if "ceiling" in reason.lower():
                    unavailable_by_ceiling += 1
                else:
                    unavailable_by_obstacle += 1
            
            # Add the new node to the multi-layer graph
            G.add_node(new_id, **new_data)
        
        # 3b. Second pass: Add edges between nodes (now all nodes exist)
        # NEW RULE: Street edges only connect available nodes (directed)
        for u, data in base_nodes:
            new_id = f"{u}_{L}"
            u_available = G.nodes[new_id]['available']
            
            for _, v_base, edge_data in G_base.out_edges(u, data=True):
                v_new_id = f"{v_base}_{L}"
                
                # Get availability of both nodes
                v_available = G.nodes[v_new_id]['available']
                
                # Only add street edge if BOTH nodes are available
                if u_available and v_available:
                    # Copy the street edge (horizontal connection) - DIRECTED
                    G.add_edge(new_id, v_new_id, **edge_data, layer_type='Street')
        
        print(f"   Layer {L} ({altitude}m / {altitude*METERS_TO_FEET:.0f}ft): {layer_available}/{layer_total} nodes available")
    
    # Print overall statistics
    print(f"\n   Overall Node Availability:")
    print(f"   - Total nodes (layers 1-{NUM_LAYERS-1}): {total_nodes}")
    print(f"   - Available: {available_nodes} ({100*available_nodes/total_nodes:.1f}%)")
    print(f"   - Unavailable (FAA ceiling): {unavailable_by_ceiling}")
    print(f"   - Unavailable (obstacle clearance): {unavailable_by_obstacle}")

    return G

# --- Step 5: Add Vertical and Diagonal Lattice Edges ---

def add_vertical_lattice(G):
    """
    Adds non-oriented (two-way) vertical and diagonal edges between adjacent layers.
    """
    print("3. Adding vertical and diagonal lattice connections...")
    
    # Store nodes by their base_id and layer for easy lookup
    nodes_by_base_id = {}
    for node_id, data in G.nodes(data=True):
        base_id = data['base_id']
        layer = data['layer']
        if base_id not in nodes_by_base_id:
            nodes_by_base_id[base_id] = {}
        nodes_by_base_id[base_id][layer] = node_id

    # Iterate through all layers L from 0 to NUM_LAYERS - 2 (up to L=3)
    for L in range(NUM_LAYERS - 1):
        
        # Iterate through every node in the current layer L
        for u_id, u_data in G.nodes(data=True):
            if u_data['layer'] != L:
                continue

            u_base = u_data['base_id'] # Get the original OSM ID (int)
            u_available = u_data['available']
            
            # 5a. Direct Vertical Connection: u(L) <-> u(L+1)
            # NEW RULE: Vertical edges connect ALL nodes (available and unavailable) - UNDIRECTED
            u_next_id = nodes_by_base_id[u_base][L + 1]
            
            # Add two-way edge (undirected) with fixed weight - NO availability check
            G.add_edge(u_id, u_next_id, length=LAYER_HEIGHT_M, layer_type='Vertical')
            G.add_edge(u_next_id, u_id, length=LAYER_HEIGHT_M, layer_type='Vertical')
            
            # 5b. Diagonal Connections: u(L) <-> v(L+1) where v is a neighbor of u
            # NEW RULE: Diagonal edges are UNDIRECTED and connect:
            #   - Available <-> Available (YES)
            #   - Available <-> Unavailable (YES)
            #   - Unavailable <-> Unavailable (NO)
            # Only add diagonal edges if ENABLE_DIAGONAL_EDGES is True
            if ENABLE_DIAGONAL_EDGES:
                base_id_to_neighbors = G_base.adj
                
                for v_base_id in base_id_to_neighbors[u_base]:
                    
                    # Node v in the NEXT layer L+1
                    v_next_id = nodes_by_base_id[v_base_id][L + 1]
                    v_next_data = G.nodes[v_next_id]
                    v_next_available = v_next_data['available']
                    
                    # Check availability rule: NOT (both unavailable)
                    if not (not u_available and not v_next_available):
                        # At least one node is available, so add diagonal edge
                        
                        # Calculate Euclidean distance between u(L) and v(L+1)
                        u_pos = np.array([u_data['pos_x'], u_data['pos_y'], u_data['altitude']])
                        v_next_pos = np.array([v_next_data['pos_x'], v_next_data['pos_y'], v_next_data['altitude']])
                        
                        diagonal_length = np.linalg.norm(v_next_pos - u_pos)
                        
                        # Add two-way diagonal edge (undirected)
                        G.add_edge(u_id, v_next_id, length=diagonal_length, layer_type='Diagonal')
                        G.add_edge(v_next_id, u_id, length=diagonal_length, layer_type='Diagonal')
    
    # NEW RULE: Remove all street edges from Layer 0
    print("4. Removing all street edges from Layer 0...")
    edges_to_remove = []
    for u, v, data in G.edges(data=True):
        # Check if both nodes are in Layer 0 and it's a street edge
        if G.nodes[u]['layer'] == 0 and G.nodes[v]['layer'] == 0:
            if data.get('layer_type', 'Street') == 'Street':
                edges_to_remove.append((u, v))
    
    for u, v in edges_to_remove:
        G.remove_edge(u, v)
    
    print(f"   Removed {len(edges_to_remove)} street edges from Layer 0")
                
    return G# --- Step 6: Visualization using Plotly ---

def visualize_3d_graph(G):
    """
    Creates an interactive 3D visualization of the graph using Plotly.
    Shows all nodes but samples edges to prevent overwhelming the browser.
    """
    FILE_NAME = "nyc_lattice_graph.html"
    print("4. Generating interactive 3D Plotly visualization...")

    # DataFrames for easier Plotly setup
    nodes_df = pd.DataFrame.from_dict(dict(G.nodes(data=True)), orient='index')
    
    # --- DEBUG: Print coordinate statistics ---
    print(f"   Node DataFrame shape: {nodes_df.shape}")
    print(f"   pos_x range: {nodes_df['pos_x'].min():.2f} to {nodes_df['pos_x'].max():.2f}")
    print(f"   pos_y range: {nodes_df['pos_y'].min():.2f} to {nodes_df['pos_y'].max():.2f}")
    print(f"   altitude range: {nodes_df['altitude'].min():.2f} to {nodes_df['altitude'].max():.2f}")
    print(f"   Available nodes: {nodes_df['available'].sum()}, Unavailable: {(~nodes_df['available']).sum()}")
    print(f"   Nodes by layer: {nodes_df['layer'].value_counts().sort_index().to_dict()}")
    print(f"   Sample node altitudes by layer:")
    for layer in range(NUM_LAYERS):
        sample = nodes_df[nodes_df['layer'] == layer]['altitude'].head(3).tolist()
        print(f"     Layer {layer}: {sample}")
    
    # --- Create separate node traces for each layer (for interactive filtering) ---
    print(f"   Creating separate traces for each layer...")
    node_traces = []
    
    # Color nodes based on availability (green = available, red = unavailable)
    nodes_df['color'] = nodes_df['available'].apply(lambda x: 'green' if x else 'red')
    
    # Create a trace for each layer
    for layer in range(NUM_LAYERS):
        layer_nodes = nodes_df[nodes_df['layer'] == layer].copy()
        altitude = layer * LAYER_HEIGHT_M
        
        # Calculate altitude in feet for display
        layer_nodes['altitude_ft'] = layer_nodes['altitude'] * METERS_TO_FEET
        
        node_trace = go.Scatter3d(
            x=layer_nodes['pos_x'], 
            y=layer_nodes['pos_y'], 
            z=layer_nodes['altitude'],
            mode='markers',
            name=f'L{layer} Nodes ({altitude}m)',
            legendgroup=f'nodes_layer{layer}',  # Independent group for nodes only
            marker=dict(
                symbol='circle',
                size=NODE_SIZE,  # Configurable node size
                color=layer_nodes['color'],
                opacity=0.6,
            ),
            hovertemplate='<b>Node ID:</b> %{text}<br>' +
                          '<b>Base ID:</b> %{customdata[0]}<br>' +
                          '<b>Layer:</b> %{customdata[1]}<br>' +
                          '<b>Latitude:</b> %{customdata[2]:.6f}<br>' +
                          '<b>Longitude:</b> %{customdata[3]:.6f}<br>' +
                          '<b>Altitude:</b> %{z:.1f}m (%{customdata[4]:.0f}ft)<br>' +
                          '<b>Available:</b> %{customdata[5]}<br>' +
                          '<b>FAA Ceiling:</b> %{customdata[6]:.0f}ft<br>' +
                          '<b>Max Obstacle:</b> %{customdata[7]:.0f}ft<br>' +
                          '<b>Status:</b> %{customdata[8]}<extra></extra>',
            text=layer_nodes['label'],
            customdata=layer_nodes[['base_id', 'layer', 'lat', 'lon', 'altitude_ft',
                                   'available', 'faa_ceiling_ft', 'max_obstacle_ft', 'unavailable_reason']],
            visible=True if layer in LAYERS_TO_DISPLAY else 'legendonly'  # Initially show only selected layers
        )
        node_traces.append(node_trace)
        print(f"     Layer {layer}: {len(layer_nodes)} nodes")

    # --- Edges (Streets and Lattice Connections) ---
    # Create separate edge traces for each layer and edge type combination
    print(f"   Creating edge traces organized by layer...")
    edge_traces = []
    
    # Sample edges to prevent browser overload
    all_edges = list(G.edges(data=True))
    print(f"   Total edges: {len(all_edges)}")
    
    # Organize edges by layer and type
    # Collect cone data for direction arrows
    cone_data_by_layer = {layer: {'x': [], 'y': [], 'z': [], 'u': [], 'v': [], 'w': [], 'text': []} 
                          for layer in range(NUM_LAYERS)}
    
    for layer in range(NUM_LAYERS):
        # Street edges within this layer
        street_coords = {'x': [], 'y': [], 'z': []}
        street_info = []  # Store hover info for each edge
        street_count = 0
        street_sampled = 0
        
        for u, v, data in all_edges:
            if data.get('layer_type', 'Street') == 'Street':
                u_layer = G.nodes[u]['layer']
                v_layer = G.nodes[v]['layer']
                
                # Only include edges where both nodes are in this layer
                if u_layer == layer and v_layer == layer:
                    street_count += 1
                    # Sample at 100% for streets
                    if random.random() < 1.0:
                        u_pos = G.nodes[u]
                        v_pos = G.nodes[v]
                        
                        street_coords['x'].extend([u_pos['pos_x'], v_pos['pos_x'], None])
                        street_coords['y'].extend([u_pos['pos_y'], v_pos['pos_y'], None])
                        street_coords['z'].extend([u_pos['altitude'], v_pos['altitude'], None])
                        
                        # Store edge information for hover (with direction)
                        length = data.get('length', 0)
                        street_name = data.get('name', 'Unnamed')
                        # Handle cases where name might be a list
                        if isinstance(street_name, list):
                            street_name = ', '.join(str(n) for n in street_name)
                        
                        # Enhanced hover with direction
                        edge_info = f"Direction: {G.nodes[u]['label']} → {G.nodes[v]['label']} | Length: {length:.1f}m | Street: {street_name}"
                        street_info.extend([edge_info, edge_info, ''])
                        street_sampled += 1
                        
                        # Add cone marker at endpoint showing direction
                        # Place cone 80% along the edge towards the endpoint
                        cone_pos_x = u_pos['pos_x'] + 0.8 * (v_pos['pos_x'] - u_pos['pos_x'])
                        cone_pos_y = u_pos['pos_y'] + 0.8 * (v_pos['pos_y'] - u_pos['pos_y'])
                        cone_pos_z = u_pos['altitude'] + 0.8 * (v_pos['altitude'] - u_pos['altitude'])
                        
                        # Direction vector (normalized)
                        dx = v_pos['pos_x'] - u_pos['pos_x']
                        dy = v_pos['pos_y'] - u_pos['pos_y']
                        dz = v_pos['altitude'] - u_pos['altitude']
                        norm = sqrt(dx**2 + dy**2 + dz**2)
                        if norm > 0:
                            dx, dy, dz = dx/norm, dy/norm, dz/norm
                        
                        cone_data_by_layer[layer]['x'].append(cone_pos_x)
                        cone_data_by_layer[layer]['y'].append(cone_pos_y)
                        cone_data_by_layer[layer]['z'].append(cone_pos_z)
                        cone_data_by_layer[layer]['u'].append(dx * 50)  # Scale for visibility
                        cone_data_by_layer[layer]['v'].append(dy * 50)
                        cone_data_by_layer[layer]['w'].append(dz * 50)
                        cone_data_by_layer[layer]['text'].append(f"{G.nodes[u]['label']} → {G.nodes[v]['label']}")
        
        # Create street edge trace for this layer
        if street_coords['x']:
            altitude = layer * LAYER_HEIGHT_M
            edge_traces.append(go.Scatter3d(
                x=street_coords['x'], 
                y=street_coords['y'], 
                z=street_coords['z'],
                mode='lines',
                name=f'L{layer} Streets ({altitude}m)',
                legendgroup=f'streets_layer{layer}',  # Independent group for streets only
                line=dict(color='grey', width=STREET_LINE_WIDTH),  # Configurable street line width
                text=street_info,
                hovertemplate='<b>Street Edge (Directed)</b><br>%{text}<extra></extra>',
                hoverinfo='text',
                opacity=0.4,
                visible=True if layer in LAYERS_TO_DISPLAY else 'legendonly'
            ))
            print(f"     Layer {layer} streets: {street_sampled}/{street_count} shown")
            
            # Add direction cone markers for this layer
            if len(cone_data_by_layer[layer]['x']) > 0:
                edge_traces.append(go.Cone(
                    x=cone_data_by_layer[layer]['x'],
                    y=cone_data_by_layer[layer]['y'],
                    z=cone_data_by_layer[layer]['z'],
                    u=cone_data_by_layer[layer]['u'],
                    v=cone_data_by_layer[layer]['v'],
                    w=cone_data_by_layer[layer]['w'],
                    name=f'L{layer} Arrows ({altitude}m)',
                    legendgroup=f'cones_layer{layer}',  # Independent group for cones only
                    showlegend=True,  # Show legend entry for each layer
                    showscale=False,  # Hide the color scale bar to prevent black rectangle
                    colorscale=[[0, 'black'], [1, 'black']],  # Black arrows for direction
                    sizemode='absolute',
                    sizeref=130,
                    text=cone_data_by_layer[layer]['text'],
                    hovertemplate='<b>Direction Arrow</b><br>%{text}<extra></extra>',
                    visible=True if layer in LAYERS_TO_DISPLAY else 'legendonly',
                    opacity=0.7
                ))
                print(f"     Layer {layer} direction cones: {len(cone_data_by_layer[layer]['x'])} added")
    
    # Vertical and Diagonal edges (connecting between layers)
    # These are shown separately as they don't belong to a single layer
    edge_types_interlayer = {'Vertical': 'blue', 'Diagonal': 'orange'}
    
    for edge_type, color in edge_types_interlayer.items():
        x_coords, y_coords, z_coords = [], [], []
        edge_info = []  # Store hover info for each edge
        edge_count = 0
        edges_sampled = 0
        
        # Sampling rate for inter-layer edges
        sample_rate = 1 if edge_type == 'Vertical' else 1
        
        for u, v, data in all_edges:
            if data.get('layer_type') == edge_type:
                edge_count += 1
                
                # Sample edges to reduce rendering load
                if random.random() < sample_rate:
                    x_coords.extend([G.nodes[u]['pos_x'], G.nodes[v]['pos_x'], None])
                    y_coords.extend([G.nodes[u]['pos_y'], G.nodes[v]['pos_y'], None])
                    z_coords.extend([G.nodes[u]['altitude'], G.nodes[v]['altitude'], None])
                    
                    # Store edge information for hover
                    length = data.get('length', 0)
                    u_layer = G.nodes[u]['layer']
                    v_layer = G.nodes[v]['layer']
                    info = f"Length: {length:.1f}m | Layers: {u_layer} ↔ {v_layer}"
                    # Add info for start point, end point, and None separator
                    edge_info.extend([info, info, ''])
                    edges_sampled += 1
        
        # Create the trace for the current edge type
        if x_coords:
            # Determine line width based on edge type
            line_width = VERTICAL_LINE_WIDTH if edge_type == 'Vertical' else DIAGONAL_LINE_WIDTH
            
            edge_traces.append(go.Scatter3d(
                x=x_coords, y=y_coords, z=z_coords,
                mode='lines',
                name=f'{edge_type} Edges ({edges_sampled}/{edge_count} shown)',
                line=dict(color=color, width=line_width),  # Configurable line width
                text=edge_info,
                hovertemplate=f'<b>{edge_type} Edge</b><br>%{{text}}<extra></extra>',
                hoverinfo='text',
                opacity=0.6
            ))
            print(f"   {edge_type} edges: {edge_count} total, {edges_sampled} sampled for visualization")

    # --- Create Figure ---
    fig = go.Figure(data=edge_traces + node_traces)

    # Use manual aspect ratio to make layers clearly visible
    # Z is set to a smaller value since actual altitude (120m) is much less than XY span (~46km)
    fig.update_layout(
        scene=dict(
            xaxis_title='Position X (Meters)',
            yaxis_title='Position Y (Meters)',
            zaxis_title='Altitude Z (Meters)',
            bgcolor='#f0f0f0',
            aspectmode='manual',
            aspectratio=dict(x=1, y=1, z=0.02),  # Z is 0.2% of the XY to make layers visible
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=1.2)  # Better initial viewing angle
            )
        ),
        title=f'5-Layer NYC Lattice Graph (Z-spacing: {LAYER_HEIGHT_M}m, {G.number_of_nodes()} nodes)',
        height=900,
        showlegend=True
    )
    
    # Save the Plotly JSON to an HTML file for embedding/display
    # auto_open=False as per your scenario request
    fig.write_html(FILE_NAME, auto_open=False) 
    print(f"5. Plotly figure saved to {FILE_NAME}")
    
    # --- Manual Opening Logic (Solution to your query) ---
    try:
        # Programmatically open the HTML file in the default web browser
        webbrowser.open_new_tab(FILE_NAME)
        print(f"6. Automatically opened {FILE_NAME} in your web browser.")
    except Exception as e:
        print(f"6. Failed to open browser automatically. Please open the file manually: {FILE_NAME}")
    
# --- Save Graph to Pickle File ---

def save_graph_to_pickle(G, filename):
    """
    Saves the graph to a pickle file for later use.
    
    Args:
        G: NetworkX DiGraph to save
        filename: Name of the pickle file
    """
    print(f"\n7. Saving graph to pickle file...")
    
    try:
        # Create metadata dictionary with graph information
        metadata = {
            'created_at': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'num_nodes': G.number_of_nodes(),
            'num_edges': G.number_of_edges(),
            'num_layers': NUM_LAYERS,
            'layer_height_m': LAYER_HEIGHT_M,
            'diagonal_edges_enabled': ENABLE_DIAGONAL_EDGES,
            'bounding_box': {
                'min_lat': MIN_LAT,
                'max_lat': MAX_LAT,
                'min_lon': MIN_LON,
                'max_lon': MAX_LON
            } if USE_BOUNDING_BOX else None,
            'place_name': PLACE_NAME if not USE_BOUNDING_BOX else None,
            'faa_ceiling_csv': FAA_CEILING_CSV,
            'min_clearance_ft': MIN_CLEARANCE_FT,
            'obstacle_search_radius_m': OBSTACLE_SEARCH_RADIUS_M
        }
        
        # Package graph and metadata together
        graph_data = {
            'graph': G,
            'metadata': metadata
        }
        
        # Save to pickle file
        with open(filename, 'wb') as f:
            pickle.dump(graph_data, f, protocol=pickle.HIGHEST_PROTOCOL)
        
        # Get file size
        import os
        file_size_mb = os.path.getsize(filename) / (1024 * 1024)
        
        print(f"   ✓ Graph successfully saved to '{filename}'")
        print(f"   ✓ File size: {file_size_mb:.2f} MB")
        print(f"   ✓ Nodes: {metadata['num_nodes']}, Edges: {metadata['num_edges']}")
        print(f"\n   To load this graph later, use:")
        print(f"   >>> import pickle")
        print(f"   >>> with open('{filename}', 'rb') as f:")
        print(f"   >>>     data = pickle.load(f)")
        print(f"   >>>     G = data['graph']")
        print(f"   >>>     metadata = data['metadata']")
        
    except Exception as e:
        print(f"   ✗ ERROR saving graph to pickle: {e}")
    
# --- Execution ---

if __name__ == '__main__':
    # Print osmnx version for debugging/info
    print(f"OSMnx Version: {ox.__version__}\n")
    
    # 0. Load obstacle and airspace constraint data
    faa_df = load_faa_ceiling_data(FAA_CEILING_CSV)
    
    # 1. Create base graph (Layer 0) - this respects USE_BOUNDING_BOX setting
    G_base = create_base_graph(PLACE_NAME)
    
    # 2. Determine bbox for building data based on the actual graph extent
    if USE_BOUNDING_BOX:
        # Use predefined bounding box
        bbox = {
            'north': MAX_LAT,
            'south': MIN_LAT,
            'east': MAX_LON,
            'west': MIN_LON
        }
        print(f"\nUsing predefined bounding box:")
        print(f"  N={bbox['north']}, S={bbox['south']}, E={bbox['east']}, W={bbox['west']}")
        print(f"  Area span: {(bbox['north']-bbox['south'])*111:.2f}km x {abs(bbox['east']-bbox['west'])*85:.2f}km\n")
    else:
        # Extract bounding box from the downloaded graph (based on PLACE_NAME)
        lats = [data['lat'] for node, data in G_base.nodes(data=True)]
        lons = [data['lon'] for node, data in G_base.nodes(data=True)]
        
        bbox = {
            'north': max(lats),
            'south': min(lats),
            'east': max(lons),
            'west': min(lons)
        }
        print(f"\nUsing bounding box from place name '{PLACE_NAME}':")
        print(f"  N={bbox['north']:.6f}, S={bbox['south']:.6f}, E={bbox['east']:.6f}, W={bbox['west']:.6f}")
        print(f"  Area span: {(bbox['north']-bbox['south'])*111:.2f}km x {abs(bbox['east']-bbox['west'])*85:.2f}km\n")
    
    buildings_gdf = load_building_data(bbox)
    
    # Update Layer 0 nodes with airspace data (for info only - they're always available)
    print("\n   Updating Layer 0 nodes with airspace data...")
    for node_id, node_data in G_base.nodes(data=True):
        available, reason, faa_ceiling, max_obstacle = check_node_availability(
            node_data, faa_df, buildings_gdf
        )
        # NEW RULE: Layer 0 always available regardless of obstacles
        node_data['available'] = True
        node_data['unavailable_reason'] = 'Layer 0 - Takeoff/Landing Zone'
        node_data['faa_ceiling_ft'] = faa_ceiling
        node_data['max_obstacle_ft'] = max_obstacle

    # 2. Create all horizontal layers and connections (Layers 1-4) with availability checks
    G_multi = create_multilayer_graph(G_base, faa_df, buildings_gdf)
    
    # 3. Add all vertical and diagonal connections (Layer 0 streets will be removed here)
    G_final = add_vertical_lattice(G_multi)

    # 4. Visualize
    visualize_3d_graph(G_final)

    # 5. Save graph to pickle file (if enabled)
    if SAVE_GRAPH_PICKLE:
        save_graph_to_pickle(G_final, PICKLE_FILENAME)

    print(f"\nGraph creation complete.")
    print(f"Total Nodes: {G_final.number_of_nodes()}")
    print(f"Total Edges: {G_final.number_of_edges()}")

# Helper function definition needed for add_vertical_lattice
# This is placed here to satisfy the single-file mandate for complex code environments.
def get_distance_meters(u_coords, v_coords):
    """
    Calculates the 3D Euclidean distance in meters between two points (x, y, z).
    """
    x1, y1, z1 = u_coords
    x2, y2, z2 = v_coords
    return sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
