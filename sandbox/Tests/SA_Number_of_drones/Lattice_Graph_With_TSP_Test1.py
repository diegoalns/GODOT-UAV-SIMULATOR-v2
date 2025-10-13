# The following script builds upon the graph structure from the user's
# provided file and adds a Simulated Annealing algorithm to find
# conflict-free, optimized routes for three UAVs.

# --- Graph Creation (from original user code) ---
import networkx as nx
from geopy.distance import geodesic
import numpy as np
import plotly.graph_objects as go  # Import Plotly
import random
import math
import copy

# Define the bounding box for geographic coordinates
min_lat, max_lat = 40.6042, 40.6125
min_lon, max_lon = -73.9458, -73.9292
min_alt, max_alt = 0, 400

# Define grid dimensions
n_lat, n_lon, n_alt = 10, 10, 3

# Generate grid points using NumPy's linspace
lats = np.linspace(min_lat, max_lat, n_lat)
lons = np.linspace(min_lon, max_lon, n_lon)
alts = np.linspace(min_alt, max_alt, n_alt)

# conversion from lat/lon/alt to position in meters
# for i in range(n_lat):
#     lats[i] = geodesic((min_lat, min_lon), (lats[i], min_lon)).meters
#     lons[i] = geodesic((min_lat, min_lon), (min_lat, lons[i])).meters
    

# Create node identifiers and their geographic positions
nodes = [(i, j, k) for i in range(n_lat) for j in range(n_lon) for k in range(n_alt)]
pos = {(i, j, k): (lats[i], lons[j], alts[k]) for i in range(n_lat) for j in range(n_lon) for k in range(n_alt)}

# Randomly assign availability
availability = {node: np.random.random() < 0.75 for node in nodes}

# Create an empty directed graph using NetworkX
G = nx.DiGraph()

# Add all nodes to the graph
G.add_nodes_from(nodes)

# Set node attributes for geographic coordinates
nx.set_node_attributes(G, pos, 'pos')
nx.set_node_attributes(G, availability, 'available')

# Define a function to calculate the slant range (3D distance) between two points
def slant_range(p1, p2):
    """Calculates the 3D distance between two points."""
    dist_2d = geodesic(p1[:2], p2[:2]).meters
    dalt = p2[2] - p1[2]
    return np.sqrt(dist_2d**2 + (100000*dalt)**2)

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

# --- End of Graph Creation ---

# --- Simulated Annealing Implementation for Multi-UAV Routing ---

def get_available_nodes_at_edges():
    """Get available nodes at the edges of the grid for route generation"""
    available_nodes = [node for node in G.nodes() if G.nodes[node]['available']]
    
    # Get nodes at the left edge (x=0) and right edge (x=n_lat-1)
    left_edge = [node for node in available_nodes if node[0] == 0]
    right_edge = [node for node in available_nodes if node[0] == n_lat-1]
    
    return left_edge, right_edge

def generate_safe_uav_routes(num_routes=6):
    """Generate UAV routes using available nodes at grid edges"""
    left_edge, right_edge = get_available_nodes_at_edges()
    
    print(f"Available nodes at left edge: {len(left_edge)}")
    print(f"Available nodes at right edge: {len(right_edge)}")
    
    routes = []
    used_origins = set()
    used_destinations = set()
    
    for i in range(min(num_routes, min(len(left_edge), len(right_edge)))):
        # Find unused origin and destination
        available_origins = [node for node in left_edge if node not in used_origins]
        available_destinations = [node for node in right_edge if node not in used_destinations]
        
        if not available_origins or not available_destinations:
            print(f"Not enough available nodes for UAV {i+1}")
            break
            
        origin = available_origins[i % len(available_origins)]
        destination = available_destinations[i % len(available_destinations)]
        
        # Ensure we have a valid path
        try:
            nx.shortest_path(G, source=origin, target=destination, weight='weight')
            routes.append({'origin': origin, 'destination': destination})
            used_origins.add(origin)
            used_destinations.add(destination)
            print(f"UAV {i+1}: {origin} -> {destination}")
        except nx.NetworkXNoPath:
            print(f"No path found for UAV {i+1}: {origin} -> {destination}")
            continue
    
    return routes

# Generate UAV routes dynamically based on available nodes
UAV_ROUTES = generate_safe_uav_routes(10)  # Try to generate up to 40 routes

def validate_uav_routes(uav_routes):
    """
    Validate that all UAV routes have available origin and destination nodes
    and that paths can potentially exist between them.
    """
    valid_routes = []
    for i, route in enumerate(uav_routes):
        origin = route['origin']
        destination = route['destination']
        
        # Check if origin and destination exist and are available
        if origin not in G.nodes():
            print(f"Warning: UAV {i} origin {origin} not in graph. Skipping route.")
            continue
        if destination not in G.nodes():
            print(f"Warning: UAV {i} destination {destination} not in graph. Skipping route.")
            continue
        if not G.nodes[origin]['available']:
            print(f"Warning: UAV {i} origin {origin} not available. Skipping route.")
            continue
        if not G.nodes[destination]['available']:
            print(f"Warning: UAV {i} destination {destination} not available. Skipping route.")
            continue
            
        # Check if a path could potentially exist
        try:
            path = nx.shortest_path(G, source=origin, target=destination, weight='weight')
            print(f"UAV {i}: Valid route from {origin} to {destination}")
            valid_routes.append(route)
        except nx.NetworkXNoPath:
            print(f"Warning: UAV {i} - No path possible from {origin} to {destination}. Skipping route.")
            continue
    
    print(f"Validated {len(valid_routes)} out of {len(uav_routes)} routes")
    return valid_routes

# Validate routes before processing
UAV_ROUTES = validate_uav_routes(UAV_ROUTES)

def path_cost(paths, conflict_penalty=10000):
    """
    Calculates the total cost of a solution, which is the sum of path lengths
    plus a penalty for any conflicts.
    A conflict is when two UAVs are at the same node at the same time step.
    The time step is represented by the index in the path list.
    """
    total_length = 0
    total_conflicts = 0
    occupied_at_time = {}

    for path in paths:
        path_len = 0
        for i in range(len(path) - 1):
            u, v = path[i], path[i+1]
            try:
                # Add edge weight to total length
                path_len += G[u][v]['weight']
            except KeyError:
                # This should not happen if the path is valid, but good for robustness
                return float('inf') # A huge cost for an invalid path
            
            # Check for conflicts at the current time step (i)
            if (u, i) in occupied_at_time:
                total_conflicts += 1
            else:
                occupied_at_time[(u, i)] = True
        
        # Check the last node of the path
        if (path[-1], len(path) - 1) in occupied_at_time:
             total_conflicts += 1
        else:
             occupied_at_time[(path[-1], len(path) - 1)] = True

        total_length += path_len

    return total_length + (total_conflicts * conflict_penalty)

def generate_initial_solution(uav_routes):
    """
    Generates an initial solution using RRT algorithm for each UAV
    from origin to destination.
    """
    paths = []
    for route in uav_routes:
        try:
            # Use RRT algorithm to find a path
            path = rrt_path_planning(route['origin'], route['destination'])
            if path:
                paths.append(path)
            else:
                # Fallback to shortest path if RRT fails
                print(f"RRT failed for {route['origin']} -> {route['destination']}, using shortest path fallback.")
                path = nx.shortest_path(G, source=route['origin'], target=route['destination'], weight='weight')
                paths.append(path)
        except nx.NetworkXNoPath:
            print(f"No path found for {route['origin']} -> {route['destination']}. Returning empty solution.")
            return []
    
    print("Initial Solution for UAV Routes (using RRT):")
    for i, path in enumerate(paths):
        print(f"  UAV {i}: {path}")
    return paths

def rrt_path_planning(start, goal, max_iterations=1000, step_size=3):
    """
    RRT (Rapidly-exploring Random Tree) path planning algorithm.
    
    Args:
        start: Starting node
        goal: Goal node  
        max_iterations: Maximum number of RRT iterations
        step_size: Maximum distance for tree expansion (in grid steps)
    
    Returns:
        Path from start to goal, or None if no path found
    """
    # Check if start and goal are valid and available
    if not (is_valid_node(start) and is_valid_node(goal)):
        return None
        
    # If start and goal are the same
    if start == goal:
        return [start]
    
    # Initialize the tree with the start node
    tree = {start: None}  # node: parent mapping
    
    for iteration in range(max_iterations):
        # 1. Sample a random point in the space
        if random.random() < 0.1:  # 10% chance to sample the goal (goal bias)
            rand_node = goal
        else:
            rand_node = sample_random_node()
        
        # 2. Find the nearest node in the tree
        nearest_node = find_nearest_node(tree.keys(), rand_node)
        
        # 3. Extend the tree towards the random point
        new_node = extend_tree(nearest_node, rand_node, step_size)
        
        # 4. Check if the new node is valid (available and has valid connections)
        if new_node and is_valid_node(new_node) and is_path_clear(nearest_node, new_node):
            tree[new_node] = nearest_node
            
            # 5. Check if we've reached the goal
            if new_node == goal:
                # Goal reached directly
                return reconstruct_path(tree, start, goal)
            elif is_near_goal(new_node, goal):
                # If we're near the goal, try to connect directly
                if is_path_clear(new_node, goal):
                    tree[goal] = new_node  # Add goal to tree
                    return reconstruct_path(tree, start, goal)
    
    print(f"RRT failed to find path from {start} to {goal} after {max_iterations} iterations")
    return None

def sample_random_node():
    """Sample a random node from the available nodes in the graph."""
    available_nodes = [node for node in G.nodes() if G.nodes[node]['available']]
    if not available_nodes:
        # Fallback to any node if no available nodes (shouldn't happen with 80% availability)
        return random.choice(list(G.nodes()))
    return random.choice(available_nodes)

def find_nearest_node(tree_nodes, target_node):
    """Find the nearest node in the tree to the target node."""
    min_dist = float('inf')
    nearest = None
    
    for node in tree_nodes:
        dist = calculate_node_distance(node, target_node)
        if dist < min_dist:
            min_dist = dist
            nearest = node
    
    return nearest

def calculate_node_distance(node1, node2):
    """Calculate 3D grid distance between two nodes."""
    return np.sqrt(sum((a - b) ** 2 for a, b in zip(node1, node2)))

def extend_tree(from_node, to_node, step_size):
    """
    Extend the tree from from_node towards to_node by step_size.
    Returns the new node position.
    """
    # Calculate direction vector
    direction = np.array(to_node) - np.array(from_node)
    distance = np.linalg.norm(direction)
    
    if distance == 0:
        return from_node
    
    # Normalize and scale by step_size
    if distance <= step_size:
        new_node = to_node
    else:
        normalized_direction = direction / distance
        new_position = np.array(from_node) + normalized_direction * step_size
        # Round to nearest integer coordinates (since we're working with grid)
        new_node = tuple(np.round(new_position).astype(int))
    
    # Ensure the new node is within bounds
    if (0 <= new_node[0] < n_lat and 
        0 <= new_node[1] < n_lon and 
        0 <= new_node[2] < n_alt):
        return new_node
    
    return None

def is_valid_node(node):
    """Check if a node is valid (exists in graph and is available)."""
    return node in G.nodes() and G.nodes[node]['available']

def is_path_clear(node1, node2):
    """Check if there's a direct edge between two nodes."""
    return G.has_edge(node1, node2)

def is_near_goal(node, goal, threshold=1.5):
    """Check if a node is close enough to the goal."""
    return calculate_node_distance(node, goal) <= threshold

def reconstruct_path(tree, start, goal):
    """Reconstruct the path from start to goal using the tree."""
    # Check if goal is in the tree
    if goal not in tree:
        print(f"Error: Goal {goal} not found in tree during path reconstruction")
        return None
        
    path = []
    current = goal
    
    while current is not None:
        path.append(current)
        current = tree[current]
    
    path.reverse()
    return path

def get_neighbor(current_solution):
    """
    Generates a new candidate solution by making a small random change.
    The change involves rerouting a segment of one of the UAVs' paths.
    """
    # Create a deep copy to avoid modifying the original solution
    new_solution = copy.deepcopy(current_solution)

    # Randomly select one UAV to modify
    uav_index = random.randint(0, len(new_solution) - 1)
    path_to_modify = new_solution[uav_index]
    
    # Check if the path is long enough to have a segment to modify
    if len(path_to_modify) < 3:
        return new_solution

    # Randomly select a start and end point for the segment to be rerouted
    start_idx = random.randint(0, len(path_to_modify) - 2)
    end_idx = random.randint(start_idx + 1, len(path_to_modify) - 1)

    start_node = path_to_modify[start_idx]
    end_node = path_to_modify[end_idx]

    try:
        # Find a new shortest path for the selected segment
        new_segment = nx.shortest_path(G, source=start_node, target=end_node, weight='weight')
        
        # If the new segment is just a straight line (no change), find another segment
        if len(new_segment) < 2:
            return get_neighbor(current_solution) # Try again with another neighbor

        # Replace the old segment with the new one
        new_path = path_to_modify[:start_idx] + new_segment + path_to_modify[end_idx+1:]
        new_solution[uav_index] = new_path
        
    except nx.NetworkXNoPath:
        # If no path exists for the segment, return the original solution
        return current_solution
    
    return new_solution

def simulated_annealing(uav_routes, initial_temp=1000, cooling_rate=0.9, min_temp=0.001):
    """
    The main Simulated Annealing algorithm to find the best conflict-free paths.
    """
    # 1. Generate an initial solution
    current_paths = generate_initial_solution(uav_routes)
    if not current_paths:
        print("Error: no initial solution found.")
        return None # No solution could be found

    best_paths = current_paths
    current_cost = path_cost(current_paths)
    best_cost = current_cost
    temperature = initial_temp

    print("Simulated Annealing started...")

    while temperature > min_temp:
        # 2. Get a neighboring solution
        new_paths = get_neighbor(current_paths)
        new_cost = path_cost(new_paths)

        # 3. Decide whether to accept the new solution
        delta = new_cost - current_cost
        if delta < 0 or random.uniform(0, 1) < math.exp(-delta / temperature):
            current_paths = new_paths
            current_cost = new_cost
        
        # 4. Update the best solution found so far
        if current_cost < best_cost:
            best_paths = current_paths
            best_cost = current_cost
        
        # 5. Cool the temperature
        temperature *= cooling_rate
        
    print(f"Simulated Annealing finished. Final cost: {best_cost:.2f}")
    print("Optimized routes for UAVS are:")
    for i, path in enumerate(best_paths):
        print(f"  UAV {i}: {path} with cost {path_cost([path]):.2f}")
    return best_paths

# Run the Simulated Annealing algorithm
optimized_paths = simulated_annealing(UAV_ROUTES)

if optimized_paths is None:
    print("Could not find a valid set of paths.")
else:
    # --- Cost Comparison: RRT vs Simulated Annealing ---
    print("\n" + "="*60)
    print("COST COMPARISON: RRT vs SIMULATED ANNEALING")
    print("="*60)
    
    # Generate initial RRT solution for comparison
    initial_rrt_paths = generate_initial_solution(UAV_ROUTES)
    
    if initial_rrt_paths:
        # Calculate costs for both solutions
        rrt_cost = path_cost(initial_rrt_paths)
        sa_cost = path_cost(optimized_paths)
        
        # Calculate cost components separately for detailed analysis
        def analyze_cost_components(paths, solution_name):
            """Analyze and display cost components (path length vs conflicts)"""
            total_length = 0
            total_conflicts = 0
            occupied_at_time = {}
            
            # Calculate path lengths and conflicts separately
            for path in paths:
                path_len = 0
                for i in range(len(path) - 1):
                    u, v = path[i], path[i+1]
                    try:
                        path_len += G[u][v]['weight']
                    except KeyError:
                        pass
                    
                    # Check for conflicts at current time step
                    if (u, i) in occupied_at_time:
                        total_conflicts += 1
                    else:
                        occupied_at_time[(u, i)] = True
                
                # Check the last node of the path
                if (path[-1], len(path) - 1) in occupied_at_time:
                    total_conflicts += 1
                else:
                    occupied_at_time[(path[-1], len(path) - 1)] = True
                
                total_length += path_len
            
            conflict_penalty_cost = total_conflicts * 10000
            total_cost = total_length + conflict_penalty_cost
            
            print(f"\n{solution_name} Solution Analysis:")
            print(f"  • Path Length Cost: {total_length:.2f} meters")
            print(f"  • Number of Conflicts: {total_conflicts}")
            print(f"  • Conflict Penalty Cost: {conflict_penalty_cost:.2f}")
            print(f"  • TOTAL COST: {total_cost:.2f}")
            
            return total_cost, total_length, total_conflicts
        
        # Analyze both solutions
        rrt_total, rrt_length, rrt_conflicts = analyze_cost_components(initial_rrt_paths, "RRT")
        sa_total, sa_length, sa_conflicts = analyze_cost_components(optimized_paths, "SIMULATED ANNEALING")
        
        # Calculate improvements
        cost_improvement = rrt_total - sa_total
        length_improvement = rrt_length - sa_length
        conflict_reduction = rrt_conflicts - sa_conflicts
        
        print(f"\n" + "-"*40)
        print("IMPROVEMENT SUMMARY:")
        print("-"*40)
        print(f"Total Cost Reduction: {cost_improvement:.2f} ({((cost_improvement/rrt_total)*100):.1f}% improvement)")
        print(f"Path Length Change: {length_improvement:.2f} meters ({((length_improvement/rrt_length)*100):.1f}% change)")
        print(f"Conflicts Eliminated: {conflict_reduction} conflicts")
        
        if sa_conflicts == 0:
            print("✅ SUCCESS: All conflicts eliminated by Simulated Annealing!")
        else:
            print(f"⚠️  WARNING: {sa_conflicts} conflicts still remain")
        
        print(f"\nNumber of UAVs: {len(optimized_paths)}")
        print(f"Average path length per UAV:")
        print(f"  • RRT: {rrt_length/len(initial_rrt_paths):.2f} meters")
        print(f"  • SA:  {sa_length/len(optimized_paths):.2f} meters")
    
    else:
        print("Could not generate initial RRT solution for comparison.")
    
    print("="*60)
    A = nx.adjacency_matrix(G)
    A.toarray()

    # --- Visualization with Plotly ---
    
    # Create lists for nodes and edges
    node_x = [pos[node][1] for node in G.nodes()]
    node_y = [pos[node][0] for node in G.nodes()]
    node_z = [pos[node][2] for node in G.nodes()]
    node_colors = ['green' if G.nodes[node]['available'] else 'red' for node in G.nodes()]
    
    edge_x, edge_y, edge_z = [], [], []
    for edge in G.edges():
        x0, y0, z0 = pos[edge[0]][1], pos[edge[0]][0], pos[edge[0]][2]
        x1, y1, z1 = pos[edge[1]][1], pos[edge[1]][0], pos[edge[1]][2]
        edge_x.extend([x0, x1, None])
        edge_y.extend([y0, y1, None])
        edge_z.extend([z0, z1, None])
    
    # Create the main edge trace (lines)
    edge_trace = go.Scatter3d(
        x=edge_x, y=edge_y, z=edge_z,
        line=dict(width=2, color='gray'),
        hoverinfo='none',
        mode='lines',
        name='Graph Edges'
    )
    
    # Create the node trace
    node_trace = go.Scatter3d(
        x=node_x, y=node_y, z=node_z,
        mode='markers',
        hoverinfo='text',
        marker=dict(size=5, color=node_colors),
        text=[f"Lat: {pos[n][0]:.4f}<br>Lon: {pos[n][1]:.4f}<br>Alt: {pos[n][2]:.0f}" for n in G.nodes()],
        textposition="top center",
        name='Nodes'
    )
    
    # Create traces for each UAV's path
    # Generate colors dynamically based on the number of UAVs
    def generate_colors(num_colors):
        """Generate a list of distinct colors for the given number of UAVs"""
        if num_colors <= 20:
            # Use predefined colors for small numbers (all valid CSS color names)
            base_colors = ['red', 'blue', 'orange', 'purple', 'green', 'cyan', 'magenta', 'yellow', 'brown', 'pink', 
                          'lightblue', 'lightgreen', 'gold', 'coral', 'violet', 'darkred', 'darkblue', 'darkgreen', 'goldenrod', 'darkcyan']
            return base_colors[:num_colors]
        else:
            # Generate colors using HSV color space for larger numbers
            import colorsys
            colors = []
            for i in range(num_colors):
                hue = i / num_colors
                saturation = 0.8 + (i % 3) * 0.1  # Vary saturation slightly
                value = 0.8 + (i % 2) * 0.2       # Vary brightness slightly
                rgb = colorsys.hsv_to_rgb(hue, saturation, value)
                # Convert to hex color
                hex_color = '#{:02x}{:02x}{:02x}'.format(int(rgb[0]*255), int(rgb[1]*255), int(rgb[2]*255))
                colors.append(hex_color)
            return colors
    
    uav_colors = generate_colors(len(optimized_paths))
    uav_path_traces = []
    for i, path in enumerate(optimized_paths):
        path_x = [pos[node][1] for node in path]
        path_y = [pos[node][0] for node in path]
        path_z = [pos[node][2] for node in path]
    
        uav_path_trace = go.Scatter3d(
            x=path_x, y=path_y, z=path_z,
            mode='lines+markers',
            line=dict(width=6, color=uav_colors[i]),
            marker=dict(size=4, color=uav_colors[i]),
            name=f'UAV {i+1} Path'
        )
        uav_path_traces.append(uav_path_trace)
    
    # Combine all traces for the final plot
    data = [edge_trace, node_trace] + uav_path_traces
    
    # Customize the layout
    fig = go.Figure(data=data)
    fig.update_layout(
        title='Simulated Annealing - Conflict-Free UAV Routes',
        scene=dict(
            xaxis_title='Longitude',
            yaxis_title='Latitude',
            zaxis_title='Altitude',
        )
    )
    
    # Display the interactive plot
    fig.show()

