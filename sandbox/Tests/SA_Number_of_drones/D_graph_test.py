#!/usr/bin/env python3
"""
3D Lattice Graph Generator and Visualizer

This script creates a fully connected 3D lattice graph using geographic coordinates
and visualizes it using Plotly. The graph represents a network of nodes in 3D space
where each node has latitude, longitude, and altitude coordinates.

Author: Converted from Jupyter notebook
Date: September 18, 2025
"""

# Import required libraries
import networkx as nx          # NetworkX library for graph operations (DiGraph type)
from geopy.distance import geodesic  # Geodesic distance calculation for geographic coordinates
import numpy as np             # NumPy for numerical operations and array handling
import plotly.graph_objects as go    # Plotly for interactive 3D visualization


def create_3d_lattice_graph(n_lat=10, n_lon=10, n_alt=3, verbose=True):
    """
    Create a 3D Lattice Graph with full connectivity between neighboring nodes.
    
    This function generates a directed graph where nodes are positioned in a 3D lattice
    structure using geographic coordinates (latitude, longitude, altitude).
    
    Args:
        n_lat: int - Number of latitude grid points (default: 10)
        n_lon: int - Number of longitude grid points (default: 10) 
        n_alt: int - Number of altitude grid points (default: 3)
        verbose: bool - Whether to print progress messages (default: True)
    
    Returns:
        nx.DiGraph: A NetworkX directed graph with nodes positioned in 3D space
    """
    if verbose:
        print(f"Creating 3D Lattice Graph with dimensions {n_lat}x{n_lon}x{n_alt}...")
    
    # Define the bounding box for geographic coordinates (float values in degrees)
    # These coordinates represent a small area around LaGuardia Airport (LGA)
    min_lat, max_lat = 40.6042, 40.6125  # Latitude range in decimal degrees (float)
    min_lon, max_lon = -73.9458, -73.9292  # Longitude range in decimal degrees (float)
    min_alt, max_alt = 0.001, 0.0015      # Altitude range in relative units (float)

    # Generate grid points using NumPy's linspace function
    # linspace creates evenly spaced arrays of float values
    lats = np.linspace(min_lat, max_lat, n_lat)  # Array of 10 latitude values (float64 array)
    lons = np.linspace(min_lon, max_lon, n_lon)  # Array of 10 longitude values (float64 array) 
    alts = np.linspace(min_alt, max_alt, n_alt)  # Array of 3 altitude values (float64 array)

    # Create node identifiers as tuples and their geographic positions
    # nodes: list of tuples (i,j,k) representing grid indices (list of tuple(int,int,int))
    nodes = [(i, j, k) for i in range(n_lat) for j in range(n_lon) for k in range(n_alt)]
    
    # pos: dictionary mapping node tuples to geographic coordinates (dict[tuple, tuple(float,float,float)])
    pos = {(i, j, k): (lats[i], lons[j], alts[k]) for i in range(n_lat) for j in range(n_lon) for k in range(n_alt)}

    # Randomly assign availability to each node (80% probability of being available)
    # availability: dictionary mapping node tuples to boolean values (dict[tuple, bool])
    availability = {node: np.random.random() < 0.8 for node in nodes}

    # Create an empty directed graph using NetworkX
    G = nx.DiGraph()  # NetworkX directed graph object

    # Add all nodes to the graph
    G.add_nodes_from(nodes)

    # Set node attributes for geographic coordinates and availability
    # These attributes are stored as dictionaries within the graph structure
    nx.set_node_attributes(G, pos, 'pos')           # Position attribute (dict)
    nx.set_node_attributes(G, availability, 'available')  # Availability attribute (dict)

    # Define a function to calculate the slant range (3D distance) between two points
    def slant_range(p1, p2):
        """
        Calculate 3D distance between two geographic points.
        
        Args:
            p1: tuple(float, float, float) - First point (lat, lon, alt)
            p2: tuple(float, float, float) - Second point (lat, lon, alt)
            
        Returns:
            float: 3D distance in meters
        """
        # Calculate 2D geodesic distance using geopy (float in meters)
        dist_2d = geodesic(p1[:2], p2[:2]).meters
        
        # Calculate altitude difference (float in relative units)
        dalt = p2[2] - p1[2]
        
        # Return 3D distance using Pythagorean theorem (float in meters)
        # Altitude is scaled by 100000 to convert to meters
        return np.sqrt(dist_2d**2 + (100000*dalt)**2)

    # Add edges to connect each node to all its neighbors, including diagonals
    # This creates a fully connected lattice where each node connects to its 26 neighbors
    for i in range(n_lat):      # Loop through latitude indices (int)
        for j in range(n_lon):  # Loop through longitude indices (int)
            for k in range(n_alt):  # Loop through altitude indices (int)
                # Skip unavailable source nodes
                if G.nodes[(i, j, k)]['available'] == False:
                    continue
                    
                # Check all 26 possible neighbor directions (3x3x3 - center)
                for di in [-1, 0, 1]:   # Latitude direction offset (int)
                    for dj in [-1, 0, 1]:   # Longitude direction offset (int)
                        for dk in [-1, 0, 1]:   # Altitude direction offset (int)
                            # Skip the center node (no self-loops)
                            if di == dj == dk == 0:
                                continue
                            
                            # Calculate neighbor coordinates (int values)
                            ni, nj, nk = i + di, j + dj, k + dk
                            
                            # Check if neighbor is within grid bounds
                            if 0 <= ni < n_lat and 0 <= nj < n_lon and 0 <= nk < n_alt:
                                # Skip unavailable target nodes
                                if G.nodes[(ni, nj, nk)]['available'] == False:
                                    continue
                                
                                # Define source and target nodes (tuple identifiers)
                                u, v = (i, j, k), (ni, nj, nk)
                                
                                # Calculate distance between nodes (float in meters)
                                dist = slant_range(pos[u], pos[v])
                                
                                # Add directed edge with weight as distance
                                G.add_edge(u, v, weight=dist)

    if verbose:
        print(f"Graph created with {G.number_of_nodes()} nodes and {G.number_of_edges()} edges")
    return G

def visualize_graph(G, pos):
    """
    Create an interactive 3D visualization of the graph using Plotly.
    
    Args:
        G: nx.DiGraph - The NetworkX directed graph to visualize
        pos: dict - Dictionary mapping nodes to their 3D coordinates
    """
    print("Creating 3D visualization...")
    
    # Create lists for node coordinates and colors
    # node_x, node_y, node_z: lists of float values for 3D coordinates
    node_x = [pos[node][1] for node in G.nodes()]  # Longitude coordinates (list of float)
    node_y = [pos[node][0] for node in G.nodes()]  # Latitude coordinates (list of float)
    node_z = [pos[node][2] for node in G.nodes()]  # Altitude coordinates (list of float)
    
    # node_colors: list of color strings based on availability (list of str)
    node_colors = ['green' if G.nodes[node]['available'] else 'red' for node in G.nodes()]

    # Create lists for edges and arrow visualization
    # These lists will store coordinates for drawing edges and directional arrows
    edge_x, edge_y, edge_z = [], [], []  # Edge line coordinates (lists of float)
    arrow_x, arrow_y, arrow_z = [], [], []  # Arrow position coordinates (lists of float)
    arrow_u, arrow_v, arrow_w = [], [], []  # Arrow direction vectors (lists of float)

    # Process each edge to create visualization data
    for edge in G.edges():  # edge: tuple of two node identifiers
        # Extract 3D coordinates for source and target nodes (float values)
        x0, y0, z0 = pos[edge[0]][1], pos[edge[0]][0], pos[edge[0]][2]  # Source coordinates
        x1, y1, z1 = pos[edge[1]][1], pos[edge[1]][0], pos[edge[1]][2]  # Target coordinates

        # Add coordinates for the edge line (None creates line breaks in Plotly)
        edge_x.extend([x0, x1, None])
        edge_y.extend([y0, y1, None])
        edge_z.extend([z0, z1, None])

        # Calculate direction vector for the arrow (float values)
        u_vec = x1 - x0  # X-direction component (float)
        v_vec = y1 - y0  # Y-direction component (float)
        w_vec = z1 - z0  # Z-direction component (float)

        # Normalize the vector to control arrow size and shape
        magnitude = np.sqrt(u_vec**2 + v_vec**2 + w_vec**2)  # Vector magnitude (float)

        if magnitude > 0:  # Avoid division by zero
            # Calculate normalized direction vectors (float values between -1 and 1)
            u_norm = u_vec / magnitude
            v_norm = v_vec / magnitude
            w_norm = w_vec / magnitude

            # Place the arrow slightly before the end of the line (95% along the edge)
            arrow_pos_ratio = 0.95  # Position ratio (float)
            arrow_x.append(x0 + u_vec * arrow_pos_ratio)
            arrow_y.append(y0 + v_vec * arrow_pos_ratio)
            arrow_z.append(z0 + w_vec * arrow_pos_ratio)

            # Append normalized vectors for consistent arrow shape
            arrow_u.append(u_norm)
            arrow_v.append(v_norm)
            arrow_w.append(w_norm)

    # Create the main edge trace (lines) using Plotly Scatter3d
    edge_trace = go.Scatter3d(
        x=edge_x, y=edge_y, z=edge_z,  # 3D coordinates for lines
        line=dict(width=2, color='gray'),  # Line styling (int width, str color)
        hoverinfo='none',  # Disable hover information
        mode='lines'  # Display as lines only
    )

    # Create the node trace using Plotly Scatter3d
    node_trace = go.Scatter3d(
        x=node_x, y=node_y, z=node_z,  # 3D coordinates for nodes
        mode='markers',  # Display as markers (points)
        hoverinfo='text',  # Show text on hover
        marker=dict(size=5, color=node_colors),  # Marker styling (int size, list of colors)
        text=[f"Lat: {pos[n][0]:.4f}<br>Lon: {pos[n][1]:.4f}<br>Alt: {pos[n][2]:.6f}" for n in G.nodes()],  # Hover text
        textposition="top center"  # Text position relative to markers
    )

    # Create the cone trace for the directional arrows
    arrow_trace = go.Cone(
        x=arrow_x, y=arrow_y, z=arrow_z,  # Arrow positions (lists of float)
        u=arrow_u, v=arrow_v, w=arrow_w,  # Arrow directions (lists of float)
        sizeref=0.1,  # Arrow size reference (float)
        sizemode="absolute",  # Size mode for arrows
        anchor="tip",  # Anchor point for arrows
        colorscale=[[0, 'red'], [1, 'red']],  # Color scale (list of color mappings)
        showscale=False  # Hide color scale bar
    )

    # Create the figure and add all traces
    fig = go.Figure(data=[edge_trace, node_trace, arrow_trace])

    # Customize the layout with titles and axis labels
    fig.update_layout(
        title='Interactive 3D Directed Geographic Lattice Graph',  # Main title (str)
        scene=dict(
            xaxis_title='Longitude',  # X-axis label (str)
            yaxis_title='Latitude',   # Y-axis label (str)
            zaxis_title='Altitude',   # Z-axis label (str)
        )
    )

    # Display the interactive plot
    fig.show()
    print("Visualization complete!")

def performance_test_graph_creation(start_nx=10, end_nx=500, step_nx=10, n_alt=3, max_time_limit=120.0):
    """
    Conduct performance testing of graph creation across different grid dimensions.
    
    This function measures the time required to create 3D lattice graphs with varying
    grid sizes from start_nx to end_nx with specified step size.
    
    Args:
        start_nx: int - Starting value for nx (n_lat = n_lon = nx) (default: 10)
        end_nx: int - Ending value for nx (inclusive) (default: 500)
        step_nx: int - Step size for nx increment (default: 10)
        n_alt: int - Fixed altitude dimension (default: 3)
        max_time_limit: float - Maximum time in seconds for a single test (default: 120.0)
        
    Returns:
        tuple: (nx_values, times) - Lists of grid sizes and corresponding creation times
    """
    import time  # Import time module for performance measurement
    import gc    # Import garbage collector for memory management
    
    print("=== Performance Test: Graph Creation Time vs Grid Size ===")
    print(f"Testing nx from {start_nx} to {end_nx} with step {step_nx}")
    print(f"Fixed n_alt = {n_alt}")
    print(f"Max time limit per test: {max_time_limit} seconds")
    print("-" * 60)
    
    # Initialize lists to store test results
    nx_values = []  # List of grid size values (list of int)
    times = []      # List of corresponding creation times in seconds (list of float)
    
    # Iterate through different grid sizes
    for nx in range(start_nx, end_nx + 1, step_nx):  # nx: current grid size (int)
        total_nodes = nx * nx * n_alt  # Calculate total nodes (int)
        theoretical_edges = total_nodes * 26  # Theoretical maximum edges (int)
        
        print(f"Testing nx = {nx} (Grid: {nx}x{nx}x{n_alt})")
        print(f"  Expected nodes: {total_nodes}")
        print(f"  Theoretical max edges: {theoretical_edges}")
        
        try:
            # Record start time using high-resolution timer (float timestamp)
            start_time = time.perf_counter()
            
            # Create the graph with current dimensions (suppress verbose output)
            G = create_3d_lattice_graph(n_lat=nx, n_lon=nx, n_alt=n_alt, verbose=False)
            
            # Record end time and calculate duration (float in seconds)
            end_time = time.perf_counter()
            creation_time = end_time - start_time
            
            # Check if creation time exceeded the limit
            if creation_time > max_time_limit:
                print(f"  WARNING: Creation time ({creation_time:.4f}s) exceeded limit ({max_time_limit}s)")
                print(f"  Stopping performance test to avoid excessive computation times.")
                # Clean up and break
                del G
                gc.collect()  # Force garbage collection
                break
            
            # Store results in lists
            nx_values.append(nx)
            times.append(creation_time)
            
            # Display results for current test
            print(f"  Actual nodes: {G.number_of_nodes()}")
            print(f"  Actual edges: {G.number_of_edges()}")
            print(f"  Creation time: {creation_time:.4f} seconds")
            print(f"  Memory cleanup... ", end="")
            
            # Clean up graph object to free memory
            del G
            gc.collect()  # Force garbage collection to free memory
            print("Done!")
            print()
            
        except Exception as e:
            print(f"  ERROR: Failed to create graph for nx={nx}")
            print(f"  Error details: {str(e)}")
            print(f"  Stopping performance test due to error.")
            break
    
    print("Performance test completed!")
    print(f"Successfully tested {len(nx_values)} different grid sizes.")
    return nx_values, times

def plot_performance_results(nx_values, times):
    """
    Create visualization plots showing the relationship between grid size and creation time.
    
    Args:
        nx_values: list of int - Grid size values tested
        times: list of float - Corresponding creation times in seconds
    """
    import matplotlib.pyplot as plt  # Import matplotlib for plotting
    
    print("Creating performance analysis plots...")
    
    # Create figure with subplots for different visualizations
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))  # Figure with 2 subplots
    
    # Plot 1: Linear scale - Time vs Grid Size
    ax1.plot(nx_values, times, 'bo-', linewidth=2, markersize=6)  # Blue line with circles
    ax1.set_xlabel('Grid Size (nx)', fontsize=12)  # X-axis label (str)
    ax1.set_ylabel('Creation Time (seconds)', fontsize=12)  # Y-axis label (str)
    ax1.set_title('Graph Creation Time vs Grid Size (Linear Scale)', fontsize=14)  # Plot title (str)
    ax1.grid(True, alpha=0.3)  # Add grid lines with transparency
    
    # Add annotations for key data points
    for i in range(0, len(nx_values), max(1, len(nx_values)//10)):  # Sample every 10th point
        ax1.annotate(f'({nx_values[i]}, {times[i]:.2f}s)', 
                    (nx_values[i], times[i]), 
                    xytext=(5, 5), textcoords='offset points',
                    fontsize=8, alpha=0.7)
    
    # Plot 2: Log-log scale to analyze complexity
    ax2.loglog(nx_values, times, 'ro-', linewidth=2, markersize=6)  # Red line with circles on log scale
    ax2.set_xlabel('Grid Size (nx) - Log Scale', fontsize=12)
    ax2.set_ylabel('Creation Time (seconds) - Log Scale', fontsize=12)
    ax2.set_title('Graph Creation Time vs Grid Size (Log-Log Scale)', fontsize=14)
    ax2.grid(True, alpha=0.3)
    
    # Fit and display polynomial trend line on log-log plot
    log_nx = np.log(nx_values)  # Natural logarithm of grid sizes (array of float)
    log_times = np.log(times)   # Natural logarithm of times (array of float)
    
    # Linear regression in log space to find complexity order
    coeffs = np.polyfit(log_nx, log_times, 1)  # Fit line: log(time) = a*log(nx) + b
    slope = coeffs[0]  # Slope represents algorithmic complexity order (float)
    
    # Create trend line for visualization
    trend_line = np.exp(np.polyval(coeffs, log_nx))  # Convert back to linear space
    ax2.plot(nx_values, trend_line, 'g--', linewidth=2, alpha=0.7, 
             label=f'Trend: O(n^{slope:.2f})')  # Green dashed trend line
    ax2.legend()
    
    # Adjust layout and display
    plt.tight_layout()
    plt.show()
    
    # Print statistical analysis
    print("\n=== Performance Analysis Results ===")
    print(f"Grid sizes tested: {min(nx_values)} to {max(nx_values)} (step: {nx_values[1]-nx_values[0]})")
    print(f"Minimum creation time: {min(times):.4f} seconds (nx = {nx_values[times.index(min(times))]})")
    print(f"Maximum creation time: {max(times):.4f} seconds (nx = {nx_values[times.index(max(times))]})")
    print(f"Average creation time: {np.mean(times):.4f} seconds")
    print(f"Estimated algorithmic complexity: O(n^{slope:.2f})")
    
    # Calculate total nodes and edges for largest graph
    max_nx = max(nx_values)  # Largest grid size tested (int)
    max_nodes = max_nx * max_nx * 3  # Total nodes in largest graph (int)
    max_edges_theoretical = max_nodes * 26  # Theoretical maximum edges (int, 26 neighbors per node)
    
    print(f"\nLargest graph tested:")
    print(f"  Grid size: {max_nx}x{max_nx}x3")
    print(f"  Total nodes: {max_nodes}")
    print(f"  Theoretical max edges: {max_edges_theoretical}")

def run_performance_experiment():
    """
    Execute the complete performance testing experiment.
    
    This function runs the performance test, collects timing data, and creates
    visualizations showing the relationship between grid size and creation time.
    """
    print("Starting comprehensive performance experiment...")
    
    # Run performance test with specified parameters
    # Note: Reduced end_nx to 200 for more manageable testing
    nx_values, times = performance_test_graph_creation(
        start_nx=10,    # Start with small grid (int)
        end_nx=200,     # End with medium-large grid (int) 
        step_nx=10,     # Increment by 10 each time (int)
        n_alt=3,        # Fixed altitude dimension (int)
        max_time_limit=120.0  # Limit individual tests to 60 seconds (float)
    )
    
    # Create performance visualization plots
    plot_performance_results(nx_values, times)
    
    # Save results to file for future analysis
    import csv  # Import CSV module for data export
    
    results_filename = 'graph_performance_results.csv'  # Output filename (str)
    print(f"\nSaving results to {results_filename}...")
    
    with open(results_filename, 'w', newline='') as csvfile:  # Open CSV file for writing
        writer = csv.writer(csvfile)  # Create CSV writer object
        writer.writerow(['Grid_Size_nx', 'Creation_Time_seconds'])  # Header row
        
        # Write data rows
        for nx, time_val in zip(nx_values, times):  # Iterate through paired data
            writer.writerow([nx, time_val])  # Write nx and time as CSV row
    
    print(f"Results saved! Total tests conducted: {len(nx_values)}")
    return nx_values, times


# Execute the performance experiment when script is run directly
if __name__ == "__main__":
    print("=== 3D Lattice Graph Performance Test ===")
    print("Running comprehensive performance experiment...")
    run_performance_experiment()
