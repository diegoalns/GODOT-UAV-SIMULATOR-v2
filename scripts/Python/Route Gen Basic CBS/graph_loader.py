"""
Graph Loader Module

This module handles the loading and initialization of the regular lattice graph
from a pickle file. It provides a clean interface for loading the graph and
extracting it from various data structures (dictionaries, direct graph objects).

The module also performs comprehensive validation to ensure the graph is properly
structured for use with NetworkX and the pathfinding algorithms.

Functions:
    load_graph_from_pickle(pickle_path): Loads and extracts the NetworkX graph
                                         from a pickle file with full validation.
"""

import sys
from pathlib import Path
import pickle
import networkx as nx


def load_graph_from_pickle(pickle_path=None):
    """
    Loads a NetworkX graph from a pickle file with comprehensive validation.
    
    This function handles different pickle file formats:
    - Direct NetworkX graph objects (Graph, DiGraph, MultiGraph, MultiDiGraph)
    - Dictionary containing a graph under 'graph' or 'G' keys
    - Dictionary containing a graph as any value
    
    The function performs the following validations:
    1. Checks if the loaded object is a valid NetworkX graph type
    2. Verifies the graph is not empty
    3. Validates that nodes have required 'pos' attribute (3D coordinates)
    4. Checks that edges have 'weight' attribute for pathfinding
    5. Verifies graph connectivity for routing purposes
    
    Args:
        pickle_path (Path or str, optional): Path to the pickle file. If None,
                                            defaults to 'regular_lattice_graph.pkl'
                                            in the same directory as this module.
                                            Type: Path object or string
    
    Returns:
        networkx.Graph: The loaded and validated NetworkX graph object.
                       Can be Graph, DiGraph, MultiGraph, or MultiDiGraph type.
    
    Raises:
        FileNotFoundError: If the pickle file doesn't exist at the specified path.
        ValueError: If no NetworkX graph is found in the pickle file, or if
                   validation fails (empty graph, missing attributes, etc.).
    """
    # Set default path if not provided
    # pickle_path: Path object or None
    if pickle_path is None:
        # Get the directory containing this module file using __file__ attribute
        # __file__ returns the path to the current Python module
        pickle_path = Path(__file__).parent / 'regular_lattice_graph.pkl'
    else:
        # Convert string to Path object if necessary for consistent path handling
        pickle_path = Path(pickle_path)
    
    # Load the pickle file using binary read mode ('rb')
    print("\n" + "="*80)
    print("│ 📊 LOADING AIRSPACE GRAPH")
    print("="*80)
    print(f"│ File Path: {pickle_path}")
    
    with open(pickle_path, 'rb') as f:
        # pickle.load() deserializes the binary data back into Python objects
        loaded_data = pickle.load(f)
    
    print(f"│ Loaded Type: {type(loaded_data).__name__}")
    
    # Extract the NetworkX graph from the loaded data
    # airspace_graph: NetworkX graph object (Graph, DiGraph, MultiGraph, or MultiDiGraph)
    airspace_graph = None
    
    # If it's a dictionary, extract the NetworkX graph
    # isinstance() checks if the object is an instance of dict type
    if isinstance(loaded_data, dict):
        print(f"│ Dictionary Keys: {', '.join(loaded_data.keys())}")
        
        # Look for the graph in common dictionary keys
        # Standard key names used in many graph pickle files
        if 'graph' in loaded_data:
            airspace_graph = loaded_data['graph']
            print("│ Graph Location: Found in 'graph' key")
        elif 'G' in loaded_data:
            airspace_graph = loaded_data['G']
            print("│ Graph Location: Found in 'G' key")
        else:
            # Search through all dictionary values for a NetworkX graph
            # Iterate over key-value pairs using items() method
            for key, value in loaded_data.items():
                # Check if it's a NetworkX graph by checking for number_of_nodes method
                # hasattr() checks if the object has the specified attribute/method
                if hasattr(value, 'number_of_nodes'):
                    airspace_graph = value
                    print(f"│ Graph Location: Found in '{key}' key")
                    break
            
            # If no graph was found in any dictionary value, raise an error
            if airspace_graph is None:
                print("├" + "─"*78 + "┤")
                print("│ ❌ ERROR: No NetworkX graph found in dictionary!")
                print("└" + "─"*78 + "┘")
                raise ValueError("No NetworkX graph found in dictionary!")
    else:
        # Assume it's directly a NetworkX graph object
        airspace_graph = loaded_data
        print("│ Graph Location: Direct graph object")
    
    # ============================================================================
    # VALIDATION 1: Check if the loaded object is a valid NetworkX graph type
    # ============================================================================
    # NetworkX supports 4 main graph types:
    # - Graph: Undirected graph
    # - DiGraph: Directed graph
    # - MultiGraph: Undirected graph with multiple edges between nodes
    # - MultiDiGraph: Directed graph with multiple edges between nodes
    if not isinstance(airspace_graph, (nx.Graph, nx.DiGraph, nx.MultiGraph, nx.MultiDiGraph)):
        print("├" + "─"*78 + "┤")
        print(f"│ ❌ ERROR: Loaded object is not a NetworkX graph. Type: {type(airspace_graph).__name__}")
        print("└" + "─"*78 + "┘")
        raise ValueError(f"Loaded object is not a NetworkX graph. Type: {type(airspace_graph)}")
    
    # Get graph statistics for reporting and validation
    # number_of_nodes() returns integer count of nodes in the graph
    # number_of_edges() returns integer count of edges in the graph
    num_nodes = airspace_graph.number_of_nodes()
    num_edges = airspace_graph.number_of_edges()
    
    print("├" + "─"*78 + "┤")
    print(f"│ {'Graph Statistics':<38} │ {'Value':<38} │")
    print("├" + "─"*38 + "┼" + "─"*38 + "┤")
    print(f"│ {'  Type':<38} │ {type(airspace_graph).__name__:<38} │")
    print(f"│ {'  Total Nodes':<38} │ {num_nodes:<38} │")
    print(f"│ {'  Total Edges':<38} │ {num_edges:<38} │")
    
    # ============================================================================
    # VALIDATION 2: Check if the graph is not empty
    # ============================================================================
    if num_nodes == 0:
        print(f"│ {'  Validation':<38} │ ❌ Graph is empty (0 nodes){'':<15} │")
        print("└" + "─"*78 + "┘")
        raise ValueError("Loaded graph is empty (0 nodes). Cannot perform pathfinding on an empty graph.")
    
    if num_edges == 0:
        print(f"│ {'  Validation':<38} │ ❌ No edges (0 edges){'':<19} │")
        print("└" + "─"*78 + "┘")
        raise ValueError("Loaded graph has no edges (0 edges). Cannot find paths in a graph with no connections.")
    
    print(f"│ {'  Validation':<38} │ ✓ Graph has nodes and edges{'':<12} │")
    
    # ============================================================================
    # VALIDATION 3: Validate and create 'pos' attribute for nodes
    # ============================================================================
    # The 'pos' attribute should contain (lat, lon, alt) coordinates as a tuple
    # Some graphs may have individual 'lat', 'lon', 'altitude' attributes instead
    # We'll check for both formats and create 'pos' if it doesn't exist
    
    print("├" + "─"*78 + "┤")
    print(f"│ {'Node Position Attributes':<78} │")
    print("├" + "─"*78 + "┤")
    
    # Check a sample node to determine the coordinate format
    # sample_node: node identifier (could be string, tuple, int, etc.)
    sample_node = list(airspace_graph.nodes())[0]
    node_data = airspace_graph.nodes[sample_node]  # Dictionary of node attributes
    
    # Boolean flag to track if we need to create 'pos' attributes
    needs_pos_creation = False
    
    # Check if 'pos' attribute already exists
    if 'pos' not in node_data:
        # Check if nodes have individual lat/lon/altitude attributes
        # These are the expected attribute names from the graph generation
        if 'lat' in node_data and 'lon' in node_data and 'altitude' in node_data:
            print(f"│   Format: Individual 'lat', 'lon', 'altitude' attributes{'':<29} │")
            print(f"│   Action: Creating 'pos' tuples for all nodes...{'':<31} │")
            needs_pos_creation = True
        else:
            # If neither format exists, raise an error
            print(f"│   ❌ ERROR: Node missing position data{'':<42} │")
            print(f"│   Available attributes: {', '.join(list(node_data.keys())[:5])}{'':<10} │")
            print("└" + "─"*78 + "┘")
            raise ValueError(f"Node {sample_node} missing position data.")
    
    # Create 'pos' attribute for all nodes if needed
    if needs_pos_creation:
        # node_count: integer counter for progress reporting
        node_count = 0
        
        # Iterate through all nodes in the graph
        for node in airspace_graph.nodes():
            # Get the node's attribute dictionary
            node_attrs = airspace_graph.nodes[node]
            
            # Extract individual coordinate values (float type)
            # lat: latitude in decimal degrees (float)
            # lon: longitude in decimal degrees (float)
            # alt: altitude in meters (float)
            lat = node_attrs['lat']
            lon = node_attrs['lon']
            alt = node_attrs['altitude']  # Use 'altitude' key, not 'alt'
            
            # Create the 'pos' tuple with (lat, lon, altitude) format
            # This matches the expected format in WebSocketServer.py
            airspace_graph.nodes[node]['pos'] = (lat, lon, alt)
            
            node_count += 1
        
        print(f"│   ✓ Created 'pos' for {node_count} nodes{'':<47} │")
    else:
        print(f"│   Format: 'pos' attribute already exists{'':<40} │")
        print(f"│   ✓ All nodes have 'pos' attribute{'':<45} │")
    
    # Now validate that all nodes have valid 'pos' attributes
    # Sample a few nodes to validate (checking all nodes would be slow for large graphs)
    # min() ensures we don't try to sample more nodes than exist
    num_samples = min(10, num_nodes)  # Sample up to 10 nodes for validation
    sample_nodes = list(airspace_graph.nodes())[:num_samples]  # Get first N nodes as list
    
    print(f"│   Validating: Sampling {num_samples} nodes...{'':<44} │")
    for node in sample_nodes:
        # Get the position data and validate it's a 3D coordinate
        # pos should be a tuple or list of (lat, lon, alt) - 3 float values
        pos = airspace_graph.nodes[node]['pos']
        
        # Check if pos is a tuple or list
        if not isinstance(pos, (tuple, list)):
            print(f"│   ❌ ERROR: Node 'pos' must be tuple/list{'':<38} │")
            print("└" + "─"*78 + "┘")
            raise ValueError(f"Node {node} 'pos' attribute must be a tuple or list, got {type(pos)}")
        
        # Check if pos has exactly 3 coordinates (lat, lon, alt)
        if len(pos) != 3:
            print(f"│   ❌ ERROR: 'pos' must have 3 coordinates (lat, lon, alt){'':<18} │")
            print("└" + "─"*78 + "┘")
            raise ValueError(f"Node {node} 'pos' must be a 3D coordinate (lat, lon, alt), got {len(pos)} values")
    
    print(f"│   ✓ All sampled nodes have valid 3D coordinates{'':<32} │")
    
    # ============================================================================
    # VALIDATION 4: Check that edges have 'weight' attribute for pathfinding
    # ============================================================================
    # NetworkX shortest_path() uses edge weights for optimal path calculation
    # Sample a few edges to check for weights
    num_edge_samples = min(10, num_edges)  # Sample up to 10 edges for validation
    sample_edges = list(airspace_graph.edges())[:num_edge_samples]  # Get first N edges as list
    
    print("├" + "─"*78 + "┤")
    print(f"│ {'Edge Weight Attributes':<78} │")
    print("├" + "─"*78 + "┤")
    print(f"│   Validating: Sampling {num_edge_samples} edges...{'':<44} │")
    
    missing_weights = False  # Boolean flag to track if any edges are missing weights
    
    for u, v in sample_edges:
        # Check if 'weight' attribute exists in edge data dictionary
        # airspace_graph.edges[u, v] returns the data dictionary for edge between nodes u and v
        if 'weight' not in airspace_graph.edges[u, v]:
            print(f"│   ⚠ WARNING: Edge missing 'weight' attribute{'':<34} │")
            missing_weights = True
            break
    
    if missing_weights:
        print(f"│   ⚠ Some edges missing weights (unweighted paths){'':<29} │")
    else:
        print(f"│   ✓ All sampled edges have 'weight' for pathfinding{'':<28} │")
    
    # ============================================================================
    # VALIDATION 5: Check graph connectivity for routing purposes
    # ============================================================================
    # A connected graph ensures that paths can be found between any two nodes
    # For undirected graphs: check if fully connected
    # For directed graphs: check if weakly connected (connected when treating edges as undirected)
    
    print("├" + "─"*78 + "┤")
    print(f"│ {'Graph Connectivity':<78} │")
    print("├" + "─"*78 + "┤")
    
    # Check if graph is undirected (Graph or MultiGraph types)
    # Directed graphs (DiGraph, MultiDiGraph) inherit from DiGraph
    if isinstance(airspace_graph, nx.Graph) and not isinstance(airspace_graph, nx.DiGraph):
        # For undirected graphs, check if fully connected
        # is_connected() returns True if there's a path between every pair of nodes
        if nx.is_connected(airspace_graph):
            print(f"│   Type: Undirected Graph{'':<51} │")
            print(f"│   ✓ Fully connected - paths exist between all nodes{'':<27} │")
        else:
            # number_connected_components() returns integer count of disconnected subgraphs
            num_components = nx.number_connected_components(airspace_graph)
            print(f"│   Type: Undirected Graph{'':<51} │")
            print(f"│   ⚠ WARNING: {num_components} disconnected components{'':<34} │")
            print(f"│   Some routes may fail{'':<55} │")
    
    # Check if graph is directed (DiGraph or MultiDiGraph types)
    elif isinstance(airspace_graph, nx.DiGraph):
        # For directed graphs, check if weakly connected
        # is_weakly_connected() treats directed edges as undirected for connectivity check
        if nx.is_weakly_connected(airspace_graph):
            print(f"│   Type: Directed Graph{'':<53} │")
            print(f"│   ✓ Weakly connected - basic connectivity exists{'':<30} │")
            
            # Additionally check if strongly connected (every node can reach every other node following edge directions)
            # is_strongly_connected() returns True only if paths exist in both directions
            if nx.is_strongly_connected(airspace_graph):
                print(f"│   ✓ Strongly connected - bidirectional paths exist{'':<28} │")
            else:
                # number_strongly_connected_components() returns integer count of strongly connected subgraphs
                num_components = nx.number_strongly_connected_components(airspace_graph)
                print(f"│   ⚠ {num_components} strongly connected components{'':<36} │")
                print(f"│   Some routes may be one-way only{'':<44} │")
        else:
            # number_weakly_connected_components() returns integer count of weakly connected subgraphs
            num_components = nx.number_weakly_connected_components(airspace_graph)
            print(f"│   Type: Directed Graph{'':<53} │")
            print(f"│   ⚠ WARNING: {num_components} weakly disconnected components{'':<31} │")
            print(f"│   Some routes may fail{'':<55} │")
    
    # ============================================================================
    # Final summary and return
    # ============================================================================
    print("├" + "─"*78 + "┤")
    print(f"│ {'RESULT':<78} │")
    print("├" + "─"*78 + "┤")
    print(f"│ ✅ Graph successfully loaded and validated!{'':<36} │")
    print(f"│    Type: {type(airspace_graph).__name__:<70} │")
    print(f"│    Nodes: {num_nodes:<69} │")
    print(f"│    Edges: {num_edges:<69} │")
    print("└" + "─"*78 + "┘\n")
    
    # Return the validated NetworkX graph object
    return airspace_graph

