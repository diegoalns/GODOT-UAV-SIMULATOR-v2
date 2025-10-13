#!/usr/bin/env python3
"""
Simulated Annealing Convergence Test

This script tests the convergence behavior of the Simulated Annealing algorithm
for UAV route planning with configurable parameters. It tracks cost vs iterations
and temperature vs cost to analyze algorithm convergence.

Author: Convergence Testing Framework
Date: September 2024
"""

import networkx as nx
from geopy.distance import geodesic
import numpy as np
import random
import math
import copy
import time
import matplotlib.pyplot as plt
from datetime import datetime
import warnings
from itertools import islice
warnings.filterwarnings('ignore')

# Set random seeds for reproducible results
random.seed(39)
np.random.seed(39)
class UAVGraphBuilder:
    """Simplified graph builder for convergence testing."""
    
    def __init__(self, n_lat=30, n_lon=30, n_alt=3):
        """
        Initialize graph builder with configurable dimensions.
        
        Args:
            n_lat (int): Number of latitude grid points
            n_lon (int): Number of longitude grid points  
            n_alt (int): Number of altitude levels
        """
        self.n_lat = n_lat
        self.n_lon = n_lon
        self.n_alt = n_alt
        
        # Fixed airspace boundaries for consistency
        self.min_lat = 40.6042
        self.max_lat = 40.6125
        self.min_lon = -73.9458
        self.max_lon = -73.9292
        self.min_alt = 0
        self.max_alt = 400

    def build_graph(self):
        """Build the airspace graph structure."""
        print(f"Building graph: {self.n_lat}x{self.n_lon}x{self.n_alt} = {self.n_lat*self.n_lon*self.n_alt} nodes")
        
        # Generate grid points
        lats = np.linspace(self.min_lat, self.max_lat, self.n_lat)
        lons = np.linspace(self.min_lon, self.max_lon, self.n_lon)
        alts = np.linspace(self.min_alt, self.max_alt, self.n_alt)

        # Create nodes and positions
        nodes = [(i, j, k) for i in range(self.n_lat) 
                for j in range(self.n_lon) 
                for k in range(self.n_alt)]

        pos = {(i, j, k): (lats[i], lons[j], alts[k]) 
               for i in range(self.n_lat) 
               for j in range(self.n_lon) 
               for k in range(self.n_alt)}

        # Random availability (75% chance)
        availability = {node: np.random.random() < 0.75 for node in nodes}

        # Create graph
        G = nx.DiGraph()
        G.add_nodes_from(nodes)
        nx.set_node_attributes(G, pos, 'pos')
        nx.set_node_attributes(G, availability, 'available')

        # Add edges
        self._add_edges(G, pos)

        available_nodes = sum(1 for node in G.nodes() if G.nodes[node]['available'])
        print(f"Graph built: {available_nodes} available nodes, {len(G.edges())} edges")

        return G

    def _add_edges(self, G, pos):
        """Add edges between neighboring nodes."""

        for i in range(self.n_lat):
            for j in range(self.n_lon):
                for k in range(self.n_alt):
                    if not G.nodes[(i, j, k)]['available']:
                        continue
                    
                    # Check all 26 neighbors
                    for di in [-1, 0, 1]:
                        for dj in [-1, 0, 1]:
                            for dk in [-1, 0, 1]:
                                if di == dj == dk == 0:
                                    continue
                                
                                ni, nj, nk = i + di, j + dj, k + dk
                                
                                if (0 <= ni < self.n_lat and 
                                    0 <= nj < self.n_lon and 
                                    0 <= nk < self.n_alt):
                                    
                                    if not G.nodes[(ni, nj, nk)]['available']:
                                        continue
                                    
                                    # Calculate distance
                                    u, v = (i, j, k), (ni, nj, nk)
                                    dist = self._calculate_distance(pos[u], pos[v])
                                    G.add_edge(u, v, weight=dist)

    def _calculate_distance(self, p1, p2):
        """Calculate 3D distance between two points."""
        dist_2d = geodesic(p1[:2], p2[:2]).meters
        dalt = p2[2] - p1[2]
        return np.sqrt(dist_2d**2 + (100000*dalt)**2)

class ConvergenceTracker:
    """Track convergence metrics during SA optimization."""
    def __init__(self):
        self.reset()
    def reset(self):
        """Reset tracking data."""
        self.iterations = []
        self.temperatures = []
        self.current_costs = []
        self.best_costs = []
        self.accepted_moves = []
        # Add shortest path cost storage
        self.shortest_path_cost = None
    def record(self, iteration, temperature, current_cost, best_cost, accepted):
        """Record convergence data point."""
        self.iterations.append(iteration)
        self.temperatures.append(temperature)
        self.current_costs.append(current_cost)
        self.best_costs.append(best_cost)
        self.accepted_moves.append(accepted)
    
    def set_shortest_path_cost(self, cost):
        """Set the shortest path cost for comparison."""
        self.shortest_path_cost = cost

class SimulatedAnnealingOptimizer:
    """SA optimizer with convergence tracking."""

    def __init__(self, graph):
        self.G = graph
        self.tracker = ConvergenceTracker()
        # Get graph dimensions from the graph structure
        nodes = list(graph.nodes())
        if nodes:
            self.n_lat = max(node[0] for node in nodes) + 1
            self.n_lon = max(node[1] for node in nodes) + 1
            self.n_alt = max(node[2] for node in nodes) + 1
        else:
            self.n_lat = self.n_lon = self.n_alt = 0
        
    def path_cost(self, paths, conflict_penalty=10000):
        """Calculate total cost of solution."""
        total_length = 0
        total_conflicts = 0
        occupied_at_time = {}

        for path in paths:
            for i in range(len(path) - 1):
                u, v = path[i], path[i+1]
                try:
                    total_length += self.G[u][v]['weight']
                except KeyError:
                    return float('inf')
                # Check for conflicts
                if (u, i) in occupied_at_time:
                    total_conflicts += 1
                else:
                    occupied_at_time[(u, i)] = True
            
            # Check last node
            if (path[-1], len(path) - 1) in occupied_at_time:
                 total_conflicts += 1
            else:
                 occupied_at_time[(path[-1], len(path) - 1)] = True

        return total_length + (total_conflicts * conflict_penalty)
    
    def generate_initial_solution(self, uav_routes):
        """Generate initial solution using RRT algorithm and compute shortest path for comparison."""
        paths = []
        shortest_paths = []
        
        print("\nComputing shortest paths for comparison...")
        
        # First, compute shortest paths for all routes
        for i, route in enumerate(uav_routes):
            try:
                shortest_path = nx.shortest_path(self.G, 
                                               source=route['origin'], 
                                               target=route['destination'], 
                                               weight='weight')
                shortest_paths.append(shortest_path)
                print(f"  Route {i+1}: Shortest path length = {len(shortest_path)} nodes")
                
            except nx.NetworkXNoPath:
                print(f"No shortest path found for {route['origin']} -> {route['destination']}")
                return [], []
        
        # Calculate shortest path total cost
        shortest_path_cost = self.path_cost(shortest_paths)
        self.tracker.set_shortest_path_cost(shortest_path_cost)
        print(f"Total shortest path cost: {shortest_path_cost:.2f}")
        
        print("\nGenerating initial RRT solutions...")
        
        # Now generate RRT paths
        for i, route in enumerate(uav_routes):
            try:
                rrt_path = self._rrt_path_planning(
                    route['origin'], 
                    route['destination'],
                    max_iterations=100000,
                    step_size=3
                )
                
                if not rrt_path:
                    print(f"RRT failed for route {i+1}, using shortest path")
                    # Use the pre-computed shortest path
                    paths.append(shortest_paths[i])
                else:
                    paths.append(rrt_path)
                    print(f"RRT Route {i+1}: Found path with length {len(rrt_path)} nodes")
                
            except Exception as e:
                print(f"Error generating path for route {i+1}: {e}")
                print(f"Using shortest path as fallback")
                paths.append(shortest_paths[i])
    
        return paths

    def _rrt_path_planning(self, start, goal, max_iterations=1000000, step_size=3, goal_bias=0.5):
        """
        RRT path planning algorithm.
        
        Args:
            start: Starting node
            goal: Goal node  
            max_iterations: Maximum number of RRT iterations
            step_size: Maximum step size for tree extension
            goal_bias: Probability of sampling goal directly
        
        Returns:
            List of nodes representing the path, or None if no path found
        """
        # Initialize RRT tree
        rrt_tree = nx.DiGraph()
        rrt_tree.add_node(start)
        
        # Get all available nodes for sampling
        available_nodes = [node for node in self.G.nodes() if self.G.nodes[node]['available']]
        
        if start not in available_nodes or goal not in available_nodes:
            return None
        
        for iteration in range(max_iterations):
            # Sample random node (with goal bias)
            if random.random() < goal_bias:
                random_node = goal
            else:
                random_node = random.choice(available_nodes)
            
            # Find nearest node in RRT tree
            nearest_node = self._find_nearest_rrt_node(rrt_tree, random_node)
            
            # Extend tree toward random node
            new_node = self._extend_rrt_tree(nearest_node, random_node, step_size)
            
            # Check if extension is valid and collision-free
            if (new_node and 
                new_node in available_nodes and 
                self._is_rrt_path_valid(nearest_node, new_node)):
                
                # Add new node to tree
                rrt_tree.add_node(new_node)
                # Calculate edge weight using graph distance
                try:
                    edge_weight = self.G[nearest_node][new_node]['weight']
                except KeyError:
                    # If direct edge doesn't exist, use Euclidean distance
                    edge_weight = self._calculate_euclidean_distance(nearest_node, new_node)
                
                rrt_tree.add_edge(nearest_node, new_node, weight=edge_weight)
                
                # Check if goal is reached
                if self._rrt_distance(new_node, goal) < 2.0:  # Within 2 nodes distance
                    # Try to connect directly to goal
                    if self._is_rrt_path_valid(new_node, goal):
                        try:
                            goal_weight = self.G[new_node][goal]['weight']
                        except KeyError:
                            goal_weight = self._calculate_euclidean_distance(new_node, goal)
                        
                        rrt_tree.add_node(goal)
                        rrt_tree.add_edge(new_node, goal, weight=goal_weight)
                        
                        # Extract path from start to goal
                        try:
                            path = nx.shortest_path(rrt_tree, source=start, target=goal, weight='weight')
                            return path
                        except nx.NetworkXNoPath:
                            continue
        
        # If no path found, return None
        return None

    def _find_nearest_rrt_node(self, rrt_tree, target_node):
        """Find the nearest node in RRT tree to target node."""
        min_distance = float('inf')
        nearest_node = None
        
        for node in rrt_tree.nodes():
            distance = self._rrt_distance(node, target_node)
            if distance < min_distance:
                min_distance = distance
                nearest_node = node
        
        return nearest_node

    def _extend_rrt_tree(self, from_node, to_node, step_size):
        """Extend RRT tree from from_node toward to_node with given step_size."""
        # Calculate direction vector
        direction = (
            to_node[0] - from_node[0],
            to_node[1] - from_node[1], 
            to_node[2] - from_node[2]
        )
        
        # Calculate distance
        distance = math.sqrt(sum(d**2 for d in direction))
        
        if distance == 0:
            return from_node
        
        # If target is within step size, go directly
        if distance <= step_size:
            return to_node
        
        # Normalize direction and scale by step_size
        scale = step_size / distance
        new_coords = (
            from_node[0] + int(direction[0] * scale),
            from_node[1] + int(direction[1] * scale),
            from_node[2] + int(direction[2] * scale)
        )
        
        # Ensure coordinates are within graph bounds
        new_coords = (
            max(0, min(self.n_lat - 1, new_coords[0])),
            max(0, min(self.n_lon - 1, new_coords[1])),
            max(0, min(self.n_alt - 1, new_coords[2]))
        )
        
        return new_coords

    def _is_rrt_path_valid(self, from_node, to_node):
        """Check if path between two nodes is valid (collision-free)."""
        # Check if both nodes are available
        if (not self.G.nodes[from_node]['available'] or 
            not self.G.nodes[to_node]['available']):
            return False
        
        # For simplicity, check if there's a direct connection or if nodes are close
        if self.G.has_edge(from_node, to_node):
            return True
        
        # Check if nodes are adjacent (within 1 step in each dimension)
        diff = (
            abs(to_node[0] - from_node[0]),
            abs(to_node[1] - from_node[1]),
            abs(to_node[2] - from_node[2])
        )
        
        return all(d <= 1 for d in diff)

    def _rrt_distance(self, node1, node2):
        """Calculate distance between two nodes for RRT."""
        return math.sqrt(
            (node1[0] - node2[0])**2 + 
            (node1[1] - node2[1])**2 + 
            (node1[2] - node2[2])**2
        )

    def _calculate_euclidean_distance(self, node1, node2):
        """Calculate Euclidean distance between two nodes using their positions."""
        pos1 = self.G.nodes[node1]['pos']
        pos2 = self.G.nodes[node2]['pos']
        
        # 2D geographic distance
        dist_2d = geodesic(pos1[:2], pos2[:2]).meters
        # Altitude difference
        dalt = pos2[2] - pos1[2]
        
        return math.sqrt(dist_2d**2 + (dalt)**2)

    def get_neighbor(self, current_solution):
        """Generate neighboring solution."""
        new_solution = copy.deepcopy(current_solution)
        uav_index = random.randint(0, len(new_solution) - 1)
        path_to_modify = new_solution[uav_index]

        if len(path_to_modify) < 3:
            return new_solution

        start_idx = random.randint(0, len(path_to_modify) - 2)
        end_idx = random.randint(start_idx + 1, len(path_to_modify) - 1)
        start_node = path_to_modify[start_idx]
        end_node = path_to_modify[end_idx]

        try:
            new_segment = nx.shortest_path(self.G, source=start_node, 
                                         target=end_node, weight='weight')
            new_path = path_to_modify[:start_idx] + new_segment + path_to_modify[end_idx+1:]
            new_solution[uav_index] = new_path
        except nx.NetworkXNoPath:
            return current_solution
        
        return new_solution

    def simulated_annealing_with_tracking(self, uav_routes, initial_temp=100000, 
                                        cooling_rate=0.80, min_temp=0.001):
        """SA algorithm with convergence tracking."""
        print("Starting Simulated Annealing with convergence tracking...")
        
        # Reset tracker
        self.tracker.reset()
        
        # Generate initial solution
        current_paths = self.generate_initial_solution(uav_routes)
        if not current_paths:
            return None, float('inf'), self.tracker

        best_paths = current_paths
        current_cost = self.path_cost(current_paths)
        best_cost = current_cost
        temperature = initial_temp
        iterations = 0

        print(f"Initial cost: {current_cost:.2f}")
        
        while temperature > min_temp:
            # Get neighboring solution
            new_paths = self.get_neighbor(current_paths)
            new_cost = self.path_cost(new_paths)
            # Print header every 20 iterations
            if iterations % 20 == 0:
                print(f"\n{'Iter':>6} | {'Temp':>10} | {'Current':>10} | {'New':>10} | {'Best':>10} | {'Delta':>10}")
                print("-" * 65)
            
            # Print current iteration data in table format
            delta = new_cost - current_cost
            print(f"{iterations:>6} | {temperature:>10.4f} | {current_cost:>10.2f} | {new_cost:>10.2f} | {best_cost:>10.2f} | {delta:>10.2f}")

            # Acceptance decision
            delta = new_cost - current_cost
            accepted = False
            
            if delta < 0:
                # Always accept better solutions
                current_paths = new_paths
                current_cost = new_cost
                accepted = True
            # Metropolis criterion (accept worse solutions with probability based on temperature and cost difference, 
            # the higher the temperature, the more likely to accept worse solutions)
            elif random.uniform(0, 1) < math.exp(-delta / temperature):
                # Accept worse solutions with probability
                current_paths = new_paths
                current_cost = new_cost
                accepted = True
            
            # Update best solution
            if current_cost < best_cost:
                best_paths = current_paths
                best_cost = current_cost
            
            # Record convergence data
            self.tracker.record(iterations, temperature, current_cost, best_cost, accepted)
            
            # Cool temperature
            temperature *= cooling_rate
            iterations += 1
            
            # Progress reporting
            if iterations % 100 == 0:
                print(f"Iteration {iterations}: Temp={temperature:.4f}, Current={current_cost:.2f}, Best={best_cost:.2f}")
        
        print(f"Optimization completed: {iterations} iterations, Final cost: {best_cost:.2f}")
        return best_paths, best_cost, self.tracker

def generate_uav_routes(graph, num_drones):
    """Generate UAV routes for testing."""
    available_nodes = [node for node in graph.nodes() if graph.nodes[node]['available']]
    
    # Get edge nodes
    max_lat_idx = max(node[0] for node in available_nodes)
    left_edge = [node for node in available_nodes if node[0] == 0]
    right_edge = [node for node in available_nodes if node[0] == max_lat_idx]
    
    routes = []
    for i in range(min(num_drones, len(left_edge), len(right_edge))):
        origin = left_edge[i % len(left_edge)]
        destination = right_edge[i % len(right_edge)]
        
        # Verify path exists
        try:
            nx.shortest_path(graph, source=origin, target=destination, weight='weight')
            routes.append({'origin': origin, 'destination': destination})
        except nx.NetworkXNoPath:
            continue
    print(f"\n Generated {len(routes)} UAV routes \n")
    for route in routes:
        print(f"  Route: {route['origin']} -> {route['destination']}")
    return routes

def plot_convergence(tracker, num_drones, graph_dims):
    """Plot convergence analysis."""
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    
    # Plot 1: Cost vs Iterations
    ax1.plot(tracker.iterations, tracker.current_costs, 'b-', linewidth=5, label='Current Cost')
    ax1.plot(tracker.iterations, tracker.best_costs, 'r-', linewidth=2, label='Best Cost')
    
    # Add shortest path cost as horizontal reference line
    if tracker.shortest_path_cost is not None:
        ax1.axhline(y=tracker.shortest_path_cost, color='green', linestyle='--', 
               linewidth=2, label=f'Shortest Path Cost ({tracker.shortest_path_cost:.2e})')
    
    ax1.set_xlabel('Iterations')
    ax1.set_ylabel('Cost')
    ax1.set_title('Cost vs Iterations')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Temperature vs Cost  
    ax2.scatter(tracker.temperatures, tracker.current_costs, c=tracker.iterations, 
                cmap='viridis', alpha=0.6, s=10)
    
    # Add shortest path cost reference line
    if tracker.shortest_path_cost is not None:
        ax2.axhline(y=tracker.shortest_path_cost, color='green', linestyle='--', 
                   linewidth=2, alpha=0.8)
    
    ax2.set_xlabel('Temperature')
    ax2.set_ylabel('Current Cost')
    ax2.set_title('Temperature vs Cost')
    ax2.set_xscale('log')
    colorbar = plt.colorbar(ax2.collections[0], ax=ax2)
    colorbar.set_label('Iteration')
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Temperature Cooling Schedule
    ax3.plot(tracker.iterations, tracker.temperatures, 'g-')
    ax3.set_xlabel('Iterations')
    ax3.set_ylabel('Temperature')
    ax3.set_title('Temperature Cooling Schedule')
    ax3.set_yscale('log')
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Acceptance Rate - IMPROVED VERSION
    window_size = max(50, len(tracker.accepted_moves) // 20)
    acceptance_rate = []
    iterations_windowed = []
    
    # Debug: Check data types and first few values
    print(f"Debug - accepted_moves type: {type(tracker.accepted_moves[0])}")
    print(f"Debug - first 10 accepted_moves: {tracker.accepted_moves[:10]}")
    print(f"Debug - window_size: {window_size}")
    
    for i in range(window_size, len(tracker.accepted_moves)):
        window = tracker.accepted_moves[i-window_size:i]
        
        # Ensure boolean conversion and explicit calculation
        accepted_count = sum(True for move in window if move)  # Count True values
        total_count = len(window)
        rate = (accepted_count / total_count) * 100 if total_count > 0 else 0
        
        acceptance_rate.append(rate)
        # Use the middle point of the window for better alignment
        iterations_windowed.append(tracker.iterations[i - window_size//2])
    
    # Additional debug info
    if len(acceptance_rate) > 0:
        print(f"Debug - acceptance rate range: {min(acceptance_rate):.1f}% to {max(acceptance_rate):.1f}%")
        print(f"Debug - first 5 acceptance rates: {acceptance_rate[:5]}")
    
    ax4.plot(iterations_windowed, acceptance_rate, 'm-', linewidth=2)
    ax4.set_xlabel('Iterations')
    ax4.set_ylabel('Acceptance Rate (%)')
    ax4.set_title(f'Acceptance Rate (Window: {window_size})')
    ax4.set_ylim(0, 100)  # Set y-axis limits for better visualization
    ax4.grid(True, alpha=0.3)
    
    plt.suptitle(f'SA Convergence Analysis - {num_drones} Drones, Graph: {graph_dims[0]}x{graph_dims[1]}x{graph_dims[2]}', 
                 fontsize=14)
    plt.tight_layout()
    
    # Save plot
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"sa_convergence_{num_drones}drones_{timestamp}.png"
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Convergence plot saved to {filename}")
    
    plt.show()

def main():
    """Main function for convergence testing."""
    print("SIMULATED ANNEALING CONVERGENCE TEST")
    print("="*50)
    
    # Configurable parameters
    NUM_DRONES = 20  # Change this to set number of drones
    GRAPH_DIMENSIONS = (50, 50, 3)  # (n_lat, n_lon, n_alt) - Change this to set graph size
    
    print(f"Configuration:")
    print(f"  Number of drones: {NUM_DRONES}")
    print(f"  Graph dimensions: {GRAPH_DIMENSIONS[0]}x{GRAPH_DIMENSIONS[1]}x{GRAPH_DIMENSIONS[2]}")
    print("="*50)
    
    # Build graph
    start_time = time.time()
    builder = UAVGraphBuilder(n_lat=GRAPH_DIMENSIONS[0], 
                             n_lon=GRAPH_DIMENSIONS[1], 
                             n_alt=GRAPH_DIMENSIONS[2])
    graph = builder.build_graph()
    
    # Generate routes
    routes = generate_uav_routes(graph, NUM_DRONES)
    actual_drones = len(routes)
    print(f"Generated {actual_drones} routes")
    
    if actual_drones == 0:
        print("No valid routes found. Try different parameters.")
        return
    
    # Run SA with convergence tracking
    optimizer = SimulatedAnnealingOptimizer(graph)
    best_paths, final_cost, tracker = optimizer.simulated_annealing_with_tracking(
        routes, 
        initial_temp=10000, 
        cooling_rate=0.99, 
        min_temp=0.00001
    )
    
    total_time = time.time() - start_time
    
    # Results summary
    print("\n" + "="*50)
    print("CONVERGENCE TEST RESULTS")
    print("="*50)
    print(f"Total execution time: {total_time:.2f} seconds")
    print(f"Final cost: {final_cost:.2f}")
    print(f"Total iterations: {len(tracker.iterations)}")
    print(f"Initial cost: {tracker.current_costs[0]:.2f}")
    print(f"Cost improvement: {tracker.current_costs[0] - final_cost:.2f}")
    print(f"Improvement percentage: {((tracker.current_costs[0] - final_cost) / tracker.current_costs[0] * 100):.1f}%")
    
    # Add shortest path comparison
    if tracker.shortest_path_cost is not None:
        print(f"\nShortest Path Comparison:")
        print(f"Shortest path cost: {tracker.shortest_path_cost:.2f}")
        print(f"SA final cost: {final_cost:.2f}")
        optimality_gap = ((final_cost - tracker.shortest_path_cost) / tracker.shortest_path_cost * 100)
        print(f"Optimality gap: {optimality_gap:.1f}%")
        
        if final_cost <= tracker.shortest_path_cost:
            print("✓ SA found solution better than or equal to shortest path!")
        else:
            print(f"△ SA solution is {optimality_gap:.1f}% above shortest path cost")
    
    # Plot convergence
    plot_convergence(tracker, actual_drones, GRAPH_DIMENSIONS)

if __name__ == "__main__":
    main()