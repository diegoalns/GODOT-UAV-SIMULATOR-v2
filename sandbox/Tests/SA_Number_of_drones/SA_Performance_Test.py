#!/usr/bin/env python3
"""
Simulated Annealing Performance Testing Framework

This script evaluates the computational performance of the Simulated Annealing
algorithm for UAV route planning with varying numbers of drones (10-100 in steps of 10).

The airspace graph data structure is generated once and reused for all tests to ensure
consistent testing conditions.

Author: Performance Testing Framework
Date: September 2024
"""

import networkx as nx
from geopy.distance import geodesic
import numpy as np
import random
import math
import copy
import time
import csv
import matplotlib.pyplot as plt
from datetime import datetime
import statistics
import pandas as pd
import warnings
warnings.filterwarnings('ignore')

# Set random seeds for reproducible results
random.seed(42)
np.random.seed(42)

class UAVGraphBuilder:
    """
    Class responsible for building the UAV airspace graph structure.
    This ensures the graph is built once and reused across all tests.
    """
    
    def __init__(self, min_lat=40.6042, max_lat=40.6125, min_lon=-73.9458, 
                 max_lon=-73.9292, min_alt=0, max_alt=400, n_lat=50, n_lon=50, n_alt=3):
        """
        Initialize graph builder with airspace parameters.
        
        Args:
            min_lat (float): Minimum latitude boundary
            max_lat (float): Maximum latitude boundary
            min_lon (float): Minimum longitude boundary
            max_lon (float): Maximum longitude boundary
            min_alt (float): Minimum altitude in meters
            max_alt (float): Maximum altitude in meters
            n_lat (int): Number of latitude grid points
            n_lon (int): Number of longitude grid points  
            n_alt (int): Number of altitude levels
        """
        self.min_lat = min_lat
        self.max_lat = max_lat
        self.min_lon = min_lon
        self.max_lon = max_lon
        self.min_alt = min_alt
        self.max_alt = max_alt
        self.n_lat = n_lat
        self.n_lon = n_lon
        self.n_alt = n_alt
        
        # Initialize graph components
        self.G = None
        self.pos = None
        self.nodes = None
        
    def build_graph(self):
        """
        Build the complete airspace graph structure.
        This method generates the 3D lattice graph with geographic coordinates.
        """
        print("Building airspace graph structure...")
        start_time = time.time()
        
        # Generate grid points using NumPy's linspace for uniform distribution
        lats = np.linspace(self.min_lat, self.max_lat, self.n_lat)
        lons = np.linspace(self.min_lon, self.max_lon, self.n_lon)
        alts = np.linspace(self.min_alt, self.max_alt, self.n_alt)
        
        # Create node identifiers as 3D grid coordinates (i, j, k)
        self.nodes = [(i, j, k) for i in range(self.n_lat) 
                     for j in range(self.n_lon) 
                     for k in range(self.n_alt)]
        
        # Map each node to its geographic position (lat, lon, alt)
        self.pos = {(i, j, k): (lats[i], lons[j], alts[k]) 
                   for i in range(self.n_lat) 
                   for j in range(self.n_lon) 
                   for k in range(self.n_alt)}
        
        # Randomly assign availability (75% chance of being available)
        # This simulates obstacles or restricted airspace
        availability = {node: np.random.random() < 0.75 for node in self.nodes}
        
        # Create empty directed graph using NetworkX
        self.G = nx.DiGraph()
        
        # Add all nodes to the graph
        self.G.add_nodes_from(self.nodes)
        
        # Set node attributes for geographic coordinates and availability
        nx.set_node_attributes(self.G, self.pos, 'pos')
        nx.set_node_attributes(self.G, availability, 'available')
        
        # Add edges connecting each available node to its neighbors (including diagonals)
        self._add_edges()
        
        build_time = time.time() - start_time
        available_nodes = sum(1 for node in self.G.nodes() if self.G.nodes[node]['available'])
        
        print(f"Graph built in {build_time:.2f} seconds")
        print(f"Total nodes: {len(self.G.nodes())}")
        print(f"Available nodes: {available_nodes} ({(available_nodes/len(self.G.nodes()))*100:.1f}%)")
        print(f"Total edges: {len(self.G.edges())}")
        
        return self.G
    
    def _add_edges(self):
        """
        Add edges between neighboring nodes in the 3D lattice.
        Each node can connect to up to 26 neighbors (3x3x3 - 1).
        """
        edge_count = 0
        
        for i in range(self.n_lat):
            for j in range(self.n_lon):
                for k in range(self.n_alt):
                    # Skip unavailable source nodes
                    if not self.G.nodes[(i, j, k)]['available']:
                        continue
                    
                    # Check all 26 possible neighbors (3x3x3 cube minus center)
                    for di in [-1, 0, 1]:
                        for dj in [-1, 0, 1]:
                            for dk in [-1, 0, 1]:
                                # Skip the center node (no self-loops)
                                if di == dj == dk == 0:
                                    continue
                                
                                # Calculate neighbor coordinates
                                ni, nj, nk = i + di, j + dj, k + dk
                                
                                # Check if neighbor is within grid bounds
                                if (0 <= ni < self.n_lat and 
                                    0 <= nj < self.n_lon and 
                                    0 <= nk < self.n_alt):
                                    
                                    # Skip unavailable destination nodes
                                    if not self.G.nodes[(ni, nj, nk)]['available']:
                                        continue
                                    
                                    # Calculate 3D distance between nodes
                                    u, v = (i, j, k), (ni, nj, nk)
                                    dist = self._slant_range(self.pos[u], self.pos[v])
                                    
                                    # Add weighted edge
                                    self.G.add_edge(u, v, weight=dist)
                                    edge_count += 1
        
        print(f"Added {edge_count} edges to the graph")
    
    def _slant_range(self, p1, p2):
        """
        Calculate the 3D distance between two geographic points.
        
        Args:
            p1 (tuple): First point as (lat, lon, alt)
            p2 (tuple): Second point as (lat, lon, alt)
            
        Returns:
            float: 3D distance in meters
        """
        # Calculate 2D distance using geodesic (great circle distance)
        dist_2d = geodesic(p1[:2], p2[:2]).meters
        
        # Calculate altitude difference
        dalt = p2[2] - p1[2]
        
        # Return 3D Euclidean distance
        # Note: altitude scaling factor of 100000 from original code
        return np.sqrt(dist_2d**2 + (100000*dalt)**2)

class UAVRouteGenerator:
    """
    Class responsible for generating UAV routes for testing.
    Ensures consistent route generation across different test runs.
    """
    
    def __init__(self, graph):
        """
        Initialize route generator with the airspace graph.
        
        Args:
            graph (nx.DiGraph): The airspace graph structure
        """
        self.G = graph
        
    def get_available_edge_nodes(self):
        """
        Get available nodes at the edges of the grid for route generation.
        
        Returns:
            tuple: (left_edge_nodes, right_edge_nodes)
        """
        available_nodes = [node for node in self.G.nodes() if self.G.nodes[node]['available']]
        
        # Get maximum grid dimensions
        max_lat_idx = max(node[0] for node in available_nodes)
        
        # Get nodes at the left edge (x=0) and right edge (x=max)
        left_edge = [node for node in available_nodes if node[0] == 0]
        right_edge = [node for node in available_nodes if node[0] == max_lat_idx]
        
        return left_edge, right_edge
    
    def generate_uav_routes(self, num_routes):
        """
        Generate UAV routes using available nodes at grid edges.
        
        Args:
            num_routes (int): Number of UAV routes to generate
            
        Returns:
            list: List of route dictionaries with 'origin' and 'destination' keys
        """
        left_edge, right_edge = self.get_available_edge_nodes()
        
        routes = []
        used_origins = set()
        used_destinations = set()
        
        max_possible_routes = min(len(left_edge), len(right_edge))
        actual_routes = min(num_routes, max_possible_routes)
        
        if actual_routes < num_routes:
            print(f"Warning: Can only generate {actual_routes} routes instead of {num_routes}")
        
        for i in range(actual_routes):
            # Find unused origin and destination
            available_origins = [node for node in left_edge if node not in used_origins]
            available_destinations = [node for node in right_edge if node not in used_destinations]
            
            if not available_origins or not available_destinations:
                break
            
            # Use modulo to cycle through available nodes if needed
            origin = available_origins[i % len(available_origins)]
            destination = available_destinations[i % len(available_destinations)]
            
            # Verify path exists
            try:
                nx.shortest_path(self.G, source=origin, target=destination, weight='weight')
                routes.append({'origin': origin, 'destination': destination})
                used_origins.add(origin)
                used_destinations.add(destination)
            except nx.NetworkXNoPath:
                print(f"No path found for UAV {i+1}: {origin} -> {destination}")
                continue
        
        return routes

class SimulatedAnnealingOptimizer:
    """
    Simulated Annealing optimizer for UAV route planning.
    This class contains all SA-related functionality.
    """
    
    def __init__(self, graph):
        """
        Initialize SA optimizer with the airspace graph.
        
        Args:
            graph (nx.DiGraph): The airspace graph structure
        """
        self.G = graph
        
    def path_cost(self, paths, conflict_penalty=10000):
        """
        Calculate the total cost of a solution (path lengths + conflict penalties).
        
        Args:
            paths (list): List of UAV paths
            conflict_penalty (float): Penalty for each conflict
            
        Returns:
            float: Total cost of the solution
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
                    path_len += self.G[u][v]['weight']
                except KeyError:
                    # Invalid path - return infinite cost
                    return float('inf')
                
                # Check for conflicts at the current time step
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
    
    def generate_initial_solution(self, uav_routes):
        """
        Generate an initial solution using shortest paths.
        
        Args:
            uav_routes (list): List of UAV route specifications
            
        Returns:
            list: List of initial paths for each UAV
        """
        paths = []
        for route in uav_routes:
            try:
                path = nx.shortest_path(self.G, source=route['origin'], 
                                      target=route['destination'], weight='weight')
                paths.append(path)
            except nx.NetworkXNoPath:
                print(f"No path found for {route['origin']} -> {route['destination']}")
                return []
        
        return paths
    
    def get_neighbor(self, current_solution):
        """
        Generate a neighboring solution by modifying a random path segment.
        
        Args:
            current_solution (list): Current solution paths
            
        Returns:
            list: Modified solution with one path segment rerouted
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
            new_segment = nx.shortest_path(self.G, source=start_node, 
                                         target=end_node, weight='weight')
            
            # Replace the old segment with the new one
            new_path = path_to_modify[:start_idx] + new_segment + path_to_modify[end_idx+1:]
            new_solution[uav_index] = new_path
            
        except nx.NetworkXNoPath:
            # If no path exists for the segment, return the original solution
            return current_solution
        
        return new_solution
    
    def simulated_annealing(self, uav_routes, initial_temp=1000, cooling_rate=0.9, 
                          min_temp=0.001, max_iterations=None):
        """
        Main Simulated Annealing algorithm for finding optimal conflict-free paths.
        
        Args:
            uav_routes (list): List of UAV route specifications
            initial_temp (float): Initial temperature for SA
            cooling_rate (float): Temperature cooling rate
            min_temp (float): Minimum temperature threshold
            max_iterations (int): Maximum number of iterations (optional)
            
        Returns:
            tuple: (optimized_paths, final_cost, iterations, time_taken)
        """
        start_time = time.time()
        
        # Generate initial solution
        current_paths = self.generate_initial_solution(uav_routes)
        if not current_paths:
            return None, float('inf'), 0, 0

        best_paths = current_paths
        current_cost = self.path_cost(current_paths)
        best_cost = current_cost
        temperature = initial_temp
        iterations = 0

        while temperature > min_temp:
            if max_iterations and iterations >= max_iterations:
                break
                
            # Get a neighboring solution
            new_paths = self.get_neighbor(current_paths)
            new_cost = self.path_cost(new_paths)

            # Decide whether to accept the new solution
            delta = new_cost - current_cost
            if delta < 0 or random.uniform(0, 1) < math.exp(-delta / temperature):
                current_paths = new_paths
                current_cost = new_cost
            
            # Update the best solution found so far
            if current_cost < best_cost:
                best_paths = current_paths
                best_cost = current_cost
            
            # Cool the temperature
            temperature *= cooling_rate
            iterations += 1
        
        time_taken = time.time() - start_time
        return best_paths, best_cost, iterations, time_taken

class PerformanceTestRunner:
    """
    Main class for running performance tests on the SA algorithm.
    """
    
    def __init__(self):
        """Initialize the performance test runner."""
        self.graph_builder = UAVGraphBuilder()
        self.G = None
        self.route_generator = None
        self.sa_optimizer = None
        self.results = []
        
    def setup(self):
        """Set up the testing environment by building the graph once."""
        print("Setting up performance testing environment...")
        print("="*60)
        
        # Build the graph structure once
        self.G = self.graph_builder.build_graph()
        
        # Initialize route generator and SA optimizer
        self.route_generator = UAVRouteGenerator(self.G)
        self.sa_optimizer = SimulatedAnnealingOptimizer(self.G)
        
        print("="*60)
        print("Setup complete. Ready to run performance tests.\n")
        
    def run_single_test(self, num_drones, test_run=1, total_runs=1):
        """
        Run a single performance test for a given number of drones.
        
        Args:
            num_drones (int): Number of drones to test
            test_run (int): Current test run number
            total_runs (int): Total number of test runs
            
        Returns:
            dict: Test results dictionary
        """
        print(f"Running test {test_run}/{total_runs} with {num_drones} drones...")
        
        # Generate UAV routes
        route_start = time.time()
        uav_routes = self.route_generator.generate_uav_routes(num_drones)
        route_gen_time = time.time() - route_start
        
        actual_drones = len(uav_routes)
        if actual_drones < num_drones:
            print(f"  Warning: Only {actual_drones} routes generated instead of {num_drones}")
        
        # Run Simulated Annealing
        sa_result = self.sa_optimizer.simulated_annealing(uav_routes)
        optimized_paths, final_cost, iterations, sa_time = sa_result
        
        # Calculate solution quality metrics
        conflicts = self._count_conflicts(optimized_paths) if optimized_paths else float('inf')
        total_path_length = self._calculate_total_path_length(optimized_paths) if optimized_paths else 0
        
        result = {
            'num_drones': actual_drones,
            'requested_drones': num_drones,
            'route_generation_time': route_gen_time,
            'sa_time': sa_time,
            'total_time': route_gen_time + sa_time,
            'final_cost': final_cost,
            'iterations': iterations,
            'conflicts': conflicts,
            'total_path_length': total_path_length,
            'success': optimized_paths is not None,
            'timestamp': datetime.now().isoformat()
        }
        
        print(f"  Completed in {sa_time:.2f}s, Cost: {final_cost:.2f}, Conflicts: {conflicts}")
        return result
    
    def _count_conflicts(self, paths):
        """Count the number of conflicts in the solution."""
        if not paths:
            return float('inf')
            
        conflicts = 0
        occupied_at_time = {}
        
        for path in paths:
            for i in range(len(path)):
                if (path[i], i) in occupied_at_time:
                    conflicts += 1
                else:
                    occupied_at_time[(path[i], i)] = True
        
        return conflicts
    
    def _calculate_total_path_length(self, paths):
        """Calculate total path length for all UAVs."""
        if not paths:
            return 0
            
        total_length = 0
        for path in paths:
            for i in range(len(path) - 1):
                u, v = path[i], path[i+1]
                try:
                    total_length += self.G[u][v]['weight']
                except KeyError:
                    pass
        
        return total_length
    
    def run_performance_tests(self, drone_counts=None, runs_per_test=3):
        """
        Run comprehensive performance tests.
        
        Args:
            drone_counts (list): List of drone counts to test (default: 10-100 step 10)
            runs_per_test (int): Number of runs per test for statistical reliability
        """
        if drone_counts is None:
            drone_counts = list(range(10, 101, 10))  # 10, 20, 30, ..., 100
        
        print(f"Starting performance tests for drone counts: {drone_counts}")
        print(f"Runs per test: {runs_per_test}")
        print("="*60)
        
        total_tests = len(drone_counts) * runs_per_test
        current_test = 0
        
        for num_drones in drone_counts:
            print(f"\nTesting with {num_drones} drones:")
            print("-" * 40)
            
            for run in range(runs_per_test):
                current_test += 1
                result = self.run_single_test(num_drones, run + 1, runs_per_test)
                result['test_id'] = current_test
                result['run_number'] = run + 1
                self.results.append(result)
        
        print("\n" + "="*60)
        print("All performance tests completed!")
        print("="*60)
        
    def analyze_results(self):
        """Analyze and display test results."""
        if not self.results:
            print("No results to analyze.")
            return
        
        print("\nPERFORMANCE ANALYSIS RESULTS")
        print("="*60)
        
        # Group results by drone count
        drone_counts = sorted(set(r['num_drones'] for r in self.results))
        
        analysis_data = []
        
        for num_drones in drone_counts:
            drone_results = [r for r in self.results if r['num_drones'] == num_drones]
            
            # Calculate statistics
            sa_times = [r['sa_time'] for r in drone_results if r['success']]
            total_times = [r['total_time'] for r in drone_results if r['success']]
            final_costs_raw = [r['final_cost'] for r in drone_results if r['success'] and r['final_cost'] != float('inf')]
            final_costs_normalized = [r['final_cost']/num_drones for r in drone_results if r['success'] and r['final_cost'] != float('inf')]
            conflicts = [r['conflicts'] for r in drone_results if r['success'] and r['conflicts'] != float('inf')]
            iterations = [r['iterations'] for r in drone_results if r['success']]
            
            if sa_times:
                analysis_data.append({
                    'drones': num_drones,
                    'avg_sa_time': statistics.mean(sa_times),
                    'std_sa_time': statistics.stdev(sa_times) if len(sa_times) > 1 else 0,
                    'min_sa_time': min(sa_times),
                    'max_sa_time': max(sa_times),
                    'avg_total_time': statistics.mean(total_times),
                    'avg_cost_raw': statistics.mean(final_costs_raw) if final_costs_raw else float('inf'),
                    'avg_cost_per_drone': statistics.mean(final_costs_normalized) if final_costs_normalized else float('inf'),
                    'avg_conflicts': statistics.mean(conflicts) if conflicts else float('inf'),
                    'avg_iterations': statistics.mean(iterations),
                    'success_rate': len([r for r in drone_results if r['success']]) / len(drone_results)
                })
        
        # Display results table - use avg_cost_per_drone instead of avg_cost
        print(f"{'Drones':<8} {'Avg Time(s)':<12} {'Std Dev':<10} {'Min Time':<10} {'Max Time':<10} {'Cost/Drone':<12} {'Conflicts':<10} {'Success%':<10}")
        print("-" * 90)
        
        for data in analysis_data:
            print(f"{data['drones']:<8} {data['avg_sa_time']:<12.3f} {data['std_sa_time']:<10.3f} "
                  f"{data['min_sa_time']:<10.3f} {data['max_sa_time']:<10.3f} {data['avg_cost_per_drone']:<12.1f} "
                  f"{data['avg_conflicts']:<10.1f} {data['success_rate']*100:<10.1f}")
        
        return analysis_data
    
    def save_results(self, filename=None):
        """Save results to CSV file."""
        if not self.results:
            print("No results to save.")
            return
        
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"sa_performance_results_{timestamp}.csv"
        
        # Save detailed results
        df = pd.DataFrame(self.results)
        df.to_csv(filename, index=False)
        print(f"Results saved to {filename}")
        
        # Save summary statistics
        analysis_data = self.analyze_results()
        if analysis_data:
            summary_filename = filename.replace('.csv', '_summary.csv')
            summary_df = pd.DataFrame(analysis_data)
            summary_df.to_csv(summary_filename, index=False)
            print(f"Summary statistics saved to {summary_filename}")
    
    def plot_results(self):
        """Create performance visualization plots."""
        if not self.results:
            print("No results to plot.")
            return
        
        # Group results by drone count and calculate means
        drone_counts = sorted(set(r['num_drones'] for r in self.results))
        avg_times = []
        std_times = []
        
        for num_drones in drone_counts:
            drone_results = [r for r in self.results if r['num_drones'] == num_drones and r['success']]
            times = [r['sa_time'] for r in drone_results]
            if times:
                avg_times.append(statistics.mean(times))
                std_times.append(statistics.stdev(times) if len(times) > 1 else 0)
            else:
                avg_times.append(0)
                std_times.append(0)
        
        # Create performance plot with 6 subplots (2x3 grid)
        plt.figure(figsize=(18, 12))
        
        # Plot 1: Execution time vs number of drones
        plt.subplot(2, 3, 1)
        plt.errorbar(drone_counts, avg_times, yerr=std_times, marker='o', capsize=5)
        plt.xlabel('Number of Drones')
        plt.ylabel('Average SA Time (seconds)')
        plt.title('Simulated Annealing Performance vs Number of Drones')
        plt.grid(True, alpha=0.3)
        
        # Plot 2: Success rate
        plt.subplot(2, 3, 2)
        success_rates = []
        for num_drones in drone_counts:
            drone_results = [r for r in self.results if r['num_drones'] == num_drones]
            success_rate = len([r for r in drone_results if r['success']]) / len(drone_results)
            success_rates.append(success_rate * 100)
        
        plt.bar(drone_counts, success_rates, alpha=0.7)
        plt.xlabel('Number of Drones')
        plt.ylabel('Success Rate (%)')
        plt.title('Algorithm Success Rate')
        plt.ylim(0, 105)
        plt.grid(True, alpha=0.3)
        
        # Plot 3: Raw (Non-normalized) Average Cost
        plt.subplot(2, 3, 3)
        avg_costs_raw = []
        std_costs_raw = []
        for num_drones in drone_counts:
            drone_results = [r for r in self.results if r['num_drones'] == num_drones and r['success']]
            costs = [r['final_cost'] for r in drone_results if r['final_cost'] != float('inf')]
            if costs:
                avg_costs_raw.append(statistics.mean(costs))
                std_costs_raw.append(statistics.stdev(costs) if len(costs) > 1 else 0)
            else:
                avg_costs_raw.append(0)
                std_costs_raw.append(0)
        
        plt.errorbar(drone_counts, avg_costs_raw, yerr=std_costs_raw, marker='s', color='green', capsize=3)
        plt.xlabel('Number of Drones')
        plt.ylabel('Average Total Cost')
        plt.title('Solution Quality - Raw Total Cost')
        plt.grid(True, alpha=0.3)
        
        # Plot 4: Normalized Average Cost (Cost per Drone)
        plt.subplot(2, 3, 4)
        avg_costs_normalized = []
        std_costs_normalized = []
        for num_drones in drone_counts:
            drone_results = [r for r in self.results if r['num_drones'] == num_drones and r['success']]
            costs_normalized = [r['final_cost']/num_drones for r in drone_results if r['final_cost'] != float('inf')]
            if costs_normalized:
                avg_costs_normalized.append(statistics.mean(costs_normalized))
                std_costs_normalized.append(statistics.stdev(costs_normalized) if len(costs_normalized) > 1 else 0)
            else:
                avg_costs_normalized.append(0)
                std_costs_normalized.append(0)
        
        plt.errorbar(drone_counts, avg_costs_normalized, yerr=std_costs_normalized, marker='o', color='blue', capsize=3)
        plt.xlabel('Number of Drones')
        plt.ylabel('Average Cost per Drone')
        plt.title('Solution Quality - Normalized Cost per Drone')
        plt.grid(True, alpha=0.3)
        
        # Plot 5: Average iterations
        plt.subplot(2, 3, 5)
        avg_iterations = []
        for num_drones in drone_counts:
            drone_results = [r for r in self.results if r['num_drones'] == num_drones and r['success']]
            iterations_list = [r['iterations'] for r in drone_results]
            avg_iterations.append(statistics.mean(iterations_list) if iterations_list else 0)
        
        plt.plot(drone_counts, avg_iterations, marker='^', color='red')
        plt.xlabel('Number of Drones')
        plt.ylabel('Average Iterations')
        plt.title('Algorithm Convergence (Iterations)')
        plt.grid(True, alpha=0.3)
        
        # Plot 6: Cost Comparison (Both Raw and Normalized on same plot with dual y-axis)
        plt.subplot(2, 3, 6)
        
        # Create primary axis for raw costs
        ax1 = plt.gca()
        line1 = ax1.plot(drone_counts, avg_costs_raw, marker='s', color='green', label='Raw Total Cost', linewidth=2)
        ax1.set_xlabel('Number of Drones')
        ax1.set_ylabel('Raw Total Cost', color='green')
        ax1.tick_params(axis='y', labelcolor='green')
        ax1.grid(True, alpha=0.3)
        
        # Create secondary axis for normalized costs
        ax2 = ax1.twinx()
        line2 = ax2.plot(drone_counts, avg_costs_normalized, marker='o', color='blue', label='Cost per Drone', linewidth=2)
        ax2.set_ylabel('Cost per Drone', color='blue')
        ax2.tick_params(axis='y', labelcolor='blue')
        
        # Add legend
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax1.legend(lines, labels, loc='upper left')
        
        plt.title('Cost Comparison: Raw vs Normalized')
        
        plt.tight_layout()
        
        # Save the plot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        plot_filename = f"sa_performance_plot_{timestamp}.png"
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"Performance plots saved to {plot_filename}")
        
        plt.show()

def main():
    """
    Main function to run the performance testing framework.
    """
    print("SIMULATED ANNEALING PERFORMANCE TESTING FRAMEWORK")
    print("="*60)
    print("Testing SA algorithm performance with varying drone counts (10-100)")
    print("Graph structure will be generated once and reused for all tests")
    print("="*60)
    
    # Initialize test runner
    test_runner = PerformanceTestRunner()
    
    # Set up the testing environment (build graph once)
    test_runner.setup()
    
    # Define test parameters
    drone_counts = list(range(10, 101, 10))  # 10, 20, 30, ..., 100
    runs_per_test = 3  # Number of runs per drone count for statistical reliability
    
    # Run performance tests
    test_runner.run_performance_tests(drone_counts, runs_per_test)
    
    # Analyze results
    test_runner.analyze_results()
    
    # Save results to CSV
    test_runner.save_results()
    
    # Create performance plots
    test_runner.plot_results()
    
    print("\nPerformance testing completed successfully!")
    print("Check the generated CSV files and plots for detailed results.")

if __name__ == "__main__":
    main()
