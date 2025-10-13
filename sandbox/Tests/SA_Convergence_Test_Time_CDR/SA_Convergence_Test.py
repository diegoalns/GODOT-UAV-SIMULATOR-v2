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
from collections import defaultdict
import heapq
from dataclasses import dataclass
from typing import List, Tuple, Dict, Set, Optional
warnings.filterwarnings('ignore')

# Set random seeds for reproducible results
random.seed(123)
np.random.seed(123)

@dataclass
class DroneModel:
    """Represents a drone with specific flight characteristics."""
    model_name: str
    max_speed: float  # m/s
    cruise_speed: float  # m/s
    min_speed: float  # m/s
    acceleration: float  # m/s²
    energy_consumption_rate: float  # Wh/km at cruise speed

# Predefined drone models
DRONE_MODELS = {
    'DJI_PHANTOM': DroneModel('DJI Phantom 4', 20.0, 15.0, 5.0, 4.0, 120.0),
    'DJI_MAVIC': DroneModel('DJI Mavic Pro', 18.0, 13.0, 3.0, 3.5, 100.0),
    'RACING_DRONE': DroneModel('Racing Drone', 35.0, 25.0, 8.0, 8.0, 200.0),
    'CARGO_DRONE': DroneModel('Cargo Drone', 15.0, 12.0, 2.0, 2.0, 180.0),
    'SURVEILLANCE': DroneModel('Surveillance', 22.0, 16.0, 4.0, 3.0, 140.0)
}

class TimeStepConverter:
    """Utilities for converting between distance, time, and discrete time steps."""
    
    def __init__(self, time_step_duration=1.0):
        """
        Initialize converter.
        
        Args:
            time_step_duration (float): Duration of each time step in seconds
        """
        self.time_step_duration = time_step_duration
    
    def distance_to_time_steps(self, distance_meters, speed_ms):
        """Convert distance to number of time steps at given speed."""
        time_seconds = distance_meters / speed_ms
        return max(1, int(math.ceil(time_seconds / self.time_step_duration)))
    
    def time_steps_to_seconds(self, time_steps):
        """Convert time steps to seconds."""
        return time_steps * self.time_step_duration
    
    def seconds_to_time_steps(self, seconds):
        """Convert seconds to time steps (rounded up)."""
        return int(math.ceil(seconds / self.time_step_duration))

@dataclass 
class TemporalTrajectory:
    """Represents a drone's path with timing information."""
    drone_id: int
    spatial_path: List[tuple]  # List of (lat_idx, lon_idx, alt_idx)
    time_points: List[float]   # Time when drone reaches each point (seconds)
    speeds: List[float]        # Speed between consecutive points (m/s)
    
    def get_position_at_time(self, query_time):
        """Get interpolated position at specific time."""
        if query_time <= self.time_points[0]:
            return self.spatial_path[0]
        if query_time >= self.time_points[-1]:
            return self.spatial_path[-1]
        
        # Find segment containing query_time
        for i in range(len(self.time_points) - 1):
            if self.time_points[i] <= query_time <= self.time_points[i + 1]:
                # For simplicity, return discrete position (no interpolation)
                # Could add continuous interpolation if needed
                return self.spatial_path[i]
        
        return self.spatial_path[-1]
    
    def get_duration(self):
        """Get total trajectory duration."""
        return self.time_points[-1] - self.time_points[0]
    
    def get_total_distance(self, spatial_graph=None):
        """
        Get total distance traveled using graph edge weights when available.
        
        Args:
            spatial_graph: NetworkX graph with edge weights (optional)
            
        Returns:
            float: Total distance in meters
        """
        total = 0
        for i in range(len(self.spatial_path) - 1):
            current_node = self.spatial_path[i]
            next_node = self.spatial_path[i + 1]
            
            if spatial_graph and spatial_graph.has_edge(current_node, next_node):
                # Use actual graph edge weight (accurate geodesic + altitude distance)
                edge_distance = spatial_graph[current_node][next_node]['weight']
                total += edge_distance
            else:
                # Fallback to simplified Euclidean distance in grid coordinates
                # This approximation is less accurate but ensures method always works
                p1, p2 = current_node, next_node
                total += math.sqrt(sum((a - b)**2 for a, b in zip(p1, p2)))
        return total
    
    def validate_distance_consistency(self, spatial_graph=None, tolerance=0.1):
        """
        Validate consistency between different distance calculation methods.
        
        Args:
            spatial_graph: NetworkX graph for accurate calculations
            tolerance: Acceptable relative difference between methods
            
        Returns:
            dict: Validation results with consistency information
        """
        if not spatial_graph:
            return {"status": "skipped", "reason": "No graph provided for validation"}
        
        accurate_distance = self.get_total_distance(spatial_graph)
        approximate_distance = self.get_total_distance()  # No graph - uses approximation
        
        if accurate_distance == 0:
            relative_diff = 0 if approximate_distance == 0 else float('inf')
        else:
            relative_diff = abs(accurate_distance - approximate_distance) / accurate_distance
        
        is_consistent = relative_diff <= tolerance
        
        return {
            "status": "consistent" if is_consistent else "inconsistent",
            "accurate_distance": accurate_distance,
            "approximate_distance": approximate_distance,
            "relative_difference": relative_diff,
            "tolerance": tolerance,
            "drone_id": self.drone_id
        }

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
        self.min_alt = 0      # feet (0 ft = 0 m)
        self.max_alt = 400    # feet (400 ft ≈ 122 m) 

    def build_graph(self):
        """Build the airspace graph structure."""
        print(f"Building graph: {self.n_lat}x{self.n_lon}x{self.n_alt} = {self.n_lat*self.n_lon*self.n_alt} nodes")
        
        # Generate grid points
        lats = np.linspace(self.min_lat, self.max_lat, self.n_lat)
        lons = np.linspace(self.min_lon, self.max_lon, self.n_lon)
        # Convert altitude from feet to meters for position coordinates
        alts_feet = np.linspace(self.min_alt, self.max_alt, self.n_alt)
        alts = alts_feet * 0.3048  # Convert feet to meters

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
        dalt = p2[2] - p1[2]  # altitude difference (positions are already in meters)
        return np.sqrt(dist_2d**2 + dalt**2)

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

class ReservationTable:
    """Efficient spatio-temporal conflict detection using hash tables."""
    
    def __init__(self, time_step_duration=1.0):
        self.time_step_duration = time_step_duration
        # Core data structure: (node, time_step) -> drone_id
        self.reservations = {}
        # Reverse lookup: drone_id -> list of (node, time_step)
        self.drone_reservations = defaultdict(list)
        # Safety distance for near-miss detection (in grid units)
        self.safety_distance = 1
    
    def clear(self):
        """Clear all reservations."""
        self.reservations.clear()
        self.drone_reservations.clear()
    
    def add_trajectory(self, drone_id, temporal_trajectory):
        """Add a complete trajectory to the reservation table."""
        self.clear_drone_reservations(drone_id)
        
        time_converter = TimeStepConverter(self.time_step_duration)
        
        for i, (node, time_seconds) in enumerate(zip(temporal_trajectory.spatial_path, 
                                                   temporal_trajectory.time_points)):
            time_step = time_converter.seconds_to_time_steps(time_seconds)
            
            # Reserve the exact time step
            key = (node, time_step)
            if key in self.reservations and self.reservations[key] != drone_id:
                return False, f"Conflict at {node} at time step {time_step}"
            
            self.reservations[key] = drone_id
            self.drone_reservations[drone_id].append(key)
        
        return True, "Success"
    
    def clear_drone_reservations(self, drone_id):
        """Remove all reservations for a specific drone."""
        for key in self.drone_reservations[drone_id]:
            if key in self.reservations and self.reservations[key] == drone_id:
                del self.reservations[key]
        self.drone_reservations[drone_id].clear()
    
    def is_occupied(self, node, time_step):
        """Check if a spatio-temporal point is occupied."""
        return (node, time_step) in self.reservations
    
    def get_occupying_drone(self, node, time_step):
        """Get the drone ID occupying a spatio-temporal point."""
        return self.reservations.get((node, time_step), None)
    
    def check_trajectory_conflicts(self, drone_id, temporal_trajectory):
        """Check if a trajectory would create conflicts."""
        conflicts = []
        time_converter = TimeStepConverter(self.time_step_duration)
        
        for i, (node, time_seconds) in enumerate(zip(temporal_trajectory.spatial_path,
                                                   temporal_trajectory.time_points)):
            time_step = time_converter.seconds_to_time_steps(time_seconds)
            key = (node, time_step)
            
            if key in self.reservations:
                occupying_drone = self.reservations[key]
                if occupying_drone != drone_id:
                    conflicts.append({
                        'node': node,
                        'time_step': time_step,
                        'time_seconds': time_seconds,
                        'conflicting_drone': occupying_drone,
                        'type': 'direct_collision'
                    })
        
        return conflicts
    
    def update_drone_trajectory(self, drone_id, old_trajectory, new_trajectory):
        """Atomically update a drone's trajectory in the reservation table."""
        # Step 1: Check if new trajectory would create conflicts
        conflicts = self.check_trajectory_conflicts(drone_id, new_trajectory)
        
        if conflicts:
            return False, conflicts
        
        # Step 2: Remove old reservations
        self.clear_drone_reservations(drone_id)
        
        # Step 3: Add new reservations
        success, message = self.add_trajectory(drone_id, new_trajectory)
        
        if not success:
            # Restore old trajectory if new one failed
            self.add_trajectory(drone_id, old_trajectory)
            return False, [message]
        
        return True, []
    
    def get_alternative_times(self, node, preferred_time_step, window=5):
        """Find alternative time steps when a node is available."""
        alternatives = []
        for t in range(max(0, preferred_time_step - window), 
                      preferred_time_step + window + 1):
            if not self.is_occupied(node, t):
                alternatives.append(t)
        return alternatives
    
    def get_occupancy_statistics(self):
        """Get statistics about current occupancy."""
        total_reservations = len(self.reservations)
        drones_with_reservations = len([d for d in self.drone_reservations.values() if d])
        
        # Time span
        if self.reservations:
            all_times = [key[1] for key in self.reservations.keys()]
            time_span = max(all_times) - min(all_times) + 1
        else:
            time_span = 0
        
        return {
            'total_reservations': total_reservations,
            'active_drones': drones_with_reservations,
            'time_span': time_span,
            'memory_usage_mb': len(str(self.reservations)) / (1024 * 1024)  # Rough estimate
        }

class PriorityBasedPlanner:
    """Generate initial feasible solution using priority-based sequential planning."""
    
    def __init__(self, spatial_graph, time_step_duration=1.0):
        self.spatial_graph = spatial_graph
        self.time_converter = TimeStepConverter(time_step_duration)
        self.reservation_table = ReservationTable(time_step_duration)
    
    def plan_all_drones(self, routes_with_drones, min_success_rate=0.7):
        """Plan all drones sequentially in priority order."""
        # Sort routes by priority (shortest path first for better initial solution)
        sorted_routes = self._prioritize_routes(routes_with_drones)
        
        temporal_trajectories = []
        failed_drones = []
        self.reservation_table.clear()
        
        print(f"Planning {len(sorted_routes)} drones in priority order...")
        
        for route_info in sorted_routes:
            trajectory = self._plan_single_drone(route_info)
            
            if trajectory is None:
                print(f"❌ Failed to find path for drone {route_info['drone_id']}")
                failed_drones.append(route_info['drone_id'])
                continue
            
            # Add to reservation table
            success, message = self.reservation_table.add_trajectory(
                route_info['drone_id'], trajectory
            )
            
            if not success:
                print(f"❌ Reservation conflict for drone {route_info['drone_id']}: {message}")
                failed_drones.append(route_info['drone_id'])
                continue
            
            temporal_trajectories.append(trajectory)
            print(f"✅ Planned drone {route_info['drone_id']}: {len(trajectory.spatial_path)} nodes, "
                  f"{trajectory.get_duration():.1f}s duration")
        
        success_rate = len(temporal_trajectories) / len(sorted_routes)
        print(f"\nPlanning complete: {len(temporal_trajectories)}/{len(sorted_routes)} drones successful "
              f"({success_rate:.1%} success rate)")
        
        if success_rate >= min_success_rate:
            if failed_drones:
                print(f"⚠ Failed drones: {failed_drones}")
            return temporal_trajectories
        else:
            print(f"❌ Success rate {success_rate:.1%} below minimum {min_success_rate:.1%}")
            return None
    
    def _prioritize_routes(self, routes_with_drones):
        """Sort routes by priority (shortest path length first)."""
        prioritized = []
        
        for i, route in enumerate(routes_with_drones):
            try:
                shortest_path = nx.shortest_path(
                    self.spatial_graph,
                    source=route['origin'],
                    target=route['destination'],
                    weight='weight'
                )
                path_length = nx.shortest_path_length(
                    self.spatial_graph,
                    source=route['origin'],
                    target=route['destination'],
                    weight='weight'
                )
                
                prioritized.append({
                    'drone_id': i,
                    'origin': route['origin'],
                    'destination': route['destination'],
                    'drone_model': route.get('drone_model', 'DJI_PHANTOM'),
                    'departure_time': route.get('departure_time', 0.0),
                    'path_length': path_length,
                    'estimated_nodes': len(shortest_path)
                })
                
            except nx.NetworkXNoPath:
                print(f"No path exists for route {i}")
                continue
        
        # Sort by path length (shortest first), then by departure time
        prioritized.sort(key=lambda x: (x['path_length'], x['departure_time']))
        return prioritized
    
    def _plan_single_drone(self, route_info):
        """Plan a single drone avoiding existing reservations."""
        origin = route_info['origin']
        destination = route_info['destination']
        drone_model = DRONE_MODELS[route_info['drone_model']]
        departure_time = route_info['departure_time']
        
        print(f"  Planning drone {route_info['drone_id']}: {origin} -> {destination}")
        
        # Try temporal A* first
        path = self._temporal_astar(origin, destination, departure_time, drone_model)
        
        if path:
            return self._convert_to_temporal_trajectory(
                route_info['drone_id'], path, departure_time, drone_model
            )
        
        # Fallback: Use regular shortest path with increasing delays
        print(f"    Temporal A* failed, trying shortest path with delays...")
        
        for delay_multiplier in [1, 2, 3, 5, 8]:
            delayed_departure = departure_time + (delay_multiplier * 10.0)  # 10, 20, 30, 50, 80 second delays
            
            try:
                # Get shortest path (ignoring temporal conflicts)
                shortest_path = nx.shortest_path(
                    self.spatial_graph,
                    source=origin,
                    target=destination,
                    weight='weight'
                )
                
                # Convert to temporal trajectory
                trajectory = self._convert_to_temporal_trajectory(
                    route_info['drone_id'], shortest_path, delayed_departure, drone_model
                )
                
                # Check if this delayed trajectory has conflicts
                conflicts = self.reservation_table.check_trajectory_conflicts(
                    route_info['drone_id'], trajectory
                )
                
                if not conflicts:
                    print(f"    Found conflict-free path with {delay_multiplier*10}s delay")
                    return trajectory
                else:
                    print(f"    Delay {delay_multiplier*10}s still has {len(conflicts)} conflicts")
                
            except nx.NetworkXNoPath:
                print(f"    No spatial path exists")
                return None
        
        print(f"    Could not find conflict-free path even with delays")
        return None
    
    def _temporal_astar(self, start, goal, start_time, drone_model, max_iterations=2000, max_time_horizon=150):
        """A* search that avoids reserved spatio-temporal points with limits."""
        from heapq import heappush, heappop
        
        print(f"    Temporal A* from {start} to {goal}, start_time={start_time:.1f}s")
        
        # Priority queue: (f_score, g_score, current_node, current_time, path)
        open_set = [(0, 0, start, start_time, [start])]
        visited = set()
        iterations = 0
        
        while open_set and iterations < max_iterations:
            f_score, g_score, current, current_time, path = heappop(open_set)
            iterations += 1
            
            # Limit search time horizon
            if current_time - start_time > max_time_horizon:
                continue
            
            # Convert current time to time step for reservation checking
            current_time_step = self.time_converter.seconds_to_time_steps(current_time)
            state = (current, current_time_step)
            
            if state in visited:
                continue
            visited.add(state)
            
            # Check if we reached the goal
            if current == goal:
                print(f"    ✓ Path found in {iterations} iterations, length: {len(path)}")
                return path
            
            # Progress reporting
            if iterations % 1000 == 0:
                print(f"    A* iteration {iterations}, open_set size: {len(open_set)}, current: {current}")
                
            # Early termination for very long paths
            if len(path) > 50:  # Reasonable path length limit
                continue
                
            # Limit open set size to prevent memory explosion
            if len(open_set) > 5000:
                print(f"    A* open set too large ({len(open_set)}), terminating")
                break
            
            # Explore neighbors
            if current in self.spatial_graph:
                for neighbor in self.spatial_graph.neighbors(current):
                    if not self.spatial_graph.nodes[neighbor]['available']:
                        continue
                    
                    # Calculate travel time to neighbor
                    edge_distance = self.spatial_graph[current][neighbor]['weight']
                    travel_time = edge_distance / drone_model.cruise_speed
                    arrival_time = current_time + travel_time
                    arrival_time_step = self.time_converter.seconds_to_time_steps(arrival_time)
                    
                    # Check if neighbor is available at arrival time
                    if self.reservation_table.is_occupied(neighbor, arrival_time_step):
                        continue
                    
                    neighbor_state = (neighbor, arrival_time_step)
                    if neighbor_state in visited:
                        continue
                    
                    # Calculate costs
                    new_g_score = g_score + edge_distance
                    h_score = self._heuristic(neighbor, goal)
                    new_f_score = new_g_score + h_score
                    
                    new_path = path + [neighbor]
                    heappush(open_set, (new_f_score, new_g_score, neighbor, arrival_time, new_path))
        
        print(f"    Temporal A* failed after {iterations} iterations")
        return None  # No path found
    
    def _heuristic(self, node1, node2):
        """Heuristic function for A* (simplified Euclidean distance)."""
        # Use simple grid-based distance to avoid expensive calculations
        # This is an approximation but should work fine for A*
        return math.sqrt(sum((a - b)**2 for a, b in zip(node1, node2))) * 100
    
    def _convert_to_temporal_trajectory(self, drone_id, spatial_path, start_time, drone_model):
        """Convert spatial path to temporal trajectory."""
        time_points = [start_time]
        speeds = []
        current_time = start_time
        
        for i in range(len(spatial_path) - 1):
            current_node = spatial_path[i]
            next_node = spatial_path[i + 1]
            
            # Get edge distance
            edge_distance = self.spatial_graph[current_node][next_node]['weight']
            
            # Use cruise speed for simplicity (could add speed optimization later)
            speed = drone_model.cruise_speed
            travel_time = edge_distance / speed
            
            current_time += travel_time
            time_points.append(current_time)
            speeds.append(speed)
        
        return TemporalTrajectory(
            drone_id=drone_id,
            spatial_path=spatial_path,
            time_points=time_points,
            speeds=speeds
        )

@dataclass
class DroneEvent:
    """Event for discrete event simulation."""
    time: float
    drone_id: int
    event_type: str  # 'enter_node', 'exit_node'
    node: tuple
    
    def __lt__(self, other):
        return self.time < other.time

class EventBasedValidator:
    """Comprehensive conflict validation using discrete event simulation."""
    
    def __init__(self, safety_distance=50.0, time_tolerance=0.1):
        """
        Initialize validator.
        
        Args:
            safety_distance (float): Minimum safe distance between drones in meters
            time_tolerance (float): Time tolerance for conflict detection in seconds
        """
        self.safety_distance = safety_distance
        self.time_tolerance = time_tolerance
    
    def validate_solution(self, temporal_trajectories, spatial_graph):
        """Comprehensive validation of a solution."""
        conflicts = []
        
        # Check direct node conflicts
        conflicts.extend(self._check_node_conflicts(temporal_trajectories))
        
        # Check proximity conflicts (disabled for performance - focus on node conflicts)
        # conflicts.extend(self._check_proximity_conflicts(temporal_trajectories, spatial_graph))
        
        # Check edge conflicts (head-on collisions)
        conflicts.extend(self._check_edge_conflicts(temporal_trajectories))
        
        return conflicts
    
    def _check_node_conflicts(self, temporal_trajectories):
        """Check for direct node occupation conflicts."""
        events = []
        conflicts = []
        
        # Create enter/exit events for each drone
        for trajectory in temporal_trajectories:
            for i, (node, time) in enumerate(zip(trajectory.spatial_path, trajectory.time_points)):
                # Enter event
                events.append(DroneEvent(time, trajectory.drone_id, 'enter_node', node))
                
                # Exit event (drone leaves when moving to next node, or at end+buffer)
                if i < len(trajectory.time_points) - 1:
                    exit_time = trajectory.time_points[i + 1]
                else:
                    exit_time = time + 1.0  # Stay at final node for 1 second
                
                events.append(DroneEvent(exit_time, trajectory.drone_id, 'exit_node', node))
        
        # Sort events by time
        events.sort()
        
        # Simulate and detect conflicts
        active_nodes = defaultdict(set)  # node -> set of drone_ids
        
        for event in events:
            if event.event_type == 'enter_node':
                if active_nodes[event.node]:
                    # Conflict detected!
                    conflicts.append({
                        'type': 'node_conflict',
                        'time': event.time,
                        'node': event.node,
                        'drones': list(active_nodes[event.node]) + [event.drone_id],
                        'severity': 'high'
                    })
                
                active_nodes[event.node].add(event.drone_id)
                
            else:  # exit_node
                active_nodes[event.node].discard(event.drone_id)
                if not active_nodes[event.node]:
                    del active_nodes[event.node]
        
        return conflicts
    
    def _check_proximity_conflicts(self, temporal_trajectories, spatial_graph):
        """Check for proximity conflicts (drones too close in space)."""
        conflicts = []
        
        # Sample time points for checking
        max_time = max(traj.time_points[-1] for traj in temporal_trajectories)
        time_samples = np.arange(0, max_time + 1, 1.0)  # Check every second
        
        for t in time_samples:
            # Get positions of all drones at time t
            drone_positions = {}
            for trajectory in temporal_trajectories:
                pos = trajectory.get_position_at_time(t)
                if pos:
                    drone_positions[trajectory.drone_id] = pos
            
            # Check all pairs for proximity
            drone_ids = list(drone_positions.keys())
            for i in range(len(drone_ids)):
                for j in range(i + 1, len(drone_ids)):
                    drone1, drone2 = drone_ids[i], drone_ids[j]
                    pos1, pos2 = drone_positions[drone1], drone_positions[drone2]
                    
                    # Calculate 3D distance
                    distance = self._calculate_3d_distance(pos1, pos2, spatial_graph)
                    
                    if distance < self.safety_distance:
                        conflicts.append({
                            'type': 'proximity_conflict',
                            'time': t,
                            'drones': [drone1, drone2],
                            'positions': [pos1, pos2],
                            'distance': distance,
                            'severity': 'medium' if distance > self.safety_distance * 0.5 else 'high'
                        })
        
        return conflicts
    
    def _check_edge_conflicts(self, temporal_trajectories):
        """Check for edge conflicts (head-on collisions)."""
        conflicts = []
        
        # Build edge usage timeline
        edge_usage = defaultdict(list)  # (node1, node2) -> [(drone_id, start_time, end_time)]
        
        for trajectory in temporal_trajectories:
            for i in range(len(trajectory.spatial_path) - 1):
                node1 = trajectory.spatial_path[i]
                node2 = trajectory.spatial_path[i + 1]
                start_time = trajectory.time_points[i]
                end_time = trajectory.time_points[i + 1]
                
                # Add both directions to catch head-on collisions
                edge_usage[(node1, node2)].append((trajectory.drone_id, start_time, end_time))
                edge_usage[(node2, node1)].append((trajectory.drone_id, start_time, end_time))
        
        # Check for overlapping usage of opposite directions
        for (node1, node2), usages in edge_usage.items():
            if len(usages) > 1:
                # Check all pairs for temporal overlap
                for i in range(len(usages)):
                    for j in range(i + 1, len(usages)):
                        drone1, start1, end1 = usages[i]
                        drone2, start2, end2 = usages[j]
                        
                        # Check if time intervals overlap
                        if self._intervals_overlap(start1, end1, start2, end2):
                            conflicts.append({
                                'type': 'edge_conflict',
                                'edge': (node1, node2),
                                'drones': [drone1, drone2],
                                'time_intervals': [(start1, end1), (start2, end2)],
                                'severity': 'high'
                            })
        
        return conflicts
    
    def _calculate_3d_distance(self, pos1, pos2, spatial_graph):
        """Calculate 3D distance between two positions (simplified)."""
        try:
            # Use simple grid-based distance to avoid expensive geographic calculations
            # Each grid step is approximately 100m based on the airspace size
            grid_distance = math.sqrt(sum((a - b)**2 for a, b in zip(pos1, pos2)))
            return grid_distance * 100  # Convert to meters (approximate)
        except Exception:
            return 1000  # Safe default distance
    
    def _intervals_overlap(self, start1, end1, start2, end2):
        """Check if two time intervals overlap."""
        return start1 < end2 and start2 < end1
    
    def print_conflict_summary(self, conflicts):
        """Print a summary of detected conflicts."""
        if not conflicts:
            print("✓ No conflicts detected - solution is feasible!")
            return
        
        print(f"⚠ {len(conflicts)} conflicts detected:")
        
        conflict_counts = defaultdict(int)
        for conflict in conflicts:
            conflict_counts[conflict['type']] += 1
        
        for conflict_type, count in conflict_counts.items():
            severity_counts = defaultdict(int)
            for conflict in conflicts:
                if conflict['type'] == conflict_type:
                    severity_counts[conflict.get('severity', 'unknown')] += 1
            
            print(f"  {conflict_type}: {count} total")
            for severity, sev_count in severity_counts.items():
                print(f"    - {severity}: {sev_count}")

class SimulatedAnnealingOptimizer:
    """SA optimizer with hybrid temporal conflict resolution."""

    def __init__(self, graph, time_step_duration=1.0):
        self.G = graph
        self.tracker = ConvergenceTracker()
        self.time_step_duration = time_step_duration
        
        # Initialize hybrid approach components
        self.priority_planner = PriorityBasedPlanner(graph, time_step_duration)
        self.reservation_table = ReservationTable(time_step_duration)
        self.event_validator = EventBasedValidator()
        
        # Get graph dimensions from the graph structure
        nodes = list(graph.nodes())
        if nodes:
            self.n_lat = max(node[0] for node in nodes) + 1
            self.n_lon = max(node[1] for node in nodes) + 1
            self.n_alt = max(node[2] for node in nodes) + 1
        else:
            self.n_lat = self.n_lon = self.n_alt = 0
        
    def path_cost(self, temporal_trajectories):
        """Calculate total cost based only on distance."""
        if not temporal_trajectories:
            return float('inf')
        
        # Calculate total distance cost only
        total_distance = 0
        
        for trajectory in temporal_trajectories:
            # Distance cost - sum of all edge weights in the path
            for i in range(len(trajectory.spatial_path) - 1):
                u, v = trajectory.spatial_path[i], trajectory.spatial_path[i + 1]
                try:
                    total_distance += self.G[u][v]['weight']
                except KeyError:
                    return float('inf')  # Invalid path
        
        return total_distance
    
    def generate_initial_solution(self, uav_routes, timeout_seconds=120):
        """Generate initial conflict-free solution using hybrid approach with timeout."""
        print("\n=== PHASE 1: INITIAL FEASIBLE SOLUTION ===")
        start_time = time.time()
        
        # Convert routes to include drone models and timing
        routes_with_drones = []
        for i, route in enumerate(uav_routes):
            # Assign drone models (could be specified in route or randomly assigned)
            drone_models = list(DRONE_MODELS.keys())
            drone_model = route.get('drone_model', np.random.choice(drone_models))
            
            routes_with_drones.append({
                'origin': route['origin'],
                'destination': route['destination'],
                'drone_model': drone_model,
                'departure_time': route.get('departure_time', 0.0)
            })
        
        print(f"Attempting to plan {len(routes_with_drones)} drones with {timeout_seconds}s timeout...")
        
        # Use priority-based planner for initial solution
        temporal_trajectories = self.priority_planner.plan_all_drones(routes_with_drones)
        
        elapsed_time = time.time() - start_time
        
        if temporal_trajectories is None:
            print(f"❌ Failed to find initial feasible solution after {elapsed_time:.1f}s")
            
            # Try with fewer drones as fallback
            print("🔄 Trying with reduced number of drones...")
            reduced_routes = routes_with_drones[:len(routes_with_drones)//2]  # Try with half the drones
            temporal_trajectories = self.priority_planner.plan_all_drones(reduced_routes, min_success_rate=0.5)
            
            if temporal_trajectories is None:
                print("❌ Even reduced drone set failed")
                # Ultimate fallback: simple solution with large time separations
                temporal_trajectories = self._generate_simple_initial_solution(uav_routes)
                if temporal_trajectories is None:
                    print("❌ All fallback strategies failed")
                    return None
            else:
                print(f"✓ Fallback successful with {len(temporal_trajectories)} drones")
        
        print(f"✓ Found initial solution with {len(temporal_trajectories)} drones in {elapsed_time:.1f}s")
        
        # Validate the solution
        conflicts = self.event_validator.validate_solution(temporal_trajectories, self.G)
        if conflicts:
            print(f"⚠ Warning: Initial solution has {len(conflicts)} conflicts")
            self.event_validator.print_conflict_summary(conflicts)
        else:
            print("✓ Initial solution is conflict-free")
        
        # Compute shortest path cost for comparison
        self._compute_shortest_path_baseline(uav_routes)
        
        return temporal_trajectories
    
    def _generate_simple_initial_solution(self, uav_routes):
        """Generate simple initial solution with large time separations (ultimate fallback)."""
        print("🔄 Trying simple solution with large time separations...")
        
        simple_trajectories = []
        departure_interval = 60.0  # 1 minute between departures
        
        for i, route in enumerate(uav_routes):
            try:
                # Use shortest path
                shortest_path = nx.shortest_path(
                    self.G, 
                    source=route['origin'], 
                    target=route['destination'], 
                    weight='weight'
                )
                
                # Large departure delay to avoid conflicts
                departure_time = i * departure_interval
                drone_model = DRONE_MODELS['DJI_PHANTOM']  # Use standard model
                
                # Convert to temporal trajectory
                trajectory = self._convert_path_to_temporal_trajectory(
                    i, shortest_path, departure_time, drone_model
                )
                
                simple_trajectories.append(trajectory)
                
            except nx.NetworkXNoPath:
                print(f"No path for route {i}")
                continue
        
        if simple_trajectories:
            print(f"✓ Simple solution generated with {len(simple_trajectories)} drones")
            return simple_trajectories
        else:
            return None
    
    def _convert_path_to_temporal_trajectory(self, drone_id, spatial_path, start_time, drone_model):
        """Convert spatial path to temporal trajectory."""
        time_points = [start_time]
        speeds = []
        current_time = start_time
        
        for i in range(len(spatial_path) - 1):
            current_node = spatial_path[i]
            next_node = spatial_path[i + 1]
            
            # Get edge distance
            edge_distance = self.G[current_node][next_node]['weight']
            
            # Use cruise speed
            speed = drone_model.cruise_speed
            travel_time = edge_distance / speed
            
            current_time += travel_time
            time_points.append(current_time)
            speeds.append(speed)
        
        return TemporalTrajectory(
            drone_id=drone_id,
            spatial_path=spatial_path,
            time_points=time_points,
            speeds=speeds
        )
    
    def _compute_shortest_path_baseline(self, uav_routes):
        """Compute shortest path baseline for comparison."""
        print("\nComputing shortest path baseline...")
        
        shortest_paths = []
        total_shortest_distance = 0
        
        for i, route in enumerate(uav_routes):
            try:
                shortest_path = nx.shortest_path(self.G, 
                                               source=route['origin'], 
                                               target=route['destination'], 
                                               weight='weight')
                shortest_distance = nx.shortest_path_length(self.G,
                                                          source=route['origin'],
                                                          target=route['destination'],
                                                          weight='weight')
                
                shortest_paths.append(shortest_path)
                total_shortest_distance += shortest_distance
                
                print(f"  Route {i+1}: {len(shortest_path)} nodes, {shortest_distance:.1f}m")
                
            except nx.NetworkXNoPath:
                print(f"No shortest path found for route {i+1}")
                return
        
        # Store baseline (ignoring conflicts for comparison)
        self.tracker.set_shortest_path_cost(total_shortest_distance)
        print(f"Total shortest path distance: {total_shortest_distance:.2f}m (ignoring conflicts)")

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
        # Altitude difference (positions are already in meters after conversion)
        dalt = pos2[2] - pos1[2]  # already in meters
        
        return math.sqrt(dist_2d**2 + (dalt)**2)

    def get_neighbor(self, current_solution):
        """Generate neighboring solution with more aggressive modifications."""
        if not current_solution:
            return current_solution
        
        # Try multiple times to generate a different neighbor
        max_attempts = 5
        for attempt in range(max_attempts):
            # Select random drone to modify
            drone_index = random.randint(0, len(current_solution) - 1)
            current_trajectory = current_solution[drone_index]
            
            # Try different types of modifications (weighted towards more impactful changes)
            modification_types = ['departure_time', 'speed_adjustment', 'reroute_segment']
            modification_type = random.choice(modification_types)
            
            new_trajectory = None
            if modification_type == 'reroute_segment':
                new_trajectory = self._modify_trajectory_segment(current_trajectory)
            elif modification_type == 'speed_adjustment':
                new_trajectory = self._modify_trajectory_speed(current_trajectory)
            else:  # departure_time - most likely to succeed
                new_trajectory = self._modify_departure_time(current_trajectory)
            
            if new_trajectory is not None:
                # Create new solution
                new_solution = current_solution.copy()
                new_solution[drone_index] = new_trajectory
                return new_solution
        
        # If all attempts failed, return original solution
        return current_solution
    
    def _modify_trajectory_segment(self, trajectory):
        """Modify a segment of the trajectory by rerouting."""
        if len(trajectory.spatial_path) < 3:
            return None
        
        # Select random segment to reroute
        start_idx = random.randint(0, len(trajectory.spatial_path) - 3)
        end_idx = random.randint(start_idx + 2, len(trajectory.spatial_path) - 1)
        
        start_node = trajectory.spatial_path[start_idx]
        end_node = trajectory.spatial_path[end_idx]
        start_time = trajectory.time_points[start_idx]
        
        try:
            # Find alternative path segment
            new_segment = nx.shortest_path(self.G, source=start_node, 
                                         target=end_node, weight='weight')
            
            # Reconstruct spatial path
            new_spatial_path = (trajectory.spatial_path[:start_idx] + 
                              new_segment + 
                              trajectory.spatial_path[end_idx + 1:])
            
            # Recompute timing for modified path
            return self._recompute_trajectory_timing(
                trajectory.drone_id, new_spatial_path, start_time, trajectory.speeds[0]
            )
            
        except nx.NetworkXNoPath:
            return None
    
    def _modify_trajectory_speed(self, trajectory):
        """Modify trajectory by changing cruise speed."""
        # Get current drone model (simplified - assume first speed is representative)
        current_speed = trajectory.speeds[0] if trajectory.speeds else 15.0
        
        # Vary speed by ±40% (more aggressive)
        speed_factor = random.uniform(0.6, 1.4)
        new_speed = max(5.0, min(35.0, current_speed * speed_factor))  # Clamp to reasonable range
        
        # Recompute timing with new speed
        return self._recompute_trajectory_timing(
            trajectory.drone_id, trajectory.spatial_path, 
            trajectory.time_points[0], new_speed
        )
    
    def _modify_departure_time(self, trajectory):
        """Modify trajectory by changing departure time."""
        # Vary departure time by ±30 seconds (more aggressive)
        time_shift = random.uniform(-30.0, 30.0)
        new_departure_time = max(0.0, trajectory.time_points[0] + time_shift)
        
        # Recompute timing with new departure time
        current_speed = trajectory.speeds[0] if trajectory.speeds else 15.0
        return self._recompute_trajectory_timing(
            trajectory.drone_id, trajectory.spatial_path, 
            new_departure_time, current_speed
        )
    
    def _recompute_trajectory_timing(self, drone_id, spatial_path, start_time, speed):
        """Recompute timing for a spatial path."""
        time_points = [start_time]
        speeds = []
        current_time = start_time
        
        for i in range(len(spatial_path) - 1):
            current_node = spatial_path[i]
            next_node = spatial_path[i + 1]
            
            try:
                edge_distance = self.G[current_node][next_node]['weight']
                travel_time = edge_distance / speed
                
                current_time += travel_time
                time_points.append(current_time)
                speeds.append(speed)
                
            except KeyError:
                return None  # Invalid path
        
        return TemporalTrajectory(
            drone_id=drone_id,
            spatial_path=spatial_path,
            time_points=time_points,
            speeds=speeds
        )
    
    def _try_conflict_resolution(self, trajectory, conflicts):
        """Attempt to resolve conflicts by minor adjustments."""
        # Simple conflict resolution: try delaying departure by 1-5 seconds
        for delay in [1.0, 2.0, 3.0, 5.0]:
            adjusted_trajectory = self._recompute_trajectory_timing(
                trajectory.drone_id,
                trajectory.spatial_path,
                trajectory.time_points[0] + delay,
                trajectory.speeds[0] if trajectory.speeds else 15.0
            )
            
            if adjusted_trajectory:
                # Check if conflicts are resolved
                test_conflicts = self.reservation_table.check_trajectory_conflicts(
                    trajectory.drone_id, adjusted_trajectory
                )
                
                if not test_conflicts:
                    return adjusted_trajectory
        
        return None  # Could not resolve conflicts

    def simulated_annealing_with_tracking(self, uav_routes, initial_temp=100000, 
                                        cooling_rate=0.99, min_temp=0.001):
        """SA algorithm with hybrid temporal conflict resolution."""
        print("\n=== PHASE 2: SIMULATED ANNEALING OPTIMIZATION ===")
        
        # Reset tracker
        self.tracker.reset()
        
        # Generate initial feasible solution
        current_trajectories = self.generate_initial_solution(uav_routes)
        if not current_trajectories:
            return None, float('inf'), self.tracker

        # Initialize reservation table with current solution
        self.reservation_table.clear()
        for trajectory in current_trajectories:
            self.reservation_table.add_trajectory(trajectory.drone_id, trajectory)

        best_trajectories = current_trajectories
        current_cost = self.path_cost(current_trajectories)
        best_cost = current_cost
        temperature = initial_temp
        iterations = 0
        stagnation_counter = 0
        max_stagnation = 500  # Stop if no improvement for 500 iterations

        print(f"Initial cost: {current_cost:.2f}")
        print(f"Starting SA optimization...")
        
        while temperature > min_temp and stagnation_counter < max_stagnation and iterations < 5000:
            # Get neighboring solution using hybrid approach
            new_trajectories = self.get_neighbor(current_trajectories)
            
            # Only evaluate cost if neighbor is different (no conflicts)
            if new_trajectories != current_trajectories:
                new_cost = self.path_cost(new_trajectories)
                
                # Acceptance decision
                delta = new_cost - current_cost
                accepted = False
                
                if delta < 0:
                    # Always accept better solutions
                    current_trajectories = new_trajectories
                    current_cost = new_cost
                    accepted = True
                    stagnation_counter = 0  # Reset stagnation
                elif random.uniform(0, 1) < math.exp(-delta / temperature):
                    # Accept worse solutions with probability (now accepts solutions with conflicts)
                    current_trajectories = new_trajectories
                    current_cost = new_cost
                    accepted = True
                else:
                    stagnation_counter += 1
                
                # Update best solution
                if current_cost < best_cost:
                    best_trajectories = current_trajectories
                    best_cost = current_cost
                    print(f"New best at iteration {iterations}: {best_cost:.2f}")
                
                # Record convergence data
                self.tracker.record(iterations, temperature, current_cost, best_cost, accepted)
                
                # Progress reporting
                if iterations % 50 == 0:
                    conflicts = self.event_validator.validate_solution(current_trajectories, self.G)
                    conflict_count = len(conflicts)
                    
                    print(f"Iter {iterations:>4}: T={temperature:>8.4f}, "
                          f"Cost={current_cost:>8.2f}, Best={best_cost:>8.2f}, "
                          f"Conflicts={conflict_count}")
                    
                    # Print reservation table statistics
                    if iterations % 200 == 0:
                        stats = self.reservation_table.get_occupancy_statistics()
                        print(f"  Reservations: {stats['total_reservations']}, "
                              f"Time span: {stats['time_span']}, "
                              f"Memory: {stats['memory_usage_mb']:.1f}MB")
            else:
                # No valid neighbor found
                stagnation_counter += 1
            
            # Cool temperature
            temperature *= cooling_rate
            iterations += 1
            
            # Adaptive cooling - slow down cooling if making progress
            if stagnation_counter < 10:
                temperature *= 1.01  # Slight heating to explore more
        
        # Final validation
        print(f"\n=== PHASE 3: FINAL VALIDATION ===")
        final_conflicts = self.event_validator.validate_solution(best_trajectories, self.G)
        
        if final_conflicts:
            print(f"⚠ Warning: Final solution has {len(final_conflicts)} conflicts")
            self.event_validator.print_conflict_summary(final_conflicts)
        else:
            print("✓ Final solution is conflict-free!")
        
        print(f"Optimization completed: {iterations} iterations")
        print(f"Final cost: {best_cost:.2f}")
        print(f"Stagnation counter: {stagnation_counter}")
        
        return best_trajectories, best_cost, self.tracker

def generate_uav_routes(graph, num_drones):
    """Generate UAV routes with drone models and timing for testing."""
    available_nodes = [node for node in graph.nodes() if graph.nodes[node]['available']]
    
    # Get edge nodes for realistic cross-airspace routes
    max_lat_idx = max(node[0] for node in available_nodes)
    left_edge = [node for node in available_nodes if node[0] == 0]
    right_edge = [node for node in available_nodes if node[0] == max_lat_idx]
    
    # Also get some internal routes for variety
    center_nodes = [node for node in available_nodes if node[0] == max_lat_idx // 2]
    
    routes = []
    drone_models = list(DRONE_MODELS.keys())
    
    for i in range(min(num_drones, len(left_edge), len(right_edge))):
        # Mix of edge-to-edge and internal routes
        if i < len(left_edge) and i < len(right_edge):
            origin = left_edge[i % len(left_edge)]
            destination = right_edge[i % len(right_edge)]
        elif i < len(center_nodes) and i < len(right_edge):
            origin = center_nodes[i % len(center_nodes)]
            destination = right_edge[i % len(right_edge)]
        else:
            # Random routes
            origin = random.choice(available_nodes)
            destination = random.choice(available_nodes)
            if origin == destination:
                continue
        
        # Verify path exists
        try:
            nx.shortest_path(graph, source=origin, target=destination, weight='weight')
            
            # Assign drone model (weighted towards more common models)
            model_weights = [3, 3, 1, 2, 1]  # Favor DJI models
            drone_model = np.random.choice(drone_models, p=np.array(model_weights)/sum(model_weights))
            
            # Stagger departure times slightly to reduce initial conflicts
            departure_time = i * 2.0  # 2 second intervals
            
            routes.append({
                'origin': origin,
                'destination': destination,
                'drone_model': drone_model,
                'departure_time': departure_time
            })
            
        except nx.NetworkXNoPath:
            continue
    
    print(f"\n=== Generated {len(routes)} UAV routes ===")
    for i, route in enumerate(routes):
        model_name = DRONE_MODELS[route['drone_model']].model_name
        print(f"  Route {i+1}: {route['origin']} -> {route['destination']}")
        print(f"    Model: {model_name} ({route['drone_model']})")
        print(f"    Departure: {route['departure_time']:.1f}s")
        print(f"    Max Speed: {DRONE_MODELS[route['drone_model']].max_speed:.1f} m/s")
    
    return routes

def plot_convergence(tracker, num_drones, graph_dims, final_trajectories=None, spatial_graph=None):
    """Plot convergence analysis with temporal trajectory information."""
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
    
    # Plot 1: Cost vs Iterations
    if tracker.iterations and tracker.current_costs:
        ax1.plot(tracker.iterations, tracker.current_costs, 'b-', linewidth=2, label='Current Cost', alpha=0.7)
        ax1.plot(tracker.iterations, tracker.best_costs, 'r-', linewidth=3, label='Best Cost')
        
        # Add shortest path cost as horizontal reference line
        if tracker.shortest_path_cost is not None:
            ax1.axhline(y=tracker.shortest_path_cost, color='green', linestyle='--', 
                   linewidth=2, label=f'Shortest Path Distance ({tracker.shortest_path_cost:.0f}m)')
    
    ax1.set_xlabel('Iterations')
    ax1.set_ylabel('Cost')
    ax1.set_title('Cost vs Iterations (Temporal SA)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Temperature vs Cost (if data available)
    if tracker.temperatures and tracker.current_costs:
        # Filter out infinite costs for better visualization
        finite_costs = []
        finite_temps = []
        finite_iters = []
        
        for i, (temp, cost, iter_num) in enumerate(zip(tracker.temperatures, tracker.current_costs, tracker.iterations)):
            if cost != float('inf') and cost < 1e6:  # Reasonable upper bound
                finite_costs.append(cost)
                finite_temps.append(temp)
                finite_iters.append(iter_num)
        
        if finite_costs:
            scatter = ax2.scatter(finite_temps, finite_costs, c=finite_iters, 
                        cmap='viridis', alpha=0.6, s=15)
            
            # Add shortest path cost reference line
            if tracker.shortest_path_cost is not None:
                ax2.axhline(y=tracker.shortest_path_cost, color='green', linestyle='--', 
                           linewidth=2, alpha=0.8)
            
            plt.colorbar(scatter, ax=ax2, label='Iteration')
    
    ax2.set_xlabel('Temperature')
    ax2.set_ylabel('Current Cost')
    ax2.set_title('Temperature vs Cost (Feasible Solutions Only)')
    ax2.set_xscale('log')
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Temperature Cooling Schedule
    if tracker.temperatures and tracker.iterations:
        ax3.plot(tracker.iterations, tracker.temperatures, 'g-', linewidth=2)
    ax3.set_xlabel('Iterations')
    ax3.set_ylabel('Temperature')
    ax3.set_title('Temperature Cooling Schedule')
    ax3.set_yscale('log')
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Solution Quality Metrics
    if final_trajectories:
        # Analyze final solution using accurate distance calculation
        total_flight_time = max(traj.get_duration() for traj in final_trajectories)
        total_distance = sum(traj.get_total_distance(spatial_graph) for traj in final_trajectories)
        avg_speed = np.mean([np.mean(traj.speeds) for traj in final_trajectories if traj.speeds])
        
        # Create bar chart of solution metrics
        metrics = ['Total Distance (km)', 'Max Flight Time (min)', 'Avg Speed (m/s)', 'Num Drones']
        values = [total_distance/1000, total_flight_time/60, avg_speed, len(final_trajectories)]
        
        bars = ax4.bar(metrics, values, color=['skyblue', 'lightcoral', 'lightgreen', 'gold'])
        ax4.set_title('Final Solution Metrics')
        ax4.set_ylabel('Value')
        
        # Add value labels on bars
        for bar, value in zip(bars, values):
            height = bar.get_height()
            ax4.text(bar.get_x() + bar.get_width()/2., height + height*0.01,
                    f'{value:.1f}', ha='center', va='bottom')
        
        ax4.grid(True, alpha=0.3, axis='y')
        
    else:
        # Fallback: Acceptance Rate
        if tracker.accepted_moves:
            window_size = max(20, len(tracker.accepted_moves) // 30)
            acceptance_rate = []
            iterations_windowed = []
            
            for i in range(window_size, len(tracker.accepted_moves)):
                window = tracker.accepted_moves[i-window_size:i]
                accepted_count = sum(1 for move in window if move)
                rate = (accepted_count / len(window)) * 100
                acceptance_rate.append(rate)
                iterations_windowed.append(tracker.iterations[i - window_size//2] if i - window_size//2 < len(tracker.iterations) else i)
            
            if acceptance_rate:
                ax4.plot(iterations_windowed, acceptance_rate, 'm-', linewidth=2)
            
            ax4.set_xlabel('Iterations')
            ax4.set_ylabel('Acceptance Rate (%)')
            ax4.set_title(f'Acceptance Rate (Window: {window_size})')
            ax4.set_ylim(0, 100)
            ax4.grid(True, alpha=0.3)
    
    plt.suptitle(f'Hybrid Temporal SA Analysis - {num_drones} Drones, Graph: {graph_dims[0]}x{graph_dims[1]}x{graph_dims[2]}', 
                 fontsize=16, fontweight='bold')
    plt.tight_layout()
    
    # Save plot
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"hybrid_temporal_sa_{num_drones}drones_{timestamp}.png"
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Convergence plot saved to {filename}")
    
    plt.show()

def print_solution_summary(final_trajectories, conflicts, execution_time, spatial_graph=None):
    """Print detailed summary of the final solution."""
    print("\n" + "="*60)
    print("HYBRID TEMPORAL SOLUTION SUMMARY")
    print("="*60)
    
    if not final_trajectories:
        print("❌ No solution found")
        return
    
    print(f"✓ Solution found for {len(final_trajectories)} drones")
    print(f"⏱ Total execution time: {execution_time:.2f} seconds")
    
    # Conflict analysis
    if conflicts:
        print(f"⚠ {len(conflicts)} conflicts detected:")
        conflict_types = {}
        for conflict in conflicts:
            conflict_type = conflict['type']
            conflict_types[conflict_type] = conflict_types.get(conflict_type, 0) + 1
        
        for ctype, count in conflict_types.items():
            print(f"  - {ctype}: {count}")
    else:
        print("✅ Solution is conflict-free!")
    
    # Flight statistics using accurate distance calculation
    total_distance = sum(traj.get_total_distance(spatial_graph) for traj in final_trajectories)
    total_flight_time = max(traj.get_duration() for traj in final_trajectories)
    avg_flight_time = np.mean([traj.get_duration() for traj in final_trajectories])
    
    print(f"\nFlight Statistics:")
    print(f"  Total distance: {total_distance/1000:.2f} km")
    print(f"  Makespan (max flight time): {total_flight_time/60:.2f} minutes")
    print(f"  Average flight time: {avg_flight_time/60:.2f} minutes")
    
    # Speed analysis
    all_speeds = []
    for traj in final_trajectories:
        if traj.speeds:
            all_speeds.extend(traj.speeds)
    
    if all_speeds:
        print(f"  Average speed: {np.mean(all_speeds):.1f} m/s")
        print(f"  Speed range: {np.min(all_speeds):.1f} - {np.max(all_speeds):.1f} m/s")
    
    # Per-drone details
    print(f"\nPer-drone details:")
    for i, traj in enumerate(final_trajectories):
        print(f"  Drone {i}: {len(traj.spatial_path)} nodes, "
              f"{traj.get_duration():.1f}s flight time, "
              f"{traj.get_total_distance():.0f}m distance")
    
    print("="*60)

def validate_solution_consistency(temporal_trajectories, spatial_graph=None):
    """
    Validate consistency of distance calculations across all trajectories.
    
    Args:
        temporal_trajectories: List of TemporalTrajectory objects
        spatial_graph: NetworkX graph for accurate calculations
        
    Returns:
        dict: Validation summary
    """
    if not spatial_graph:
        return {"status": "skipped", "reason": "No graph provided"}
    
    print(f"\n📊 DISTANCE CALCULATION VALIDATION:")
    print(f"{'Drone':<6} {'Accurate(m)':<12} {'Approx(m)':<12} {'Diff(%)':<10} {'Status':<12}")
    print(f"{'-'*6} {'-'*12} {'-'*12} {'-'*10} {'-'*12}")
    
    validation_results = []
    total_accurate = 0
    total_approximate = 0
    
    for traj in temporal_trajectories:
        result = traj.validate_distance_consistency(spatial_graph)
        validation_results.append(result)
        
        if result["status"] != "skipped":
            accurate = result["accurate_distance"]
            approximate = result["approximate_distance"]
            diff_pct = result["relative_difference"] * 100
            status = "✓ GOOD" if result["status"] == "consistent" else "⚠ ISSUE"
            
            print(f"{traj.drone_id:<6} {accurate:<12.1f} {approximate:<12.1f} {diff_pct:<10.1f} {status:<12}")
            
            total_accurate += accurate
            total_approximate += approximate
    
    if total_accurate > 0:
        overall_diff = abs(total_accurate - total_approximate) / total_accurate * 100
        print(f"\n📈 OVERALL VALIDATION:")
        print(f"  Total Accurate Distance: {total_accurate:.1f}m")
        print(f"  Total Approximate Distance: {total_approximate:.1f}m") 
        print(f"  Overall Difference: {overall_diff:.1f}%")
        
        status = "CONSISTENT" if overall_diff <= 10.0 else "INCONSISTENT"
        print(f"  Validation Status: {status}")
        
        return {
            "status": "completed",
            "overall_difference_pct": overall_diff,
            "total_accurate": total_accurate,
            "total_approximate": total_approximate,
            "individual_results": validation_results
        }
    else:
        return {"status": "failed", "reason": "No valid distance calculations"}

def print_detailed_routes(temporal_trajectories, spatial_graph):
    """Print detailed route information for each drone."""
    print("\n" + "="*80)
    print("DETAILED ROUTE INFORMATION")
    print("="*80)
    
    for i, trajectory in enumerate(temporal_trajectories):
        print(f"\n🚁 DRONE {i} ROUTE DETAILS:")
        print(f"   Drone ID: {trajectory.drone_id}")
        print(f"   Total Nodes: {len(trajectory.spatial_path)}")
        print(f"   Flight Duration: {trajectory.get_duration():.2f} seconds ({trajectory.get_duration()/60:.2f} minutes)")
        print(f"   Total Distance: {trajectory.get_total_distance(spatial_graph):.2f} meters")
        
        if trajectory.speeds:
            avg_speed = np.mean(trajectory.speeds)
            min_speed = np.min(trajectory.speeds)
            max_speed = np.max(trajectory.speeds)
            print(f"   Average Speed: {avg_speed:.2f} m/s")
            print(f"   Speed Range: {min_speed:.2f} - {max_speed:.2f} m/s")
        
        print(f"\n   📍 WAYPOINT DETAILS:")
        print(f"   {'#':<3} {'Node':<12} {'Lat':<10} {'Lon':<11} {'Alt':<8} {'Time(s)':<8} {'Speed(m/s)':<10} {'Segment(m)':<12}")
        print(f"   {'-'*3} {'-'*12} {'-'*10} {'-'*11} {'-'*8} {'-'*8} {'-'*10} {'-'*12}")
        
        for j, (node, time_point) in enumerate(zip(trajectory.spatial_path, trajectory.time_points)):
            # Get geographic position
            if node in spatial_graph.nodes:
                pos = spatial_graph.nodes[node]['pos']
                lat, lon, alt = pos[0], pos[1], pos[2]
            else:
                lat, lon, alt = 0.0, 0.0, 0.0
            
            # Get speed for this segment
            if j < len(trajectory.speeds):
                speed = trajectory.speeds[j]
            else:
                speed = 0.0
            
            # Calculate segment distance
            if j > 0:
                prev_node = trajectory.spatial_path[j-1]
                try:
                    segment_dist = spatial_graph[prev_node][node]['weight']
                except (KeyError, IndexError):
                    segment_dist = 0.0
            else:
                segment_dist = 0.0
            
            print(f"   {j+1:<3} {str(node):<12} {lat:<10.6f} {lon:<11.6f} {alt:<8.3f} {time_point:<8.2f} {speed:<10.2f} {segment_dist:<12.2f}")
        
        # Print edge information
        print(f"\n   🔗 EDGE TRAVERSAL DETAILS:")
        print(f"   {'Segment':<8} {'From Node':<12} {'To Node':<12} {'Distance(m)':<12} {'Time(s)':<8} {'Speed(m/s)':<10}")
        print(f"   {'-'*8} {'-'*12} {'-'*12} {'-'*12} {'-'*8} {'-'*10}")
        
        for j in range(len(trajectory.spatial_path) - 1):
            from_node = trajectory.spatial_path[j]
            to_node = trajectory.spatial_path[j + 1]
            
            # Get edge weight
            try:
                edge_weight = spatial_graph[from_node][to_node]['weight']
            except (KeyError, IndexError):
                edge_weight = 0.0
            
            # Calculate segment time and speed
            segment_time = trajectory.time_points[j + 1] - trajectory.time_points[j]
            segment_speed = trajectory.speeds[j] if j < len(trajectory.speeds) else 0.0
            
            print(f"   {j+1:<8} {str(from_node):<12} {str(to_node):<12} {edge_weight:<12.2f} {segment_time:<8.2f} {segment_speed:<10.2f}")
        
        print(f"\n   📊 ROUTE SUMMARY:")
        print(f"   - Start Position: {trajectory.spatial_path[0]} at {trajectory.time_points[0]:.2f}s")
        print(f"   - End Position: {trajectory.spatial_path[-1]} at {trajectory.time_points[-1]:.2f}s")
        print(f"   - Total Segments: {len(trajectory.spatial_path) - 1}")
        
        if trajectory.speeds:
            print(f"   - Speed Statistics:")
            print(f"     * Mean: {np.mean(trajectory.speeds):.2f} m/s")
            print(f"     * Std Dev: {np.std(trajectory.speeds):.2f} m/s")
            print(f"     * Min: {np.min(trajectory.speeds):.2f} m/s")
            print(f"     * Max: {np.max(trajectory.speeds):.2f} m/s")
        
        print(f"   {'='*70}")
    
    # Print overall route comparison
    print(f"\n📈 COMPARATIVE ROUTE ANALYSIS:")
    print(f"{'Drone':<6} {'Distance(m)':<12} {'Duration(s)':<12} {'Avg Speed(m/s)':<15} {'Efficiency':<10}")
    print(f"{'-'*6} {'-'*12} {'-'*12} {'-'*15} {'-'*10}")
    
    total_distance = sum(traj.get_total_distance(spatial_graph) for traj in temporal_trajectories)
    total_time = sum(traj.get_duration() for traj in temporal_trajectories)
    
    for i, trajectory in enumerate(temporal_trajectories):
        distance = trajectory.get_total_distance(spatial_graph)
        duration = trajectory.get_duration()
        avg_speed = np.mean(trajectory.speeds) if trajectory.speeds else 0.0
        
        # Efficiency metric (distance per unit time)
        efficiency = distance / duration if duration > 0 else 0.0
        
        print(f"{i:<6} {distance:<12.2f} {duration:<12.2f} {avg_speed:<15.2f} {efficiency:<10.2f}")
    
    print(f"\n📋 FLEET SUMMARY:")
    print(f"   Total Fleet Distance: {total_distance:.2f} meters ({total_distance/1000:.3f} km)")
    print(f"   Total Fleet Time: {total_time:.2f} seconds ({total_time/60:.2f} minutes)")
    print(f"   Average Distance per Drone: {total_distance/len(temporal_trajectories):.2f} meters")
    print(f"   Average Duration per Drone: {total_time/len(temporal_trajectories):.2f} seconds")
    print(f"   Fleet Makespan (max completion time): {max(traj.get_duration() for traj in temporal_trajectories):.2f} seconds")
    
    print("="*80)

def create_3d_route_visualization(graph, temporal_trajectories, title="3D UAV Routes Visualization"):
    """
    Create 3D visualization of the graph and drone routes using Plotly.
    Similar to D_graph_test.ipynb visualization method.
    """
    try:
        import plotly.graph_objects as go
    except ImportError:
        print("Plotly not available. Install with: pip install plotly")
        return None
    
    # Get node positions and availability
    pos = nx.get_node_attributes(graph, 'pos')
    availability = nx.get_node_attributes(graph, 'available')
    
    # Create lists for nodes
    node_x = [pos[node][1] for node in graph.nodes() if node in pos]  # Longitude
    node_y = [pos[node][0] for node in graph.nodes() if node in pos]  # Latitude  
    node_z = [pos[node][2] for node in graph.nodes() if node in pos]  # Altitude
    node_colors = ['green' if availability.get(node, True) else 'red' for node in graph.nodes() if node in pos]
    node_text = [f"Node: {node}<br>Lat: {pos[node][0]:.6f}<br>Lon: {pos[node][1]:.6f}<br>Alt: {pos[node][2]:.1f}m" 
                 for node in graph.nodes() if node in pos]
    
    # Create lists for graph edges (background network) - sample edges for performance
    edge_x, edge_y, edge_z = [], [], []
    edge_sample = list(graph.edges())[:min(1000, len(graph.edges()))]  # Limit for performance
    
    for edge in edge_sample:
        if edge[0] in pos and edge[1] in pos:
            x0, y0, z0 = pos[edge[0]][1], pos[edge[0]][0], pos[edge[0]][2]
            x1, y1, z1 = pos[edge[1]][1], pos[edge[1]][0], pos[edge[1]][2]
            
            edge_x.extend([x0, x1, None])
            edge_y.extend([y0, y1, None])
            edge_z.extend([z0, z1, None])
    
    # Create traces
    traces = []
    
    # Background graph edges
    edge_trace = go.Scatter3d(
        x=edge_x, y=edge_y, z=edge_z,
        line=dict(width=1, color='lightgray'),
        hoverinfo='none',
        mode='lines',
        name='Graph Edges',
        opacity=0.3
    )
    traces.append(edge_trace)
    
    # Graph nodes
    node_trace = go.Scatter3d(
        x=node_x, y=node_y, z=node_z,
        mode='markers',
        hoverinfo='text',
        marker=dict(size=2, color=node_colors, opacity=0.4),
        text=node_text,
        name='Available Nodes'
    )
    traces.append(node_trace)
    
    # Drone routes
    if temporal_trajectories:
        drone_colors = ['red', 'blue', 'orange', 'purple', 'brown', 'pink', 'cyan', 'magenta']
        
        for i, trajectory in enumerate(temporal_trajectories):
            drone_color = drone_colors[i % len(drone_colors)]
            
            # Route coordinates
            route_x, route_y, route_z = [], [], []
            route_text = []
            
            for j, (node, time_point) in enumerate(zip(trajectory.spatial_path, trajectory.time_points)):
                if node in pos:
                    route_x.append(pos[node][1])  # Longitude
                    route_y.append(pos[node][0])  # Latitude
                    route_z.append(pos[node][2])  # Altitude
                    
                    speed = trajectory.speeds[j] if j < len(trajectory.speeds) else 0.0
                    route_text.append(
                        f"Drone {i} (ID: {trajectory.drone_id})<br>" +
                        f"Waypoint: {j+1}/{len(trajectory.spatial_path)}<br>" +
                        f"Time: {time_point:.2f}s<br>" +
                        f"Speed: {speed:.1f} m/s<br>" +
                        f"Node: {node}<br>" +
                        f"Lat: {pos[node][0]:.6f}<br>" +
                        f"Lon: {pos[node][1]:.6f}<br>" +
                        f"Alt: {pos[node][2]:.1f}m"
                    )
            
            # Route line
            if route_x:
                route_trace = go.Scatter3d(
                    x=route_x, y=route_y, z=route_z,
                    mode='lines+markers',
                    line=dict(width=6, color=drone_color),
                    marker=dict(size=5, color=drone_color),
                    hoverinfo='text',
                    text=route_text,
                    name=f'Drone {i} (ID: {trajectory.drone_id})',
                    opacity=0.9
                )
                traces.append(route_trace)
                
                # Start marker (green diamond)
                start_trace = go.Scatter3d(
                    x=[route_x[0]], y=[route_y[0]], z=[route_z[0]],
                    mode='markers',
                    marker=dict(size=10, color='green', symbol='diamond'),
                    hoverinfo='text',
                    text=[f"START - Drone {i}<br>Node: {trajectory.spatial_path[0]}<br>Time: {trajectory.time_points[0]:.2f}s"],
                    name=f'Start {i}',
                    showlegend=False
                )
                traces.append(start_trace)
                
                # End marker (red X)
                end_trace = go.Scatter3d(
                    x=[route_x[-1]], y=[route_y[-1]], z=[route_z[-1]],
                    mode='markers',
                    marker=dict(size=10, color='darkred', symbol='x'),
                    hoverinfo='text',
                    text=[f"END - Drone {i}<br>Node: {trajectory.spatial_path[-1]}<br>Time: {trajectory.time_points[-1]:.2f}s"],
                    name=f'End {i}',
                    showlegend=False
                )
                traces.append(end_trace)
    
    # Create figure
    fig = go.Figure(data=traces)
    
    # Layout optimized for full browser screen usage
    fig.update_layout(
        title={
            'text': title,
            'x': 0.5,
            'xanchor': 'center',
            'font': {'size': 20}
        },
        scene=dict(
            xaxis_title='Longitude',
            yaxis_title='Latitude', 
            zaxis_title='Altitude (m)',
            camera=dict(
                eye=dict(x=1.3, y=1.3, z=1.2)
            ),
            aspectmode='cube'
        ),
        # Use relative sizing for responsive full-screen display
        width=None,  # Remove fixed width to use full browser width
        height=None,  # Remove fixed height to use full browser height
        autosize=True,  # Enable automatic sizing
        showlegend=True,
        legend=dict(
            x=0.02,
            y=0.98,
            bgcolor='rgba(255,255,255,0.8)',
            font=dict(size=12)
        ),
        # Add margin settings to maximize plot area
        margin=dict(
            l=20,    # Left margin
            r=20,    # Right margin  
            t=60,    # Top margin (space for title)
            b=20,    # Bottom margin
            pad=4    # Padding
        ),
        # Configure HTML div to use viewport dimensions
        paper_bgcolor='white',
        plot_bgcolor='white'
    )
    
    return fig

def visualize_routes_3d(graph, temporal_trajectories):
    """Generate and display 3D route visualization."""
    print("\n" + "="*60)
    print("3D ROUTE VISUALIZATION")
    print("="*60)
    
    if not temporal_trajectories:
        print("❌ No trajectories to visualize")
        return
    
    fig = create_3d_route_visualization(
        graph, 
        temporal_trajectories,
        title=f"SA Optimized UAV Routes - {len(temporal_trajectories)} Drones"
    )
    
    if fig:
        print(f"📊 Displaying 3D visualization of {len(temporal_trajectories)} drone routes...")
        
        # Save HTML file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"sa_routes_3d_{len(temporal_trajectories)}drones_{timestamp}.html"
        
        try:
            # Write HTML with full viewport configuration
            fig.write_html(
                filename,
                config={
                    'displayModeBar': True,
                    'displaylogo': False,
                    'modeBarButtonsToRemove': ['pan2d', 'select2d', 'lasso2d'],
                    'responsive': True
                },
                div_id="plotly-div",
                include_plotlyjs=True,
                full_html=True,
                # Add custom CSS for full viewport usage
                post_script="""
                <style>
                    body { margin: 0; padding: 0; height: 100vh; overflow: hidden; }
                    #plotly-div { width: 100vw; height: 100vh; }
                    .plotly-graph-div { width: 100% !important; height: 100% !important; }
                </style>
                <script>
                    // Ensure plot resizes with window
                    window.addEventListener('resize', function() {
                        Plotly.Plots.resize('plotly-div');
                    });
                </script>
                """
            )
            print(f"💾 Full-screen 3D visualization saved to: {filename}")
        except Exception as e:
            print(f"⚠️ Could not save file: {e}")
        
        # Display
        try:
            fig.show()
            print("✅ 3D visualization displayed successfully!")
        except Exception as e:
            print(f"⚠️ Could not display: {e}")
            print("💡 Open the saved HTML file in your browser to view the visualization")
        
        # Print route summary
        print(f"\n📈 Visualization Summary:")
        print(f"   Nodes displayed: {len(graph.nodes())}")
        print(f"   Edges displayed: {min(1000, len(graph.edges()))}")
        print(f"   Drone routes: {len(temporal_trajectories)}")
        
        total_distance = sum(traj.get_total_distance() for traj in temporal_trajectories)
        max_time = max(traj.get_duration() for traj in temporal_trajectories)
        print(f"   Total route distance: {total_distance:.2f}m")
        print(f"   Fleet makespan: {max_time:.2f}s")
        
    else:
        print("❌ Could not create visualization (Plotly may not be installed)")
        print("💡 Install Plotly with: pip install plotly")
    
    print("="*60)

def main():
    """Main function for hybrid temporal SA testing."""
    print("HYBRID TEMPORAL SIMULATED ANNEALING TEST")
    print("="*60)
    
    # Configurable parameters
    NUM_DRONES = 40  # Reduced for better initial feasibility
    GRAPH_DIMENSIONS = (50, 50, 3)  # (n_lat, n_lon, n_alt)
    TIME_STEP_DURATION = 1.0  # seconds per time step
    
    print(f"Configuration:")
    print(f"  Number of drones: {NUM_DRONES}")
    print(f"  Graph dimensions: {GRAPH_DIMENSIONS[0]}x{GRAPH_DIMENSIONS[1]}x{GRAPH_DIMENSIONS[2]}")
    print(f"  Time step duration: {TIME_STEP_DURATION}s")
    print(f"  Drone models available: {len(DRONE_MODELS)}")
    print("="*60)
    
    # Build graph
    start_time = time.time()
    print("Building spatial graph...")
    builder = UAVGraphBuilder(n_lat=GRAPH_DIMENSIONS[0], 
                             n_lon=GRAPH_DIMENSIONS[1], 
                             n_alt=GRAPH_DIMENSIONS[2])
    graph = builder.build_graph()
    
    # Generate routes with drone models
    routes = generate_uav_routes(graph, NUM_DRONES)
    actual_drones = len(routes)
    
    if actual_drones == 0:
        print("❌ No valid routes found. Try different parameters.")
        return
    
    # Initialize hybrid optimizer
    optimizer = SimulatedAnnealingOptimizer(graph, TIME_STEP_DURATION)
    
    # Run hybrid SA optimization
    best_trajectories, final_cost, tracker = optimizer.simulated_annealing_with_tracking(
        routes, 
        initial_temp=1000,     # Lower initial temp
        cooling_rate=0.98,    # Faster cooling
        min_temp=0.01          # Higher min temp
    )
    
    total_time = time.time() - start_time
    
    # Final comprehensive validation
    if best_trajectories:
        final_conflicts = optimizer.event_validator.validate_solution(best_trajectories, graph)
    else:
        final_conflicts = []
    
    # Print detailed solution summary
    print_solution_summary(best_trajectories, final_conflicts, total_time, graph)
    
    # Validate distance calculation consistency
    if best_trajectories:
        validation_results = validate_solution_consistency(best_trajectories, graph)
    
    # Print detailed route information
    if best_trajectories:
        print_detailed_routes(best_trajectories, graph)
    
    # Performance analysis
    if tracker.iterations and tracker.current_costs:
        print(f"\nPerformance Analysis:")
        print(f"  Total iterations: {len(tracker.iterations)}")
        if len(tracker.current_costs) > 1:
            initial_cost = next((cost for cost in tracker.current_costs if cost != float('inf')), float('inf'))
            if initial_cost != float('inf') and final_cost != float('inf'):
                improvement = initial_cost - final_cost
                improvement_pct = (improvement / initial_cost * 100) if initial_cost > 0 else 0
                print(f"  Cost improvement: {improvement:.2f} ({improvement_pct:.1f}%)")
        
        # Convergence analysis
        finite_costs = [cost for cost in tracker.best_costs if cost != float('inf')]
        if len(finite_costs) > 10:
            early_avg = np.mean(finite_costs[:len(finite_costs)//4])
            late_avg = np.mean(finite_costs[-len(finite_costs)//4:])
            print(f"  Early avg cost: {early_avg:.2f}")
            print(f"  Late avg cost: {late_avg:.2f}")
            print(f"  Convergence ratio: {late_avg/early_avg:.3f}")
    
    # Shortest path comparison
    if tracker.shortest_path_cost is not None:
        print(f"\nBaseline Comparison:")
        print(f"  Shortest path distance (ignoring conflicts): {tracker.shortest_path_cost:.2f}m")
        if final_cost != float('inf'):
            if best_trajectories:
                total_distance = sum(traj.get_total_distance(graph) for traj in best_trajectories)
                distance_ratio = total_distance / tracker.shortest_path_cost
                print(f"  Hybrid solution distance: {total_distance:.2f}m")
                print(f"  Distance overhead: {(distance_ratio - 1) * 100:.1f}%")
                
                # Time efficiency - corrected calculation
                makespan = max(traj.get_duration() for traj in best_trajectories)
                avg_speed = np.mean([np.mean(traj.speeds) for traj in best_trajectories if traj.speeds])
                if avg_speed > 0:
                    theoretical_time = tracker.shortest_path_cost / avg_speed
                    time_efficiency = theoretical_time / makespan * 100
                    print(f"  Time efficiency: {time_efficiency:.1f}%")
                else:
                    print(f"  Time efficiency: Cannot calculate (zero speed)")
    
    # Memory usage analysis
    if hasattr(optimizer, 'reservation_table'):
        stats = optimizer.reservation_table.get_occupancy_statistics()
        print(f"\nMemory Usage:")
        print(f"  Reservation table entries: {stats['total_reservations']}")
        print(f"  Time span covered: {stats['time_span']} time steps")
        print(f"  Estimated memory: {stats['memory_usage_mb']:.2f} MB")
    
    # Plot results
    if best_trajectories:
        plot_convergence(tracker, actual_drones, GRAPH_DIMENSIONS, best_trajectories, graph)
    else:
        plot_convergence(tracker, actual_drones, GRAPH_DIMENSIONS, spatial_graph=graph)
    
    # Create 3D route visualization
    if best_trajectories:
        try:
            print("\n" + "="*50)
            print("GENERATING 3D VISUALIZATION")
            print("="*50)
            
            # Create enhanced 3D visualization
            visualize_routes_3d(graph, best_trajectories)
            
            # Also create the standalone visualization with all features
            drone_models_dict = {}
            for i, traj in enumerate(best_trajectories):
                drone_models_dict[traj.drone_id] = traj.drone_model
            
            fig = create_3d_route_visualization(graph, best_trajectories, drone_models_dict)
            
            # Save as HTML file with full-screen configuration
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            html_filename = f'3d_routes_comprehensive_{timestamp}.html'
            fig.write_html(
                html_filename,
                config={
                    'displayModeBar': True,
                    'displaylogo': False,
                    'modeBarButtonsToRemove': ['pan2d', 'select2d', 'lasso2d'],
                    'responsive': True
                },
                div_id="plotly-div-main",
                include_plotlyjs=True,
                full_html=True,
                # Add custom CSS and JavaScript for full viewport usage
                post_script="""
                <style>
                    body { 
                        margin: 0; 
                        padding: 0; 
                        height: 100vh; 
                        overflow: hidden; 
                        font-family: Arial, sans-serif;
                    }
                    #plotly-div-main { 
                        width: 100vw; 
                        height: 100vh; 
                        position: fixed;
                        top: 0;
                        left: 0;
                    }
                    .plotly-graph-div { 
                        width: 100% !important; 
                        height: 100% !important; 
                    }
                    /* Hide scrollbars */
                    html, body {
                        overflow: hidden;
                    }
                </style>
                <script>
                    // Ensure plot resizes with window and takes full viewport
                    window.addEventListener('resize', function() {
                        Plotly.Plots.resize('plotly-div-main');
                    });
                    // Set focus and prevent scrolling
                    window.addEventListener('load', function() {
                        document.body.style.overflow = 'hidden';
                        Plotly.Plots.resize('plotly-div-main');
                    });
                </script>
                """
            )
            print(f"✅ Full-screen comprehensive 3D visualization saved to: {html_filename}")
            
            # Try to open in browser
            try:
                import webbrowser
                import os
                webbrowser.open('file://' + os.path.realpath(html_filename))
                print("🌐 Opening comprehensive visualization in browser...")
            except Exception as e:
                print(f"Note: Could not auto-open browser: {e}")
                print(f"Please manually open: {html_filename}")
                
        except Exception as e:
            print(f"⚠ Error generating 3D visualization: {e}")
            print("Continuing without enhanced visualization...")

if __name__ == "__main__":
    main()