"""
Conflict-Based Search (CBS) Pathfinder Module

This module implements a simplified CBS algorithm for multi-agent pathfinding
where existing drone routes are frozen and only the new requesting drone can be replanned.

Key Features:
- Node-based conflict detection (10-second threshold)
- Constrained A* pathfinding considering simulation time
- Edge weight-based temporal calculations
- Integration with drone registry for constraint extraction
"""

import heapq
import networkx as nx
from typing import List, Dict, Tuple, Optional, Set
from dataclasses import dataclass


# ============================================================================
# DATA STRUCTURES
# ============================================================================

@dataclass
class NodeConstraint:
    """
    Represents a constraint that prevents a drone from being at a specific node
    during a specific time window.
    
    Attributes:
        node_id: str - Graph node ID that is constrained
        forbidden_start: float - Start of forbidden time window (seconds)
        forbidden_end: float - End of forbidden time window (seconds)
    """
    node_id: str
    forbidden_start: float
    forbidden_end: float
    
    def conflicts_with(self, node_id: str, time: float) -> bool:
        """
        Check if given node and time conflicts with this constraint.
        
        Args:
            node_id: str - Node ID to check
            time: float - Time to check (seconds)
        
        Returns:
            bool - True if there's a conflict, False otherwise
        """
        return self.node_id == node_id and self.forbidden_start <= time <= self.forbidden_end


@dataclass
class SearchNode:
    """
    Represents a node in the A* search space.
    Each node contains the current graph node and the simulation time when reached.
    
    Attributes:
        graph_node: str - Current graph node ID
        time: float - Simulation time when this node is reached (seconds)
        g_cost: float - Cost from start to this node (cumulative distance in meters)
        h_cost: float - Heuristic cost from this node to goal (estimated distance in meters)
        parent: Optional[SearchNode] - Parent node in search tree for path reconstruction
    """
    graph_node: str
    time: float
    g_cost: float
    h_cost: float
    parent: Optional['SearchNode'] = None
    
    @property
    def f_cost(self) -> float:
        """
        Total estimated cost: g_cost (actual) + h_cost (heuristic).
        
        Returns:
            float - Total estimated cost (meters)
        """
        return self.g_cost + self.h_cost
    
    def __lt__(self, other):
        """
        Comparison operator for priority queue ordering.
        Nodes with lower f_cost have higher priority.
        
        Args:
            other: SearchNode - Other node to compare
        
        Returns:
            bool - True if this node has lower f_cost
        """
        return self.f_cost < other.f_cost


# ============================================================================
# CONSTRAINT EXTRACTION FROM REGISTRY
# ============================================================================

def extract_node_constraints(registry: Dict, conflict_threshold: float = 10.0) -> List[NodeConstraint]:
    """
    Extract node constraints from active drone registry.
    Creates constraints for each node that active drones will occupy, with
    a time window expanded by the conflict threshold.
    
    Args:
        registry: Dict - Active drones registry with structure:
                        {drone_id: {"route_nodes": [str], "overfly_times": [float]}}
        conflict_threshold: float - Time threshold in seconds for conflict detection (default: 10.0)
    
    Returns:
        List[NodeConstraint] - List of node constraints extracted from registry
    """
    constraints = []  # List to store all extracted constraints (list of NodeConstraint)
    
    # Iterate through each active drone in registry
    for drone_id, route_data in registry.items():
        route_nodes = route_data.get("route_nodes", [])  # List of node IDs (list of str)
        overfly_times = route_data.get("overfly_times", [])  # List of overfly times (list of float)
        
        # Create constraint for each node in the route
        for i, node_id in enumerate(route_nodes):
            if i < len(overfly_times):
                overfly_time = overfly_times[i]  # Time when drone overflies this node (float, seconds)
                
                # Create time window: [overfly_time - threshold, overfly_time + threshold]
                # This prevents new drone from being at same node within threshold seconds
                constraint = NodeConstraint(
                    node_id=node_id,  # Node ID that is constrained (str)
                    forbidden_start=overfly_time - conflict_threshold,  # Start of forbidden window (float, seconds)
                    forbidden_end=overfly_time + conflict_threshold  # End of forbidden window (float, seconds)
                )
                constraints.append(constraint)  # Add constraint to list
    
    return constraints  # Return all extracted constraints (list of NodeConstraint)


# ============================================================================
# HEURISTIC FUNCTION
# ============================================================================

def heuristic(graph: nx.Graph, current_node: str, goal_node: str) -> float:
    """
    Calculate heuristic distance between two nodes using shortest path length.
    Uses NetworkX shortest path length with edge weights as heuristic estimate.
    
    Args:
        graph: nx.Graph - NetworkX graph with edge weights
        current_node: str - Current node ID
        goal_node: str - Goal node ID
    
    Returns:
        float - Estimated distance in meters (heuristic cost)
    """
    try:
        # Calculate shortest path length using edge weights (distance in meters)
        # This provides admissible heuristic (never overestimates)
        return nx.shortest_path_length(graph, current_node, goal_node, weight='weight')  # float, meters
    except nx.NetworkXNoPath:
        # If no path exists, return large value (infinity)
        return float('inf')  # float, meters


# ============================================================================
# CONSTRAINT CHECKING
# ============================================================================

def check_constraints(node_id: str, time: float, constraints: List[NodeConstraint]) -> bool:
    """
    Check if a node-time pair violates any constraints.
    
    Args:
        node_id: str - Graph node ID to check
        time: float - Simulation time to check (seconds)
        constraints: List[NodeConstraint] - List of constraints to check against
    
    Returns:
        bool - True if constraint is violated (conflict exists), False if valid
    """
    # Check each constraint
    for constraint in constraints:
        if constraint.conflicts_with(node_id, time):
            return True  # Conflict found - constraint violated
    
    return False  # No conflicts - node-time pair is valid


# ============================================================================
# CONSTRAINED A* PATHFINDING
# ============================================================================

def constrained_astar(
    graph: nx.Graph,
    start_node: str,
    goal_node: str,
    start_time: float,
    speed: float,
    constraints: List[NodeConstraint],
    max_iterations: int = 10000
) -> Optional[Tuple[List[str], List[float]]]:
    """
    Perform constrained A* search to find conflict-free path.
    Considers simulation time and edge weights to calculate arrival times at each node.
    
    Args:
        graph: nx.Graph - NetworkX graph with edge weights (distances in meters)
        start_node: str - Starting node ID
        goal_node: str - Goal node ID
        start_time: float - Simulation time when route starts (seconds)
        speed: float - Drone speed in m/s
        constraints: List[NodeConstraint] - List of node constraints to avoid
        max_iterations: int - Maximum iterations to prevent infinite loops (default: 10000)
    
    Returns:
        Optional[Tuple[List[str], List[float]]] - Tuple of (path_nodes, overfly_times) if path found, None otherwise
    """
    # Check if start node violates constraints
    if check_constraints(start_node, start_time, constraints):
        # Start node is occupied - need to wait or find alternative
        # Try delaying start by small increments
        delayed_start = start_time  # Current delayed start time (float, seconds)
        max_delay = 60.0  # Maximum delay to try (float, seconds)
        delay_increment = 1.0  # Delay increment (float, seconds)
        
        # Try to find valid start time
        while delayed_start < start_time + max_delay:
            delayed_start += delay_increment
            if not check_constraints(start_node, delayed_start, constraints):
                start_time = delayed_start  # Use delayed start time
                break
        else:
            # Could not find valid start time within delay limit
            return None  # No valid path found
    
    # Initialize open set (priority queue) with start node
    open_set = []  # Priority queue: [(f_cost, SearchNode)]
    start_search_node = SearchNode(
        graph_node=start_node,  # Starting graph node ID (str)
        time=start_time,  # Start simulation time (float, seconds)
        g_cost=0.0,  # Cost from start (float, meters)
        h_cost=heuristic(graph, start_node, goal_node),  # Heuristic to goal (float, meters)
        parent=None  # No parent (start node)
    )
    heapq.heappush(open_set, (start_search_node.f_cost, start_search_node))  # Add to priority queue
    
    # Initialize closed set to track visited nodes
    closed_set: Set[Tuple[str, float]] = set()  # Set of (node_id, time) tuples already explored
    
    iteration_count = 0  # Iteration counter (int)
    
    # Main A* search loop
    while open_set and iteration_count < max_iterations:
        iteration_count += 1  # Increment iteration counter
        
        # Get node with lowest f_cost from priority queue
        _, current = heapq.heappop(open_set)  # SearchNode with lowest f_cost
        
        # Check if we've reached the goal
        if current.graph_node == goal_node:
            # Reconstruct path by following parent pointers
            path_nodes = []  # List of node IDs in path (list of str)
            overfly_times = []  # List of overfly times (list of float)
            
            # Trace back from goal to start
            node = current  # Current node in path reconstruction (SearchNode)
            while node is not None:
                path_nodes.append(node.graph_node)  # Add node to path
                overfly_times.append(node.time)  # Add time to overfly times
                node = node.parent  # Move to parent node
            
            # Reverse to get path from start to goal
            path_nodes.reverse()  # Reverse list: goal → start becomes start → goal
            overfly_times.reverse()  # Reverse times to match path order
            
            return (path_nodes, overfly_times)  # Return path and overfly times
        
        # Check if this node-time pair has already been explored
        node_time_key = (current.graph_node, current.time)  # Unique key for node-time pair (tuple)
        if node_time_key in closed_set:
            continue  # Skip if already explored
        
        # Mark as explored
        closed_set.add(node_time_key)
        
        # Explore neighbors of current node
        for neighbor_node in graph.neighbors(current.graph_node):
            # Get edge weight (distance in meters)
            if graph.has_edge(current.graph_node, neighbor_node):
                edge_weight = graph[current.graph_node][neighbor_node].get('weight', 0.0)  # Edge weight in meters (float)
            else:
                # Fallback: calculate distance from node positions if edge doesn't exist
                pos1 = graph.nodes[current.graph_node]['pos']  # Position tuple (lat, lon, alt) (tuple of float)
                pos2 = graph.nodes[neighbor_node]['pos']  # Position tuple (lat, lon, alt) (tuple of float)
                # Calculate 3D distance
                dx = pos2[0] - pos1[0]  # Latitude difference (float, degrees)
                dy = pos2[1] - pos1[1]  # Longitude difference (float, degrees)
                dz = pos2[2] - pos1[2]  # Altitude difference (float, meters)
                edge_weight = (dx**2 + dy**2 + dz**2)**0.5  # 3D distance approximation (float, meters)
            
            # Calculate arrival time at neighbor node
            # Time = current_time + (distance / speed)
            traversal_time = edge_weight / speed if speed > 0 else 0.0  # Time to traverse edge (float, seconds)
            arrival_time = current.time + traversal_time  # Arrival time at neighbor (float, seconds)
            
            # Check if neighbor node-time pair violates constraints
            if check_constraints(neighbor_node, arrival_time, constraints):
                continue  # Skip this neighbor - constraint violation
            
            # Check if already explored
            neighbor_key = (neighbor_node, arrival_time)  # Unique key for neighbor-time pair (tuple)
            if neighbor_key in closed_set:
                continue  # Skip if already explored
            
            # Calculate costs
            g_cost_new = current.g_cost + edge_weight  # New g_cost: cumulative distance (float, meters)
            h_cost_new = heuristic(graph, neighbor_node, goal_node)  # Heuristic to goal (float, meters)
            
            # Create search node for neighbor
            neighbor_search_node = SearchNode(
                graph_node=neighbor_node,  # Neighbor node ID (str)
                time=arrival_time,  # Arrival time at neighbor (float, seconds)
                g_cost=g_cost_new,  # Cumulative cost from start (float, meters)
                h_cost=h_cost_new,  # Heuristic cost to goal (float, meters)
                parent=current  # Parent node for path reconstruction (SearchNode)
            )
            
            # Add to open set
            heapq.heappush(open_set, (neighbor_search_node.f_cost, neighbor_search_node))
    
    # No path found within iteration limit
    return None  # Return None if no path found


# ============================================================================
# CBS HIGH-LEVEL SEARCH
# ============================================================================

def cbs_find_round_trip_path(
    graph: nx.Graph,
    start_node: str,
    goal_node: str,
    start_time: float,
    speed: float,
    registry: Dict,
    wait_time_at_destination: float = 60.0,
    conflict_threshold: float = 10.0,
    max_cbs_iterations: int = 10
) -> Optional[Tuple[List[str], List[float]]]:
    """
    Conflict-Based Search (CBS) algorithm for finding conflict-free round trip path.
    Plans complete route: origin → destination → origin with wait time at destination.
    Iteratively finds path and resolves conflicts by adding constraints.
    
    Args:
        graph: nx.Graph - NetworkX graph with edge weights
        start_node: str - Starting node ID (also return destination)
        goal_node: str - Destination node ID (outbound goal)
        start_time: float - Simulation time when route starts (seconds)
        speed: float - Drone speed in m/s
        registry: Dict - Active drones registry for constraint extraction
        wait_time_at_destination: float - Wait time at destination before return (seconds, default: 60.0)
        conflict_threshold: float - Time threshold for conflict detection in seconds (default: 10.0)
        max_cbs_iterations: int - Maximum CBS iterations to prevent infinite loops (default: 10)
    
    Returns:
        Optional[Tuple[List[str], List[float]]] - Tuple of (path_nodes, overfly_times) if path found, None otherwise
    """
    # Extract initial constraints from registry
    constraints = extract_node_constraints(registry, conflict_threshold)  # List of NodeConstraint
    
    # ========================================================================
    # PHASE 1: Plan outbound path (origin → destination)
    # ========================================================================
    outbound_result = None  # Result of outbound pathfinding (tuple or None)
    outbound_path_nodes = []  # Outbound path nodes (list of str)
    outbound_overfly_times = []  # Outbound overfly times (list of float)
    
    # Try to find conflict-free outbound path
    for iteration in range(max_cbs_iterations):
        # Run constrained A* for outbound journey
        outbound_result = constrained_astar(
            graph=graph,  # NetworkX graph (nx.Graph)
            start_node=start_node,  # Start node ID (str)
            goal_node=goal_node,  # Goal node ID (str)
            start_time=start_time,  # Start simulation time (float, seconds)
            speed=speed,  # Drone speed (float, m/s)
            constraints=constraints,  # Current constraints (list of NodeConstraint)
            max_iterations=10000  # Maximum A* iterations (int)
        )
        
        # Check if outbound path was found
        if outbound_result is None:
            # No outbound path found with current constraints
            return None  # Return None - no conflict-free path exists
        
        outbound_path_nodes, outbound_overfly_times = outbound_result  # Unpack outbound path and times
        
        # Verify outbound path is conflict-free
        conflicts_found = False  # Flag to track if conflicts exist (bool)
        new_constraints = []  # List to store new constraints from conflicts (list of NodeConstraint)
        
        # Check each node in outbound path against registry
        for i, node_id in enumerate(outbound_path_nodes):
            overfly_time = outbound_overfly_times[i]  # Planned overfly time (float, seconds)
            
            # Check against all active drones in registry
            for drone_id, route_data in registry.items():
                route_nodes = route_data.get("route_nodes", [])  # Active drone's route nodes (list of str)
                route_overfly_times = route_data.get("overfly_times", [])  # Active drone's overfly times (list of float)
                
                # Check each node in active drone's route
                for j, active_node_id in enumerate(route_nodes):
                    if active_node_id == node_id:  # Same node
                        if j < len(route_overfly_times):
                            active_overfly_time = route_overfly_times[j]  # Active drone's overfly time (float, seconds)
                            
                            # Check if times are within conflict threshold
                            time_difference = abs(overfly_time - active_overfly_time)  # Time difference (float, seconds)
                            if time_difference < conflict_threshold:
                                # Conflict detected - verify it's not already constrained
                                already_constrained = False  # Flag to track if constraint exists (bool)
                                for existing_constraint in constraints:
                                    if existing_constraint.node_id == node_id:
                                        if (existing_constraint.forbidden_start <= active_overfly_time <= 
                                            existing_constraint.forbidden_end):
                                            already_constrained = True  # Constraint already exists
                                            break
                                
                                if not already_constrained:
                                    # New conflict - add constraint
                                    conflicts_found = True  # Mark conflict found
                                    constraint = NodeConstraint(
                                        node_id=node_id,  # Conflicting node ID (str)
                                        forbidden_start=active_overfly_time - conflict_threshold,  # Forbidden start time (float, seconds)
                                        forbidden_end=active_overfly_time + conflict_threshold  # Forbidden end time (float, seconds)
                                    )
                                    new_constraints.append(constraint)  # Add to new constraints
        
        # If no conflicts found, outbound path is conflict-free
        if not conflicts_found:
            break  # Exit loop - outbound path is valid
        
        # Conflicts found - add new constraints and replan
        constraints.extend(new_constraints)  # Add new constraints to existing constraints
    
    # Check if outbound path was successfully found
    if outbound_result is None or len(outbound_path_nodes) == 0:
        return None  # No outbound path found
    
    # ========================================================================
    # PHASE 2: Calculate arrival time at destination and wait period
    # ========================================================================
    arrival_at_destination_time = outbound_overfly_times[-1]  # Arrival time at destination (float, seconds)
    wait_end_time = arrival_at_destination_time + wait_time_at_destination  # End of wait period (float, seconds)
    
    # Add constraints for destination node during wait period
    # This prevents other drones from conflicting during the wait
    wait_constraint = NodeConstraint(
        node_id=goal_node,  # Destination node ID (str)
        forbidden_start=arrival_at_destination_time - conflict_threshold,  # Forbidden start time (float, seconds)
        forbidden_end=wait_end_time + conflict_threshold  # Forbidden end time including wait (float, seconds)
    )
    constraints.append(wait_constraint)  # Add wait constraint
    
    # ========================================================================
    # PHASE 3: Plan return path (destination → origin)
    # ========================================================================
    return_result = None  # Result of return pathfinding (tuple or None)
    return_path_nodes = []  # Return path nodes (list of str)
    return_overfly_times = []  # Return overfly times (list of float)
    
    # Try to find conflict-free return path
    for iteration in range(max_cbs_iterations):
        # Run constrained A* for return journey
        return_result = constrained_astar(
            graph=graph,  # NetworkX graph (nx.Graph)
            start_node=goal_node,  # Start node ID (destination) (str)
            goal_node=start_node,  # Goal node ID (origin) (str)
            start_time=wait_end_time,  # Start simulation time after wait (float, seconds)
            speed=speed,  # Drone speed (float, m/s)
            constraints=constraints,  # Current constraints (list of NodeConstraint)
            max_iterations=10000  # Maximum A* iterations (int)
        )
        
        # Check if return path was found
        if return_result is None:
            # No return path found with current constraints
            return None  # Return None - no conflict-free return path exists
        
        return_path_nodes, return_overfly_times = return_result  # Unpack return path and times
        
        # Verify return path is conflict-free
        conflicts_found = False  # Flag to track if conflicts exist (bool)
        new_constraints = []  # List to store new constraints from conflicts (list of NodeConstraint)
        
        # Check each node in return path against registry
        for i, node_id in enumerate(return_path_nodes):
            overfly_time = return_overfly_times[i]  # Planned overfly time (float, seconds)
            
            # Check against all active drones in registry
            for drone_id, route_data in registry.items():
                route_nodes = route_data.get("route_nodes", [])  # Active drone's route nodes (list of str)
                route_overfly_times = route_data.get("overfly_times", [])  # Active drone's overfly times (list of float)
                
                # Check each node in active drone's route
                for j, active_node_id in enumerate(route_nodes):
                    if active_node_id == node_id:  # Same node
                        if j < len(route_overfly_times):
                            active_overfly_time = route_overfly_times[j]  # Active drone's overfly time (float, seconds)
                            
                            # Check if times are within conflict threshold
                            time_difference = abs(overfly_time - active_overfly_time)  # Time difference (float, seconds)
                            if time_difference < conflict_threshold:
                                # Conflict detected - verify it's not already constrained
                                already_constrained = False  # Flag to track if constraint exists (bool)
                                for existing_constraint in constraints:
                                    if existing_constraint.node_id == node_id:
                                        if (existing_constraint.forbidden_start <= active_overfly_time <= 
                                            existing_constraint.forbidden_end):
                                            already_constrained = True  # Constraint already exists
                                            break
                                
                                if not already_constrained:
                                    # New conflict - add constraint
                                    conflicts_found = True  # Mark conflict found
                                    constraint = NodeConstraint(
                                        node_id=node_id,  # Conflicting node ID (str)
                                        forbidden_start=active_overfly_time - conflict_threshold,  # Forbidden start time (float, seconds)
                                        forbidden_end=active_overfly_time + conflict_threshold  # Forbidden end time (float, seconds)
                                    )
                                    new_constraints.append(constraint)  # Add to new constraints
        
        # If no conflicts found, return path is conflict-free
        if not conflicts_found:
            break  # Exit loop - return path is valid
        
        # Conflicts found - add new constraints and replan
        constraints.extend(new_constraints)  # Add new constraints to existing constraints
    
    # Check if return path was successfully found
    if return_result is None or len(return_path_nodes) == 0:
        return None  # No return path found
    
    # ========================================================================
    # PHASE 4: Combine outbound and return paths into complete round trip
    # ========================================================================
    # Combine path nodes: outbound + destination (duplicate for wait) + return (skip first node as it's same as destination)
    complete_path_nodes = outbound_path_nodes.copy()  # Start with outbound path (list of str)
    complete_path_nodes.append(goal_node)  # Add duplicate destination node for wait period (str)
    complete_path_nodes.extend(return_path_nodes[1:])  # Add return path (skip first node - already at destination) (list of str)
    
    # Combine overfly times: outbound + wait period + return (skip first time as it's same as wait_end_time)
    complete_overfly_times = outbound_overfly_times.copy()  # Start with outbound times (list of float)
    complete_overfly_times.append(wait_end_time)  # Add wait end time (destination node with wait) (float, seconds)
    complete_overfly_times.extend(return_overfly_times[1:])  # Add return times (skip first time) (list of float)
    
    # Verify path and times have same length (critical for waypoint creation)
    if len(complete_path_nodes) != len(complete_overfly_times):
        # This should never happen, but check for safety
        raise ValueError(f"Path nodes ({len(complete_path_nodes)}) and overfly times ({len(complete_overfly_times)}) length mismatch")
    
    return (complete_path_nodes, complete_overfly_times)  # Return complete round trip path


def cbs_find_path(
    graph: nx.Graph,
    start_node: str,
    goal_node: str,
    start_time: float,
    speed: float,
    registry: Dict,
    conflict_threshold: float = 10.0,
    max_cbs_iterations: int = 10,
    round_trip: bool = True,
    wait_time_at_destination: float = 60.0
) -> Optional[Tuple[List[str], List[float]]]:
    """
    Conflict-Based Search (CBS) algorithm for finding conflict-free path.
    Supports both one-way and round trip planning.
    
    Args:
        graph: nx.Graph - NetworkX graph with edge weights
        start_node: str - Starting node ID
        goal_node: str - Goal node ID
        start_time: float - Simulation time when route starts (seconds)
        speed: float - Drone speed in m/s
        registry: Dict - Active drones registry for constraint extraction
        conflict_threshold: float - Time threshold for conflict detection in seconds (default: 10.0)
        max_cbs_iterations: int - Maximum CBS iterations to prevent infinite loops (default: 10)
        round_trip: bool - If True, plan round trip (origin → destination → origin) (default: True)
        wait_time_at_destination: float - Wait time at destination before return in seconds (default: 60.0)
    
    Returns:
        Optional[Tuple[List[str], List[float]]] - Tuple of (path_nodes, overfly_times) if path found, None otherwise
    """
    if round_trip:
        # Plan complete round trip route
        return cbs_find_round_trip_path(
            graph=graph,  # NetworkX graph (nx.Graph)
            start_node=start_node,  # Start node ID (str)
            goal_node=goal_node,  # Goal node ID (str)
            start_time=start_time,  # Start simulation time (float, seconds)
            speed=speed,  # Drone speed (float, m/s)
            registry=registry,  # Active drones registry (dict)
            wait_time_at_destination=wait_time_at_destination,  # Wait time at destination (float, seconds)
            conflict_threshold=conflict_threshold,  # Conflict threshold (float, seconds)
            max_cbs_iterations=max_cbs_iterations  # Maximum CBS iterations (int)
        )
    else:
        # Original one-way pathfinding (kept for backward compatibility)
        # Extract initial constraints from registry
        constraints = extract_node_constraints(registry, conflict_threshold)  # List of NodeConstraint
        
        # Try to find conflict-free path
        for iteration in range(max_cbs_iterations):
            # Run constrained A* with current constraints
            result = constrained_astar(
                graph=graph,  # NetworkX graph (nx.Graph)
                start_node=start_node,  # Start node ID (str)
                goal_node=goal_node,  # Goal node ID (str)
                start_time=start_time,  # Start simulation time (float, seconds)
                speed=speed,  # Drone speed (float, m/s)
                constraints=constraints,  # Current constraints (list of NodeConstraint)
                max_iterations=10000  # Maximum A* iterations (int)
            )
            
            # Check if path was found
            if result is None:
                # No path found with current constraints
                return None  # Return None - no conflict-free path exists
            
            path_nodes, overfly_times = result  # Unpack path and overfly times
            
            # Verify path is conflict-free by checking against registry
            # Note: constrained_astar should already avoid conflicts, but we verify here
            conflicts_found = False  # Flag to track if conflicts exist (bool)
            new_constraints = []  # List to store new constraints from conflicts (list of NodeConstraint)
            
            # Check each node in the planned path against registry
            for i, node_id in enumerate(path_nodes):
                overfly_time = overfly_times[i]  # Planned overfly time (float, seconds)
                
                # Check against all active drones in registry
                for drone_id, route_data in registry.items():
                    route_nodes = route_data.get("route_nodes", [])  # Active drone's route nodes (list of str)
                    route_overfly_times = route_data.get("overfly_times", [])  # Active drone's overfly times (list of float)
                    
                    # Check each node in active drone's route
                    for j, active_node_id in enumerate(route_nodes):
                        if active_node_id == node_id:  # Same node
                            if j < len(route_overfly_times):
                                active_overfly_time = route_overfly_times[j]  # Active drone's overfly time (float, seconds)
                                
                                # Check if times are within conflict threshold
                                # Conflict exists if planned time is within threshold of active drone's time
                                time_difference = abs(overfly_time - active_overfly_time)  # Time difference (float, seconds)
                                if time_difference < conflict_threshold:
                                    # Conflict detected - verify it's not already constrained
                                    # Check if this conflict is already covered by existing constraints
                                    already_constrained = False  # Flag to track if constraint exists (bool)
                                    for existing_constraint in constraints:
                                        if existing_constraint.node_id == node_id:
                                            # Check if active_overfly_time is within existing constraint window
                                            if (existing_constraint.forbidden_start <= active_overfly_time <= 
                                                existing_constraint.forbidden_end):
                                                already_constrained = True  # Constraint already exists
                                                break
                                    
                                    if not already_constrained:
                                        # New conflict not covered by constraints - add constraint
                                        conflicts_found = True  # Mark conflict found
                                        
                                        # Create constraint to avoid this node at this time
                                        constraint = NodeConstraint(
                                            node_id=node_id,  # Conflicting node ID (str)
                                            forbidden_start=active_overfly_time - conflict_threshold,  # Forbidden start time (float, seconds)
                                            forbidden_end=active_overfly_time + conflict_threshold  # Forbidden end time (float, seconds)
                                        )
                                        new_constraints.append(constraint)  # Add to new constraints
            
            # If no conflicts found, path is conflict-free
            if not conflicts_found:
                return (path_nodes, overfly_times)  # Return conflict-free path
            
            # Conflicts found - add new constraints and replan
            constraints.extend(new_constraints)  # Add new constraints to existing constraints
        
        # Maximum iterations reached - return best path found so far
        # This may still have conflicts, but it's the best we can do
        return result  # Return last found path (may have conflicts)

