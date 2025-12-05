class_name DroneManager
extends Node

var drones: Dictionary = {}
var visualization_system: VisualizationSystem

func set_visualization_system(vis_system: VisualizationSystem):
	visualization_system = vis_system

func create_test_drone(id: String, start: Vector3, end: Vector3, model: String, origin_node_id: String = "", dest_node_id: String = "", precomputed_route: Array = []) -> Drone:
	"""
	Create a test drone instance and initialize it with flight parameters
	
	Args:
		id: String - Unique drone identifier (e.g., "FP000001")
		start: Vector3 - Starting world position in Godot coordinates
		end: Vector3 - Destination world position in Godot coordinates
		model: String - Drone model type (e.g., "Heavy Quadcopter")
		origin_node_id: String - Origin graph node ID for Python path planning (e.g., "L0_X0_Y0")
		dest_node_id: String - Destination graph node ID for Python path planning (e.g., "L0_X6_Y2")
		precomputed_route: Array - Optional precomputed route waypoints from Python (empty array if not provided)
	
	Returns:
		Drone - The created and initialized drone instance
	"""
	# Check if drone with this ID already exists - prevents multiple instances with same ID
	if drones.has(id):
		push_warning("Attempting to create duplicate drone with ID %s. Cleaning up existing drone first." % id)
		
		# Properly remove the existing drone to prevent multiple instances
		var existing_drone = drones[id]
		existing_drone.queue_free()  # Remove from scene tree - Area3D will be destroyed
		drones.erase(id)  # Remove from dictionary to clear reference
		
		# Remove from visualization system if it exists to prevent visual artifacts
		if visualization_system:
			visualization_system.remove_drone(existing_drone)
	
	# Create new drone instance as Area3D node
	var drone = Drone.new()
	drones[id] = drone  # Store drone reference in dictionary with String ID as key
	add_child(drone)  # Add drone node to scene tree - required before calling any Timer.start() methods
	
	# Initialize drone after adding to scene tree to ensure all child nodes (like timers) are properly connected
	# Pass both Vector3 positions (for Godot navigation) and Node IDs (for efficient Python path planning)
	# If precomputed_route is provided, drone will use it instead of requesting route via WebSocket
	drone.initialize(id, start, end, model, origin_node_id, dest_node_id, precomputed_route)
	
	# Note: Collision detection now handled automatically by Area3D signals
	# No need to set collision manager reference anymore
	
	# Add to visualization system for 3D rendering
	if visualization_system:
		visualization_system.add_drone(drone)
	
	return drone

func update_all(delta: float):
	for drone in drones.values():
		drone.update(delta)
		
		# Update visualization
		if visualization_system:
			visualization_system.update_drone_position(drone)

func remove_completed_drones():
	var to_remove = []
	for id in drones.keys():
		if drones[id].completed:
			to_remove.append(id)
	for id in to_remove:
		var drone = drones[id]
		drone.queue_free()  # Remove from scene
		drones.erase(id)    # Remove from dictionary

# Note: get_all_drones() function removed - no longer needed with Area3D collision system
# Collision detection now happens automatically through Godot's physics engine

func get_all_drones() -> Dictionary:
	"""
	Get all active drones (kept for compatibility with other systems like logging)
	
	Returns:
		Dictionary: All active drones keyed by drone_id
	"""
	return drones
