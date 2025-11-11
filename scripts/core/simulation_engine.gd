class_name SimulationEngine
extends Node

@onready var drone_manager = DroneManager.new()
@onready var flight_plan_manager = FlightPlanManager.new()
@onready var visualization_system = VisualizationSystem.new()
@onready var logger = SimpleLogger.new()

var simulation_time: float = 0.0
# Static reference for global access to simulation time
static var current_simulation_time: float = 0.0
var running: bool = false
var speed_multiplier: float = 1.0
var time_step: float = 1.0
var real_runtime: float = 0.0
var headless_mode: bool = false
var ui: SimpleUI  # Store UI instance for access in _physics_process

func _ready():
	add_child(visualization_system)
	add_child(drone_manager)
	add_child(flight_plan_manager)
	add_child(logger)

	# Connect systems
	drone_manager.set_visualization_system(visualization_system)
	
	# Load flight plans into queue
	flight_plan_manager.load_flight_plans()

	await get_tree().process_frame
	
	# Add drone ports to visualization
	var ports = flight_plan_manager.get_drone_ports()
	for port_id in ports.keys():
		var lat = ports[port_id]["lat"]
		var lon = ports[port_id]["lon"]
		var pos = flight_plan_manager.latlon_to_position(lat, lon)
		visualization_system.add_drone_port(pos, port_id)

	# Add UI
	var canvas_layer = CanvasLayer.new()
	ui = SimpleUI.new()
	canvas_layer.add_child(ui)
	add_child(canvas_layer)

	# Set drone ports in UI
	ui.set_drone_ports(ports.keys())

	# Connect UI signals
	ui.start_requested.connect(_on_start_requested)
	ui.pause_requested.connect(_on_pause_requested)
	ui.speed_changed.connect(_on_speed_changed)
	ui.headless_mode_changed.connect(_on_headless_mode_changed)
	ui.port_selected.connect(_on_port_selected)

func _on_start_requested():
	running = true

func _on_pause_requested():
	running = false

func _on_speed_changed(multiplier: float):
	speed_multiplier = multiplier

func _on_headless_mode_changed(enabled: bool):
	headless_mode = enabled
	visualization_system.set_enabled(!enabled)
	
func _physics_process(delta: float):
	Engine.physics_ticks_per_second = 100  # Set to 360 physics FPS
	if not running:
		return
		
	# Simulation time and real runtime calculation - float type updated every physics frame
	simulation_time += time_step * speed_multiplier
	current_simulation_time = simulation_time  # Update static reference for global access
	real_runtime += delta
	
	# Queue-based drone launching - efficient O(k) where k = number of ready plans
	# Get all flight plans ready to launch at current simulation time
	# This function automatically removes processed plans from the queue
	var plans_to_launch = flight_plan_manager.get_next_pending_plans(simulation_time)
	
	# Launch each ready drone - Array of Dictionary objects
	# Print table header only when launching drones
	if plans_to_launch.size() > 0:
		print("\n" + "─".repeat(90))
		print("│ 🚁 LAUNCHING DRONES - Simulation Time: %.2f seconds" % simulation_time)
		print("├" + "─".repeat(12) + "┬" + "─".repeat(20) + "┬" + "─".repeat(20) + "┬" + "─".repeat(34) + "┤")
		print("│ %-10s │ %-18s │ %-18s │ %-32s │" % ["Drone ID", "Model", "ETD (sec)", "Route (Nodes)"])
		print("├" + "─".repeat(12) + "┼" + "─".repeat(20) + "┼" + "─".repeat(20) + "┼" + "─".repeat(34) + "┤")
	
	for plan in plans_to_launch:
		# Convert latitude/longitude coordinates to Vector3 world positions
		var origin = flight_plan_manager.latlon_to_position(plan.origin_lat, plan.origin_lon)
		var destination = flight_plan_manager.latlon_to_position(plan.dest_lat, plan.dest_lon)
		
		# Print drone launch information in table format
		var route_info = "%s → %s" % [plan.origin_node_id, plan.dest_node_id]
		print("│ %-10s │ %-18s │ %-18.2f │ %-32s │" % [plan.id, plan.model, plan.etd_seconds, route_info])
		
		# Create and initialize the drone with route request to Python server
		# Pass both Vector3 positions (for Godot) and Node IDs (for Python path planning)
		drone_manager.create_test_drone(plan.id, origin, destination, plan.model, plan.origin_node_id, plan.dest_node_id)
	
	# Close the table if drones were launched
	if plans_to_launch.size() > 0:
		print("└" + "─".repeat(12) + "┴" + "─".repeat(20) + "┴" + "─".repeat(20) + "┴" + "─".repeat(34) + "┘")
	
	# Update all created drones
	drone_manager.update_all(time_step * speed_multiplier)
	
	# Log data
	logger.update(time_step, simulation_time, drone_manager.drones)
	
	# Remove completed drones from memory
	drone_manager.remove_completed_drones()
	
	# Update time label in UI
	ui.update_time(simulation_time, real_runtime)

	#if int(simulation_time) % 1 == 0 and simulation_time - delta < int(simulation_time):
	# print("Simulation time: %.5f seconds" % simulation_time)
	# print("Real runtime: %.5f seconds" % real_runtime)

func _on_port_selected(port_id: String):
	var ports = flight_plan_manager.get_drone_ports()
	if ports.has(port_id):
		var lat = ports[port_id]["lat"]
		var lon = ports[port_id]["lon"]
		var pos = flight_plan_manager.latlon_to_position(lat, lon)
		visualization_system.move_balloon_to_port(pos)

func get_terrain_altitude_at_position(world_pos: Vector3) -> float:
	"""
	Get terrain altitude at a specific world position
	@param world_pos: Vector3 - World position to query
	@return float - Altitude value at that position, or -1 if terrain not ready
	"""
	if visualization_system and visualization_system.is_terrain_ready():
		return visualization_system.get_terrain_altitude_at_position(world_pos)
	return -1.0

func get_terrain_info() -> Dictionary:
	"""
	Get terrain system information for debugging/display purposes
	@return Dictionary - Terrain information or empty dict if not ready
	"""
	if visualization_system and visualization_system.is_terrain_ready():
		return visualization_system.get_terrain_info()
	return {}
