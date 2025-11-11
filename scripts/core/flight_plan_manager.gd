class_name FlightPlanManager
extends Node

# Queue-based storage for flight plans - Array type used as a queue data structure
# Plans are sorted by ETD time and removed from front as they are processed
var flight_plan_queue: Array = []

# Statistics tracking - int type counters for total plans loaded and processed
var total_plans_loaded: int = 0
var total_plans_processed: int = 0

# Configuration: CSV file path for flight plans data
# Change this constant to load a different flight plan file
const FLIGHT_PLAN_FILE = "res://data/Regular_Lattice_Manhattan_96 FP_2DP_2Hrs.csv.csv"

const ORIGIN_LAT = 40.55417343
const ORIGIN_LON = -73.99583928

# NOTE: _ready() removed - load_flight_plans() is now called explicitly by SimulationEngine
# This prevents duplicate loading of flight plans which was causing duplicate drone creation

func load_flight_plans():
	"""
	Load flight plans from CSV file into a queue
	Reads CSV data and stores each flight plan as a Dictionary in the queue Array
	After loading, sorts the queue by ETD (Estimated Time of Departure) for efficient processing
	"""
	var file = FileAccess.open(FLIGHT_PLAN_FILE, FileAccess.READ)
	if not file:
		print("Error: Could not open flight plans file: ", FLIGHT_PLAN_FILE)
		return
	
	# Skip header line - first line of CSV contains column names
	file.get_csv_line()
	
	# Read all flight plans from CSV and add to queue
	while not file.eof_reached():
		var data = file.get_csv_line()
		# Ensure row has minimum required columns (13 data fields needed including node IDs)
		# CSV structure: [0]FlightPlanID, [1]DronePortID, [2]ETD, [3]ETD_Seconds, [4]OriginLat, 
		#                [5]OriginLon, [6]OriginNodeID, [7]DestinationLat, [8]DestinationLon, 
		#                [9]DestinationNodeID, [10]DroneModel, [11]EstimatedFlightTime, [12]Ceiling
		if data.size() > 10:
			# Create Dictionary to store flight plan data from CSV columns
			var flight_plan = {
				"id": data[0],                      # String: Flight plan ID (e.g., "FP000001")
				"port": data[1],                    # String: Drone port ID (e.g., "DP1", "DP2")
				"etd_seconds": float(data[3]),      # float: Estimated Time of Departure in seconds
				"origin_lat": float(data[4]),       # float: Origin latitude coordinate
				"origin_lon": float(data[5]),       # float: Origin longitude coordinate
				"origin_node_id": data[6],          # String: Origin graph node ID (e.g., "L0_X0_Y0")
				"dest_lat": float(data[7]),         # float: Destination latitude coordinate (FIXED: was data[6])
				"dest_lon": float(data[8]),         # float: Destination longitude coordinate (FIXED: was data[7])
				"dest_node_id": data[9],            # String: Destination graph node ID (e.g., "L0_X6_Y2")
				"model": data[10],                  # String: Drone model type (FIXED: was data[8])
				"estimated_flight_time": float(data[11]),  # float: Estimated flight duration in minutes
				"ceiling": float(data[12])          # float: Maximum altitude ceiling in meters
			}
			# Add flight plan to end of queue array
			flight_plan_queue.append(flight_plan)
			total_plans_loaded += 1
	
	file.close()
	
	# Sort queue by ETD time (ascending order) - this allows efficient O(1) front access
	# Custom sort function compares etd_seconds field of two Dictionary objects
	flight_plan_queue.sort_custom(func(a, b): return a.etd_seconds < b.etd_seconds)
	
	# Print table-formatted summary
	print("\n" + "=".repeat(80))
	print("│ 📋 FLIGHT PLANS LOADED")
	print("=".repeat(80))
	print("│ Total Plans: %d" % flight_plan_queue.size())
	if not flight_plan_queue.is_empty():
		print("│ First ETD: %.2f seconds" % flight_plan_queue.front().etd_seconds)
		print("│ Last ETD: %.2f seconds" % flight_plan_queue.back().etd_seconds)
		var duration = flight_plan_queue.back().etd_seconds - flight_plan_queue.front().etd_seconds
		print("│ Duration: %.2f seconds (%.2f minutes)" % [duration, duration / 60.0])
	print("=".repeat(80) + "\n")

func get_next_pending_plans(current_time: float) -> Array:
	"""
	Get all flight plans that are ready to launch at the current simulation time
	Uses queue approach: checks front of queue and removes ready plans
	
	Args:
		current_time: float - Current simulation time in seconds
	
	Returns:
		Array - Array of Dictionary objects representing flight plans ready to launch
				Empty array if no plans are ready
	
	Note: This function modifies the queue by removing processed plans (pop_front)
		  Time complexity is O(k) where k is number of ready plans (much better than O(n))
	"""
	var plans_to_launch: Array = []
	
	# Check front of queue for ready plans - queue is sorted by ETD time
	# Continue checking while queue has plans and front plan is ready
	while not flight_plan_queue.is_empty():
		# Peek at front plan without removing it yet - Dictionary type
		var next_plan = flight_plan_queue.front()
		
		# Check if this plan's departure time has arrived or passed
		if next_plan.etd_seconds <= current_time:
			# Remove plan from front of queue and add to launch list
			plans_to_launch.append(flight_plan_queue.pop_front())
			total_plans_processed += 1
		else:
			# Queue is sorted, so if this plan isn't ready, no plans after it are ready
			# Break early for efficiency - no need to check remaining plans
			break
	
	return plans_to_launch

func get_queue_statistics() -> Dictionary:
	"""
	Get statistics about the flight plan queue
	
	Returns:
		Dictionary with the following int/float fields:
		- total_loaded: Total number of plans originally loaded from CSV
		- total_processed: Number of plans that have been dispatched
		- remaining: Number of plans still waiting in queue
		- next_etd: ETD time of next plan in queue (0.0 if queue is empty)
	"""
	return {
		"total_loaded": total_plans_loaded,
		"total_processed": total_plans_processed,
		"remaining": flight_plan_queue.size(),
		"next_etd": flight_plan_queue.front().etd_seconds if not flight_plan_queue.is_empty() else 0.0
	}

# Simple lat/lon to position conversion
func latlon_to_position(lat: float, lon: float) -> Vector3:
	var meters_per_deg_lat = 111320.0
	var meters_per_deg_lon = 111320.0 * cos(deg_to_rad(ORIGIN_LAT))
	var x = (lon - ORIGIN_LON) * meters_per_deg_lon
	# Invert Z calculation: higher latitude (north) → negative Z (north in Godot), lower latitude (south) → positive Z (south in Godot)
	var z = (ORIGIN_LAT - lat) * meters_per_deg_lat
	return Vector3(x, 0, z)

func get_drone_ports() -> Dictionary:
	"""
	Extract unique drone ports from the flight plan queue
	Iterates through all queued plans to find unique port locations
	
	Returns:
		Dictionary where:
		- Key: String - Port ID (e.g., "DP1", "DP2")
		- Value: Dictionary with "lat" (float) and "lon" (float) fields
	
	Note: Does not modify the queue, only reads from it
	"""
	var ports = {}  # Dictionary type to store unique port locations
	
	# Iterate through all plans in queue to find unique ports
	for plan in flight_plan_queue:
		var port_id = plan.port  # String type - drone port identifier
		
		# Check if this port hasn't been added yet (only store first occurrence)
		if not ports.has(port_id):
			# Create nested Dictionary with port location data
			ports[port_id] = {
				"lat": plan.origin_lat,   # float: Latitude coordinate of port
				"lon": plan.origin_lon    # float: Longitude coordinate of port
			}
	
	# Print table-formatted droneport summary
	print("\n" + "─".repeat(80))
	print("│ 🏢 DRONEPORTS IDENTIFIED")
	print("├" + "─".repeat(15) + "┬" + "─".repeat(25) + "┬" + "─".repeat(36) + "┤")
	print("│ %-13s │ %-23s │ %-34s │" % ["Port ID", "Latitude", "Longitude"])
	print("├" + "─".repeat(15) + "┼" + "─".repeat(25) + "┼" + "─".repeat(36) + "┤")
	for port_id in ports.keys():
		var lat = ports[port_id]["lat"]
		var lon = ports[port_id]["lon"]
		print("│ %-13s │ %23.8f │ %34.8f │" % [port_id, lat, lon])
	print("└" + "─".repeat(15) + "┴" + "─".repeat(25) + "┴" + "─".repeat(36) + "┘\n")
	
	return ports
