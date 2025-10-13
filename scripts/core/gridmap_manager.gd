class_name GridMapManager
extends Node

# GridMap node reference - will be set from outside
var gridmap_node: GridMap
# MeshLibrary resource reference
var mesh_library: MeshLibrary
# CSV data storage - Dictionary with lat/lon as keys and altitude as values
var terrain_data: Dictionary = {}
# Coordinate system parameters - each tile is 927m x 702m
var tile_width: float = 702.0  # Width of each tile in meters (X axis)
var tile_height: float = 927.0  # Height of each tile in meters (Z axis)

# Coordinate conversion constants (same as FlightPlanManager)
const ORIGIN_LAT = 40.891674  # Reference latitude for coordinate conversion
const ORIGIN_LON = -74.000032  # Reference longitude for coordinate conversion

func _ready():
	# Load the mesh library resource
	mesh_library = load("res://resources/Meshs/cell_library.meshlib")
	if not mesh_library:
		push_error("Failed to load cell_library.meshlib")
		return
	
	print("GridMapManager: MeshLibrary loaded successfully")

func initialize_gridmap(gridmap: GridMap):
	"""
	Initialize the GridMap node with the mesh library and set up basic properties
	@param gridmap: GridMap - The GridMap node to initialize
	"""
	gridmap_node = gridmap
	gridmap_node.cell_size = Vector3(702, 0.5, 927)
	# Note: mesh_library is already set by the visualization system
	
	print("GridMapManager: GridMap initialized with cell size: ", gridmap_node.cell_size)

func load_terrain_data():
	"""
	Load terrain data from the FAA UAS facility CSV file
	Stores each data point with its exact coordinates and altitude
	"""
	var file = FileAccess.open("res://data/Filtered_FAA_UAS_FacilityMap_Data_LGA.csv", FileAccess.READ)
	if not file:
		push_error("Failed to open Filtered_FAA_UAS_FacilityMap_Data_LGA.csv")
		return false
	
	# Skip header line
	var header = file.get_line()
	print("GridMapManager: CSV Header: ", header)
	
	# Clear existing terrain data
	terrain_data.clear()
	
	# Parse data lines
	var line_count = 0
	while not file.eof_reached():
		var line = file.get_line().strip_edges()
		if line == "":
			continue
			
		var parts = line.split(",")
		if parts.size() != 3:
			continue
			
		# Parse CSV values: CEILING,LATITUDE,LONGITUDE
		var ceiling = parts[0].to_float()  # Altitude/ceiling value
		var latitude = parts[1].to_float()  # Latitude coordinate
		var longitude = parts[2].to_float()  # Longitude coordinate
		
		# Store each point as a dictionary with all its data
		var point_data = {
			"latitude": latitude,
			"longitude": longitude,
			"altitude": ceiling
		}
		
		# Use a simple index as key instead of coordinate string
		terrain_data[line_count] = point_data
		line_count += 1
	
	file.close()
	
	print("GridMapManager: Loaded %d terrain data points" % line_count)
	print("GridMapManager: Ready to place tiles directly at coordinate positions")
	
	return true

func altitude_to_mesh_item(altitude: float) -> int:
	"""
	Convert altitude value to mesh library item index
	Mapping: 0→1, 50→2, 100→3, 200→4, 300→5, 400→6
	@param altitude: float - The altitude/ceiling value from CSV
	@return int - The mesh library item index (1-based)
	"""
	match int(altitude):
		0:
			return 0    # Item 1 in mesh library (0-indexed)
		50:
			return 1    # Item 2 in mesh library (0-indexed)
		100:
			return 2    # Item 3 in mesh library (0-indexed)
		200:
			return 3    # Item 4 in mesh library (0-indexed)
		300:
			return 4    # Item 5 in mesh library (0-indexed)
		400:
			return 5    # Item 6 in mesh library (0-indexed)
		_:
			# Default case for unknown altitudes - use item 1 (index 0)
			print("GridMapManager: Unknown altitude %f, using default item 1" % altitude)
			return 0

func latlon_to_world_position(latitude: float, longitude: float) -> Vector3:
	"""
	Convert latitude/longitude coordinates to world position (same as FlightPlanManager)
	@param latitude: float - Latitude coordinate
	@param longitude: float - Longitude coordinate  
	@return Vector3 - World position in meters
	"""
	# Use same conversion as FlightPlanManager for consistency
	var meters_per_deg_lat = 111320.0  # Meters per degree latitude
	var meters_per_deg_lon = 111320.0 * cos(deg_to_rad(ORIGIN_LAT))  # Meters per degree longitude at this latitude
	
	var x = (longitude - ORIGIN_LON) * meters_per_deg_lon  # X position in meters
	var z = (ORIGIN_LAT - latitude) * meters_per_deg_lat   # Z position in meters
	
	return Vector3(x, 0, z)

func world_position_to_grid_coords(world_pos: Vector3, altitude: float) -> Vector3i:
	"""
	Convert world position to GridMap grid coordinates with height based on altitude
	Each tile is centered at its coordinate position and has dimensions 702m x 927m
	@param world_pos: Vector3 - World position in meters
	@param altitude: float - Altitude from CSV data in meters (used for Y coordinate)
	@return Vector3i - Grid coordinates for GridMap including height
	"""
	# Calculate grid position by dividing world position by tile size
	var grid_x = int(round(world_pos.x / tile_width))   # Grid X coordinate
	var grid_z = int(round(world_pos.z / tile_height))  # Grid Z coordinate
	
	# Convert altitude to grid Y coordinate (each meter of altitude = 1 grid unit)
	# Use altitude directly as Y coordinate to create proper elevation
	var grid_y = int(altitude*0.3048)  # Use CSV altitude directly as grid height
	
	return Vector3i(grid_x, grid_y, grid_z)

func populate_gridmap():
	"""
	Populate the GridMap with terrain tiles - one tile per CSV data point
	Each tile is 927m x 702m and centered at the exact coordinate from the CSV
	"""
	if not gridmap_node:
		push_error("GridMapManager: GridMap node not initialized")
		return false
		
	if terrain_data.is_empty():
		push_error("GridMapManager: No terrain data loaded")
		return false
	
	print("GridMapManager: Starting to populate GridMap with %d terrain points..." % terrain_data.size())
	
	var tiles_placed = 0
	
	# Iterate through all terrain data points (each point is a dictionary with lat, lon, altitude)
	for point_index in terrain_data.keys():
		var point_data = terrain_data[point_index]  # Get the dictionary for this point
		var latitude = point_data["latitude"]        # Extract latitude
		var longitude = point_data["longitude"]      # Extract longitude  
		var altitude = point_data["altitude"]        # Extract altitude
		
		# Convert lat/lon to world position (in meters)
		var world_pos = latlon_to_world_position(latitude, longitude)
		
		# Convert world position to grid coordinates, including altitude as height
		var grid_pos = world_position_to_grid_coords(world_pos, altitude)
		
		# Get appropriate mesh item for this altitude
		var mesh_item = altitude_to_mesh_item(altitude)
		
		# Place the tile in the GridMap at the calculated grid position (now includes height)
		gridmap_node.set_cell_item(grid_pos, mesh_item)
		tiles_placed += 1
		
		# Progress logging every 100 tiles
		if tiles_placed % 100 == 0:
			print("GridMapManager: Placed %d/%d tiles... Latest: [%.6f, %.6f] alt=%.0fm -> %s -> item %d" % [
				tiles_placed, terrain_data.size(), latitude, longitude, altitude, grid_pos, mesh_item
			])
	
	print("GridMapManager: Successfully placed %d tiles in GridMap" % tiles_placed)
	print("GridMapManager: Each tile represents 927m x 702m with height based on CSV altitude (0-400m)")
	
	return true

func get_terrain_altitude_at_position(world_pos: Vector3) -> float:
	"""
	Get the terrain altitude at a specific world position
	Finds the closest terrain tile to the given position
	@param world_pos: Vector3 - World position to query
	@return float - Altitude value at that position, or -1 if not found
	"""
	if terrain_data.is_empty():
		return -1.0
	
	# Find the closest terrain data point
	var closest_altitude = -1.0
	var min_distance = INF
	
	# Iterate through all terrain points to find the closest one
	for point_index in terrain_data.keys():
		var point_data = terrain_data[point_index]
		var latitude = point_data["latitude"]
		var longitude = point_data["longitude"]
		var altitude = point_data["altitude"]
		
		# Convert this point's lat/lon to world position
		var point_world_pos = latlon_to_world_position(latitude, longitude)
		
		# Calculate distance to the query position
		var distance = world_pos.distance_to(point_world_pos)
		
		# Keep track of the closest point
		if distance < min_distance:
			min_distance = distance
			closest_altitude = altitude
	
	return closest_altitude

func get_grid_info() -> Dictionary:
	"""
	Get information about the loaded grid for debugging/display purposes
	@return Dictionary - Grid information including data count and tile specifications
	"""
	if terrain_data.is_empty():
		return {}
	
	# Calculate actual coordinate bounds from loaded data
	var min_lat = INF
	var max_lat = -INF
	var min_lon = INF
	var max_lon = -INF
	
	for point_index in terrain_data.keys():
		var point_data = terrain_data[point_index]
		var lat = point_data["latitude"]
		var lon = point_data["longitude"]
		
		min_lat = min(min_lat, lat)
		max_lat = max(max_lat, lat)
		min_lon = min(min_lon, lon)
		max_lon = max(max_lon, lon)
	
	return {
		"data_points": terrain_data.size(),
		"approach": "direct_tile_placement",
		"tile_size_meters": Vector2(tile_width, tile_height),
		"coordinate_bounds": {
			"min_lat": min_lat,
			"max_lat": max_lat,
			"min_lon": min_lon,
			"max_lon": max_lon
		},
		"origin_reference": {
			"lat": ORIGIN_LAT,
			"lon": ORIGIN_LON
		}
	}
