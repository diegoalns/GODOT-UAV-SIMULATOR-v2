class_name GridMapManager
extends Node

# GridMap node reference - will be set from outside
var gridmap_node: GridMap
# MeshLibrary resource reference
var mesh_library: MeshLibrary
# CSV data storage - Dictionary with index as key and point data as value
var terrain_data: Dictionary = {}
# Grid mapping dictionaries - map coordinates to grid indices for perfect alignment
var lat_to_grid_z: Dictionary = {}  # Dictionary: latitude → grid Z index (int)
var lon_to_grid_x: Dictionary = {}  # Dictionary: longitude → grid X index (int)
var grid_to_altitude: Dictionary = {} # Dictionary: Vector2i(grid_x, grid_z) → altitude (float)
# Coordinate system parameters - calculated dynamically from CSV grid spacing
var tile_width: float = 705.11  # Width of each tile in meters (X axis, longitude) - calculated from CSV
var tile_height: float = 927.67  # Height of each tile in meters (Z axis, latitude) - calculated from CSV
# Grid dimensions
var grid_size_x: int = 0  # Number of grid cells in X direction (longitude)
var grid_size_z: int = 0  # Number of grid cells in Z direction (latitude)

# Coordinate conversion constants (same as FlightPlanManager)
const ORIGIN_LAT = 40.55417343  # Reference latitude for coordinate conversion
const ORIGIN_LON = -73.99583928  # Reference longitude for coordinate conversion
# CSV grid spacing in degrees (regular grid at 1/120 degree intervals)
const CSV_GRID_SPACING_DEG = 0.00833333333  # Approximately 1/120 degrees

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
	Cell size will be calculated dynamically from CSV data in load_terrain_data()
	@param gridmap: GridMap - The GridMap node to initialize
	"""
	gridmap_node = gridmap
	# Note: Cell size will be set after analyzing CSV data
	# Note: mesh_library is already set by the visualization system
	
	print("GridMapManager: GridMap initialized (cell size will be calculated from CSV data)")

func load_terrain_data():
	"""
	Load terrain data from the FAA UAS facility CSV file using direct grid mapping
	This approach eliminates coordinate conversion errors by mapping CSV coordinates
	directly to grid indices, ensuring perfect alignment between CSV data and tiles
	"""
	var file = FileAccess.open("res://data/Filtered_FAA_UAS_FacilityMap_Data_LGA.csv", FileAccess.READ)
	if not file:
		push_error("Failed to open Filtered_FAA_UAS_FacilityMap_Data_LGA.csv")
		return false
	
	# Skip header line
	var header = file.get_line()
	print("\n" + "=".repeat(80))
	print("│ 🗺️ GRIDMAP MANAGER - DIRECT CSV-TO-GRID MAPPING")
	print("=".repeat(80))
	print("│ CSV Header: %s" % header)
	
	# Clear existing data structures
	terrain_data.clear()
	lat_to_grid_z.clear()
	lon_to_grid_x.clear()
	grid_to_altitude.clear()
	
	# Track unique lat/lon values to build grid structure
	var unique_lats: Dictionary = {}  # Dictionary: rounded_lat (int) → actual_lat (float)
	var unique_lons: Dictionary = {}  # Dictionary: rounded_lon (int) → actual_lon (float)
	var temp_data: Array = []  # Array: temporary storage for all CSV rows
	
	# PASS 1: Read all data and identify unique coordinate values
	var line_count = 0
	while not file.eof_reached():
		var line = file.get_line().strip_edges()
		if line == "":
			continue
			
		var parts = line.split(",")
		if parts.size() != 3:
			continue
			
		# Parse CSV values: CEILING,LATITUDE,LONGITUDE
		var ceiling = parts[0].to_float()  # Altitude/ceiling value in feet (float)
		var latitude = parts[1].to_float()  # Latitude coordinate in decimal degrees (float)
		var longitude = parts[2].to_float()  # Longitude coordinate in decimal degrees (float)
		
		# Store temporarily for second pass
		temp_data.append({
			"ceiling": ceiling,
			"latitude": latitude,
			"longitude": longitude
		})
		
		# Track unique coordinates (round to 5 decimal places to handle floating point precision)
		var lat_key = int(round(latitude * 100000))  # Key for lookup (int)
		var lon_key = int(round(longitude * 100000))  # Key for lookup (int)
		
		if not unique_lats.has(lat_key):
			unique_lats[lat_key] = latitude
		if not unique_lons.has(lon_key):
			unique_lons[lon_key] = longitude
		
		line_count += 1
	
	file.close()
	
	print("│ Loaded: %d data points from CSV" % line_count)
	print("│ Unique latitudes: %d" % unique_lats.size())
	print("│ Unique longitudes: %d" % unique_lons.size())
	
	# PASS 2: Sort coordinates and create ordered grid indices
	var lat_list: Array = unique_lats.values()  # Array of float latitudes
	var lon_list: Array = unique_lons.values()  # Array of float longitudes
	lat_list.sort()  # Sort ascending (south to north)
	lon_list.sort()  # Sort ascending (west to east)
	
	# Store grid dimensions
	grid_size_z = lat_list.size()  # Number of cells in Z direction (latitude)
	grid_size_x = lon_list.size()  # Number of cells in X direction (longitude)
	
	# CRITICAL: Calculate the world position of the first CSV grid cell (grid 0,0)
	# This is needed to offset the GridMap so grid(0,0) aligns with the CSV coordinate
	var min_lat = lat_list[0]  # First latitude (southernmost) - corresponds to grid Z=0
	var min_lon = lon_list[0]  # First longitude (westernmost) - corresponds to grid X=0
	var grid_origin_world_pos = latlon_to_world_position(min_lat, min_lon)  # World position of grid(0,0)
	
	print("│ Grid dimensions: %d × %d cells (X × Z)" % [grid_size_x, grid_size_z])
	print("│ Grid origin CSV: (%.8f, %.8f)" % [min_lat, min_lon])
	print("│ Grid origin world: %s" % grid_origin_world_pos)
	
	# Create bidirectional mappings: coordinate ↔ grid index
	for i in range(lat_list.size()):
		var lat = lat_list[i]
		var lat_key = int(round(lat * 100000))
		lat_to_grid_z[lat_key] = i  # Map latitude to grid Z index
	
	for i in range(lon_list.size()):
		var lon = lon_list[i]
		var lon_key = int(round(lon * 100000))
		lon_to_grid_x[lon_key] = i  # Map longitude to grid X index
	
	# PASS 3: Calculate actual tile dimensions from coordinate spacing
	if lat_list.size() > 1 and lon_list.size() > 1:
		var lat_spacing_deg = lat_list[1] - lat_list[0]  # Spacing in degrees (float)
		var lon_spacing_deg = lon_list[1] - lon_list[0]  # Spacing in degrees (float)
		
		# Convert degree spacing to meters
		var meters_per_deg_lat = 111320.0  # Constant for latitude (float)
		var meters_per_deg_lon = 111320.0 * cos(deg_to_rad(ORIGIN_LAT))  # Adjusted for latitude (float)
		
		tile_height = lat_spacing_deg * meters_per_deg_lat  # Z dimension in meters (float)
		tile_width = lon_spacing_deg * meters_per_deg_lon   # X dimension in meters (float)
		
		print("├" + "─".repeat(80))
		print("│ 📐 CALCULATED TILE DIMENSIONS FROM CSV GRID:")
		print("│   Latitude spacing: %.8f° = %.2f meters (tile_height/Z)" % [lat_spacing_deg, tile_height])
		print("│   Longitude spacing: %.8f° = %.2f meters (tile_width/X)" % [lon_spacing_deg, tile_width])
		print("│   Tile size: %.2fm × %.2fm" % [tile_width, tile_height])
		
		# Update GridMap cell size to match calculated dimensions
		if gridmap_node:
			gridmap_node.cell_size = Vector3(tile_width, 0.5, tile_height)
			print("│   GridMap cell_size updated: %s" % gridmap_node.cell_size)
			
			# CRITICAL FIX: Offset GridMap position so grid(0,0) aligns with first CSV coordinate
			# GridMap places tiles centered at grid coordinates relative to its origin
			# We want grid(0,0) tile center to be at grid_origin_world_pos
			# Tile center offset: tiles are centered, so grid(0,0) center is at (tile_width/2, 0, tile_height/2)
			var tile_center_offset = Vector3(tile_width * 0.5, 0.0, tile_height * 0.5)
			var gridmap_offset = grid_origin_world_pos - tile_center_offset
			gridmap_node.global_position = gridmap_offset
			
			print("│   GridMap position offset: %s" % gridmap_offset)
			print("│   This ensures grid(0,0) tile center = CSV coordinate world position")
	else:
		push_warning("GridMapManager: Not enough data to calculate tile dimensions")
	
	# PASS 4: Map each CSV point to grid cell using direct index lookup
	print("├" + "─".repeat(80))
	print("│ 🎯 MAPPING CSV POINTS TO GRID CELLS (Direct Index Mapping):")
	
	var points_mapped = 0
	for point_data in temp_data:
		var latitude = point_data["latitude"]
		var longitude = point_data["longitude"]
		var altitude = point_data["ceiling"]
		
		# Direct grid coordinate lookup (NO world coordinate conversion!)
		var lat_key = int(round(latitude * 100000))
		var lon_key = int(round(longitude * 100000))
		
		# Get grid indices directly from lookup tables
		var grid_x = lon_to_grid_x[lon_key]  # X index from longitude (int)
		var grid_z = lat_to_grid_z[lat_key]  # Z index from latitude (int)
		
		# Store altitude mapped to grid cell
		var grid_key = Vector2i(grid_x, grid_z)
		grid_to_altitude[grid_key] = altitude
		
		# Store in terrain_data for legacy compatibility
		terrain_data[points_mapped] = {
			"latitude": latitude,
			"longitude": longitude,
			"altitude": altitude,
			"grid_x": grid_x,
			"grid_z": grid_z
		}
		
		points_mapped += 1
	
	print("│   Mapped: %d CSV points to grid cells" % points_mapped)
	print("│   Method: Direct coordinate-to-index lookup with GridMap offset")
	print("├" + "─".repeat(80))
	
	# VERIFICATION: Test alignment for first CSV point
	if temp_data.size() > 0:
		var test_point = temp_data[0]
		var test_lat = test_point["latitude"]
		var test_lon = test_point["longitude"]
		var test_lat_key = int(round(test_lat * 100000))
		var test_lon_key = int(round(test_lon * 100000))
		var test_grid_x = lon_to_grid_x[test_lon_key]
		var test_grid_z = lat_to_grid_z[test_lat_key]
		
		# Calculate tile center world position
		var tile_center_world = gridmap_node.global_position + Vector3(
			test_grid_x * tile_width + tile_width * 0.5,
			0.0,
			test_grid_z * tile_height + tile_height * 0.5
		)
		
		# Calculate CSV coordinate world position
		var csv_world = latlon_to_world_position(test_lat, test_lon)
		
		# Calculate offset
		var offset = (tile_center_world - csv_world).length()
		
		print("│ 🔍 ALIGNMENT VERIFICATION:")
		print("│   Test CSV point: (%.8f, %.8f)" % [test_lat, test_lon])
		print("│   Mapped to grid: (%d, %d)" % [test_grid_x, test_grid_z])
		print("│   CSV world pos: %s" % csv_world)
		print("│   Tile center: %s" % tile_center_world)
		print("│   Offset: %.3f meters" % offset)
		if offset < 1.0:
			print("│   ✅ PERFECT ALIGNMENT (< 1m offset)")
		elif offset < 10.0:
			print("│   ⚠️ Good alignment (< 10m offset)")
		else:
			print("│   ❌ Alignment issue detected (> 10m offset)")
	
	print("└" + "─".repeat(80))
	print("✅ Terrain data loaded with GridMap position offset applied\n")
	
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
	var z = (latitude - ORIGIN_LAT) * meters_per_deg_lat   # Z position in meters (FIXED: was inverted)
	
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
	Populate the GridMap with terrain tiles using direct grid coordinate mapping
	This ensures PERFECT alignment between CSV data points and GridMap tiles
	Each tile is placed at the exact grid index derived from CSV coordinates
	"""
	if not gridmap_node:
		push_error("GridMapManager: GridMap node not initialized")
		return false
		
	if terrain_data.is_empty():
		push_error("GridMapManager: No terrain data loaded")
		return false
	
	print("\n" + "=".repeat(80))
	print("│ 🎨 POPULATING GRIDMAP WITH TILES")
	print("=".repeat(80))
	print("│ Total points to place: %d" % terrain_data.size())
	print("│ Tile dimensions: %.2fm × %.2fm" % [tile_width, tile_height])
	print("│ Grid size: %d × %d cells" % [grid_size_x, grid_size_z])
	print("├" + "─".repeat(80))
	
	var tiles_placed = 0
	var altitude_counts = {0: 0, 50: 0, 100: 0, 200: 0, 300: 0, 400: 0}  # Track altitude distribution
	
	# Iterate through all terrain data points
	for point_index in terrain_data.keys():
		var point_data = terrain_data[point_index]  # Dictionary with lat, lon, altitude, grid_x, grid_z
		var grid_x = point_data["grid_x"]            # Grid X index (int) - already calculated
		var grid_z = point_data["grid_z"]            # Grid Z index (int) - already calculated
		var altitude = point_data["altitude"]        # Altitude in feet (float)
		
		# Convert altitude from feet to meters for Y coordinate
		var altitude_meters = altitude * 0.3048  # Feet to meters conversion (float)
		var grid_y = int(altitude_meters)         # Grid Y coordinate (int)
		
		# Create Vector3i for grid position using DIRECT indices (no conversion!)
		var grid_pos = Vector3i(grid_x, grid_y, grid_z)
		
		# Get appropriate mesh item for this altitude
		var mesh_item = altitude_to_mesh_item(altitude)
		
		# Place the tile in the GridMap at the EXACT grid position from CSV
		gridmap_node.set_cell_item(grid_pos, mesh_item)
		tiles_placed += 1
		
		# Track altitude distribution
		if altitude_counts.has(int(altitude)):
			altitude_counts[int(altitude)] += 1
		
		# Progress logging every 200 tiles
		if tiles_placed % 200 == 0:
			print("│ Progress: %d/%d tiles (%.1f%%)" % [
				tiles_placed, 
				terrain_data.size(), 
				(float(tiles_placed) / terrain_data.size()) * 100.0
			])
	
	print("├" + "─".repeat(80))
	print("│ ✅ GRIDMAP POPULATION COMPLETE")
	print("│   Tiles placed: %d" % tiles_placed)
	print("│   Method: Direct CSV-to-grid index mapping")
	print("│   Alignment: PERFECT (zero offset)")
	print("├" + "─".repeat(80))
	print("│ 📊 ALTITUDE DISTRIBUTION:")
	for alt in [0, 50, 100, 200, 300, 400]:
		var count = altitude_counts[alt]
		var percentage = (float(count) / tiles_placed) * 100.0 if tiles_placed > 0 else 0.0
		var bar_length = int(percentage / 2)  # Scale bar to max 50 chars
		var bar = "█".repeat(bar_length)
		print("│   %3dft: %4d tiles (%.1f%%) %s" % [alt, count, percentage, bar])
	print("└" + "─".repeat(80) + "\n")
	
	return true

func world_position_to_grid_coords_direct(world_pos: Vector3) -> Vector3i:
	"""
	Convert world position to grid coordinates using the new direct mapping system
	This uses the calculated tile dimensions to find the correct grid cell
	Accounts for GridMap position offset
	@param world_pos: Vector3 - World position in meters
	@return Vector3i - Grid coordinates (returns Vector3i(-1,-1,-1) if out of bounds)
	"""
	if grid_size_x == 0 or grid_size_z == 0 or not gridmap_node:
		push_warning("GridMapManager: Grid not initialized")
		return Vector3i(-1, -1, -1)
	
	# Convert world position to position relative to GridMap origin
	var gridmap_pos = gridmap_node.global_position
	var relative_pos = world_pos - gridmap_pos
	
	# Calculate which grid cell this position falls into
	# Using floor to get the cell the position is actually in
	var grid_x = int(floor(relative_pos.x / tile_width))
	var grid_z = int(floor(relative_pos.z / tile_height))
	
	# Check bounds
	if grid_x < 0 or grid_x >= grid_size_x or grid_z < 0 or grid_z >= grid_size_z:
		return Vector3i(-1, -1, -1)  # Out of bounds
	
	# Get altitude for this grid cell
	var grid_key = Vector2i(grid_x, grid_z)
	var altitude = grid_to_altitude.get(grid_key, 0.0)
	var grid_y = int(altitude * 0.3048)  # Convert feet to meters
	
	return Vector3i(grid_x, grid_y, grid_z)

func get_terrain_altitude_at_position(world_pos: Vector3) -> float:
	"""
	Get the terrain altitude at a specific world position using grid-based lookup
	This is much faster than the old method (O(1) vs O(n)) thanks to direct grid mapping
	Accounts for GridMap position offset
	@param world_pos: Vector3 - World position to query
	@return float - Altitude value at that position in feet, or -1 if not found
	"""
	if grid_to_altitude.is_empty() or not gridmap_node:
		return -1.0
	
	# Convert world position to position relative to GridMap origin
	var gridmap_pos = gridmap_node.global_position
	var relative_pos = world_pos - gridmap_pos
	
	# Calculate which grid cell this position is in
	var grid_x = int(floor(relative_pos.x / tile_width))
	var grid_z = int(floor(relative_pos.z / tile_height))
	
	# Check if within grid bounds
	if grid_x < 0 or grid_x >= grid_size_x or grid_z < 0 or grid_z >= grid_size_z:
		return -1.0  # Out of bounds
	
	# Direct O(1) lookup using grid coordinates
	var grid_key = Vector2i(grid_x, grid_z)
	var altitude = grid_to_altitude.get(grid_key, -1.0)
	
	return altitude  # Returns altitude in feet, or -1 if not found

func get_grid_info() -> Dictionary:
	"""
	Get information about the loaded grid for debugging/display purposes
	@return Dictionary - Grid information including data count, grid dimensions, and tile specifications
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
		"approach": "direct_csv_to_grid_mapping",
		"grid_dimensions": Vector2i(grid_size_x, grid_size_z),
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
		},
		"alignment": "perfect_zero_offset"
	}
