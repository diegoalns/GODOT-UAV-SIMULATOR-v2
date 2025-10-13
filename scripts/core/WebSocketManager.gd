extends Node

# Signal emitted when the WebSocket connection is successfully established
signal connected
# Signal emitted when the WebSocket connection is closed or lost
signal disconnected
# Signal emitted when data is received from the WebSocket server
signal data_received(data)

# Instance of the low-level WebSocketPeer, used for managing the WebSocket connection
var ws_peer = WebSocketPeer.new()
var default_url = "ws://localhost:8765"
var reconnect_timer = null
var is_connected = false

func _ready():
	print("WebSocketManager is trying to connect to the server...")
	# Create reconnect timer
	reconnect_timer = Timer.new()
	reconnect_timer.one_shot = true
	reconnect_timer.wait_time = 3.0
	reconnect_timer.timeout.connect(_on_reconnect_timer_timeout)
	add_child(reconnect_timer)
	connect_to_server(default_url)

# Initiates a connection to the WebSocket server at the given URL
func connect_to_server(url):
	print("Connecting to server at ", url)
	var err = ws_peer.connect_to_url(url)
	if err != OK:
		print("Failed to initiate connection: ", err)
		schedule_reconnect()
		
# Called every frame; used to poll the WebSocket for new events and data
func _process(_delta):
	ws_peer.poll()
	
	# Check connection state
	var state = ws_peer.get_ready_state()
	#print("WebSocket state: ", state)
	
	if state == WebSocketPeer.STATE_OPEN:
		if not is_connected:
			print("Successfully connected to WebSocket server!")
			is_connected = true
			ws_peer.send_text("Hello from Godot WebSocketManager!")
			emit_signal("connected")
	elif state == WebSocketPeer.STATE_CLOSED:
		if is_connected:
			print("Connection to WebSocket server lost")
			is_connected = false
			emit_signal("disconnected")
			schedule_reconnect()
	
	# Process messages
	while ws_peer.get_available_packet_count() > 0:
		var packet = ws_peer.get_packet()
		emit_signal("data_received", packet)
		#print("Received data: ", packet.get_string_from_utf8())

func schedule_reconnect():
	print("Scheduling reconnection attempt in ", reconnect_timer.wait_time, " seconds")
	reconnect_timer.start()

func _on_reconnect_timer_timeout():
	print("Attempting to reconnect...")
	connect_to_server(default_url)

func send_message(message):
	if is_connected:
		ws_peer.send_text(message)
		return true
	else:
		print("Cannot send message - not connected to server")
		return false
