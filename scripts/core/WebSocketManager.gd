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
	print("\n" + "=".repeat(80))
	print("│ 🌐 WEBSOCKET CLIENT MANAGER")
	print("=".repeat(80))
	print("│ Initializing connection to Python server...")
	print("│ Target: %s" % default_url)
	print("=".repeat(80) + "\n")
	
	# Create reconnect timer
	reconnect_timer = Timer.new()
	reconnect_timer.one_shot = true
	reconnect_timer.wait_time = 3.0
	reconnect_timer.timeout.connect(_on_reconnect_timer_timeout)
	add_child(reconnect_timer)
	connect_to_server(default_url)

# Initiates a connection to the WebSocket server at the given URL
func connect_to_server(url):
	var err = ws_peer.connect_to_url(url)
	if err != OK:
		print("│ ❌ Connection failed (Error: %d)" % err)
		schedule_reconnect()
		
# Called every frame; used to poll the WebSocket for new events and data
func _process(_delta):
	ws_peer.poll()
	
	# Check connection state
	var state = ws_peer.get_ready_state()
	#print("WebSocket state: ", state)
	
	if state == WebSocketPeer.STATE_OPEN:
		if not is_connected:
			print("├" + "─".repeat(78) + "┤")
			print("│ ✅ CONNECTION ESTABLISHED")
			print("│ Status: Connected to %s" % default_url)
			print("└" + "─".repeat(78) + "┘\n")
			is_connected = true
			emit_signal("connected")
	elif state == WebSocketPeer.STATE_CLOSED:
		if is_connected:
			print("\n" + "├" + "─".repeat(78) + "┤")
			print("│ ⚠ CONNECTION LOST")
			print("│ Attempting to reconnect...")
			print("└" + "─".repeat(78) + "┘\n")
			is_connected = false
			emit_signal("disconnected")
			schedule_reconnect()
	
	# Process messages
	while ws_peer.get_available_packet_count() > 0:
		var packet = ws_peer.get_packet()
		emit_signal("data_received", packet)
		#print("Received data: ", packet.get_string_from_utf8())

func schedule_reconnect():
	# Silently schedule reconnect - no output needed
	reconnect_timer.start()

func _on_reconnect_timer_timeout():
	# Silently attempt reconnect - no output needed
	connect_to_server(default_url)

func send_message(message):
	if is_connected:
		ws_peer.send_text(message)
		return true
	else:
		push_warning("Cannot send message - WebSocket not connected to server")
		return false
