# ESPNowMesh Library v1.1.0

A reliable, multi-hop mesh networking library for ESP32 devices using ESP-NOW.

## Features

### Core Mesh Functionality
- Multi-hop ESP-NOW mesh with TTL and MAC path tracking
- Loop prevention using path and message deduplication
- Broadcast and directed unicast messaging support
- RSSI-based forwarding decision
- Auto discovery and neighbor table maintenance
- WiFi channel configuration for optimal performance
- Message caching with automatic expiration

### Routing Enhancements
- Smart unicast routing using strongest neighbor
- Retry fallback with ESP-NOW delivery status
- Optional fallback to broadcast
- Role-based node classification (e.g., `sensor`, `relay`, `light`)
- Configurable RSSI threshold for forwarding decisions
- Path tracking to prevent routing loops

### End-to-End Acknowledgments
- Application-level acknowledgments (beyond link-layer ESP-NOW ACK)
- Configurable timeout and retry logic for reliable delivery
- Success and failure callbacks for guaranteed delivery semantics
- Perfect for critical commands that must be confirmed as delivered
- Unique message ID generation for tracking

### Configuration Support
- JSON configuration distribution over mesh
- Support for configuration updates to remote nodes

### Long Range Mode (ESP32 Only)
- Extend communication range up to 4x compared to standard mode
- Fixed 1Mbps PHY rate optimized for distance
- Compatible with all other ESPNowMesh features
- Perfect for outdoor or large area IoT deployments
- Must be enabled at initialization time

### Serial Command Interface
- Interactive control via Serial Monitor:
  - `/d`, `/discovery` – trigger discovery
  - `/l`, `/list` – list neighbors
  - `/s <message>`, `/send <message>` – send broadcast
  - `/t <mac> <message>`, `/target <mac> <message>` – send unicast  
  - `/sr <mac> <message>`, `/sendreliable <mac> <message>` – send with acknowledgment
  - `/r <role>`, `/role <role>` – set node role
  - `/ttl <value>` – set default TTL
  - `/debug on|off` – toggle debug logging
  - `/ping` – send ping to all nodes
  - `/status` – display mesh status
  - `/help`, `/?` – show available commands
  - Custom commands support

### Easy API
```cpp
mesh.begin();
mesh.setRole("sensor");
mesh.onReceive(callback);
mesh.send("Hello", nullptr, 4);
mesh.enableAutoDiscovery(10000); // every 10s
```

## Sending Messages in ESPNowMesh

The library provides multiple ways to send messages across your mesh network, depending on your reliability needs and target audience.

### Broadcast Messages (to all nodes)

Send a message to all nodes in the mesh network:

```cpp
// Basic broadcast to all nodes with default TTL (4)
mesh.send("Hello, everyone!");

// With explicit nullptr and custom TTL
mesh.send("Hello with TTL 3", nullptr, 3);
```

### Unicast Messages (to specific node)

Send a message to a specific node by MAC address:

```cpp
// Define target MAC address
uint8_t targetMac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}; 

// Send unicast message
mesh.send("Hello, specific node!", targetMac);

// With custom TTL
mesh.send("Hello with TTL 5", targetMac, 5);
```

### Reliable Messages (with acknowledgment)

Send messages with end-to-end acknowledgment and automatic retries:

```cpp
// Setup delivery callbacks first
mesh.onSendSuccess(onMessageDelivered);
mesh.onSendFailure(onMessageFailed);

// Configure acknowledgment behavior
mesh.setAckTimeout(3000);  // Wait 3 seconds for acknowledgment
mesh.setAckRetries(3);     // Retry up to 3 times

// Send with reliability guarantees
mesh.sendReliably("Critical command", targetMac);
```

### Example: Real-World Usage

```cpp
// In setup():
mesh.onSendSuccess([](uint32_t msg_id, const uint8_t* dest_mac) {
  Serial.println("Message delivered successfully!");
});

mesh.onSendFailure([](uint32_t msg_id, const uint8_t* dest_mac) {
  Serial.println("Message delivery failed after all retries!");
});

// In your application code:
void turnOnLight(uint8_t* lightNodeMac) {
  // For non-critical messages:
  mesh.send("LIGHT_ON", lightNodeMac);
  
  // OR for critical commands with confirmation:
  mesh.sendReliably("LIGHT_ON", lightNodeMac);
}
```

## Examples

This library includes several examples:
- `MeshDemo` – Basic broadcast mesh
- `DiscoveryDemo` – Manual neighbor discovery
- `OTAConfigDemo` – JSON-style OTA configuration
- `AutoDiscoveryDemo` – Auto-discovery with neighbor caching
- `UnicastForwardingDemo` – Unicast smart routing
- `RetryFallbackDemo` – Message retry and fallback
- `SerialCommandDemo` – Full-featured CLI via Serial Monitor
- `ReliableAckDemo` – End-to-end acknowledgments for guaranteed delivery
- `ButtonLedDemo` – Simple IoT control over mesh
- `SimpleLedButtonControl` – Basic button-to-LED mesh demo
- `FixedMeshDemo` – Predefined mesh topology
- `SerialTerminalDemo` – Interactive mesh terminal interface
- `ComprehensiveMeshDemo` – Complete demonstration of all features
- `LongRangeDemo` – Extended range communication using LR mode

## Requirements

- ESP32-based board
- Arduino core for ESP32 version 3.0.0 or higher
- ArduinoJson (for OTA config support)

## Installation

1. Download this ZIP file
2. In Arduino IDE: `Sketch > Include Library > Add .ZIP Library...`

## License

MIT License

## Public API Methods

### Initialization
- `void begin(int rssi_threshold = -80, uint8_t wifi_channel = DEFAULT_WIFI_CHANNEL, bool long_range = false);`  
  Initializes Wi-Fi in STA mode and sets up ESP-NOW with optional RSSI threshold, WiFi channel, and Long Range mode.

- `void setRole(const char* role);`  
  Sets the string role of the node (e.g., "sensor", "relay").

- `const char* getRole() const;`  
  Gets the current role of the node.

- `void onReceive(MeshCallback cb);`  
  Registers a callback to handle incoming mesh messages.

- `void enableLongRange(bool enable);`  
  Enables or disables Long Range mode for extended communication distance.

- `bool isLongRangeEnabled() const;`  
  Returns whether Long Range mode is currently enabled.

### Messaging
- `void send(const char* message, const uint8_t* target_mac = nullptr, uint8_t ttl = MESH_TTL_DEFAULT, uint32_t msg_id = 0);`  
  Sends a message across the mesh. If `target_mac` is null, it's a broadcast. Optional custom message ID can be provided.

- `void sendConfig(const char* jsonConfig, const uint8_t* target_mac = nullptr, uint8_t ttl = MESH_TTL_DEFAULT);`  
  Sends JSON configuration data across the mesh with automatic message ID generation.

### Discovery
- `void enableAutoDiscovery(unsigned long interval_ms);`  
  Enables periodic discovery requests. Interval is in milliseconds.

- `void broadcastDiscovery();`  
  Sends a one-time discovery request to populate neighbor tables.

- `Neighbor* getNeighbors(uint8_t& count);`  
  Returns the current neighbor list and sets `count` to the number of entries.

### Unicast Routing
- `void setUnicastForwarding(bool enabled);`  
  Enables smart unicast forwarding to best neighbor (based on RSSI).

```cpp
// Example mesh routing configuration:
mesh.setUnicastForwarding(true);  // Enable smart routing (uses signal strength to pick best path)
```

### Retry and Fallback
- `void setRetryFallback(bool enabled);`  
  Enables retry logic using ESP-NOW send status.

- `void setMaxRetries(uint8_t retries);`  
  Maximum number of retry attempts for failed unicast sends.

- `void setFallbackToBroadcast(bool enabled);`  
  If all retries fail, fallback to broadcast if enabled.

```cpp
// Example mesh reliability configuration:
mesh.setRetryFallback(true);      // Enable automatic retry for failed messages
mesh.setMaxRetries(3);            // Set maximum retry attempts (default is 3)
mesh.setFallbackToBroadcast(true);// If direct path fails, try broadcasting as last resort
```

### End-to-End Acknowledgments
- `void sendReliably(const char* msg, const uint8_t* target_mac, uint8_t ttl = MESH_TTL_DEFAULT);`  
  Sends a message with end-to-end acknowledgment and automatic retries for guaranteed delivery.

- `void setAckTimeout(uint32_t timeout_ms);`  
  Sets how long (ms) to wait for an ACK before retrying.

- `void setAckRetries(uint8_t retries);`  
  Sets how many times to retry before giving up.

- `void onSendSuccess(AckCallback cb);`  
  Registers a callback for successful deliveries.

- `void onSendFailure(AckCallback cb);`  
  Registers a callback for failed deliveries after all retries.

- `void _handleAck(uint32_t ack_id, const uint8_t* sender);`  
  Handle incoming ACK - normally called internally, but exposed for custom ACK handling.

### Message ID Management
- `uint32_t generateMsgId();`  
  Generates a unique message ID for tracking purposes.

### Message Deduplication
- `void clearDuplicateCache();`  
  Clears the message deduplication cache manually.

### Debugging
- `void enableDebug(bool enabled);`  
  Turns on verbose Serial logging for mesh routing behavior.

### Utility
- `void loop();`  
  Must be called inside your main loop for discovery, acknowledgments, and internal timers.

### Long Range Mode

```cpp
// Enable Long Range mode at initialization (best for outdoor/long distance)
mesh.begin(-95, 1, true);  // RSSI threshold (-95 dBm), channel 1, Long Range enabled

// Check if Long Range mode is active
if (mesh.isLongRangeEnabled()) {
  Serial.println("Operating in extended range mode");
}
```

## Notes on Long Range Mode

Long Range (LR) mode is a special ESP32-specific feature that significantly extends the range of Wi-Fi communication at the cost of bandwidth. When using LR mode:

1. All communicating devices must be ESP32 and have Long Range mode enabled
2. Long Range mode must be set at initialization time and cannot be changed at runtime
3. The data rate is fixed at 1 Mbps (much slower than standard Wi-Fi)
4. Range can be extended up to approximately 4x standard range
5. The lower data rate means you should keep messages small
6. It works best with unobstructed line of sight
7. Consider using a lower RSSI threshold (e.g., -95 instead of -80) since signals will travel further

## SerialTerminal API

The `SerialTerminal` class provides a command-line interface for interacting with the mesh:

- `SerialTerminal(ESPNowMesh& meshInstance, Stream& serial = Serial);`  
  Create a terminal interface connected to the mesh and a stream.

- `void process();`  
  Process any available serial input (call this in loop).

- `void setCommandPrefix(char prefix);`  
  Change the command prefix (default is '/').

- `void enableEcho(bool enable);`  
  Enable or disable echoing of typed characters.

- `void enablePrompt(bool enable);`  
  Enable or disable the command prompt.

- `void setPrompt(const String& prompt);`  
  Set a custom command prompt string.

- `bool addCommand(const String& command, const String& helpText, CommandHandler handler);`  
  Add custom commands to the terminal interface.

## Structures

### MeshPacket
```cpp
struct MeshPacket {
  uint8_t sender[6];      // Original sender's MAC
  uint8_t receiver[6];    // Final recipient's MAC (or broadcast)
  uint8_t ttl;            // Time To Live - decremented at each hop
  uint8_t path_len;       // Current path length
  uint8_t path[MESH_MAX_PATH][6];  // MAC addresses of nodes the message passed through
  uint32_t msg_id;        // Unique message identifier
  char payload[MESH_PAYLOAD_LEN];  // Message content
};
```

### Neighbor
```cpp
struct Neighbor {
  uint8_t mac[6];         // Neighbor's MAC address
  String role;            // Role string ("sensor", "relay", etc.)
  int rssi;               // Signal strength
  unsigned long lastSeen; // Timestamp of last contact
};
```

### Callback Signatures
```cpp
// Basic message callback
typedef void (*MeshCallback)(const char* msg, const uint8_t* sender);

// Acknowledgment callback
typedef void (*AckCallback)(uint32_t msg_id, const uint8_t* dest_mac);

// SerialTerminal command handler
typedef bool (*CommandHandler)(const String& args, ESPNowMesh& mesh);
```

## Configuration Constants

- `MESH_MAX_PATH`: Maximum number of hops tracked in message path (default 8)
- `MESH_PAYLOAD_LEN`: Maximum message payload length (default 64)
- `MESH_CACHE_SIZE`: Size of message deduplication cache (default 10)
- `MESH_TTL_DEFAULT`: Default Time-To-Live value for messages (default 4)
- `DEFAULT_WIFI_CHANNEL`: Default WiFi channel for ESP-NOW (default 1)
- `MESH_MAX_PENDING_ACKS`: Maximum pending acknowledgments (default 10)
- `MESH_DEFAULT_ACK_TIMEOUT`: Default timeout for ACKs in ms (default 2000)
- `MESH_DEFAULT_ACK_RETRIES`: Default number of retries (default 3)
- `LR_MODE_ENABLED`: Long Range mode enabled flag (value 1)
- `LR_MODE_DISABLED`: Long Range mode disabled flag (value 0)
