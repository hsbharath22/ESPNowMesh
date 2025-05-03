/*
 * ComprehensiveMeshDemo - ESPNowMesh Complete Feature Demo
 * 
 * This example demonstrates all major features of the ESPNowMesh library:
 * - Mesh setup and initialization
 * - Auto-discovery of neighbors
 * - Broadcasting messages to all nodes
 * - Unicast messages to specific nodes
 * - Message forwarding across multiple hops
 * - Role-based node classification
 * - Retry and fallback mechanisms
 * - Integration with SerialTerminal for command interface
 * 
 * Instructions:
 * 1. Upload this sketch to multiple ESP32 boards
 * 2. Open Serial Monitor at 115200 baud on each board
 * 3. Use the serial commands to interact with the mesh network
 * 
 * Type /help to see all available commands
 */

#include <ESPNowMesh.h>
#include <SerialTerminal.h>
#include <esp_wifi.h> // Required for channel setting

// Create mesh instance
ESPNowMesh mesh;

// Create terminal instance with reference to mesh
SerialTerminal terminal(mesh);

// Configuration
uint8_t defaultTTL = 4;  // Default Time-To-Live for messages
unsigned long lastMsgTime = 0;  // For periodic status messages
unsigned long messagesSent = 0;  // Counter for sent messages
unsigned long messagesReceived = 0;  // Counter for received messages
String currentRole = "node";  // Track our current role locally
const uint8_t WIFI_CHANNEL = 1;  // Fixed WiFi channel for reliable communication

// Custom command handler for heartbeat
bool cmdHeartbeat(const String& args, ESPNowMesh& mesh) {
  char statusMsg[64];
  snprintf(statusMsg, sizeof(statusMsg), "HEARTBEAT|%s|%lu|%lu", 
           currentRole.c_str(), messagesSent, messagesReceived);
  
  mesh.send(statusMsg, nullptr, defaultTTL);
  messagesSent++;
  
  Serial.print("Heartbeat sent: ");
  Serial.println(statusMsg);
  return true;
}

// Custom command handler for statistics
bool cmdStats(const String& args, ESPNowMesh& mesh) {
  Serial.println("\n=== Message Statistics ===");
  Serial.printf("Messages sent: %lu\n", messagesSent);
  Serial.printf("Messages received: %lu\n", messagesReceived);
  return true;
}

// Callback function for incoming mesh messages
void onMeshMessage(const char* msg, const uint8_t* sender) {
  messagesReceived++;
  
  // Print the sender's MAC address
  Serial.print("\nReceived message from: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", sender[i]);
    if (i < 5) Serial.print(":");
  }
  
  Serial.print("\nMessage: ");
  Serial.println(msg);
  
  // Check for any special commands in the message
  if (strcmp(msg, "PING") == 0) {
    // Auto-respond to ping messages
    char response[32];
    snprintf(response, sizeof(response), "PONG_%s", currentRole.c_str());
    mesh.send(response, sender, defaultTTL);
    Serial.println("Responded to PING with PONG");
  }
  
  // Print prompt after receiving message (for better UX)
  Serial.print("> ");
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);  // Give serial time to initialize
  
  Serial.println("\n\n=== ESPNowMesh Comprehensive Demo ===");
  
  // Configure mesh with features
  currentRole = "node";  // Store default role locally
  mesh.setRole(currentRole.c_str());  // Set in the mesh instance
  mesh.enableDebug(true);  // Start with debug enabled
  
  // Initialize the mesh network - IMPORTANT: specify the WiFi channel
  Serial.printf("Initializing mesh network on channel %d...\n", WIFI_CHANNEL);
  mesh.begin(-80, WIFI_CHANNEL);  // RSSI threshold and WiFi channel
  mesh.onReceive(onMeshMessage);  // Register message callback
  
  // Enable advanced features
  mesh.enableAutoDiscovery(10000);  // Auto-discover every 10 seconds
  mesh.setUnicastForwarding(true);  // Enable smart routing
  mesh.setRetryFallback(true);      // Enable retry mechanism
  mesh.setFallbackToBroadcast(true); // Fallback to broadcast if retries fail
  
  // Register custom commands for our application
  terminal.addCommand("heartbeat", "Send a heartbeat message", cmdHeartbeat);
  terminal.addCommand("stats", "Show message statistics", cmdStats);
  
  // Print initial information
  Serial.println("Mesh network initialized");
  Serial.println("Initial MAC Address: " + WiFi.macAddress());
  
  // Send initial discovery to find neighbors
  Serial.println("Sending initial discovery broadcast...");
  for (int i = 0; i < 3; i++) {
    mesh.broadcastDiscovery();
    delay(200);  // Short delay between broadcasts
  }
  
  Serial.println("\nSetup complete!");
  Serial.println("Type /help for available commands");
  Serial.print("> ");
}

void loop() {
  // Process mesh events
  mesh.loop();
  
  // Process terminal commands
  terminal.process();
  
  // Periodically send status update (every 30 seconds)
  if (millis() - lastMsgTime > 30000) {
    lastMsgTime = millis();
    
    // Broadcast a heartbeat message with node info
    char statusMsg[64];
    snprintf(statusMsg, sizeof(statusMsg), "HEARTBEAT|%s|%lu|%lu", 
             currentRole.c_str(), messagesSent, messagesReceived);
    
    mesh.send(statusMsg, nullptr, defaultTTL);
    messagesSent++;
    
    // Print a note but don't display the full neighbor table automatically
    // to avoid disrupting any ongoing terminal interaction
    Serial.print("\nSent periodic heartbeat. Use /l to see neighbors.\n> ");
  }
}