/*
 * SerialTerminalDemo - Demonstrates the integrated SerialTerminal functionality
 * 
 * This example shows how to use the SerialTerminal class, which provides
 * a standardized command interface for ESPNowMesh applications. All the common
 * mesh commands are handled automatically by the terminal.
 * 
 * The example also shows how to add custom commands for application-specific needs.
 * 
 * Instructions:
 * 1. Upload this sketch to an ESP32 board
 * 2. Open Serial Monitor at 115200 baud
 * 3. Use commands like /help to explore the available features
 */

#include <ESPNowMesh.h>
#include <SerialTerminal.h>

// Create mesh instance
ESPNowMesh mesh;

// Create terminal instance with a reference to our mesh
SerialTerminal terminal(mesh);

// We'll track message statistics for our custom commands
unsigned long messagesSent = 0;
unsigned long messagesReceived = 0;

// Handler for incoming mesh messages
void onMeshMessage(const char* msg, const uint8_t* sender) {
  messagesReceived++;
  
  // Print a notification about the message
  Serial.print("\nReceived message from: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", sender[i]);
    if (i < 5) Serial.print(":");
  }
  
  Serial.print("\nMessage: ");
  Serial.println(msg);
  
  // Add the command prompt back after printing received message
  Serial.print("> ");
}

// Custom command handler for getting message stats
bool cmdStats(const String& args, ESPNowMesh& mesh) {
  Serial.println("\n=== Message Statistics ===");
  Serial.printf("Messages sent: %lu\n", messagesSent);
  Serial.printf("Messages received: %lu\n", messagesReceived);
  Serial.printf("Success rate: %s\n", 
    (messagesSent > 0) ? String(100.0 * messagesReceived / messagesSent, 1) + "%" : "N/A");
  Serial.println("=========================");
  return true; // Command was handled
}

// Custom command handler to reset statistics
bool cmdResetStats(const String& args, ESPNowMesh& mesh) {
  messagesSent = 0;
  messagesReceived = 0;
  Serial.println("Message statistics have been reset.");
  return true;
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);  // Give serial time to initialize
  
  Serial.println("\n\n=== ESPNowMesh Serial Terminal Demo ===");
  
  // Set initial role
  mesh.setRole("node");
  
  // Initialize the mesh network
  mesh.begin(-80, 1);  // RSSI threshold and WiFi channel
  mesh.onReceive(onMeshMessage);  // Register message callback
  
  // Enable auto-discovery
  mesh.enableAutoDiscovery(10000);  // Auto-discover every 10 seconds
  
  // Enable debug output
  mesh.enableDebug(true);
  
  // Register our custom commands
  terminal.addCommand("stats", "Show message statistics", cmdStats);
  terminal.addCommand("resetstats", "Reset message statistics", cmdResetStats);
  
  // Initial discovery to find neighbors
  mesh.broadcastDiscovery();
  
  // Print setup information
  Serial.println("Mesh network initialized");
  Serial.println("MAC Address: " + WiFi.macAddress());
  Serial.println("\nType /help to see available commands");
  
  // Print the initial prompt
  Serial.print("> ");
}

void loop() {
  // Process mesh events
  mesh.loop();
  
  // Process terminal commands
  terminal.process();
}