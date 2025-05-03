/*
 * SerialCommandDemo - Demonstrates the centralized SerialTerminal functionality
 * 
 * This example shows how the ESPNowMesh library now includes a built-in
 * terminal interface with SerialTerminal class. This eliminates the need
 * for duplicating command handling code in each example.
 * 
 * Instructions:
 * 1. Upload this sketch to an ESP32 board
 * 2. Open Serial Monitor at 115200 baud
 * 3. Use the integrated terminal interface with commands like:
 *    - /help or /? - Show all available commands
 *    - /d - Trigger discovery
 *    - /s <message> - Send broadcast message
 *    - /t <MAC> <message> - Send unicast message
 *    - /sr <MAC> <message> - Send reliable message with ACK
 */

#include <ESPNowMesh.h>
#include <SerialTerminal.h>

// Create mesh instance
ESPNowMesh mesh;

// Create terminal instance with reference to our mesh
SerialTerminal terminal(mesh);

// Counter for messages
unsigned int msgCount = 0;

// Handler for incoming mesh messages
void onMeshMessage(const char* msg, const uint8_t* sender) {
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

// Custom command handler for counting
bool cmdCount(const String& args, ESPNowMesh& mesh) {
  msgCount++;
  Serial.printf("Count incremented to: %u\n", msgCount);
  return true;
}

// Custom command handler to reset counter
bool cmdReset(const String& args, ESPNowMesh& mesh) {
  msgCount = 0;
  Serial.println("Count reset to zero");
  return true;
}

// Custom command handler to send a timestamped message
bool cmdTimestamped(const String& args, ESPNowMesh& mesh) {
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "TIMESTAMP:%lu|%u", millis(), msgCount++);
  
  Serial.printf("Sending timestamped message: %s\n", buffer);
  mesh.send(buffer, nullptr, 4);  // Broadcast with TTL=4
  
  return true;
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);  // Give serial time to initialize
  
  Serial.println("\n\n=== ESPNowMesh Serial Terminal Demo ===");
  
  // Set role and configure mesh
  mesh.setRole("commander");
  mesh.begin(-80, 1);  // RSSI threshold and WiFi channel
  mesh.onReceive(onMeshMessage);
  mesh.enableAutoDiscovery(10000);
  mesh.enableDebug(true);
  
  // Configure terminal - optional customizations
  terminal.setCommandPrefix('/');  // Default is already '/'
  terminal.enableEcho(true);       // Echo characters (default is true)
  terminal.setPrompt("> ");        // Set custom prompt
  
  // Register custom commands
  terminal.addCommand("count", "Increment message counter", cmdCount);
  terminal.addCommand("reset", "Reset message counter to zero", cmdReset);
  terminal.addCommand("timestamp", "Send timestamped message", cmdTimestamped);
  
  // Initial discovery
  mesh.broadcastDiscovery();
  
  // Print setup information
  Serial.println("Serial terminal initialized");
  Serial.println("MAC Address: " + WiFi.macAddress());
  Serial.print("Type /help or /? for available commands\n> ");
}

void loop() {
  // Process mesh events
  mesh.loop();
  
  // Process terminal commands
  terminal.process();
}
