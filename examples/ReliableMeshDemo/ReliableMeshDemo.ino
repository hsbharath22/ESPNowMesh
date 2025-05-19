/*
 * ReliableMeshDemo - ESPNowMesh Library Example with Fixed Channel
 *
 * This example demonstrates the improved ESPNowMesh library
 * with the fixed WiFi channel feature for reliable mesh networking.
 * It uses the integrated SerialTerminal class for command interface.
 *
 * Instructions:
 * 1. Upload this sketch to multiple ESP32 boards
 * 2. Open Serial Monitor at 115200 baud
 * 3. Use commands to interact with the mesh network
 *
 * Available commands (type /help or /? for full list):
 * - /d - Trigger discovery
 * - /s <message> - Send broadcast message
 * - /l - List neighbors
 * - /r <role> - Set node role (sensor, relay, etc.)
 * - /sr <MAC> <message> - Send reliable message with acknowledgment
 */

#include <ESPNowMesh.h>
#include <SerialTerminal.h>

// Create mesh instance
ESPNowMesh mesh;

// Create terminal instance with a reference to our mesh
SerialTerminal terminal(mesh);

// Configuration
#define MESH_WIFI_CHANNEL 1 // Fixed WiFi channel for reliable communication
unsigned long messagesSent = 0;
unsigned long messagesReceived = 0;
unsigned long messageAcksReceived = 0;
String currentRole = "node";        // Track our current role locally
const bool LONG_RANGE_MODE = false; // Enable Long Range mode

// Handler for incoming mesh messages
void onMeshMessage(const char *msg, const uint8_t *sender)
{
  // Filter out internal messages that should be handled by the library
  if (strncmp(msg, "DISCOVERY_REQ", 13) == 0 ||
      strncmp(msg, "DISCOVERY_RSP", 13) == 0)
  {
    // These are internal library messages, don't process them in the user callback
    return;
  }

  messagesReceived++;

  // Print a notification about the message
  Serial.print("\n[RECEIVED] From: ");
  for (int i = 0; i < 6; i++)
  {
    Serial.printf("%02X", sender[i]);
    if (i < 5)
      Serial.print(":");
  }

  Serial.print(" -> ");
  Serial.println(msg);

 

  // Add the command prompt back after printing received message
  Serial.print("> ");
}

// Success callback for reliable messages
void onMessageDelivered(uint32_t msg_id, const uint8_t *dest_mac)
{
  Serial.print("\nMessage delivered successfully! ID: 0x");
  Serial.print(msg_id, HEX);
  Serial.print(" to: ");

  for (int i = 0; i < 6; i++)
  {
    Serial.printf("%02X", dest_mac[i]);
    if (i < 5)
      Serial.print(":");
  }

  Serial.println();
  Serial.print("> ");
}

// Failure callback for reliable messages
void onMessageFailed(uint32_t msg_id, const uint8_t *dest_mac)
{
  Serial.print("\nMessage delivery FAILED! ID: 0x");
  Serial.print(msg_id, HEX);
  Serial.print(" to: ");

  for (int i = 0; i < 6; i++)
  {
    Serial.printf("%02X", dest_mac[i]);
    if (i < 5)
      Serial.print(":");
  }

  Serial.println();
  Serial.print("> ");
}

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize

  Serial.println("\n\n=== ESPNowMesh Reliable Demo with Terminal ===");

  // Set initial role
  currentRole = "node";
  mesh.setRole(currentRole.c_str());
  mesh.enableDebug(true);
  // Enable Long Range mode
  // mesh.enableLongRange(true);
  // Initialize the mesh network with fixed channel
  Serial.printf("Initializing mesh with fixed WiFi channel %d\n", MESH_WIFI_CHANNEL);
  mesh.begin(-80, MESH_WIFI_CHANNEL, LONG_RANGE_MODE);
  mesh.onReceive(onMeshMessage);

  // Enable advanced features
  mesh.enableAutoDiscovery(60000);   // Auto-discover every 10 seconds
  mesh.setUnicastForwarding(true);   // Enable smart routing
  mesh.setRetryFallback(true);       // Enable retry mechanism
  mesh.setFallbackToBroadcast(true); // Fallback to broadcast if retries fail

  // Configure reliable messaging
  mesh.setAckTimeout(3000);               // 3 seconds timeout for ACKs
  mesh.setAckRetries(3);                  // Retry 3 times before giving up
  mesh.onSendSuccess(onMessageDelivered); // Register delivery callback
  mesh.onSendFailure(onMessageFailed);    // Register failure callback

  // Initial discovery
  Serial.println("Sending initial discovery broadcasts...");
  for (int i = 0; i < 3; i++)
  {
    mesh.broadcastDiscovery();
    delay(200); // Short delay between broadcasts
  }

  // Print setup information
  Serial.println("\nMesh setup complete!");
  Serial.println("MAC Address: " + WiFi.macAddress());
  Serial.println("\nType /help or /? to see available commands");

  // Print the initial prompt
  Serial.print("> ");
}

void loop()
{
  // Process mesh events
  mesh.loop();

  // Process terminal commands
  terminal.process();
}