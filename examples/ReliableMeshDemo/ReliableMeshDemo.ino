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
#define MESH_WIFI_CHANNEL 1   // Fixed WiFi channel for reliable communication
unsigned long messagesSent = 0;
unsigned long messagesReceived = 0;
unsigned long messageAcksReceived = 0;
String currentRole = "node";  // Track our current role locally

// Handler for incoming mesh messages
void onMeshMessage(const char* msg, const uint8_t* sender) {
  messagesReceived++;
  
  // Print a notification about the message
  Serial.print("\n[RECEIVED] From: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", sender[i]);
    if (i < 5) Serial.print(":");
  }
  
  Serial.print(" -> ");
  Serial.println(msg);
  
  // Check if it's an ACK message
  if (strncmp(msg, "ACK|", 4) == 0) {
    messageAcksReceived++;
    
    // Extract the message ID from the ACK
    uint32_t ack_id = strtoul(msg + 4, nullptr, 10);
    
    // Let the mesh handle the acknowledgment
    mesh._handleAck(ack_id, sender);
  } 
  // For regular messages that need acknowledgment, send an ACK back with the same message ID
  else if (strncmp(msg, "RELIABLE_MSG|", 13) == 0) {
    // Extract the message ID from the start of the reliable message
    char* idStr = strchr(msg, '|');
    if (idStr) {
      idStr++; // Skip past the first |
      char* contentStart = strchr(idStr, '|');
      if (contentStart) {
        // Extract just the ID part
        char idBuf[16] = {0};
        size_t idLen = contentStart - idStr;
        if (idLen < sizeof(idBuf)) {
          memcpy(idBuf, idStr, idLen);
          uint32_t msgId = strtoul(idBuf, nullptr, 10);
          
          // Send an acknowledgment with the extracted message ID
          char ackMsg[32];
          snprintf(ackMsg, sizeof(ackMsg), "ACK|%u", msgId);
          
          Serial.printf("\nSending acknowledgment for message ID %u\n", msgId);
          mesh.send(ackMsg, sender, 4);
          
          // Process the actual content (everything after the second |)
          Serial.print("Processing reliable message content: ");
          Serial.println(contentStart + 1);
        }
      }
    }
  }
  
  // Add the command prompt back after printing received message
  Serial.print("> ");
}

// Custom command handler for mesh statistics
bool cmdMeshStats(const String& args, ESPNowMesh& mesh) {
  Serial.println("\n=== MESH STATISTICS ===");
  Serial.printf("Messages sent: %lu\n", messagesSent);
  Serial.printf("Messages received: %lu\n", messagesReceived);
  Serial.printf("Acknowledgments received: %lu\n", messageAcksReceived);
  Serial.printf("Delivery ratio: %s\n", 
    (messagesSent > 0) ? String(100.0 * messageAcksReceived / messagesSent, 1) + "%" : "N/A");
  Serial.println("=======================");
  return true;
}

// Custom command handler for sending a structured test message
bool cmdTestMessage(const String& args, ESPNowMesh& mesh) {
  // Create a JSON-like test message with timestamp and sequence number
  static uint16_t sequence = 0;
  char testMsg[64];
  
  snprintf(testMsg, sizeof(testMsg), "{\"seq\":%u,\"time\":%lu,\"role\":\"%s\"}", 
           sequence++, millis(), currentRole.c_str());
  
  Serial.printf("Sending test message: %s\n", testMsg);
  mesh.send(testMsg, nullptr, 3); // Send with TTL=3
  messagesSent++;
  
  return true;
}

// Add a custom command handler for sending reliable messages
bool cmdSendReliable(const String& args, ESPNowMesh& mesh) {
  // Find the first space to separate MAC and message
  int spacePos = args.indexOf(' ');
  if (spacePos == -1) {
    Serial.println("Usage: /sr <MAC> <message>");
    return false;
  }
  
  // Extract MAC address
  String macStr = args.substring(0, spacePos);
  macStr.replace(":", ""); // Remove colons if present
  
  // Validate MAC format
  if (macStr.length() != 12) {
    Serial.println("Invalid MAC format. Must be 12 hex chars with or without colons.");
    return false;
  }
  
  // Convert MAC string to bytes
  uint8_t targetMac[6];
  for (int i = 0; i < 6; i++) {
    char hex[3] = { macStr.charAt(i*2), macStr.charAt(i*2+1), 0 };
    targetMac[i] = strtoul(hex, nullptr, 16);
  }
  
  // Extract message content
  String message = args.substring(spacePos + 1);
  
  // Print info about what we're sending
  Serial.print("Sending reliable message to MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", targetMac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  
  // Generate a message ID for tracking
  uint32_t msgId = mesh.generateMsgId();
  
  // Format the reliable message with an embedded ID that can be extracted on receipt
  char reliableMsg[64];
  snprintf(reliableMsg, sizeof(reliableMsg), "RELIABLE_MSG|%u|%s", 
           msgId, message.c_str());
  
  // Send using the reliable API
  mesh.sendReliably(reliableMsg, targetMac, 4);
  messagesSent++;
  
  Serial.printf("Reliable message sent: %s\n", message.c_str());
  Serial.println("Waiting for acknowledgment...");
  
  return true;
}

// Success callback for reliable messages
void onMessageDelivered(uint32_t msg_id, const uint8_t* dest_mac) {
  Serial.print("\nMessage delivered successfully! ID: 0x");
  Serial.print(msg_id, HEX);
  Serial.print(" to: ");
  
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", dest_mac[i]);
    if (i < 5) Serial.print(":");
  }
  
  Serial.println();
  Serial.print("> ");
}

// Failure callback for reliable messages
void onMessageFailed(uint32_t msg_id, const uint8_t* dest_mac) {
  Serial.print("\nMessage delivery FAILED! ID: 0x");
  Serial.print(msg_id, HEX);
  Serial.print(" to: ");
  
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", dest_mac[i]);
    if (i < 5) Serial.print(":");
  }
  
  Serial.println();
  Serial.print("> ");
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);  // Give serial time to initialize
  
  Serial.println("\n\n=== ESPNowMesh Reliable Demo with Terminal ===");
  
  // Set initial role
  currentRole = "node";
  mesh.setRole(currentRole.c_str());
  mesh.enableDebug(true);
  
  // Initialize the mesh network with fixed channel
  Serial.printf("Initializing mesh with fixed WiFi channel %d\n", MESH_WIFI_CHANNEL);
  mesh.begin(-80, MESH_WIFI_CHANNEL);
  mesh.onReceive(onMeshMessage);
  
  // Enable advanced features
  mesh.enableAutoDiscovery(10000);  // Auto-discover every 10 seconds
  mesh.setUnicastForwarding(true);  // Enable smart routing
  mesh.setRetryFallback(true);      // Enable retry mechanism
  mesh.setFallbackToBroadcast(true);// Fallback to broadcast if retries fail
  
  // Configure reliable messaging
  mesh.setAckTimeout(3000);         // 3 seconds timeout for ACKs
  mesh.setAckRetries(3);            // Retry 3 times before giving up
  mesh.onSendSuccess(onMessageDelivered);  // Register delivery callback
  mesh.onSendFailure(onMessageFailed);     // Register failure callback
  
  // Register our custom commands
  terminal.addCommand("stats", "Show mesh network statistics", cmdMeshStats);
  terminal.addCommand("test", "Send a structured test message", cmdTestMessage);
  terminal.addCommand("sr", "Send reliable message with ACK: /sr <MAC> <message>", cmdSendReliable);
  
  // Initial discovery
  Serial.println("Sending initial discovery broadcasts...");
  for (int i = 0; i < 3; i++) {
    mesh.broadcastDiscovery();
    delay(200);  // Short delay between broadcasts
  }
  
  // Print setup information
  Serial.println("\nMesh setup complete!");
  Serial.println("MAC Address: " + WiFi.macAddress());
  Serial.println("\nType /help or /? to see available commands");
  
  // Print the initial prompt
  Serial.print("> ");
}

void loop() {
  // Process mesh events
  mesh.loop();
  
  // Process terminal commands
  terminal.process();
}