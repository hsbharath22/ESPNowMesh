/*
 * LongRangeDemo - ESPNowMesh Library Example with Long Range Mode
 * 
 * This example demonstrates using the Long Range (LR) mode feature
 * of the ESPNowMesh library to achieve extended communication range
 * between ESP32 devices at the cost of bandwidth.
 * 
 * Instructions:
 * 1. Upload this sketch to multiple ESP32 boards
 * 2. Open Serial Monitor at 115200 baud
 * 3. Use commands to interact with the mesh network
 * 4. Test the maximum range between devices
 *
 * Note: Long Range mode only works between ESP32 devices and 
 *       uses a fixed 1Mbps PHY rate while extending the range
 *       by approximately 4 times compared to standard mode.
 */

#include <ESPNowMesh.h>
#include <SerialTerminal.h>

// Create mesh instance
ESPNowMesh mesh;

// Create terminal instance with a reference to our mesh
SerialTerminal terminal(mesh);

// Configuration
const uint8_t WIFI_CHANNEL = 1;  // Fixed channel for better performance
const int RSSI_THRESHOLD = -95;  // Extended threshold for Long Range mode (-95 instead of -80)
unsigned long lastStatusReport = 0;
bool longRangeEnabled = true;    // Start with Long Range enabled

// Message handler callback
void onMeshMessage(const char* msg, const uint8_t* sender) {
  Serial.print("Message from ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", sender[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" -> ");
  Serial.println(msg);
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);  // Give serial time to initialize
  
  Serial.println("\n\n=== ESPNowMesh Long Range Demo ===");
  Serial.println("This demo uses the Long Range mode to extend communication range.");
  Serial.println("Note: Long Range mode trades bandwidth for distance (fixed 1Mbps PHY rate).");
  Serial.println("Type /help for available commands.\n");
  
  // Configure terminal
  terminal.begin();
  terminal.addHelpItem("/lr [0|1]", "Enable (1) or disable (0) Long Range mode");
  terminal.addHelpItem("/rssi", "Show current RSSI threshold");
  terminal.addHelpItem("/rssi <value>", "Set RSSI threshold (e.g. /rssi -95)");
  terminal.addHelpItem("/status", "Show current mesh status and configuration");
  
  // Configure mesh
  mesh.setRole("lr-node");
  mesh.enableDebug(true);
  
  // Enable Long Range mode
  mesh.enableLongRange(longRangeEnabled);
  
  // Initialize the mesh network
  Serial.printf("Initializing mesh network on channel %d with RSSI threshold %d...\n", 
                WIFI_CHANNEL, RSSI_THRESHOLD);
  mesh.begin(RSSI_THRESHOLD, WIFI_CHANNEL);
  mesh.onReceive(onMeshMessage);
  
  // Enable advanced features
  mesh.setUnicastForwarding(true);
  mesh.setRetryFallback(true);
  mesh.enableAutoDiscovery(10000);  // Auto-discover every 10 seconds
  
  showStatus();
}

void loop() {
  // Process the mesh maintenance tasks
  mesh.loop();
  
  // Process terminal commands
  terminal.loop();
  
  // Show periodic status report (every 60 seconds)
  unsigned long currentTime = millis();
  if (currentTime - lastStatusReport > 60000) {
    lastStatusReport = currentTime;
    showStatus();
  }
}

// Display the current status of the mesh and LR mode
void showStatus() {
  Serial.println("\n=== Mesh Status ===");
  Serial.printf("Long Range mode: %s\n", mesh.isLongRangeEnabled() ? "ENABLED" : "disabled");
  Serial.printf("RSSI threshold: %d dBm\n", RSSI_THRESHOLD);
  Serial.printf("WiFi channel: %d\n", WIFI_CHANNEL);
  Serial.printf("Node role: %s\n", mesh.getRole());

  // Print neighbors
  uint8_t count;
  ESPNowMesh::Neighbor* neighbors = mesh.getNeighbors(count);
  
  Serial.printf("Neighbors: %d\n", count);
  for (uint8_t i = 0; i < count; i++) {
    Serial.printf("  %d: %02X:%02X:%02X:%02X:%02X:%02X Role: %s RSSI: %d dBm\n",
                 i + 1,
                 neighbors[i].mac[0], neighbors[i].mac[1], neighbors[i].mac[2],
                 neighbors[i].mac[3], neighbors[i].mac[4], neighbors[i].mac[5],
                 neighbors[i].role.c_str(), neighbors[i].rssi);
  }
  
  Serial.println("============");
}

// Our custom command handler
bool terminalCallback(const char* command, const char* params, SerialTerminal* term) {
  // Long Range mode command
  if (strcmp(command, "/lr") == 0) {
    if (strlen(params) > 0) {
      // Enable or disable Long Range mode
      longRangeEnabled = (params[0] == '1');
      mesh.enableLongRange(longRangeEnabled);
      Serial.printf("Long Range mode %s\n", longRangeEnabled ? "ENABLED" : "disabled");
    } else {
      // Toggle Long Range mode
      longRangeEnabled = !longRangeEnabled;
      mesh.enableLongRange(longRangeEnabled);
      Serial.printf("Long Range mode toggled: %s\n", longRangeEnabled ? "ENABLED" : "disabled");
    }
    return true;
  }
  
  // RSSI threshold command
  else if (strcmp(command, "/rssi") == 0) {
    if (strlen(params) > 0) {
      int newThreshold = atoi(params);
      if (newThreshold < 0) {
        // Update the RSSI threshold
        mesh.begin(newThreshold, WIFI_CHANNEL);
        Serial.printf("RSSI threshold set to %d dBm\n", newThreshold);
      } else {
        Serial.println("Error: RSSI threshold should be negative (e.g., -95)");
      }
    } else {
      // Just show current threshold
      Serial.printf("Current RSSI threshold: %d dBm\n", RSSI_THRESHOLD);
    }
    return true;
  }
  
  // Status command
  else if (strcmp(command, "/status") == 0) {
    showStatus();
    return true;
  }
  
  // Let the default terminal handle other commands
  return false;
}