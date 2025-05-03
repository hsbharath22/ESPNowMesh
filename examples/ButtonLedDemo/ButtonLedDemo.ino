/* ButtonLedDemo - ESPNowMesh Example
 * 
 * This example demonstrates using ESPNowMesh to toggle an LED on one ESP32 
 * by pressing a button on another ESP32.
 * 
 * Instructions:
 * 1. Upload this sketch to two ESP32 boards
 * 2. Connect a button to pin 37 (with pull-up) on one board
 * 3. Connect an LED to pin 10 on the other board
 * 
 * On each board, uncomment either BUTTON_NODE or LED_NODE define
 * to configure the board's role.
 */

#include <ESPNowMesh.h>
#include <esp_wifi.h>

ESPNowMesh mesh;

// Device Type Configuration - Uncomment ONE of these lines
#define BUTTON_NODE  // For the button node
// #define LED_NODE  // For the LED node

#ifdef BUTTON_NODE
  const int BUTTON_PIN = 37; // Button pin (with internal pull-up)
  bool lastButtonState = HIGH;
#endif

#ifdef LED_NODE
  const int LED_PIN = 10;    // LED pin
  bool ledState = LOW;
#endif

// Function to print MAC addresses in a readable format
void printMac(const uint8_t* mac) {
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
}

// Callback for mesh message reception
void onMeshMessage(const char* msg, const uint8_t* sender) {
  Serial.print("Received message from: ");
  printMac(sender);
  Serial.print(" | Message: ");
  Serial.println(msg);
  
  #ifdef LED_NODE
    // Check if message is the toggle command
    if (strcmp(msg, "TOGGLE_LED") == 0) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      Serial.print("LED toggled to: ");
      Serial.println(ledState ? "ON" : "OFF");
      
      // Send acknowledgment
      Serial.print("Sending ACK to: ");
      printMac(sender);
      Serial.println();
      
      mesh.send("ACK_TOGGLE", sender, 4);
    }
  #endif
  
  #ifdef BUTTON_NODE
    // Check for acknowledgment
    if (strcmp(msg, "ACK_TOGGLE") == 0) {
      Serial.println("LED toggle confirmed!");
    }
  #endif
}

// Function to print current neighbors
void printNeighbors() {
  uint8_t count;
  auto* neighbors = mesh.getNeighbors(count);
  
  Serial.print("Current neighbors: ");
  Serial.println(count);
  
  for (int i = 0; i < count; i++) {
    Serial.print("  Neighbor ");
    Serial.print(i + 1);
    Serial.print(": ");
    printMac(neighbors[i].mac);
    Serial.print(", Role: ");
    Serial.print(neighbors[i].role);
    Serial.print(", RSSI: ");
    Serial.print(neighbors[i].rssi);
    Serial.print(", Last seen: ");
    Serial.print((millis() - neighbors[i].lastSeen) / 1000);
    Serial.println(" seconds ago");
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000); // Allow time for serial to connect
  
  Serial.println("\n=== ESPNowMesh Button/LED Demo ===");
  
  // Initialize WiFi properly
  WiFi.mode(WIFI_STA);
  delay(100); // Short delay for mode to apply
  
  // Print MAC address
  Serial.print("Initial MAC address: ");
  Serial.println(WiFi.macAddress());
  
  // Clean WiFi state without trying to set channel explicitly
  WiFi.disconnect();
  delay(100);
  
  #ifdef BUTTON_NODE
    Serial.println("Configured as BUTTON NODE");
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    mesh.setRole("button");
  #endif
  
  #ifdef LED_NODE
    Serial.println("Configured as LED NODE");
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, ledState);
    mesh.setRole("led");
  #endif
  
  // Initialize mesh network with debugging enabled
  Serial.println("Initializing mesh network...");
  mesh.enableDebug(true);
  mesh.begin();  // Uses our improved version that doesn't force channel
  mesh.onReceive(onMeshMessage);
  mesh.enableAutoDiscovery(5000);  // Auto-discover every 5 seconds
  
  // Enable reliability features
  mesh.setUnicastForwarding(true); // Use smart unicast routing
  mesh.setRetryFallback(true);     // Retry if send fails
  mesh.setFallbackToBroadcast(true); // Fallback to broadcast if retries fail
  
  // Initial discovery broadcasts with delays
  Serial.println("Sending initial discovery broadcasts");
  delay(500); // Give mesh initialization time
  
  // Send three discovery broadcasts
  for (int i = 0; i < 3; i++) {
    Serial.printf("Initial discovery broadcast %d/3\n", i+1);
    mesh.broadcastDiscovery();
    delay(250);
  }
  
  // Verify final configuration
  Serial.print("Final MAC address: ");
  Serial.println(WiFi.macAddress());
  
  Serial.printf("WiFi Status: %d\n", WiFi.status());
  Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
  
  Serial.println("Setup complete - waiting for mesh communications");
  Serial.println("-------------------------------------------");
}

void loop() {
  // Process mesh network events
  mesh.loop();
  
  // Periodic actions
  static unsigned long lastNeighborPrint = 0;
  static unsigned long lastDiagnostics = 0;
  static unsigned long messagesSent = 0;
  
  // Periodically print neighbor info & diagnostics 
  if (millis() - lastNeighborPrint > 10000) {  // Every 10 seconds
    lastNeighborPrint = millis();
    printNeighbors();
    
    // Send a test message every 10 seconds from button node
    #ifdef BUTTON_NODE
      Serial.println("Sending periodic test message");
      mesh.send("TEST_MESSAGE", nullptr, 4);
      messagesSent++;
    #endif
  }
  
  // Show diagnostics every 8 seconds
  if (millis() - lastDiagnostics > 8000) {
    lastDiagnostics = millis();
    
    Serial.println("\n--- DIAGNOSTIC INFO ---");
    Serial.print("Current MAC: ");
    Serial.println(WiFi.macAddress());
    
    Serial.printf("WiFi Status: %d", WiFi.status());
    switch (WiFi.status()) {
      case WL_CONNECTED: Serial.println(" (CONNECTED)"); break;
      case WL_DISCONNECTED: Serial.println(" (DISCONNECTED)"); break;
      case WL_IDLE_STATUS: Serial.println(" (IDLE)"); break;
      default: Serial.println();
    }
    
    Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
    #ifdef BUTTON_NODE
    Serial.printf("Messages sent: %lu\n", messagesSent);
    #endif
    Serial.println("----------------------");
  }
  
  #ifdef BUTTON_NODE
    // Read button state (pulled up, so LOW when pressed)
    bool currentButtonState = digitalRead(BUTTON_PIN);
    
    // Check for button press (transition from HIGH to LOW)
    if (currentButtonState == LOW && lastButtonState == HIGH) {
      Serial.println("\nButton pressed - sending toggle command");
      
      // Send toggle message multiple times to increase reliability
      for (int i = 0; i < 3; i++) {
        mesh.send("TOGGLE_LED", nullptr, 4);
        messagesSent++;
        delay(20);  // Small delay between sends
      }
      
      delay(50);  // Simple debounce
    }
    
    // Update last button state
    lastButtonState = currentButtonState;
  #endif
  
  // Small delay
  delay(50);
}
