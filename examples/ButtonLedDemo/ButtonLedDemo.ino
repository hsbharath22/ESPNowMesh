/* ButtonLedDemo - ESPNowMesh Example
 * 
 * This example demonstrates using ESPNowMesh to toggle LEDs between ESP32 devices.
 * Both devices have a button and LED, so you can press the button on either device
 * to toggle the LED on the other device.
 * 
 * Instructions:
 * 1. Upload this sketch to two ESP32 boards
 * 2. Connect a button to pin 37 (with pull-up) on both boards
 * 3. Connect an LED to pin 10 on both boards
 * 
 * The same code runs on both devices - no configuration changes needed!
 */

#include <ESPNowMesh.h>

ESPNowMesh mesh;

const int BUTTON_PIN = 37;  // Button pin (with internal pull-up)
const int LED_PIN = 10;     // LED pin

bool ledState = LOW;        // Current LED state
bool lastButtonState = HIGH; // Last button state (pulled up, so HIGH when not pressed)

uint8_t myMac[6];           // This device's MAC address

// Function to print MAC addresses in a readable format
void printMac(const uint8_t* mac) {
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
}

// Callback for mesh message reception
void onMeshMessage(const char* msg, const uint8_t* sender) {
  Serial.print("Message from: ");
  printMac(sender);
  Serial.print(" | Content: ");
  Serial.println(msg);

  // Check if this is a command to toggle the LED
  if (strcmp(msg, "TOGGLE_LED") == 0) {
    // Toggle LED state
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    Serial.print("LED toggled to: ");
    Serial.println(ledState ? "ON" : "OFF");
    
    // Send acknowledgment back to sender
    mesh.send("ACK_TOGGLE", sender);
  }
  // Check if this is an acknowledgment of a toggle command
  else if (strcmp(msg, "ACK_TOGGLE") == 0) {
    Serial.println("Remote LED toggle confirmed!");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== ESPNowMesh Button/LED Demo ===");
  
  // Get this device's MAC address
  WiFi.mode(WIFI_STA);
  memcpy(myMac, WiFi.macAddress().c_str(), 6);
  
  Serial.print("My MAC address: ");
  printMac(myMac);
  Serial.println();
  
  // Setup hardware
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, ledState);
  
  // Initialize mesh network
  mesh.begin();
  mesh.onReceive(onMeshMessage);
  mesh.setRole("button-led-node");
  
  // Enable reliability features
  mesh.setRetryFallback(true);
  mesh.enableAutoDiscovery(35000);  // Discover other nodes every 5 seconds
  
  Serial.println("Setup complete - press button to toggle LEDs on other nodes");
}

void loop() {
  // Process mesh network events
  mesh.loop();
  
  // Read button state (pulled up, so LOW when pressed)
  bool currentButtonState = digitalRead(BUTTON_PIN);
  
  // Check for button press (transition from HIGH to LOW)
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    Serial.println("Button pressed - sending toggle command to all nodes");
    mesh.send("TOGGLE_LED");  // Broadcast to all nodes
    delay(50);  // Simple debounce
  }
  
  // Update last button state
  lastButtonState = currentButtonState;
  
  delay(10); // Small delay for stability
}
