
#include <ESPNowMesh.h>
#include <ArduinoJson.h>

ESPNowMesh mesh;

void onMeshMsg(const char* msg, const uint8_t* sender) {
  Serial.print("From ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", sender[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" -> ");
  Serial.println(msg);
}

void setup() {
  Serial.begin(115200);
  mesh.setRole("sensor");
  mesh.begin();
  mesh.onReceive(onMeshMsg);

  delay(3000);

  // Broadcast a configuration update
  mesh.sendConfig(nullptr, R"({
    "type": "config",
    "blink": 500,
    "threshold": 42,
    "sleep": false,
    "role": "sensor"
  })");
}

void loop() {
  delay(15000);
}
