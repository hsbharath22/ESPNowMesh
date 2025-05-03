
#include <ESPNowMesh.h>

ESPNowMesh mesh;

void onMeshMsg(const char* msg, const uint8_t* sender) {
  Serial.print("Received mesh msg: ");
  Serial.println(msg);
}

void setup() {
  Serial.begin(115200);
  mesh.begin();                  // optional RSSI threshold
  mesh.onReceive(onMeshMsg);     // register callback

  delay(3000);                   // wait for mesh nodes
  mesh.send("Hello Mesh!");      // broadcast
}

void loop() {
  delay(10000);
}
