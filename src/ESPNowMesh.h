#ifndef ESP_NOW_MESH_H
#define ESP_NOW_MESH_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define MESH_MAX_PATH     8
#define MESH_PAYLOAD_LEN  64
#define MESH_CACHE_SIZE   10
#define MESH_TTL_DEFAULT  4
#define DEFAULT_WIFI_CHANNEL 1  // Use channel 1 by default for better reliability
#define MESH_MAX_PENDING_ACKS 10  // Maximum number of pending ACKs to track
#define MESH_DEFAULT_ACK_TIMEOUT 2000  // Default timeout for ACKs (ms)
#define MESH_DEFAULT_ACK_RETRIES 3  // Default number of retries for reliable messages

class ESPNowMesh {
public:
  typedef void (*MeshCallback)(const char* msg, const uint8_t* sender);
  typedef void (*AckCallback)(uint32_t msg_id, const uint8_t* dest_mac);

  struct MeshPacket {
    uint8_t sender[6];
    uint8_t receiver[6];
    uint8_t ttl;
    uint8_t path_len;
    uint8_t path[MESH_MAX_PATH][6];
    uint32_t msg_id;
    char payload[MESH_PAYLOAD_LEN];
  };

  struct Neighbor {
    uint8_t mac[6];
    String role;
    int rssi;
    unsigned long lastSeen;
  };
  
  // Structure to track pending acknowledgments
  struct PendingAck {
    bool active;
    uint32_t msg_id;
    uint8_t dest_mac[6];
    char payload[MESH_PAYLOAD_LEN];
    unsigned long timestamp;
    uint8_t attempts;
    uint8_t ttl;
  };

  // Public API
  void begin(int rssi_threshold = -80, uint8_t wifi_channel = DEFAULT_WIFI_CHANNEL);
  void send(const char* msgText, const uint8_t* target_mac = nullptr, uint8_t ttl = MESH_TTL_DEFAULT, uint32_t msg_id = 0);
  void sendConfig(const char* jsonConfig, const uint8_t* target_mac = nullptr, uint8_t ttl = MESH_TTL_DEFAULT);
  void onReceive(MeshCallback cb);
  void loop();
  void enableAutoDiscovery(unsigned long interval);
  void broadcastDiscovery();
  void enableDebug(bool enabled);
  void setRole(const char* role);
  void setUnicastForwarding(bool enabled);
  void setRetryFallback(bool enabled);
  void setFallbackToBroadcast(bool enabled);
  void setMaxRetries(uint8_t retries);
  Neighbor* getNeighbors(uint8_t& count);
  const char* getRole() const; // Get current role

  // Added functions to match implementation
  void clearDuplicateCache(); // Helper to clear message cache
  
  // New API for end-to-end acknowledgments
  void sendReliably(const char* msg, const uint8_t* target_mac, uint8_t ttl = MESH_TTL_DEFAULT);
  void setAckTimeout(uint32_t timeout_ms);
  void setAckRetries(uint8_t retries);
  void onSendSuccess(AckCallback cb);
  void onSendFailure(AckCallback cb);
  
  // Handle incoming ACK - normally called internally, but exposed for custom ACK handling
  void _handleAck(uint32_t ack_id, const uint8_t* sender);
  
  // Moved from private to public to allow application access
  uint32_t generateMsgId();  // Helper to generate unique message IDs

private:
  bool retryFallback = false;
  bool fallbackToBroadcast = true;
  uint8_t maxRetries = 2;
  bool useUnicast = true;  // Changed from false to true: enable unicast forwarding by default
  bool debugMode = false;
  String selfRole = "unknown";
  int rssiThreshold;
  uint8_t cacheIndex = 0;
  unsigned long lastDiscovery = 0;
  unsigned long discoveryInterval = 0;
  uint8_t neighborCount = 0;
  uint8_t wifiChannel = DEFAULT_WIFI_CHANNEL;
  uint16_t messageSeq = 0;  // Added: Message sequence counter for unique IDs
  
  // End-to-end acknowledgment variables
  uint32_t ackTimeout = MESH_DEFAULT_ACK_TIMEOUT;
  uint8_t ackRetries = MESH_DEFAULT_ACK_RETRIES;
  PendingAck pendingAcks[MESH_MAX_PENDING_ACKS];
  AckCallback onSuccessCallback = nullptr;
  AckCallback onFailureCallback = nullptr;

  struct RetryState {
    MeshPacket msg;
    int attempts;
    int lastTriedIndex;
  } retryState;

  struct MsgCache {
    uint8_t sender[6];
    uint32_t msg_id;
    unsigned long timestamp; // Added: timestamp field for cache expiry
  } cache[MESH_CACHE_SIZE];

  Neighbor neighbors[MESH_CACHE_SIZE];
  MeshCallback userCallback = nullptr;

  static ESPNowMesh* instance;
  // Updated callback signatures for ESP32 Arduino core 3.2.0
  static void _onRecvStatic(const esp_now_recv_info_t *info, const uint8_t *data, int len);
  static void _onSendStatic(const uint8_t *mac_addr, esp_now_send_status_t status);

  void _onRecv(const uint8_t* mac, const uint8_t* data, int len, int rssi);
  void _onSend(const uint8_t* mac, esp_now_send_status_t status);
  void updateNeighbor(const uint8_t* mac, const char* role, int rssi);
  bool isBroadcast(const uint8_t* mac);
  bool isDuplicate(uint8_t* sender, uint32_t msg_id);
  void addToCache(uint8_t* sender, uint32_t msg_id);
  bool isInPath(const uint8_t* mac, const MeshPacket& msg);
  void debugLog(const char* fmt, ...);
  
  // New efficient peer management method
  bool managePeer(const uint8_t* mac, uint8_t channel = 0, bool encrypt = false);
  
  // Helpers for end-to-end acknowledgment
  int findFreePendingSlot();
  void checkPendingAcks();
  void resendPendingMessage(int index);
  void clearPendingAck(int index);
  bool isPendingAckActive(int index);

  // Helper method for finding the best route to a target
  uint8_t* findBestRoute(const uint8_t* targetMac, int& bestRssi);
};

#endif
