#include "ESPNowMesh.h"
#include <ArduinoJson.h>

ESPNowMesh *ESPNowMesh::instance = nullptr;

void ESPNowMesh::begin(int rssi_threshold, uint8_t wifi_channel)
{
  // Save RSSI threshold and WiFi channel settings
  this->rssiThreshold = rssi_threshold;
  this->wifiChannel = wifi_channel;

  // Initialize WiFi in STA mode
  WiFi.mode(WIFI_STA);
  delay(100); // Allow time for WiFi mode to set

  // Save current MAC address
  uint8_t mac[6];
  WiFi.macAddress(mac);

  if (debugMode)
  {
    Serial.print("[MESH] Initializing with MAC: ");
    for (int i = 0; i < 6; i++)
    {
      Serial.printf("%02X", mac[i]);
      if (i < 5)
        Serial.print(":");
    }
    Serial.println();
  }

  // Clean disconnect
  WiFi.disconnect();
  delay(100);

  // Set the WiFi channel explicitly for more reliable ESP-NOW communication
  if (debugMode)
  {
    Serial.printf("[MESH] Setting WiFi channel to %d\n", wifiChannel);
  }
  esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
  delay(100);

  // First deinitialize ESP-NOW if it was initialized before
  if (esp_now_is_peer_exist(nullptr))
  {
    if (debugMode)
      Serial.println("[MESH] Deinitializing existing ESP-NOW setup");
    esp_now_deinit();
    delay(100);
  }

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    if (debugMode)
      Serial.println("[MESH] ESP-NOW init failed!");
    return;
  }

  if (debugMode)
    Serial.println("[MESH] ESP-NOW initialized");

  // Register callbacks - must be done after ESP-NOW init
  esp_now_register_recv_cb(_onRecvStatic);
  esp_now_register_send_cb(_onSendStatic);

  // Set up broadcast peer with specified channel
  esp_now_peer_info_t peer = {};
  memset(peer.peer_addr, 0xFF, 6);
  peer.channel = wifiChannel; // Use our fixed channel
  peer.encrypt = false;

  // Remove any existing peers first
  esp_now_del_peer(peer.peer_addr);

  if (esp_now_add_peer(&peer) != ESP_OK)
  {
    if (debugMode)
      Serial.println("[MESH] Failed to add broadcast peer");
  }
  else
  {
    if (debugMode)
      Serial.println("[MESH] Broadcast peer added");
  }

  // Save instance for static callbacks
  instance = this;

  // Verify MAC address is still valid
  if (debugMode)
  {
    Serial.print("[MESH] Final MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.print("[MESH] Role: ");
    Serial.println(selfRole);
    Serial.printf("[MESH] WiFi channel: %d\n", wifiChannel);
  }
}

void ESPNowMesh::setRole(const char *role)
{
  selfRole = role;
}

const char *ESPNowMesh::getRole() const
{
  return selfRole.c_str();
}

// Implement the new message ID generation function
uint32_t ESPNowMesh::generateMsgId()
{
  // Use both millis() and a sequence counter to ensure uniqueness
  // High 16 bits: time component, Low 16 bits: sequence counter
  uint32_t timeComponent = (millis() & 0xFFFF) << 16;
  uint32_t seqComponent = (++messageSeq) & 0xFFFF;
  return timeComponent | seqComponent;
}

// Implement cache clearing function
void ESPNowMesh::clearDuplicateCache()
{
  for (int i = 0; i < MESH_CACHE_SIZE; i++)
  {
    cache[i].msg_id = 0;
    memset(cache[i].sender, 0, 6);
    cache[i].timestamp = 0;
  }
  cacheIndex = 0;
  if (debugMode)
  {
    debugLog("Message deduplication cache cleared");
  }
}

// Modify the send method to use the optional message ID parameter
void ESPNowMesh::send(const char *msgText, const uint8_t *target_mac, uint8_t ttl, uint32_t msg_id)
{
  // Create message packet
  MeshPacket msg;

  // Get our MAC address
  WiFi.macAddress(msg.sender);

  // Set receiver (target or broadcast)
  if (target_mac)
  {
    memcpy(msg.receiver, target_mac, 6);
    if (debugMode)
    {
      Serial.print("[MESH] Preparing unicast message to: ");
      for (int i = 0; i < 6; i++)
      {
        Serial.printf("%02X", target_mac[i]);
        if (i < 5)
          Serial.print(":");
      }
      Serial.println();
    }

    // Use efficient peer management
    if (!isBroadcast(target_mac)) {
      managePeer(target_mac, wifiChannel, false);
    }
  }
  else
  {
    memset(msg.receiver, 0xFF, 6); // Broadcast
    if (debugMode)
      Serial.println("[MESH] Preparing broadcast message");
  }

  // Set message fields
  msg.ttl = ttl;
  msg.path_len = 0;

  // Use provided message ID if not zero, otherwise generate a new one
  if (msg_id != 0)
  {
    msg.msg_id = msg_id;
    if (debugMode)
    {
      debugLog("Using provided message ID: 0x%08X", msg.msg_id);
    }
  }
  else
  {
    // Generate a unique message ID using the improved method
    msg.msg_id = generateMsgId();
    if (debugMode)
    {
      debugLog("Generated unique message ID: 0x%08X", msg.msg_id);
    }
  }

  // Copy payload with safety checks
  size_t msgLen = strlen(msgText);
  if (msgLen >= MESH_PAYLOAD_LEN)
  {
    if (debugMode)
      Serial.println("[MESH] Warning: Message truncated (too long)");
    msgLen = MESH_PAYLOAD_LEN - 1;
  }
  memcpy(msg.payload, msgText, msgLen);
  msg.payload[msgLen] = '\0'; // Ensure null termination

  // Try unicast if enabled and target is not broadcast
  if (useUnicast && target_mac && !isBroadcast(msg.receiver))
  {
    // Use findBestRoute to get the best next hop
    int bestRSSI = -128;
    uint8_t* bestMac = findBestRoute(target_mac, bestRSSI);

    if (bestMac && bestRSSI >= rssiThreshold && !isBroadcast(bestMac))
    {
      if (debugMode)
      {
        debugLog("Sending via unicast neighbor with RSSI: %d", bestRSSI);
      }

      if (debugMode) { // More detailed logs
        char payloadPreview[17] = {0};
        strncpy(payloadPreview, (char*)msg.payload, 16);
        debugLog("ESP-NOW SEND: Unicast to %02X:%02X:%02X:%02X:%02X:%02X, from %02X:%02X:%02X:%02X:%02X:%02X, payload: %.16s%s",
                bestMac[0], bestMac[1], bestMac[2], bestMac[3], bestMac[4], bestMac[5],
                msg.sender[0], msg.sender[1], msg.sender[2], msg.sender[3], msg.sender[4], msg.sender[5],
                payloadPreview, strlen((char*)msg.payload) > 16 ? "..." : "");
      }
      
      // Make sure the peer is properly registered
      managePeer(bestMac, wifiChannel, false);
      
      esp_err_t result = esp_now_send(bestMac, (uint8_t *)&msg, sizeof(msg));

      if (result != ESP_OK)
      {
        if (debugMode)
          debugLog("Unicast send failed: %d, falling back to broadcast", result);
      }
      else
      {
        if (retryFallback)
          retryState = {msg, 0, 0};
        return;
      }
    }
    else
    {
      if (debugMode)
        debugLog("No suitable unicast neighbor or poor signal, using broadcast");
    }
  }

  // Broadcast fallback or direct broadcast
  // Ensure broadcast peer is added with correct parameters
  uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  
  // Use efficient peer management
  managePeer(broadcastAddr, wifiChannel, false);

  if (debugMode) {
    char payloadPreview[17] = {0};
    strncpy(payloadPreview, (char*)msg.payload, 16);
    debugLog("ESP-NOW SEND: Broadcast to FF:FF:FF:FF:FF:FF, from %02X:%02X:%02X:%02X:%02X:%02X, payload: %.16s%s",
            msg.sender[0], msg.sender[1], msg.sender[2], msg.sender[3], msg.sender[4], msg.sender[5],
            payloadPreview, strlen((char*)msg.payload) > 16 ? "..." : "");
  }
  esp_err_t result = esp_now_send(broadcastAddr, (uint8_t *)&msg, sizeof(msg));

  if (result != ESP_OK)
  {
    if (debugMode)
      debugLog("Broadcast send failed: %d", result);
  }
  else
  {
    if (debugMode)
      debugLog("Message sent via broadcast");
  }
}

void ESPNowMesh::sendConfig(const char *jsonConfig, const uint8_t *target_mac, uint8_t ttl)
{
  send(jsonConfig, target_mac, ttl, 0); // Pass 0 as the message ID to auto-generate one
}

void ESPNowMesh::broadcastDiscovery()
{
  if (debugMode)
    Serial.println("[MESH] Broadcasting discovery request");

  // Ensure broadcast peer is registered using efficient peer management
  uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  managePeer(broadcastAddr, wifiChannel, false);

  // Send multiple discovery broadcasts with random delays to increase chances of success
  for (int i = 0; i < 3; i++)
  {
    // Create a discovery message
    char discoveryMsg[32] = "DISCOVERY_REQ";

    if (debugMode) {
      uint8_t ourMac[6];
      WiFi.macAddress(ourMac);
      debugLog("ESP-NOW SEND: Discovery broadcast from %02X:%02X:%02X:%02X:%02X:%02X, payload: %s",
              ourMac[0], ourMac[1], ourMac[2], ourMac[3], ourMac[4], ourMac[5],
              discoveryMsg);
    }
    esp_err_t result = esp_now_send(broadcastAddr, (uint8_t *)discoveryMsg, strlen(discoveryMsg) + 1);

    if (result != ESP_OK && debugMode)
    {
      Serial.printf("[MESH] Discovery broadcast failed: %d\n", result);
    }

    delay(random(50, 150)); // Random delay to avoid collisions
  }

  // Also send through mesh protocol as fallback
  delay(20);
  send("DISCOVERY_REQ", nullptr, 1, 0);
}

void ESPNowMesh::enableAutoDiscovery(unsigned long interval)
{
  discoveryInterval = interval;
  lastDiscovery = 0;
}

void ESPNowMesh::enableDebug(bool enabled) { debugMode = enabled; }
void ESPNowMesh::setUnicastForwarding(bool enabled) { useUnicast = enabled; }
void ESPNowMesh::setRetryFallback(bool e) { retryFallback = e; }
void ESPNowMesh::setFallbackToBroadcast(bool e) { fallbackToBroadcast = e; }
void ESPNowMesh::setMaxRetries(uint8_t r) { maxRetries = r; }

ESPNowMesh::Neighbor *ESPNowMesh::getNeighbors(uint8_t &count)
{
  count = neighborCount;
  return neighbors;
}

// Add these setter methods somewhere in the implementation file
void ESPNowMesh::setNeighborExpiry(unsigned long expiryTime) {
  neighborExpiryTime = expiryTime;
  if (debugMode) {
    debugLog("Neighbor expiry time set to %lu ms", expiryTime);
  }
}

void ESPNowMesh::setNeighborCleanupInterval(unsigned long interval) {
  neighborCleanupInterval = interval;
  if (debugMode) {
    debugLog("Neighbor cleanup interval set to %lu ms", interval);
  }
}

// Implement the optimized loop method using the state machine approach
void ESPNowMesh::loop() {
  unsigned long currentTime = millis();
  
  // Run one maintenance task per loop iteration using a state machine
  // This spreads processing across multiple loop calls for better performance
  if (currentTime >= nextTaskCheck) {
    switch (loopTaskIndex) {
      case 0: // Auto-discovery check
        checkForDiscovery(currentTime);
        break;
        
      case 1: // Neighbor cleanup
        cleanupStaleNeighbors(currentTime);
        break;
        
      case 2: // Message cache cleanup
        cleanupMessageCache(currentTime);
        break;
    }
    
    // Move to next task for next loop iteration
    loopTaskIndex = (loopTaskIndex + 1) % 3; // 3 tasks total
    nextTaskCheck = currentTime + 10; // Small delay to next task
  }
  
  // Always process pending ACKs on every loop as they're time-critical
  checkPendingAcks();
}

// Helper method to check for and handle auto-discovery
void ESPNowMesh::checkForDiscovery(unsigned long currentTime) {
  if (discoveryInterval == 0) return; // Discovery disabled
  
  if (currentTime - lastDiscovery > discoveryInterval) {
    lastDiscovery = currentTime;
    broadcastDiscovery();
    
    // Log neighbor status if in debug mode
    if (debugMode) {
      logNeighborStatus();
    }
  }
}

// Helper method to log neighbor information
void ESPNowMesh::logNeighborStatus() {
  debugLog("Current neighbor count: %d", neighborCount);
  
  for (uint8_t i = 0; i < neighborCount; i++) {
    debugLog("  Neighbor %d: %02X:%02X:%02X:%02X:%02X:%02X, Role: %s, RSSI: %d, Last seen: %lu ms ago",
             i + 1,
             neighbors[i].mac[0], neighbors[i].mac[1], neighbors[i].mac[2],
             neighbors[i].mac[3], neighbors[i].mac[4], neighbors[i].mac[5],
             neighbors[i].role.c_str(),
             neighbors[i].rssi,
             millis() - neighbors[i].lastSeen);
  }
}

// Helper method to clean up stale neighbors
void ESPNowMesh::cleanupStaleNeighbors(unsigned long currentTime) {
  if (currentTime - lastNeighborCleanup <= neighborCleanupInterval) return;
  
  lastNeighborCleanup = currentTime;
  
  // Remove neighbors that haven't been seen for longer than expiryTime
  int removedCount = 0;
  for (uint8_t i = 0; i < neighborCount; i++) {
    unsigned long timeSinceLastSeen = currentTime - neighbors[i].lastSeen;
    
    // Check if this neighbor has expired
    if (timeSinceLastSeen > neighborExpiryTime) {
      if (debugMode) {
        debugLog("Removing stale neighbor %02X:%02X:%02X:%02X:%02X:%02X, Role: %s, not seen for %lu ms",
               neighbors[i].mac[0], neighbors[i].mac[1], neighbors[i].mac[2],
               neighbors[i].mac[3], neighbors[i].mac[4], neighbors[i].mac[5],
               neighbors[i].role.c_str(), timeSinceLastSeen);
      }
      
      // Shift remaining neighbors to compact the array
      for (uint8_t j = i; j < neighborCount - 1; j++) {
        memcpy(&neighbors[j], &neighbors[j+1], sizeof(Neighbor));
      }
      
      // Decrement counts and counters
      neighborCount--;
      i--; // Process this index again since it now contains the next element
      removedCount++;
    }
  }
  
  if (debugMode && removedCount > 0) {
    debugLog("Removed %d stale neighbors, %d active neighbors remain", removedCount, neighborCount);
  }
}

// Helper method to clean up the message deduplication cache
void ESPNowMesh::cleanupMessageCache(unsigned long currentTime) {
  if (currentTime - lastCacheCleanup <= 30000) return; // Run every 30 seconds
  
  lastCacheCleanup = currentTime;

  // Simply clear very old entries to prevent false positives
  const unsigned long CACHE_EXPIRY_MS = 60000; // 1 minute

  int cleanedEntries = 0;
  for (int i = 0; i < MESH_CACHE_SIZE; i++) {
    if (cache[i].msg_id != 0 &&
        ((currentTime > cache[i].timestamp &&
          currentTime - cache[i].timestamp > CACHE_EXPIRY_MS) ||
         (currentTime < cache[i].timestamp &&
          currentTime + (0xFFFFFFFF - cache[i].timestamp) > CACHE_EXPIRY_MS))) {
      // Entry is too old, clear it
      cache[i].msg_id = 0;
      memset(cache[i].sender, 0, 6);
      cache[i].timestamp = 0;
      cleanedEntries++;
    }
  }

  if (debugMode && cleanedEntries > 0) {
    debugLog("Cleaned %d expired entries from message cache", cleanedEntries);
  }
}

// Add the missing checkPendingAcks method
void ESPNowMesh::checkPendingAcks() {
  unsigned long currentTime = millis();
  
  // Check each pending ACK slot
  for (int i = 0; i < MESH_MAX_PENDING_ACKS; i++) {
    // Skip inactive slots
    if (!pendingAcks[i].active) continue;
    
    // Check if this ACK has timed out
    unsigned long waitTime = currentTime - pendingAcks[i].timestamp;
    if (waitTime > ackTimeout) {
      // Check if we have retries left
      if (pendingAcks[i].attempts < ackRetries) {
        // Still have retries left, resend the message
        resendPendingMessage(i);
      } else {
        // No more retries, consider the message as failed
        if (debugMode) {
          debugLog("Message delivery failed after %d attempts: ID 0x%08X to %02X:%02X:%02X:%02X:%02X:%02X",
                   pendingAcks[i].attempts,
                   pendingAcks[i].msg_id,
                   pendingAcks[i].dest_mac[0], pendingAcks[i].dest_mac[1], pendingAcks[i].dest_mac[2],
                   pendingAcks[i].dest_mac[3], pendingAcks[i].dest_mac[4], pendingAcks[i].dest_mac[5]);
        }
        
        // Call the failure callback if registered
        if (onFailureCallback) {
          onFailureCallback(pendingAcks[i].msg_id, pendingAcks[i].dest_mac);
        }
        
        // Clear this pending ACK slot
        clearPendingAck(i);
      }
    }
  }
}

void ESPNowMesh::_onRecvStatic(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
  if (!instance)
    return;

  // Special handling for direct discovery messages (non-mesh packet)
  if (len < sizeof(MeshPacket) && len > 0)
  {
    // Could be a direct discovery message - try to interpret as string
    char msg[64] = {0};
    int msgLen = (len < 63) ? len : 63;
    memcpy(msg, data, msgLen);
    msg[msgLen] = '\0'; // Ensure null termination

    if (instance->debugMode)
    { // Only show in verbose mode
      Serial.print("[MESH] Received small packet: '");
      Serial.print(msg);
      Serial.print("' from ");
      for (int i = 0; i < 6; i++)
      {
        Serial.printf("%02X", info->src_addr[i]);
        if (i < 5)
          Serial.print(":");
      }
      Serial.println();
    }

    if (strcmp(msg, "DISCOVERY_REQ") == 0)
    {
      // Add peer for response using efficient peer management
      instance->managePeer(info->src_addr, instance->wifiChannel, false);

      // Send direct discovery response after adding peer
      char rsp[64];
      snprintf(rsp, sizeof(rsp), "DISCOVERY_RSP|%s|%lu",
               instance->selfRole.c_str(), millis());

      // Add random delay to avoid collisions
      delay(random(10, 50));

      if (instance->debugMode) {
        uint8_t ourMac[6];
        WiFi.macAddress(ourMac);
        instance->debugLog("ESP-NOW SEND: Direct discovery response to %02X:%02X:%02X:%02X:%02X:%02X, from %02X:%02X:%02X:%02X:%02X:%02X, payload: %s",
                info->src_addr[0], info->src_addr[1], info->src_addr[2], info->src_addr[3], info->src_addr[4], info->src_addr[5],
                ourMac[0], ourMac[1], ourMac[2], ourMac[3], ourMac[4], ourMac[5],
                rsp);
      }
      esp_err_t result = esp_now_send(info->src_addr, (uint8_t *)rsp, strlen(rsp) + 1);

      if (result != ESP_OK && instance->debugMode)
      {
        Serial.printf("[MESH] Failed to send direct discovery response: %d\n", result);
      }
      return;
    }
    else if (strncmp(msg, "DISCOVERY_RSP|", 14) == 0)
    {
      // Extract role
      char *role = strchr(msg, '|');
      if (role)
      {
        role++; // Skip past the |
        // Find the next | if it exists
        char *end = strchr(role, '|');
        if (end)
          *end = '\0'; // Terminate at the |

        if (instance->debugMode)
        {
          Serial.print("[MESH] Found neighbor with role: ");
          Serial.println(role);
        }

        // Update neighbor table with RSSI from ESP-NOW info struct
        int rssi = info->rx_ctrl->rssi;
        if (instance->debugMode)
        {
          Serial.printf("[MESH] Packet RSSI: %d dBm\n", rssi);
        }
        instance->updateNeighbor(info->src_addr, role, rssi);

        // Add as peer using efficient peer management
        instance->managePeer(info->src_addr, instance->wifiChannel, false);
      }
      return;
    }
  }

  // Standard mesh packet handling - pass RSSI info to _onRecv
  instance->_onRecv(info->src_addr, data, len, info->rx_ctrl->rssi);
}

void ESPNowMesh::_onSendStatic(const uint8_t *mac, esp_now_send_status_t status)
{
  if (!instance || !instance->retryFallback || instance->retryState.msg.ttl == 0)
    return;
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    instance->retryState.msg.ttl = 0;
    return;
  }
  instance->retryState.attempts++;
  if (instance->retryState.attempts > instance->maxRetries)
  {
    if (instance->fallbackToBroadcast) {
      if (instance->debugMode) {
        char payloadPreview[17] = {0};
        strncpy(payloadPreview, (char*)instance->retryState.msg.payload, 16);
        instance->debugLog("ESP-NOW SEND: Fallback broadcast from %02X:%02X:%02X:%02X:%02X:%02X, payload: %.16s%s",
                instance->retryState.msg.sender[0], instance->retryState.msg.sender[1], instance->retryState.msg.sender[2],
                instance->retryState.msg.sender[3], instance->retryState.msg.sender[4], instance->retryState.msg.sender[5],
                payloadPreview, strlen((char*)instance->retryState.msg.payload) > 16 ? "..." : "");
      }
      esp_now_send(nullptr, (uint8_t *)&instance->retryState.msg, sizeof(instance->retryState.msg));
    }
    instance->retryState.msg.ttl = 0;
    return;
  }
  // try next neighbor
  int best = -128;
  uint8_t *bestMac = nullptr;
  for (uint8_t i = 0; i < instance->neighborCount; i++)
  {
    if (!instance->isInPath(instance->neighbors[i].mac, instance->retryState.msg) && instance->neighbors[i].rssi > best)
    {
      best = instance->neighbors[i].rssi;
      bestMac = instance->neighbors[i].mac;
    }
  }
  if (bestMac) {
    if (instance->debugMode) {
      char payloadPreview[17] = {0};
      strncpy(payloadPreview, (char*)instance->retryState.msg.payload, 16);
      instance->debugLog("ESP-NOW SEND: Retry via %02X:%02X:%02X:%02X:%02X:%02X, from %02X:%02X:%02X:%02X:%02X:%02X, payload: %.16s%s",
              bestMac[0], bestMac[1], bestMac[2], bestMac[3], bestMac[4], bestMac[5],
              instance->retryState.msg.sender[0], instance->retryState.msg.sender[1], instance->retryState.msg.sender[2],
              instance->retryState.msg.sender[3], instance->retryState.msg.sender[4], instance->retryState.msg.sender[5],
              payloadPreview, strlen((char*)instance->retryState.msg.payload) > 16 ? "..." : "");
    }
    esp_now_send(bestMac, (uint8_t *)&instance->retryState.msg, sizeof(instance->retryState.msg));
  }
}

// Add implementation for the isInPath function
bool ESPNowMesh::isInPath(const uint8_t *mac, const MeshPacket &msg)
{
  // Check if the MAC address is already in the message path
  for (int i = 0; i < msg.path_len; i++)
  {
    if (memcmp(mac, msg.path[i], 6) == 0)
    {
      if (debugMode)
      {
        Serial.print("[MESH] MAC found in path at position ");
        Serial.println(i);
      }
      return true;
    }
  }
  return false;
}

// Modify _onRecv to properly handle unicast messages
void ESPNowMesh::_onRecv(const uint8_t *mac, const uint8_t *data, int len, int rssi)
{
  if (len != sizeof(MeshPacket))
  {
    if (debugMode)
      Serial.println("[MESH] Invalid packet size");
    return;
  }

  MeshPacket msg;
  memcpy(&msg, data, sizeof(msg));
  // Now using the rssi parameter passed from _onRecvStatic

  // Ensure null termination of payload
  msg.payload[MESH_PAYLOAD_LEN - 1] = '\0';

  if (debugMode)
  {
    Serial.print("[MESH] Received from ");
    for (int i = 0; i < 6; i++)
    {
      Serial.printf("%02X", mac[i]);
      if (i < 5)
        Serial.print(":");
    }
    Serial.printf(", RSSI: %d, Msg: %s\n", rssi, msg.payload);
  }

  // Handle acknowledgment messages
  if (strncmp(msg.payload, "ACK|", 4) == 0)
  {
    uint32_t ack_id = strtoul(msg.payload + 4, nullptr, 10);
    if (debugMode)
    {
      debugLog("Received ACK message for ID 0x%08X", ack_id);
    }

    // Process the ACK locally - maybe we're waiting for it
    _handleAck(ack_id, mac);

    return;
  }

  // Check if this is a duplicate message
  if (isDuplicate(msg.sender, msg.msg_id))
  {
    if (debugMode)
      Serial.println("[MESH] Duplicate message, ignoring");
    return;
  }

  // Add to deduplication cache
  addToCache(msg.sender, msg.msg_id);

  // Get our MAC address
  uint8_t ourMac[6];
  WiFi.macAddress(ourMac);

  // Check if the message is intended for us (broadcast or direct to our MAC)
  bool isUnicast = !isBroadcast(msg.receiver);
  bool isForUs = isUnicast ? (memcmp(msg.receiver, ourMac, 6) == 0) : true; // All broadcasts are for us

  // Flag for internal protocol messages that shouldn't be passed to user callback
  bool isInternalProtocolMsg = (strcmp(msg.payload, "DISCOVERY_REQ") == 0 ||
                                strncmp(msg.payload, "DISCOVERY_RSP|", 14) == 0);

  if (isForUs)
  {
    if (debugMode)
    {
      if (isUnicast)
      {
        Serial.println("[MESH] Unicast message addressed to us received");
      }
      else
      {
        Serial.println("[MESH] Broadcast message received");
      }
    }

    // Deliver to application only if not an internal protocol message
    if (userCallback && !isInternalProtocolMsg)
    {
      if (debugMode)
        Serial.println("[MESH] Message is for us, calling user callback");
      userCallback(msg.payload, msg.sender);
    }
    else if (debugMode && isInternalProtocolMsg)
    {
      Serial.println("[MESH] Internal protocol message, not calling user callback");
    }

    // Process internal protocol messages
    if (strcmp(msg.payload, "DISCOVERY_REQ") == 0)
    {
      if (debugMode)
        Serial.println("[MESH] Processing discovery request");

      char rsp[64];
      snprintf(rsp, sizeof(rsp), "DISCOVERY_RSP|%s|%lu", selfRole.c_str(), millis());

      delay(random(10, 50)); // Avoid collisions
      send(rsp, mac, 1, 0);
      // No need to continue processing after handling discovery request
    }
    else if (strncmp(msg.payload, "DISCOVERY_RSP|", 14) == 0)
    {
      if (debugMode)
        Serial.println("[MESH] Processing discovery response");

      char buf[64];
      strncpy(buf, msg.payload + 14, sizeof(buf) - 1);
      buf[sizeof(buf) - 1] = '\0';

      char *role = strtok(buf, "|");
      if (role)
      {
        if (debugMode)
        {
          Serial.print("[MESH] Adding/updating neighbor with role: ");
          Serial.println(role);
        }
        updateNeighbor(mac, role, rssi);
      }
      // No need to continue processing after handling discovery response
    }
    else if (isUnicast)
    {
      // Send acknowledgment if unicast - ensure we do this for any unicast message
      if (debugMode)
      {
        debugLog("Sending ACK for message ID 0x%08X to %02X:%02X:%02X:%02X:%02X:%02X",
                 msg.msg_id, msg.sender[0], msg.sender[1], msg.sender[2],
                 msg.sender[3], msg.sender[4], msg.sender[5]);
      }

      // Create ACK message
      char ackMsg[32];
      snprintf(ackMsg, sizeof(ackMsg), "ACK|%u", msg.msg_id);
      
      // Use the existing send method with msg.sender as target
      // This will automatically handle best route selection and peer management
      send(ackMsg, msg.sender, MESH_TTL_DEFAULT, 0);
    }
  }
  else if (debugMode)
  {
    Serial.println("[MESH] Message is not intended for us");
  }

  // Forward message if:
  // - TTL is not zero
  // - We're not already in the path (to prevent loops)
  // - It's a broadcast message OR a unicast message not for us
  if (msg.ttl > 0 && !isInPath(ourMac, msg) &&
      (!isUnicast || !isForUs))
  {
    if (debugMode)
    {
      Serial.printf("[MESH] Forwarding message (TTL: %d -> %d)\n", msg.ttl, msg.ttl - 1);
    }

    // Add ourselves to the path to prevent loops
    if (msg.path_len < MESH_MAX_PATH)
    {
      memcpy(msg.path[msg.path_len++], ourMac, 6);
    }

    // Decrement TTL
    msg.ttl--;

    // Only forward if signal is strong enough
    if (rssi >= rssiThreshold)
    {
      esp_err_t result = ESP_FAIL;
      
      // For unicast messages, try to use directed unicast forwarding
      if (isUnicast && useUnicast)
      {
        // If we know the target node directly, send to it directly
        bool directPathFound = false;
        
        // Check if the target is one of our immediate neighbors
        for (uint8_t i = 0; i < neighborCount; i++)
        {
          if (memcmp(msg.receiver, neighbors[i].mac, 6) == 0)
          {
            // Target node is a direct neighbor - forward directly to it
            if (debugMode)
            {
              debugLog("Target is direct neighbor, forwarding unicast directly to %02X:%02X:%02X:%02X:%02X:%02X",
                      msg.receiver[0], msg.receiver[1], msg.receiver[2],
                      msg.receiver[3], msg.receiver[4], msg.receiver[5]);
            }
            
            if (debugMode) {
              uint8_t ourMac[6];
              WiFi.macAddress(ourMac);
              char payloadPreview[17] = {0};
              strncpy(payloadPreview, (char*)msg.payload, 16);
              debugLog("ESP-NOW SEND: Direct forwarding to %02X:%02X:%02X:%02X:%02X:%02X, from %02X:%02X:%02X:%02X:%02X:%02X, payload: %.16s%s",
                      msg.receiver[0], msg.receiver[1], msg.receiver[2], msg.receiver[3], msg.receiver[4], msg.receiver[5],
                      ourMac[0], ourMac[1], ourMac[2], ourMac[3], ourMac[4], ourMac[5],
                      payloadPreview, strlen((char*)msg.payload) > 16 ? "..." : "");
            }
            result = esp_now_send(msg.receiver, (uint8_t *)&msg, sizeof(msg));
            directPathFound = true;
            break;
          }
        }
        
        // If direct path wasn't found, try to find best next hop
        if (!directPathFound)
        {
          // Find best neighbor based on RSSI to forward the message
          int bestRSSI = -128;
          uint8_t *bestMac = nullptr;
          
          for (uint8_t i = 0; i < neighborCount; i++)
          {
            if (!isInPath(neighbors[i].mac, msg) && neighbors[i].rssi > bestRSSI)
            {
              bestRSSI = neighbors[i].rssi;
              bestMac = neighbors[i].mac;
            }
          }
          
          if (bestMac)
          {
            if (debugMode)
            {
              debugLog("Forwarding unicast message via best neighbor %02X:%02X:%02X:%02X:%02X:%02X, RSSI: %d",
                      bestMac[0], bestMac[1], bestMac[2], bestMac[3], bestMac[4], bestMac[5], bestRSSI);
            }
            
            if (debugMode) {
              uint8_t ourMac[6];
              WiFi.macAddress(ourMac);
              char payloadPreview[17] = {0};
              strncpy(payloadPreview, (char*)msg.payload, 16);
              debugLog("ESP-NOW SEND: Forward via best neighbor %02X:%02X:%02X:%02X:%02X:%02X, from %02X:%02X:%02X:%02X:%02X:%02X, payload: %.16s%s",
                      bestMac[0], bestMac[1], bestMac[2], bestMac[3], bestMac[4], bestMac[5],
                      ourMac[0], ourMac[1], ourMac[2], ourMac[3], ourMac[4], ourMac[5],
                      payloadPreview, strlen((char*)msg.payload) > 16 ? "..." : "");
            }
            result = esp_now_send(bestMac, (uint8_t *)&msg, sizeof(msg));
          }
          else if (debugMode)
          {
            debugLog("No suitable unicast neighbor found for forwarding, falling back to broadcast");
          }
        }
      }
      
      // Broadcast fallback - for broadcast messages or if unicast forwarding failed
      if (!isUnicast || result != ESP_OK)
      {
        // Use efficient peer management for broadcast address
        uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        managePeer(broadcastAddr, wifiChannel, false);
        
        if (debugMode && isUnicast)
        {
          debugLog("Unicast forwarding failed or disabled, using broadcast");
        }
        
        if (debugMode) {
          uint8_t ourMac[6];
          WiFi.macAddress(ourMac);
          char payloadPreview[17] = {0};
          strncpy(payloadPreview, (char*)msg.payload, 16);
          debugLog("ESP-NOW SEND: Broadcast forwarding from %02X:%02X:%02X:%02X:%02X:%02X, payload: %.16s%s",
                  ourMac[0], ourMac[1], ourMac[2], ourMac[3], ourMac[4], ourMac[5],
                  payloadPreview, strlen((char*)msg.payload) > 16 ? "..." : "");
        }
        result = esp_now_send(nullptr, (uint8_t *)&msg, sizeof(msg));
      }

      if (result != ESP_OK)
      {
        if (debugMode)
          Serial.println("[MESH] Forward failed");
      }
      else if (debugMode)
      {
        Serial.println("[MESH] Message forwarded successfully");
      }
    }
    else if (debugMode)
    {
      Serial.printf("[MESH] Not forwarding - RSSI too low (%d < %d)\n",
                    rssi, rssiThreshold);
    }
  }
  else if (debugMode)
  {
    // Explain why we're not forwarding
    if (msg.ttl == 0)
    {
      Serial.println("[MESH] Not forwarding - TTL expired");
    }
    else if (isInPath(ourMac, msg))
    {
      Serial.println("[MESH] Not forwarding - already in path");
    }
    else if (isUnicast && isForUs)
    {
      Serial.println("[MESH] Not forwarding unicast message addressed to us");
    }
  }
}

// Improve the isDuplicate method to be more reliable and add timestamp checking
bool ESPNowMesh::isDuplicate(uint8_t *sender, uint32_t msg_id)
{
  unsigned long currentTime = millis();
  const unsigned long MESSAGE_TTL_MS = 30000; // Consider messages older than 30 seconds as non-duplicates

  for (int i = 0; i < MESH_CACHE_SIZE; i++)
  {
    // Check if entry exists and matches
    if (cache[i].msg_id == msg_id && memcmp(cache[i].sender, sender, 6) == 0)
    {
      // Check if the message is too old (handle millis() overflow)
      if (currentTime >= cache[i].timestamp &&
          (currentTime - cache[i].timestamp) < MESSAGE_TTL_MS)
      {
        if (debugMode)
        {
          debugLog("Duplicate message ID 0x%08X from %02X:%02X:%02X:%02X:%02X:%02X detected",
                   msg_id, sender[0], sender[1], sender[2], sender[3], sender[4], sender[5]);
        }
        return true;
      }
      else
      {
        // Entry exists but it's too old, update it as if it's new
        cache[i].timestamp = currentTime;
        if (debugMode)
        {
          debugLog("Old cache entry refreshed for message ID 0x%08X", msg_id);
        }
        return false;
      }
    }
  }
  return false;
}

// Improve addToCache method to include timestamp
void ESPNowMesh::addToCache(uint8_t *sender, uint32_t msg_id)
{
  memcpy(cache[cacheIndex].sender, sender, 6);
  cache[cacheIndex].msg_id = msg_id;
  cache[cacheIndex].timestamp = millis();

  if (debugMode)
  {
    debugLog("Added message ID 0x%08X from %02X:%02X:%02X:%02X:%02X:%02X to deduplication cache at index %d",
             msg_id, sender[0], sender[1], sender[2], sender[3], sender[4], sender[5], cacheIndex);
  }

  cacheIndex = (cacheIndex + 1) % MESH_CACHE_SIZE;
}

void ESPNowMesh::updateNeighbor(const uint8_t *mac, const char *role, int rssi)
{
  // Check if neighbor already exists
  for (uint8_t i = 0; i < neighborCount; i++)
  {
    if (memcmp(mac, neighbors[i].mac, 6) == 0)
    {
      // Update existing neighbor
      neighbors[i].lastSeen = millis();
      neighbors[i].rssi = rssi;
      neighbors[i].role = role;

      debugLog("Updated neighbor %02X:%02X:%02X:%02X:%02X:%02X, Role: %s, RSSI: %d",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], role, rssi);
      return;
    }
  }

  // Add new neighbor if space available
  if (neighborCount < MESH_CACHE_SIZE)
  {
    memcpy(neighbors[neighborCount].mac, mac, 6);
    neighbors[neighborCount].role = role;
    neighbors[neighborCount].rssi = rssi;
    neighbors[neighborCount].lastSeen = millis();

    debugLog("Added new neighbor %02X:%02X:%02X:%02X:%02X:%02X, Role: %s, RSSI: %d",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], role, rssi);

    neighborCount++;
  }
  else
  {
    debugLog("Neighbor table full, dropping new neighbor");
  }
}

bool ESPNowMesh::isBroadcast(const uint8_t *mac)
{
  static const uint8_t b[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  return !memcmp(mac, b, 6);
}

void ESPNowMesh::onReceive(MeshCallback cb)
{
  userCallback = cb;
  debugLog("Receive callback registered");
}

void ESPNowMesh::debugLog(const char *fmt, ...)
{
  if (!debugMode)
    return;

  char buffer[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  Serial.print("[MESH] ");
  Serial.println(buffer);
}

// Implementation of end-to-end acknowledgment API methods
void ESPNowMesh::sendReliably(const char *msg, const uint8_t *target_mac, uint8_t ttl)
{
  if (!target_mac)
  {
    if (debugMode)
    {
      debugLog("Error: sendReliably requires a target MAC address");
    }
    return;
  }

  // Check if there's a free slot for tracking this message
  int slot = findFreePendingSlot();
  if (slot < 0)
  {
    if (debugMode)
    {
      debugLog("Error: No free slots for reliable message tracking");
    }
    return;
  }

  // Generate a unique message ID - this will be used for tracking the acknowledgment
  uint32_t msgId = generateMsgId();

  if (debugMode)
  {
    debugLog("Sending reliable message with ID 0x%08X to %02X:%02X:%02X:%02X:%02X:%02X",
             msgId, target_mac[0], target_mac[1], target_mac[2],
             target_mac[3], target_mac[4], target_mac[5]);
  }

  // Store in pending slot with the tracking ID
  pendingAcks[slot].active = true;
  pendingAcks[slot].msg_id = msgId;
  memcpy(pendingAcks[slot].dest_mac, target_mac, 6);
  strlcpy(pendingAcks[slot].payload, msg, MESH_PAYLOAD_LEN);
  pendingAcks[slot].timestamp = millis();
  pendingAcks[slot].attempts = 1; // First attempt
  pendingAcks[slot].ttl = ttl;

  // Send the message using the regular send method but with our specific message ID
  send(msg, target_mac, ttl, msgId);
}

// Set acknowledgment timeout (milliseconds)
void ESPNowMesh::setAckTimeout(uint32_t timeout_ms)
{
  ackTimeout = timeout_ms;
  if (debugMode)
  {
    debugLog("ACK timeout set to %u ms", timeout_ms);
  }
}

// Set maximum acknowledgment retries
void ESPNowMesh::setAckRetries(uint8_t retries)
{
  ackRetries = retries;
  if (debugMode)
  {
    debugLog("ACK max retries set to %u", retries);
  }
}

// Register success callback
void ESPNowMesh::onSendSuccess(AckCallback cb)
{
  onSuccessCallback = cb;
  if (debugMode)
  {
    debugLog("Success callback registered");
  }
}

// Register failure callback
void ESPNowMesh::onSendFailure(AckCallback cb)
{
  onFailureCallback = cb;
  if (debugMode)
  {
    debugLog("Failure callback registered");
  }
}

// Find a free slot for pending acknowledgment tracking
int ESPNowMesh::findFreePendingSlot()
{
  for (int i = 0; i < MESH_MAX_PENDING_ACKS; i++)
  {
    if (!pendingAcks[i].active)
    {
      return i;
    }
  }
  return -1; // No free slots
}

// Check if a pending ACK slot is active
bool ESPNowMesh::isPendingAckActive(int index)
{
  if (index < 0 || index >= MESH_MAX_PENDING_ACKS)
  {
    return false;
  }
  return pendingAcks[index].active;
}

// Clear a pending ACK slot
void ESPNowMesh::clearPendingAck(int index)
{
  if (index < 0 || index >= MESH_MAX_PENDING_ACKS)
  {
    return;
  }
  pendingAcks[index].active = false;
}

// Resend a pending message
void ESPNowMesh::resendPendingMessage(int index)
{
  if (!isPendingAckActive(index))
  {
    return;
  }

  pendingAcks[index].attempts++;
  pendingAcks[index].timestamp = millis();

  if (debugMode)
  {
    debugLog("Resending message ID 0x%08X (attempt %u/%u)",
             pendingAcks[index].msg_id,
             pendingAcks[index].attempts,
             ackRetries);
  }

  // Resend the message
  send(pendingAcks[index].payload,
       pendingAcks[index].dest_mac,
       pendingAcks[index].ttl,
       pendingAcks[index].msg_id);
}

// Handle an incoming ACK
void ESPNowMesh::_handleAck(uint32_t ack_id, const uint8_t *sender)
{
  if (debugMode)
  {
    debugLog("Handling ACK for message ID 0x%08X from %02X:%02X:%02X:%02X:%02X:%02X",
             ack_id, sender[0], sender[1], sender[2], sender[3], sender[4], sender[5]);
  }

  // Look for this ID in our pending ACKs
  for (int i = 0; i < MESH_MAX_PENDING_ACKS; i++)
  {
    if (pendingAcks[i].active && pendingAcks[i].msg_id == ack_id)
    {
      // Debug logging to troubleshoot
      if (debugMode)
      {
        debugLog("Found pending ACK with matching ID 0x%08X", ack_id);
        debugLog("ACK received from: %02X:%02X:%02X:%02X:%02X:%02X",
                 sender[0], sender[1], sender[2], sender[3], sender[4], sender[5]);
        debugLog("Original message sent to: %02X:%02X:%02X:%02X:%02X:%02X",
                 pendingAcks[i].dest_mac[0], pendingAcks[i].dest_mac[1], pendingAcks[i].dest_mac[2],
                 pendingAcks[i].dest_mac[3], pendingAcks[i].dest_mac[4], pendingAcks[i].dest_mac[5]);
      }

      // Mark message as delivered when ACK ID matches
      if (debugMode)
      {
        debugLog("ACK validated for message ID 0x%08X, marking as delivered", ack_id);
      }

      // Call success callback if registered
      if (onSuccessCallback)
      {
        onSuccessCallback(ack_id, pendingAcks[i].dest_mac);
      }

      // Clear this pending ACK
      clearPendingAck(i);
      return;
    }
  }

  // If we got here, no matching pending ACK was found
  if (debugMode)
  {
    debugLog("Received ACK for unknown message ID 0x%08X", ack_id);
  }
}

// New efficient peer management that uses esp_now_mod_peer() when possible
bool ESPNowMesh::managePeer(const uint8_t* mac, uint8_t channel, bool encrypt)
{
  if (!mac) return false;
  
  // Skip peer management for broadcast addresses
  if (isBroadcast(mac)) {
    return true;  // Return success but do nothing
  }

  // Use the instance channel if not specified
  if (channel == 0) {
    channel = wifiChannel;
  }

  // Create peer info structure
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = channel;
  peerInfo.encrypt = encrypt;
  
  // Check if peer already exists
  bool peerExists = esp_now_is_peer_exist(mac);
  
  if (peerExists) {
    // Try to modify the existing peer first (more efficient)
    esp_err_t result = esp_now_mod_peer(&peerInfo);
    
    if (result == ESP_OK) {
      if (debugMode) {
        debugLog("Updated peer %02X:%02X:%02X:%02X:%02X:%02X channel: %d", 
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], channel);
      }
      return true;
    }
    else {
      // Modification failed, try delete and re-add
      if (debugMode) {
        debugLog("Failed to modify peer, trying delete and re-add");
      }
      esp_now_del_peer(mac);
    }
  }
  
  // Add the peer
  esp_err_t result = esp_now_add_peer(&peerInfo);
  
  if (result == ESP_OK) {
    if (debugMode) {
      debugLog("%s peer %02X:%02X:%02X:%02X:%02X:%02X channel: %d", 
              peerExists ? "Re-added" : "Added new", 
              mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], channel);
    }
    return true;
  }
  else {
    if (debugMode) {
      debugLog("Failed to add peer: %d", result);
    }
    return false;
  }
}

// Helper method to find the best route to a target
uint8_t* ESPNowMesh::findBestRoute(const uint8_t* targetMac, int& bestRssi)
{
  bestRssi = -128;
  static uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  
  // First check if the target is a direct neighbor with good signal
  if (targetMac) {
    bool targetIsNeighbor = false;
    int targetRssi = -128;
    
    for (uint8_t i = 0; i < neighborCount; i++) {
      if (memcmp(targetMac, neighbors[i].mac, 6) == 0) {
        targetIsNeighbor = true;
        targetRssi = neighbors[i].rssi;
        if (debugMode) {
          debugLog("Target is a direct neighbor with RSSI: %d", targetRssi);
        }
        break;
      }
    }
    
    if (targetIsNeighbor && targetRssi >= rssiThreshold) {
      // Target is reachable directly with good signal
      bestRssi = targetRssi;
      if (debugMode) {
        debugLog("Routing directly to target: %02X:%02X:%02X:%02X:%02X:%02X (RSSI: %d)",
               targetMac[0], targetMac[1], targetMac[2], targetMac[3], targetMac[4], targetMac[5], targetRssi);
      }
      return (uint8_t*)targetMac;
    }
  }
  
  // If target isn't directly reachable or has poor signal, find best next hop
  for (uint8_t i = 0; i < neighborCount; i++) {
    // Skip the target if it had poor signal
    if (targetMac && memcmp(targetMac, neighbors[i].mac, 6) == 0)
      continue;
      
    if (neighbors[i].rssi > bestRssi) {
      bestRssi = neighbors[i].rssi;
      if (debugMode) {
        debugLog("Considering neighbor %02X:%02X:%02X:%02X:%02X:%02X as best route (RSSI: %d)",
               neighbors[i].mac[0], neighbors[i].mac[1], neighbors[i].mac[2],
               neighbors[i].mac[3], neighbors[i].mac[4], neighbors[i].mac[5], neighbors[i].rssi);
      }
      return neighbors[i].mac;
    }
  }
  
  // If no good neighbors found, use broadcast as fallback
  if (debugMode) {
    debugLog("No good neighbors found for routing, using broadcast");
  }
  bestRssi = -100; // Indicate this is a fallback
  return broadcastAddr;
}
