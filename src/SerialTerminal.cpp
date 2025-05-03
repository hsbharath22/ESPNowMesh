#include "SerialTerminal.h"

// Constructor
SerialTerminal::SerialTerminal(ESPNowMesh& meshInstance, Stream& serialPort)
    : mesh(meshInstance), serial(serialPort) {
    
    // Initialize custom commands array
    for (int i = 0; i < MAX_CUSTOM_COMMANDS; i++) {
        customCommands[i].active = false;
    }
}

// Process serial input
void SerialTerminal::process() {
    // Check for serial data
    while (serial.available() > 0) {
        char c = serial.read();
        
        // Echo character if enabled
        if (echoEnabled) {
            serial.write(c);
        }
        
        // Process character
        if (c == '\n' || c == '\r') {
            // Process complete command
            if (commandBuffer.length() > 0) {
                if (echoEnabled && c == '\r') {
                    serial.write('\n'); // Add newline if we got CR
                }
                
                handleCommand(commandBuffer);
                commandBuffer = "";
                
                // Print prompt if enabled
                if (promptEnabled) {
                    serial.print(prompt);
                }
            }
        } 
        else if (c == 8 || c == 127) {  // Backspace or Delete
            if (commandBuffer.length() > 0) {
                commandBuffer.remove(commandBuffer.length() - 1);
                if (echoEnabled) {
                    // Send backspace-space-backspace to erase character
                    serial.write(8);
                    serial.write(' ');
                    serial.write(8);
                }
            }
        }
        else {
            commandBuffer += c;
        }
    }
}

// Set command prefix character
void SerialTerminal::setCommandPrefix(char prefix) {
    commandPrefix = prefix;
}

// Enable/disable command echo
void SerialTerminal::enableEcho(bool enable) {
    echoEnabled = enable;
}

// Enable/disable prompt
void SerialTerminal::enablePrompt(bool enable) {
    promptEnabled = enable;
}

// Set custom prompt string
void SerialTerminal::setPrompt(const String& newPrompt) {
    prompt = newPrompt;
}

// Add a custom command
bool SerialTerminal::addCommand(const String& command, const String& helpText, CommandHandler handler) {
    if (customCommandCount >= MAX_CUSTOM_COMMANDS) {
        return false;
    }
    
    customCommands[customCommandCount].command = command;
    customCommands[customCommandCount].helpText = helpText;
    customCommands[customCommandCount].handler = handler;
    customCommands[customCommandCount].active = true;
    customCommandCount++;
    
    return true;
}

// Print MAC address in readable format
void SerialTerminal::printMac(const uint8_t* mac) {
    for (int i = 0; i < 6; i++) {
        serial.printf("%02X", mac[i]);
        if (i < 5) serial.print(":");
    }
}

// Convert MAC string (with or without colons) to bytes
bool SerialTerminal::macStringToBytes(const String& macStr, uint8_t* targetMac) {
    String processedMac = macStr;
    processedMac.replace(":", ""); // Remove colons if present
    
    // Validate MAC length
    if (processedMac.length() != 12) {
        return false;
    }
    
    // Convert MAC string to bytes
    for (int i = 0; i < 6; i++) {
        char byteHex[3] = {processedMac.charAt(i*2), processedMac.charAt(i*2+1), 0};
        targetMac[i] = strtoul(byteHex, nullptr, 16);
    }
    
    return true;
}

// Handle commands
void SerialTerminal::handleCommand(String cmd) {
    cmd.trim();
    
    // Check for empty command
    if (cmd.length() == 0) {
        return;
    }
    
    // Check if command has prefix
    if (cmd.charAt(0) != commandPrefix) {
        serial.println("Commands must start with " + String(commandPrefix));
        return;
    }
    
    // Remove prefix for processing
    cmd = cmd.substring(1);
    
    // Extract command and arguments
    int spaceIndex = cmd.indexOf(' ');
    String command, args;
    
    if (spaceIndex != -1) {
        command = cmd.substring(0, spaceIndex);
        args = cmd.substring(spaceIndex + 1);
    } else {
        command = cmd;
        args = "";
    }
    
    // Convert to lowercase for case-insensitive comparison
    command.toLowerCase();
    
    // Process built-in commands
    if (command == "d" || command == "discovery") {
        cmdDiscovery();
    }
    else if (command == "l" || command == "list") {
        cmdListNeighbors();
    }
    else if (command == "s" || command == "send") {
        cmdBroadcast(args);
    }
    else if (command == "t" || command == "target") {
        cmdUnicast(args);
    }
    else if (command == "sr" || command == "sendreliable") {
        cmdReliable(args);
    }
    else if (command == "r" || command == "role") {
        cmdSetRole(args);
    }
    else if (command == "ttl") {
        cmdSetTTL(args);
    }
    else if (command == "debug") {
        cmdDebug(args);
    }
    else if (command == "ping") {
        cmdPing();
    }
    else if (command == "status") {
        printStatus();
    }
    else if (command == "help" || command == "?") {
        printHelp();
    }
    else {
        // Check custom commands
        bool foundCustom = false;
        for (int i = 0; i < customCommandCount; i++) {
            if (customCommands[i].active && customCommands[i].command.equalsIgnoreCase(command)) {
                foundCustom = customCommands[i].handler(args, mesh);
                break;
            }
        }
        
        if (!foundCustom) {
            serial.println("Unknown command. Type " + String(commandPrefix) + "help or " + 
                          String(commandPrefix) + "? for available commands.");
        }
    }
}

// Command implementations
void SerialTerminal::cmdDiscovery() {
    mesh.broadcastDiscovery();
    serial.println("Discovery broadcast initiated");
}

void SerialTerminal::cmdListNeighbors() {
    uint8_t count;
    auto* neighbors = mesh.getNeighbors(count);
    
    serial.println("\n=== NEIGHBOR LIST ===");
    serial.printf("Total count: %d\n", count);
    
    for (int i = 0; i < count; i++) {
        serial.print("  Node ");
        serial.print(i + 1);
        serial.print(": ");
        printMac(neighbors[i].mac);
        serial.print("\n    Role: ");
        serial.print(neighbors[i].role);
        serial.print("\n    RSSI: ");
        serial.print(neighbors[i].rssi);
        serial.print(" dBm\n    Last seen: ");
        serial.print((millis() - neighbors[i].lastSeen) / 1000);
        serial.println(" seconds ago");
    }
    serial.println("=====================");
}

void SerialTerminal::cmdBroadcast(const String& msg) {
    if (msg.length() == 0) {
        serial.println("Error: Message cannot be empty");
        return;
    }
    
    mesh.send(msg.c_str(), nullptr, defaultTTL);
    serial.print("Broadcast message sent: ");
    serial.println(msg);
}

void SerialTerminal::cmdUnicast(const String& args) {
    int spacePos = args.indexOf(' ');
    if (spacePos == -1) {
        serial.println("Error: Format should be " + String(commandPrefix) + "t <MAC> <message>");
        return;
    }
    
    // Extract MAC and message
    String macStr = args.substring(0, spacePos);
    String msg = args.substring(spacePos + 1);
    
    if (msg.length() == 0) {
        serial.println("Error: Message cannot be empty");
        return;
    }
    
    // Convert MAC string to bytes
    uint8_t targetMac[6];
    if (!macStringToBytes(macStr, targetMac)) {
        serial.println("Error: Invalid MAC address format. Expected 12 hex characters (with or without colons)");
        return;
    }
    
    // Show parsed MAC
    serial.print("Sending to MAC: ");
    printMac(targetMac);
    serial.println();
    
    // Send the message
    mesh.send(msg.c_str(), targetMac, defaultTTL);
    serial.print("Unicast message sent: ");
    serial.println(msg);
}

// New reliable send command implementation
void SerialTerminal::cmdReliable(const String& args) {
    int spacePos = args.indexOf(' ');
    if (spacePos == -1) {
        serial.println("Error: Format should be " + String(commandPrefix) + "sr <MAC> <message>");
        return;
    }
    
    // Extract MAC and message
    String macStr = args.substring(0, spacePos);
    String msg = args.substring(spacePos + 1);
    
    if (msg.length() == 0) {
        serial.println("Error: Message cannot be empty");
        return;
    }
    
    // Convert MAC string to bytes
    uint8_t targetMac[6];
    if (!macStringToBytes(macStr, targetMac)) {
        serial.println("Error: Invalid MAC address format. Expected 12 hex characters (with or without colons)");
        return;
    }
    
    // Show parsed MAC
    serial.print("Sending reliable message to MAC: ");
    printMac(targetMac);
    serial.println();
    
    // Send the reliable message
    mesh.sendReliably(msg.c_str(), targetMac, defaultTTL);
    serial.print("Reliable message sent: ");
    serial.println(msg);
    serial.println("Waiting for acknowledgment...");
}

void SerialTerminal::cmdSetRole(const String& role) {
    if (role.length() == 0) {
        serial.println("Error: Role cannot be empty");
        return;
    }
    
    mesh.setRole(role.c_str());
    serial.print("Role set to: ");
    serial.println(role);
}

void SerialTerminal::cmdSetTTL(const String& ttlStr) {
    if (ttlStr.length() == 0) {
        serial.println("Error: TTL value required");
        return;
    }
    
    int ttl = ttlStr.toInt();
    if (ttl <= 0 || ttl > 10) {
        serial.println("Error: TTL must be between 1 and 10");
        return;
    }
    
    defaultTTL = ttl;
    serial.print("Default TTL set to: ");
    serial.println(defaultTTL);
}

void SerialTerminal::cmdDebug(const String& mode) {
    if (mode.length() == 0) {
        serial.println("Error: Specify 'on' or 'off'");
        return;
    }
    
    bool debugOn = mode.equalsIgnoreCase("on");
    mesh.enableDebug(debugOn);
    serial.print("Debug mode set to: ");
    serial.println(debugOn ? "ON" : "OFF");
}

void SerialTerminal::cmdPing() {
    mesh.send("PING", nullptr, defaultTTL);
    serial.println("PING broadcast sent");
}

// Print available commands
void SerialTerminal::printHelp() {
    serial.println("\nAvailable commands:");
    serial.println("  " + String(commandPrefix) + "d, " + String(commandPrefix) + "discovery - Trigger discovery");
    serial.println("  " + String(commandPrefix) + "l, " + String(commandPrefix) + "list - List neighbors");
    serial.println("  " + String(commandPrefix) + "s, " + String(commandPrefix) + "send <message> - Send broadcast message");
    serial.println("  " + String(commandPrefix) + "t, " + String(commandPrefix) + "target <mac> <message> - Send unicast message");
    serial.println("  " + String(commandPrefix) + "sr, " + String(commandPrefix) + "sendreliable <mac> <message> - Send reliable message with ACK");
    serial.println("  " + String(commandPrefix) + "r, " + String(commandPrefix) + "role <role> - Set node role");
    serial.println("  " + String(commandPrefix) + "ttl <value> - Set default TTL value");
    serial.println("  " + String(commandPrefix) + "debug on|off - Enable/disable debug output");
    serial.println("  " + String(commandPrefix) + "status - Show current mesh status");
    serial.println("  " + String(commandPrefix) + "ping - Send ping to all nodes");
    serial.println("  " + String(commandPrefix) + "help, " + String(commandPrefix) + "? - Show this help message");
    
    // Print custom commands if any
    if (customCommandCount > 0) {
        serial.println("\nCustom commands:");
        for (int i = 0; i < customCommandCount; i++) {
            if (customCommands[i].active) {
                serial.print("  " + String(commandPrefix) + customCommands[i].command + " - ");
                serial.println(customCommands[i].helpText);
            }
        }
    }
}

// Print status information
void SerialTerminal::printStatus() {
    serial.println("\n=== MESH STATUS ===");
    serial.print("MAC Address: ");
    serial.println(WiFi.macAddress());
    serial.print("Role: ");
    serial.println(mesh.getRole());
    serial.print("WiFi Status: ");
    serial.print(WiFi.status());
    
    switch (WiFi.status()) {
        case WL_CONNECTED: serial.println(" (CONNECTED)"); break;
        case WL_DISCONNECTED: serial.println(" (DISCONNECTED)"); break;
        case WL_IDLE_STATUS: serial.println(" (IDLE)"); break;
        default: serial.println(" (OTHER)");
    }
    
    serial.print("RSSI: ");
    serial.print(WiFi.RSSI());
    serial.println(" dBm");
    
    serial.println("Mesh Settings:");
    serial.print("- Default TTL: ");
    serial.println(defaultTTL);
    
    // Show neighbor information
    cmdListNeighbors();
}