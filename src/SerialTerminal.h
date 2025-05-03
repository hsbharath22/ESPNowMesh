/**
 * SerialTerminal.h - Serial Command Interface for ESPNowMesh
 * 
 * This class provides a consistent interface for processing serial commands
 * in ESPNowMesh applications. It allows for standard and custom commands,
 * making it easy to control mesh network operations from the serial monitor.
 */

#ifndef SERIAL_TERMINAL_H
#define SERIAL_TERMINAL_H

#include <Arduino.h>
#include "ESPNowMesh.h"

// Maximum number of custom commands that can be registered
#define MAX_CUSTOM_COMMANDS 10

class SerialTerminal {
public:
    // Command handler function type - returns true if command was handled
    typedef bool (*CommandHandler)(const String& args, ESPNowMesh& mesh);

    // Constructor
    SerialTerminal(ESPNowMesh& meshInstance, Stream& serial = Serial);
    
    // Process any available serial input (call this in loop)
    void process();
    
    // Configuration
    void setCommandPrefix(char prefix);
    void enableEcho(bool enable);
    void enablePrompt(bool enable);
    void setPrompt(const String& prompt);
    
    // Custom commands
    bool addCommand(const String& command, const String& helpText, CommandHandler handler);
    
    // Print help and status information
    void printHelp();
    void printStatus();
    
private:
    struct CustomCommand {
        String command;
        String helpText;
        CommandHandler handler;
        bool active;
    };
    
    ESPNowMesh& mesh;
    Stream& serial;
    char commandPrefix = '/';
    bool echoEnabled = true;
    bool promptEnabled = true;
    String prompt = "> ";
    uint8_t defaultTTL = MESH_TTL_DEFAULT;
    
    // Custom command storage
    CustomCommand customCommands[MAX_CUSTOM_COMMANDS];
    uint8_t customCommandCount = 0;
    
    // Command buffer
    String commandBuffer;
    bool processingCommand = false;
    
    // Internal command handlers
    void handleCommand(String cmd);
    
    // Standard command handlers
    void cmdDiscovery();
    void cmdListNeighbors();
    void cmdBroadcast(const String& msg);
    void cmdUnicast(const String& args);
    void cmdReliable(const String& args); // New reliable send method
    void cmdSetRole(const String& role);
    void cmdSetTTL(const String& ttlStr);
    void cmdDebug(const String& mode);
    void cmdPing();
    
    // Helper functions
    void printMac(const uint8_t* mac);
    bool macStringToBytes(const String& macStr, uint8_t* targetMac);
};

#endif // SERIAL_TERMINAL_H