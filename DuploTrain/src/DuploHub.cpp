/**
 * DuploHub - A class to encapsulate LEGO DUPLO Train Hub functionality
 * 
 * (c) Copyright 2025
 * Released under MIT License
 * 
 */

#include "Arduino.h"
#include "DuploHub.h"

// Default constructor
DuploHub::DuploHub() : motorPort((byte)PoweredUpHubPort::A), wasConnected(false),
                       onConnectedCallback(nullptr), onDisconnectedCallback(nullptr) {
}

// Constructor with port specification
DuploHub::DuploHub(byte port) : motorPort(port), wasConnected(false),
                                onConnectedCallback(nullptr), onDisconnectedCallback(nullptr) {
}

// Initialize the hub
void DuploHub::init() {
    hub.init();
}

// Initialize the hub with specific address
void DuploHub::init(const std::string& address) {
    hub.init(address);
}

// Attempt to connect to the hub
bool DuploHub::connect() {
    hub.connectHub();
    if (hub.isConnected()) {
        Serial.println("Connected to HUB");
        Serial.print("Hub address: ");
        Serial.println(hub.getHubAddress().toString().c_str());
        Serial.print("Hub name: ");
        Serial.println(hub.getHubName().c_str());
        return true;
    } else {
        Serial.println("Failed to connect to HUB");
        return false;
    }
}

// Check if hub is connected
bool DuploHub::isConnected() {
    return hub.isConnected();
}

// Check if hub is connecting
bool DuploHub::isConnecting() {
    return hub.isConnecting();
}

// Check if hub is disconnected (not connecting and not connected)
bool DuploHub::isDisconnected() {
    return !hub.isConnecting() && !hub.isConnected();
}

// Set hub name
void DuploHub::setHubName(const char* name) {
    char hubName[strlen(name) + 1];
    strcpy(hubName, name);
    hub.setHubName(hubName);
}

// Get hub address
std::string DuploHub::getHubAddress() {
    return hub.getHubAddress().toString();
}

// Get hub name
std::string DuploHub::getHubName() {
    return hub.getHubName();
}

// Set LED color
void DuploHub::setLedColor(Color color) {
    hub.setLedColor(color);
}

// Set motor speed
void DuploHub::setMotorSpeed(int speed) {
    hub.setBasicMotorSpeed(motorPort, speed);
}

// Stop motor
void DuploHub::stopMotor() {
    hub.stopBasicMotor(motorPort);
}

// Set motor port
void DuploHub::setMotorPort(byte port) {
    motorPort = port;
}

// Get motor port
byte DuploHub::getMotorPort() {
    return motorPort;
}

// Set callback for when hub connects
void DuploHub::setOnConnectedCallback(ConnectionCallback callback) {
    onConnectedCallback = callback;
}

// Set callback for when hub disconnects
void DuploHub::setOnDisconnectedCallback(ConnectionCallback callback) {
    onDisconnectedCallback = callback;
}

// Main update method - handles connection logic
void DuploHub::update() {
    static bool wasEverConnected = false;  // Local static variable to track if we ever had a connection
    
    // Initialize if not connected and not connecting
    if (isDisconnected()) {
        if (!wasEverConnected) {
            // First time trying to connect
            Serial.println("Attempting initial connection to hub...");
        } else {
            // Attempting to reconnect after previous connection was lost
            Serial.println("Attempting to reconnect to hub...");
        }
        hub.init();
    }
    
    // Handle connection process
    if (isConnecting()) {
        connect();
    }
    
    // Check for connection state changes and trigger callbacks
    bool currentlyConnected = isConnected();
    
    if (currentlyConnected && !wasConnected) {
        // Connection established
        wasEverConnected = true;  // Mark that we've had at least one successful connection
        Serial.println("Train hub is connected");
        if (onConnectedCallback != nullptr) {
            onConnectedCallback();
        }
    } else if (!currentlyConnected && wasConnected) {
        // Connection lost
        Serial.println("Train hub is disconnected");
        if (onDisconnectedCallback != nullptr) {
            onDisconnectedCallback();
        }
    }
    
    // Update the connection state for next iteration
    wasConnected = currentlyConnected;
}
