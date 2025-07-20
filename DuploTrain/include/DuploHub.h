/**
 * DuploHub - A class to encapsulate LEGO DUPLO Train Hub functionality
 * 
 * (c) Copyright 2025
 * Released under MIT License
 * 
 */

#ifndef DUPLO_HUB_H
#define DUPLO_HUB_H

#include "Lpf2Hub.h"

// Callback function types
typedef void (*ConnectionCallback)();

class DuploHub {
private:
    Lpf2Hub hub;
    byte motorPort;
    bool wasConnected;  // Track previous connection state
    
    // Callback functions
    ConnectionCallback onConnectedCallback;
    ConnectionCallback onDisconnectedCallback;
    
public:
    // Constructor
    DuploHub();
    DuploHub(byte port);
    
    // Initialization and connection management
    void init();
    void init(const std::string& address);
    bool connect();
    bool isConnected();
    bool isConnecting();
    bool isDisconnected();
    
    // Hub information and settings
    void setHubName(const char* name);
    std::string getHubAddress();
    std::string getHubName();
    
    // LED control
    void setLedColor(Color color);
    
    // Motor control
    void setMotorSpeed(int speed);
    void stopMotor();
    void setMotorPort(byte port);
    byte getMotorPort();
    
    // Callback registration
    void setOnConnectedCallback(ConnectionCallback callback);
    void setOnDisconnectedCallback(ConnectionCallback callback);
    
    // Main update loop
    void update();
};

#endif // DUPLO_HUB_H
