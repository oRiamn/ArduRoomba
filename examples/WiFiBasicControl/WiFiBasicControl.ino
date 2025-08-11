#include "ArduRoomba.h"
#include "wireless/ArduRoombaWiFi.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

// WiFi credentials - UPDATE THESE FOR YOUR NETWORK
const char* WIFI_SSID = "YourWiFiNetwork";
const char* WIFI_PASSWORD = "YourWiFiPassword";
const char* DEVICE_HOSTNAME = "arduroomba";

// Optional: Set API key for authentication (leave empty to disable)
const char* API_KEY = ""; // Example: "your-secret-api-key"

// Web server port
const uint16_t SERVER_PORT = 80;

// Roomba connection pins
const uint8_t ROOMBA_RX_PIN = 2;  // Arduino pin connected to Roomba TX
const uint8_t ROOMBA_TX_PIN = 3;  // Arduino pin connected to Roomba RX  
const uint8_t ROOMBA_BRC_PIN = 4; // Arduino pin connected to Roomba BRC

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

ArduRoomba::ArduRoomba roomba(ROOMBA_RX_PIN, ROOMBA_TX_PIN, ROOMBA_BRC_PIN);
ArduRoomba::ArduRoombaWiFi wifi(roomba);

// Status tracking
unsigned long lastStatusPrint = 0;
const unsigned long STATUS_INTERVAL = 30000; // Print status every 30 seconds

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000); // Give serial time to initialize
    
    Serial.println();
    Serial.println("=================================");
    Serial.println("ArduRoomba WiFi Control Example");
    Serial.println("=================================");
    Serial.println();
    
    // Enable debug output
    wifi.setDebugEnabled(true);
    
    // Initialize Roomba
    Serial.println("Initializing Roomba...");

     // Enable debug output
    roomba.setDebug(true);
    
    // Initialize connection to Roomba
    if (roomba.begin()) {
        Serial.println("Roomba connected successfully!");
    } else {
        Serial.println("Failed to connect to Roomba!");
        Serial.println("Check wiring and power.");
    }

    // Initialize WiFi
    Serial.println();
    Serial.println("Connecting to WiFi...");
    Serial.print("SSID: ");
    Serial.println(WIFI_SSID);
    
    ArduRoomba::ErrorCode wifiResult = wifi.begin(WIFI_SSID, WIFI_PASSWORD, DEVICE_HOSTNAME);
    
    if (wifiResult == ArduRoomba::ErrorCode::SUCCESS) {
        // Get connection info
        IPAddress ip;
        int32_t rssi;
        String ssid;
        wifi.getConnectionInfo(ip, rssi, ssid);
        
        Serial.println("✓ WiFi connected successfully!");
        Serial.print("IP Address: ");
        Serial.println(ip);
        Serial.print("Signal Strength: ");
        Serial.print(rssi);
        Serial.println(" dBm");
        
        // Set API key if provided
        if (strlen(API_KEY) > 0) {
            wifi.setAPIKey(API_KEY);
            Serial.println("✓ API authentication enabled");
        }
        
        // Start web server
        Serial.println();
        Serial.println("Starting web server...");
        ArduRoomba::ErrorCode serverResult = wifi.startWebServer(SERVER_PORT);
        
        if (serverResult == ArduRoomba::ErrorCode::SUCCESS) {
            Serial.println("✓ Web server started successfully!");
            Serial.println();
            Serial.println("=================================");
            Serial.println("READY FOR CONNECTIONS");
            Serial.println("=================================");
            Serial.println();
            Serial.print("Web Interface: ");
            Serial.println(wifi.getServerURL());
            Serial.println();
            
            if (strlen(API_KEY) > 0) {
                Serial.println("Authentication required: Include 'X-API-Key: " + String(API_KEY) + "' header");
                Serial.println();
            }
            
            Serial.println("Example API calls:");
            Serial.println("  curl " + wifi.getServerURL() + "/api/status");
            Serial.println("  curl -X POST " + wifi.getServerURL() + "/api/movement/forward");
            Serial.println("  curl -X POST " + wifi.getServerURL() + "/api/movement/stop");
            Serial.println();
            
        } else {
            Serial.print("✗ Failed to start web server. Error code: ");
            Serial.println(static_cast<uint8_t>(serverResult));
        }
        
    } else {
        Serial.print("✗ WiFi connection failed. Error code: ");
        Serial.println(static_cast<uint8_t>(wifiResult));
        Serial.println("Check your WiFi credentials and try again.");
    }
    
    Serial.println("Setup complete. Entering main loop...");
    Serial.println();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Handle incoming web requests
    wifi.handleClient();
    
    // Print periodic status updates
    if (millis() - lastStatusPrint > STATUS_INTERVAL) {
        printStatus();
        lastStatusPrint = millis();
    }
    
    // Check for WiFi disconnection and attempt reconnection
    if (wifi.getStatus() == ArduRoomba::WiFiStatus::DISCONNECTED) {
        Serial.println("WiFi disconnected. Attempting reconnection...");
        wifi.reconnect();
    }
    
    // Small delay to prevent overwhelming the system
    delay(10);
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Print current system status
 */
void printStatus() {
    Serial.println("--- Status Update ---");
    
    // WiFi status
    ArduRoomba::WiFiStatus wifiStatus = wifi.getStatus();
    Serial.print("WiFi: ");
    switch (wifiStatus) {
        case ArduRoomba::WiFiStatus::CONNECTED:
            Serial.print("Connected");
            break;
        case ArduRoomba::WiFiStatus::DISCONNECTED:
            Serial.print("Disconnected");
            break;
        case ArduRoomba::WiFiStatus::CONNECTING:
            Serial.print("Connecting");
            break;
        case ArduRoomba::WiFiStatus::AP_MODE:
            Serial.print("Access Point Mode");
            break;
        default:
            Serial.print("Unknown");
            break;
    }
    
    if (wifiStatus == ArduRoomba::WiFiStatus::CONNECTED || 
        wifiStatus == ArduRoomba::WiFiStatus::AP_MODE) {
        IPAddress ip;
        int32_t rssi;
        String ssid;
        wifi.getConnectionInfo(ip, rssi, ssid);
        Serial.print(" (");
        Serial.print(ip);
        Serial.print(")");
    }
    Serial.println();
    
    // Server status
    Serial.print("Web Server: ");
    Serial.println(wifi.isServerRunning() ? "Running" : "Stopped");
    
    // Statistics
    unsigned long uptime;
    uint32_t requests;
    uint16_t errors;
    wifi.getStatistics(uptime, requests, errors);
    
    Serial.print("Uptime: ");
    Serial.print(uptime / 1000);
    Serial.print("s | Requests: ");
    Serial.print(requests);
    Serial.print(" | Errors: ");
    Serial.println(errors);
    
    // Memory status
    Serial.print("Free Heap: ");
#if defined(ESP32) || defined(ESP8266)
    Serial.print(ESP.getFreeHeap());
#else
    Serial.print("N/A");
#endif
    Serial.println(" bytes");
    
    Serial.println();
}