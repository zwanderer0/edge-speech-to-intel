/*
 * ESP32 Voice Recorder - zwanderer
 * 
 * Captures audio via I2S microphone and streams to AI Voice Recorder
 * Features: WebSocket streaming, VAD integration, status indicators
 * 
 * Hardware Requirements:
 * - ESP32 development board
 * - INMP441 I2S microphone
 * - Status LED (optional)
 * - Push button (optional)
 * 
 * Wiring:
 * ESP32 GPIO22 -> INMP441 SCK
 * ESP32 GPIO23 -> INMP441 WS
 * ESP32 GPIO21 -> INMP441 SD
 * ESP32 3.3V   -> INMP441 VCC
 * ESP32 GND    -> INMP441 GND
 * ESP32 GND    -> INMP441 L/R
 * 
 * Author: zwanderer
 * License: MIT
 */

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <driver/i2s.h>

// ============================================================================
// CONFIGURATION SECTION - MODIFY THESE VALUES
// ============================================================================

// WiFi Configuration
const char* ssid = "YOUR_WIFI_SSID";         // Replace with your WiFi name
const char* password = "YOUR_WIFI_PASSWORD"; // Replace with your WiFi password

// Server Configuration  
const char* websocket_host = "192.168.1.100"; // Replace with your computer's IP
const int websocket_port = 8080;
const char* websocket_path = "/audio";

// Hardware Pin Configuration
#define I2S_WS_PIN 22      // Word Select (LRCLK) - Blue wire
#define I2S_SCK_PIN 23     // Serial Clock (BCLK) - White wire  
#define I2S_SD_PIN 21      // Serial Data (DIN) - Grey wire
#define I2S_PORT I2S_NUM_0

#define LED_PIN 2          // Status LED pin
#define BUTTON_PIN 0       // Manual recording button pin

// Audio Configuration
#define SAMPLE_RATE 16000          // 16kHz sample rate for voice
#define BITS_PER_SAMPLE 16         // 16-bit audio
#define CHANNELS 1                 // Mono audio
#define BUFFER_SIZE 1024           // Audio buffer size
#define DMA_BUFFER_COUNT 8         // Number of DMA buffers
#define DMA_BUFFER_LEN 1024        // Length of each DMA buffer

// Voice Activity Detection
#define VAD_THRESHOLD 500          // Adjust based on environment noise
#define MIN_AUDIO_LEVEL 100        // Minimum level to consider as audio

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

WebSocketsClient webSocket;
bool isConnected = false;
bool isRecording = false;
bool buttonPressed = false;
unsigned long lastButtonPress = 0;

int16_t audioBuffer[BUFFER_SIZE];
unsigned long lastStatusUpdate = 0;
unsigned long recordingStartTime = 0;
int packetsTransmitted = 0;

// ============================================================================
// MAIN SETUP AND LOOP
// ============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32 Voice Recorder - zwanderer ===");
    
    // Initialize hardware
    setupPins();
    setupI2S();
    
    // Connect to WiFi
    connectWiFi();
    
    // Initialize WebSocket
    setupWebSocket();
    
    Serial.println("=== ESP32 Voice Recorder Ready ===");
    printDeviceInfo();
}

void loop() {
    // Handle WebSocket communication
    webSocket.loop();
    
    // Handle button press with debouncing
    handleButton();
    
    // Capture and transmit audio if connected
    if (isConnected) {
        captureAndTransmitAudio();
    }
    
    // Send periodic status updates
    sendPeriodicStatus();
    
    // Small delay to prevent CPU overload
    delay(1);
}

// ============================================================================
// HARDWARE INITIALIZATION
// ============================================================================

void setupPins() {
    Serial.println("Initializing GPIO pins...");
    
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Initial LED state (off)
    digitalWrite(LED_PIN, LOW);
    
    Serial.println("GPIO pins initialized");
}

void setupI2S() {
    Serial.println("Initializing I2S interface...");
    
    // I2S configuration for microphone input
    i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = i2s_bits_per_sample_t(BITS_PER_SAMPLE),
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = DMA_BUFFER_COUNT,
        .dma_buf_len = DMA_BUFFER_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    
    // Pin configuration for I2S
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD_PIN
    };
    
    // Install I2S driver
    esp_err_t result = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (result != ESP_OK) {
        Serial.printf("ERROR: Failed to install I2S driver: %d\n", result);
        handleFatalError("I2S driver installation failed");
        return;
    }
    
    // Set I2S pins
    result = i2s_set_pin(I2S_PORT, &pin_config);
    if (result != ESP_OK) {
        Serial.printf("ERROR: Failed to set I2S pins: %d\n", result);
        handleFatalError("I2S pin configuration failed");
        return;
    }
    
    Serial.printf("I2S initialized: %dHz, %d-bit, %d channel(s)\n", 
                  SAMPLE_RATE, BITS_PER_SAMPLE, CHANNELS);
}

// ============================================================================
// WIFI CONNECTION
// ============================================================================

void connectWiFi() {
    Serial.printf("Connecting to WiFi: %s\n", ssid);
    
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
        
        // Blink LED during connection attempt
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nERROR: Failed to connect to WiFi");
        handleFatalError("WiFi connection failed");
        return;
    }
    
    Serial.println("\nWiFi connected successfully!");
    Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
    
    // Success blink pattern
    blinkSuccess();
}

// ============================================================================
// WEBSOCKET COMMUNICATION
// ============================================================================

void setupWebSocket() {
    Serial.printf("Connecting to WebSocket: ws://%s:%d%s\n", 
                  websocket_host, websocket_port, websocket_path);
    
    webSocket.begin(websocket_host, websocket_port, websocket_path);
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);
    webSocket.enableHeartbeat(15000, 3000, 2);
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("WebSocket: Disconnected");
            isConnected = false;
            digitalWrite(LED_PIN, LOW);
            break;
            
        case WStype_CONNECTED:
            Serial.printf("WebSocket: Connected to %s\n", payload);
            isConnected = true;
            digitalWrite(LED_PIN, HIGH);
            
            // Send initial configuration
            sendConfiguration();
            blinkSuccess();
            break;
            
        case WStype_TEXT:
            handleTextMessage((char*)payload);
            break;
            
        case WStype_BIN:
            Serial.printf("WebSocket: Received binary data (%u bytes)\n", length);
            break;
            
        case WStype_ERROR:
            Serial.printf("WebSocket: Error - %s\n", payload);
            break;
            
        case WStype_PING:
            Serial.println("WebSocket: Ping received");
            break;
            
        case WStype_PONG:
            Serial.println("WebSocket: Pong received");
            break;
            
        default:
            Serial.printf("WebSocket: Unknown event type: %d\n", type);
            break;
    }
}

void sendConfiguration() {
    DynamicJsonDocument doc(1024);
    doc["type"] = "config";
    doc["device"] = "ESP32";
    doc["author"] = "zwanderer";
    doc["version"] = "1.0.0";
    doc["sample_rate"] = SAMPLE_RATE;
    doc["channels"] = CHANNELS;
    doc["bits_per_sample"] = BITS_PER_SAMPLE;
    doc["buffer_size"] = BUFFER_SIZE;
    doc["mac_address"] = WiFi.macAddress();
    doc["ip_address"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
    
    String message;
    serializeJson(doc, message);
    webSocket.sendTXT(message);
    
    Serial.println("Configuration sent to server");
}

void handleTextMessage(const char* message) {
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
        Serial.printf("JSON parsing error: %s\n", error.c_str());
        return;
    }
    
    String type = doc["type"];
    
    if (type == "start_recording") {
        startRecording("server_command");
    } 
    else if (type == "stop_recording") {
        stopRecording("server_command");
    } 
    else if (type == "status_request") {
        sendStatus();
    }
    else if (type == "ping") {
        sendPong();
    }
    else {
        Serial.printf("Unknown message type: %s\n", type.c_str());
    }
}

// ============================================================================
// AUDIO CAPTURE AND PROCESSING
// ============================================================================

void captureAndTransmitAudio() {
    size_t bytesRead = 0;
    
    // Read audio data from I2S
    esp_err_t result = i2s_read(I2S_PORT, audioBuffer, 
                               sizeof(audioBuffer), &bytesRead, 
                               portMAX_DELAY);
    
    if (result != ESP_OK) {
        Serial.printf("ERROR: I2S read failed: %d\n", result);
        return;
    }
    
    if (bytesRead == 0) {
        return;
    }
    
    size_t samplesRead = bytesRead / sizeof(int16_t);
    
    // Perform Voice Activity Detection
    bool hasVoiceActivity = performVAD(audioBuffer, samplesRead);
    
    // Transmit audio if there's voice activity or recording is forced
    if (hasVoiceActivity || isRecording) {
        transmitAudioData(audioBuffer, bytesRead);
        
        // Blink LED during audio transmission
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}

bool performVAD(int16_t* buffer, size_t samples) {
    // Calculate RMS (Root Mean Square) energy
    long sumSquares = 0;
    int16_t maxAmplitude = 0;
    int zeroCrossings = 0;
    
    for (size_t i = 0; i < samples; i++) {
        sumSquares += (long)buffer[i] * buffer[i];
        
        // Track maximum amplitude
        int16_t absValue = abs(buffer[i]);
        if (absValue > maxAmplitude) {
            maxAmplitude = absValue;
        }
        
        // Count zero crossings (indicates voice vs noise)
        if (i > 0 && ((buffer[i-1] >= 0) != (buffer[i] >= 0))) {
            zeroCrossings++;
        }
    }
    
    // Calculate RMS
    int16_t rmsLevel = sqrt(sumSquares / samples);
    
    // Voice activity conditions:
    // 1. RMS level above threshold
    // 2. Maximum amplitude above minimum level
    // 3. Reasonable number of zero crossings (voice characteristic)
    bool hasVoice = (rmsLevel > VAD_THRESHOLD) && 
                    (maxAmplitude > MIN_AUDIO_LEVEL) && 
                    (zeroCrossings > 5) && 
                    (zeroCrossings < samples / 4);
    
    // Debug output (uncomment for testing)
    // Serial.printf("RMS: %d, Max: %d, ZC: %d, Voice: %s\n", 
    //               rmsLevel, maxAmplitude, zeroCrossings, hasVoice ? "YES" : "NO");
    
    return hasVoice;
}

void transmitAudioData(int16_t* buffer, size_t bytesToSend) {
    if (!isConnected) {
        return;
    }
    
    // Send audio data as binary via WebSocket
    bool success = webSocket.sendBIN((uint8_t*)buffer, bytesToSend);
    
    if (success) {
        packetsTransmitted++;
    } else {
        Serial.println("WARNING: Failed to send audio data");
    }
}

// ============================================================================
// RECORDING CONTROL
// ============================================================================

void startRecording(const char* trigger) {
    if (!isRecording) {
        isRecording = true;
        recordingStartTime = millis();
        packetsTransmitted = 0;
        
        Serial.printf("Recording STARTED (trigger: %s)\n", trigger);
        
        // Send status to server
        sendRecordingStatus("started", trigger);
        
        // LED indication
        digitalWrite(LED_PIN, HIGH);
    }
}

void stopRecording(const char* trigger) {
    if (isRecording) {
        isRecording = false;
        unsigned long duration = millis() - recordingStartTime;
        
        Serial.printf("Recording STOPPED (trigger: %s, duration: %lu ms, packets: %d)\n", 
                      trigger, duration, packetsTransmitted);
        
        // Send status to server
        sendRecordingStatus("stopped", trigger);
        
        // LED indication
        if (isConnected) {
            digitalWrite(LED_PIN, HIGH);
        } else {
            digitalWrite(LED_PIN, LOW);
        }
    }
}

void sendRecordingStatus(const char* status, const char* trigger) {
    if (!isConnected) return;
    
    DynamicJsonDocument doc(512);
    doc["type"] = "recording_status";
    doc["status"] = status;
    doc["trigger"] = trigger;
    doc["timestamp"] = millis();
    doc["device"] = "ESP32";
    doc["author"] = "zwanderer";
    
    if (strcmp(status, "stopped") == 0) {
        doc["duration_ms"] = millis() - recordingStartTime;
        doc["packets_sent"] = packetsTransmitted;
    }
    
    String message;
    serializeJson(doc, message);
    webSocket.sendTXT(message);
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================

void handleButton() {
    bool currentButtonState = digitalRead(BUTTON_PIN) == LOW;
    unsigned long currentTime = millis();
    
    // Button debouncing
    if (currentButtonState && !buttonPressed && 
        (currentTime - lastButtonPress) > 200) {
        
        buttonPressed = true;
        lastButtonPress = currentTime;
        
        // Toggle recording state
        if (isRecording) {
            stopRecording("button_press");
        } else {
            startRecording("button_press");
        }
        
        // Button press feedback
        blinkFeedback();
    } else if (!currentButtonState) {
        buttonPressed = false;
    }
}

// ============================================================================
// STATUS AND MONITORING
// ============================================================================

void sendPeriodicStatus() {
    unsigned long currentTime = millis();
    
    // Send status every 30 seconds
    if (isConnected && (currentTime - lastStatusUpdate) > 30000) {
        sendStatus();
        lastStatusUpdate = currentTime;
    }
}

void sendStatus() {
    if (!isConnected) return;
    
    DynamicJsonDocument doc(1024);
    doc["type"] = "device_status";
    doc["device"] = "ESP32";
    doc["author"] = "zwanderer";
    doc["timestamp"] = millis();
    doc["connected"] = isConnected;
    doc["recording"] = isRecording;
    doc["wifi_rssi"] = WiFi.RSSI();
    doc["free_heap"] = ESP.getFreeHeap();
    doc["uptime_ms"] = millis();
    doc["packets_transmitted"] = packetsTransmitted;
    doc["ip_address"] = WiFi.localIP().toString();
    doc["mac_address"] = WiFi.macAddress();
    
    String message;
    serializeJson(doc, message);
    webSocket.sendTXT(message);
}

void sendPong() {
    if (!isConnected) return;
    
    DynamicJsonDocument doc(256);
    doc["type"] = "pong";
    doc["timestamp"] = millis();
    doc["device"] = "ESP32";
    
    String message;
    serializeJson(doc, message);
    webSocket.sendTXT(message);
}

void printDeviceInfo() {
    Serial.println("\n--- Device Information ---");
    Serial.printf("Chip Model: %s\n", ESP.getChipModel());
    Serial.printf("Chip Revision: %d\n", ESP.getChipRevision());
    Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Flash Size: %d bytes\n", ESP.getFlashChipSize());
    Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("MAC Address: %s\n", WiFi.macAddress().c_str());
    Serial.println("-------------------------\n");
}

// ============================================================================
// LED CONTROL AND FEEDBACK
// ============================================================================

void blinkSuccess() {
    // Three quick blinks to indicate success
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    
    // Restore LED state based on connection
    digitalWrite(LED_PIN, isConnected ? HIGH : LOW);
}

void blinkFeedback() {
    // Single blink for button feedback
    bool currentState = digitalRead(LED_PIN);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, currentState);
}

void blinkError() {
    // Rapid blinking to indicate error
    for (int i = 0; i < 10; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
}

// ============================================================================
// ERROR HANDLING
// ============================================================================

void handleFatalError(const char* error) {
    Serial.printf("FATAL ERROR: %s\n", error);
    Serial.println("System will restart in 5 seconds...");
    
    // Error indication
    blinkError();
    
    // Wait and restart
    delay(5000);
    ESP.restart();
}

// ============================================================================
// DIAGNOSTIC FUNCTIONS
// ============================================================================

void printAudioStats() {
    static unsigned long lastPrint = 0;
    static int packetCount = 0;
    
    packetCount++;
    
    if (millis() - lastPrint > 5000) { // Print every 5 seconds
        Serial.printf("Audio Stats - Packets: %d, Free Heap: %d, RSSI: %d dBm\n", 
                      packetCount, ESP.getFreeHeap(), WiFi.RSSI());
        lastPrint = millis();
        packetCount = 0;
    }
}