# Passive Embedded Sensor Network for Edge Speech Intelligence

This document describes the deployment of ESP32-based passive sensors for continuous speech monitoring in smart building environments. These sensors form a distributed mesh network optimized for 24/7 operation with minimal maintenance.

**Author:** zwanderer  
**Focus:** Embedded passive sensors for ambient intelligence  
**Target:** Residential and commercial smart building deployment

## Deployment Architecture

### Sensor Node Design Philosophy

The passive sensor network is designed around human factors principles:

- **Ambient Integration:** Sensors blend seamlessly into the environment
- **Privacy by Design:** No local audio storage, processing occurs at edge
- **Minimal Maintenance:** 5+ year deployment lifecycle with remote updates
- **Adaptive Operation:** Automatic calibration based on environmental conditions

### Network Topology

```
Building Layout:
├── Living Areas (2-3 sensors per 500 sq ft)
├── Meeting Rooms (1 sensor per room + backup)
├── Corridors (1 sensor every 50 feet)
└── Utility Areas (minimal coverage, motion-triggered)

Data Flow:
Sensor Nodes -> Mesh Network -> Edge Gateway -> RKNPU Processing
```

## Hardware Implementation

### Passive Sensor Node Specifications

**Microcontroller:**
- ESP32-WROOM-32D (preferred for stability)
- ESP32-S3 (for advanced features like on-device VAD)
- Dual-core 240MHz with WiFi 6 support
- 520KB SRAM, 4MB+ flash storage

**Audio Capture:**
- INMP441 I2S MEMS microphone (primary)
- High SNR (61 dB) and flat frequency response
- Omnidirectional pickup pattern
- AOP (Acoustic Overload Point): 120 dB SPL

**Power Management:**
- Primary: PoE+ (Power over Ethernet) for permanent installations
- Backup: 3.7V LiPo with solar charging for wireless deployments
- Deep sleep mode: <10μA consumption during silence periods
- Active listening: 80mA average, 150mA peak during transmission

**Environmental Sensors (Optional):**
- Temperature/humidity (SHT30)
- Motion detection (PIR or microwave)
- Light level sensing (ambient adjustment)
- Acoustic noise baseline measurement

### Wiring Configuration

**Standard I2S Microphone Connection:**
```
ESP32 Pin    Function      INMP441 Pin   Wire Color
GPIO22       I2S_SCLK      SCK          White
GPIO23       I2S_LRCLK     WS           Blue  
GPIO21       I2S_SDOUT     SD           Gray
3.3V         Power         VCC          Red
GND          Ground        GND          Black
GND          Channel       L/R          Black (Left channel)
```

**Power over Ethernet (Recommended):**
```
PoE Module   ESP32 Pin     Function
+5V         -> VIN         Main power input
GND         -> GND         Power ground
DATA+       -> GPIO16      Ethernet data (optional)
DATA-       -> GPIO17      Ethernet data (optional)
```

**Status and Control:**
```
GPIO2       -> LED         System status indicator
GPIO0       -> Button      Manual configuration/reset
GPIO4       -> Motion PIR  Occupancy detection
GPIO5       -> ENV_SDA     Environmental sensor I2C
GPIO18      -> ENV_SCL     Environmental sensor I2C
```

## Firmware Implementation

### Core Passive Monitoring Firmware

```cpp
/*
 * Passive Sensor Node for Edge Speech Intelligence
 * Optimized for 24/7 deployment with minimal maintenance
 * 
 * Author: zwanderer
 * License: MIT
 * Target: ESP32-WROOM-32D/ESP32-S3
 */

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <driver/i2s.h>
#include <esp_sleep.h>
#include <esp_wifi.h>

// Deployment configuration
#define DEPLOYMENT_MODE "PASSIVE_MONITORING"
#define SENSOR_LOCATION "ROOM_UNSET"  // Updated via configuration
#define FIRMWARE_VERSION "1.0.0"
#define DEVICE_TYPE "PASSIVE_SENSOR"

// Audio capture configuration  
#define I2S_WS_PIN 22
#define I2S_SCK_PIN 23
#define I2S_SD_PIN 21
#define SAMPLE_RATE 16000
#define BITS_PER_SAMPLE 16
#define CHANNELS 1

// Power management for 90% silence optimization
#define SILENCE_THRESHOLD_MS 5000    // 5 seconds silence before sleep
#define DEEP_SLEEP_THRESHOLD_MS 300000  // 5 minutes for deep sleep
#define WAKE_INTERVAL_SECONDS 300    // Wake every 5 minutes for check-in

// Network configuration
#define MESH_SSID_PREFIX "EdgeSpeech_"
#define EDGE_GATEWAY_PORT 8080
#define HEARTBEAT_INTERVAL_MS 60000  // 1 minute status updates

// Hardware pins
#define STATUS_LED_PIN 2
#define CONFIG_BUTTON_PIN 0
#define MOTION_SENSOR_PIN 4
#define I2C_SDA_PIN 5
#define I2C_SCL_PIN 18

// Global state
struct SensorState {
    bool is_connected = false;
    bool is_monitoring = false;
    bool motion_detected = false;
    unsigned long last_speech_time = 0;
    unsigned long last_heartbeat = 0;
    unsigned long total_uptime = 0;
    float battery_voltage = 0.0;
    String sensor_id;
    String location;
} sensor_state;

WebSocketsClient webSocket;
TaskHandle_t audioProcessingTask;
TaskHandle_t networkManagementTask;

void setup() {
    Serial.begin(115200);
    
    // Initialize hardware
    setupGPIO();
    setupI2S();
    setupPowerManagement();
    
    // Load configuration from EEPROM/SPIFFS
    loadConfiguration();
    
    // Initialize network
    setupNetworkStack();
    
    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(audioProcessingTaskFunc, "AudioTask", 8192, NULL, 2, &audioProcessingTask, 0);
    xTaskCreatePinnedToCore(networkManagementTaskFunc, "NetworkTask", 4096, NULL, 1, &networkManagementTask, 1);
    
    // Setup wake-up sources for deep sleep
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);  // Button wake
    esp_sleep_enable_timer_wakeup(WAKE_INTERVAL_SECONDS * 1000000);  // Timer wake
    
    Serial.printf("Passive sensor %s initialized at %s\n", 
                  sensor_state.sensor_id.c_str(), 
                  sensor_state.location.c_str());
}

void loop() {
    // Main loop handles system coordination
    handleSystemMaintenance();
    
    // Check for deep sleep conditions
    if (shouldEnterDeepSleep()) {
        enterDeepSleep();
    }
    
    delay(1000);  // 1 second main loop
}

void setupGPIO() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);
    pinMode(MOTION_SENSOR_PIN, INPUT);
    
    // Initial LED pattern indicates initialization
    blinkStatusLED(3, 200);
}

void setupI2S() {
    i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = i2s_bits_per_sample_t(BITS_PER_SAMPLE),
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,      // Reduced for power efficiency
        .dma_buf_len = 256,      // Smaller buffers
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD_PIN
    };
    
    esp_err_t result = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (result == ESP_OK) {
        i2s_set_pin(I2S_NUM_0, &pin_config);
        Serial.println("I2S microphone initialized for passive monitoring");
    } else {
        Serial.printf("I2S initialization failed: %d\n", result);
        handleFatalError("I2S setup failed");
    }
}

void setupPowerManagement() {
    // Configure WiFi power saving
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    
    // Reduce CPU frequency during idle
    setCpuFrequencyMhz(80);  // Reduce from 240MHz default
    
    // Configure ADC for battery monitoring
    analogSetAttenuation(ADC_11db);
    
    Serial.println("Power management configured for 24/7 operation");
}

void audioProcessingTaskFunc(void* parameter) {
    const size_t buffer_size = 512;
    int16_t audio_buffer[buffer_size];
    size_t bytes_read;
    
    unsigned long silence_start = 0;
    bool in_silence_period = true;
    
    while (true) {
        if (sensor_state.is_monitoring) {
            // Read audio data
            esp_err_t result = i2s_read(I2S_NUM_0, audio_buffer, 
                                       sizeof(audio_buffer), &bytes_read, 
                                       portMAX_DELAY);
            
            if (result == ESP_OK && bytes_read > 0) {
                // Simple energy-based VAD for local processing
                bool has_speech = detectSpeechActivity(audio_buffer, bytes_read / sizeof(int16_t));
                
                if (has_speech) {
                    if (in_silence_period) {
                        // Transition from silence to speech
                        in_silence_period = false;
                        sensor_state.last_speech_time = millis();
                        
                        // Wake up network if sleeping
                        wakeNetworkStack();
                    }
                    
                    // Transmit audio data to edge processor
                    transmitAudioData(audio_buffer, bytes_read);
                    
                    // Update LED status
                    digitalWrite(STATUS_LED_PIN, HIGH);
                    
                } else {
                    if (!in_silence_period) {
                        // Start of silence period
                        in_silence_period = true;
                        silence_start = millis();
                    }
                    
                    // Check for extended silence
                    if (millis() - silence_start > SILENCE_THRESHOLD_MS) {
                        // Enter power-saving mode
                        enterSilenceMode();
                    }
                    
                    digitalWrite(STATUS_LED_PIN, LOW);
                }
            }
        } else {
            // Monitoring disabled - deep sleep
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
        
        vTaskDelay(pdMS_TO_TICKS(30));  // 30ms processing cycle
    }
}

bool detectSpeechActivity(int16_t* buffer, size_t samples) {
    // Energy-based VAD optimized for passive monitoring
    const int16_t energy_threshold = 800;  // Adjusted for indoor environments
    const int16_t noise_floor = 100;
    
    long sum_squares = 0;
    int16_t max_amplitude = 0;
    int zero_crossings = 0;
    
    for (size_t i = 0; i < samples; i++) {
        int16_t sample = buffer[i];
        sum_squares += (long)sample * sample;
        
        if (abs(sample) > max_amplitude) {
            max_amplitude = abs(sample);
        }
        
        if (i > 0 && ((buffer[i-1] >= 0) != (buffer[i] >= 0))) {
            zero_crossings++;
        }
    }
    
    int16_t rms_energy = sqrt(sum_squares / samples);
    
    // Speech detection criteria for passive monitoring
    bool has_energy = rms_energy > energy_threshold;
    bool above_noise_floor = max_amplitude > noise_floor;
    bool has_voice_characteristics = (zero_crossings > 3) && (zero_crossings < samples / 8);
    
    // Consider motion detection for context
    bool motion_context = digitalRead(MOTION_SENSOR_PIN) == HIGH;
    
    return has_energy && above_noise_floor && has_voice_characteristics && motion_context;
}

void transmitAudioData(int16_t* buffer, size_t bytes) {
    if (sensor_state.is_connected) {
        // Send binary audio data via WebSocket
        webSocket.sendBIN((uint8_t*)buffer, bytes);
        
        // Blink LED to indicate transmission
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    }
}

void networkManagementTaskFunc(void* parameter) {
    while (true) {
        webSocket.loop();
        
        // Send periodic heartbeat
        if (millis() - sensor_state.last_heartbeat > HEARTBEAT_INTERVAL_MS) {
            sendHeartbeat();
            sensor_state.last_heartbeat = millis();
        }
        
        // Handle configuration updates
        handleConfigurationUpdates();
        
        // Monitor network health
        monitorNetworkHealth();
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1 second network loop
    }
}

void sendHeartbeat() {
    if (!sensor_state.is_connected) return;
    
    DynamicJsonDocument doc(512);
    doc["type"] = "heartbeat";
    doc["sensor_id"] = sensor_state.sensor_id;
    doc["location"] = sensor_state.location;
    doc["uptime_ms"] = millis();
    doc["battery_voltage"] = readBatteryVoltage();
    doc["motion_detected"] = sensor_state.motion_detected;
    doc["last_speech_ms"] = sensor_state.last_speech_time;
    doc["firmware_version"] = FIRMWARE_VERSION;
    doc["free_heap"] = ESP.getFreeHeap();
    doc["wifi_rssi"] = WiFi.RSSI();
    
    String message;
    serializeJson(doc, message);
    webSocket.sendTXT(message);
}

void enterSilenceMode() {
    // Reduce power consumption during silence
    sensor_state.is_monitoring = false;
    
    // Reduce WiFi activity
    esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    
    // Lower LED brightness
    analogWrite(STATUS_LED_PIN, 64);  // Dim status LED
    
    Serial.println("Entering silence mode - reducing power consumption");
}

void wakeNetworkStack() {
    // Resume full operation when speech detected
    sensor_state.is_monitoring = true;
    
    // Restore WiFi performance
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    
    // Full LED brightness
    digitalWrite(STATUS_LED_PIN, HIGH);
    
    Serial.println("Network stack awakened - speech activity detected");
}

bool shouldEnterDeepSleep() {
    // Deep sleep conditions for passive sensors
    unsigned long silence_duration = millis() - sensor_state.last_speech_time;
    bool no_motion = digitalRead(MOTION_SENSOR_PIN) == LOW;
    bool low_battery = readBatteryVoltage() < 3.4;  // Critical battery level
    
    return (silence_duration > DEEP_SLEEP_THRESHOLD_MS) && no_motion && !low_battery;
}

void enterDeepSleep() {
    Serial.println("Entering deep sleep - extended silence period");
    
    // Save state to RTC memory
    rtc_store_sensor_state();
    
    // Configure wake-up sources
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 1);  // Motion sensor wake
    esp_sleep_enable_timer_wakeup(WAKE_INTERVAL_SECONDS * 1000000);
    
    // Enter deep sleep
    esp_deep_sleep_start();
}

float readBatteryVoltage() {
    // Read battery voltage for power management
    int raw = analogRead(A0);
    return (raw / 4095.0) * 3.3 * 2;  // Voltage divider compensation
}

void blinkStatusLED(int count, int delay_ms) {
    for (int i = 0; i < count; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(delay_ms);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(delay_ms);
    }
}

void handleFatalError(const char* error) {
    Serial.printf("FATAL ERROR: %s\n", error);
    
    // Rapid LED blinking to indicate error
    for (int i = 0; i < 20; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
    }
    
    // Restart after error indication
    ESP.restart();
}
```

## Deployment Considerations

### Human Factors in Sensor Placement

**Visual Integration:**
- Ceiling-mounted sensors blend with standard smoke detectors
- Wall-mounted units integrate with existing electrical outlets
- Color matching to room decor reduces visual impact
- Minimal form factor (< 2 inches diameter)

**Acoustic Positioning:**
- 8-10 foot maximum distance from conversation areas
- Avoid placement near HVAC vents or noise sources
- Consider room acoustics and reverberation patterns
- Multiple sensors for redundancy in critical areas

**Privacy Considerations:**
- Clear visual indicators when system is active
- Physical privacy switches for instant disable
- No cameras or visual recording capabilities
- Audible notifications for system status changes

### Installation Best Practices

**Power Infrastructure:**
```bash
# PoE+ Switch Configuration
# Minimum 25.5W per port for sensor + peripherals
# Gigabit Ethernet for mesh backhaul
# VLAN isolation for sensor traffic

# Example Ubiquiti UniFi configuration:
configure
set interfaces ethernet eth0 unit 0 family ethernet-switching vlan members sensor-vlan
set vlans sensor-vlan vlan-id 100
set vlans sensor-vlan interface eth0.0
```

**Network Security:**
- Certificate-based authentication for sensor nodes
- Encrypted WebSocket connections (WSS)
- Network segmentation with dedicated VLAN
- Regular security updates via over-the-air (OTA)

**Maintenance Schedule:**
- Monthly: Automated health checks and reporting
- Quarterly: Firmware updates and security patches
- Annually: Physical inspection and cleaning
- 5-year: Hardware refresh cycle

### Environmental Calibration

**Automatic Adaptation:**
```cpp
void calibrateForEnvironment() {
    // Measure background noise for 30 seconds
    float noise_floor = measureBackgroundNoise();
    
    // Adjust VAD threshold based on environment
    int dynamic_threshold = 500 + (noise_floor * 0.1);
    
    // Store calibration in persistent memory
    storeCalibrationData(dynamic_threshold, noise_floor);
    
    Serial.printf("Environment calibrated: threshold=%d, noise=%.1fdB\n", 
                  dynamic_threshold, noise_floor);
}
```

**Seasonal Adjustments:**
- HVAC system noise compensation
- Window/door status awareness
- Occupancy pattern learning
- Acoustic signature adaptation

## System Integration

### Edge Gateway Communication

**Sensor Registration Protocol:**
```json
{
  "type": "sensor_registration",
  "sensor_id": "ESP32_AABBCCDDEE",
  "location": "conference_room_a",
  "capabilities": ["audio_capture", "motion_detection", "environmental"],
  "firmware_version": "1.0.0",
  "hardware_revision": "v2.1",
  "power_source": "poe_plus"
}
```

**Real-time Data Streaming:**
```json
{
  "type": "audio_stream",
  "sensor_id": "ESP32_AABBCCDDEE", 
  "timestamp": 1640995200000,
  "sample_rate": 16000,
  "channels": 1,
  "format": "pcm_16",
  "vad_confidence": 0.85,
  "motion_context": true
}
```

### Quality Assurance

**Automated Testing:**
- Daily audio quality validation
- Network connectivity monitoring
- Power consumption tracking
- Environmental sensor verification

**Performance Metrics:**
- Speech detection accuracy (target: >95%)
- Network latency (target: <100ms)
- Power efficiency (target: <2W average)
- Uptime reliability (target: >99.9%)

---

**Author:** zwanderer  
**Last Updated:** 2024  
**License:** MIT  
**Purpose:** Production deployment of passive speech intelligence sensors