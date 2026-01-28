/*
  ==============================================================================
  PROJECT: C3mini_jammer_basic_vn_v1.0
  AUTHOR: NhanMinhz 🇻🇳
  DISCORD: nhan2k91221
  GITHUB: https://github.com/NhanMinhz/C3mini_jammer_basic
  
  HARDWARE:
    - ESP32-C3 SuperMini
    - 2x NRF24L01 (E01-2G4M27D) modules
    - Pinout:
      NRF1: CE=20, CSN=21
      NRF2: CE=0, CSN=1
      SPI: SCK=6, MISO=5, MOSI=7
      LED: GPIO8 (Active LOW)
      BUTTON: GPIO9 (BOOT button, internal pull-up)
  
  FEATURES:
    - 4 Jamming Modes: Standby, Bluetooth, WiFi, Full Spectrum
    - 120 seconds per mode, auto-return to standby
    - Button control: short press = next mode, long press (3s) = reset to standby
    - Auto module detection with 3 retries
    - Max performance: 250µs delay between packets
    - Serial logging for debugging
    - Optimized for 80 Bluetooth channels, 14 WiFi channels, 126 full channels
  
  LICENSE: MIT
  VERSION: 1.0.0
  LAST UPDATED: Jan 2026
  ==============================================================================
*/

// ==============================================================================
// INCLUDES & LIBRARIES
// ==============================================================================
#include <SPI.h>
#include <RF24.h>

// ==============================================================================
// HARDWARE PIN DEFINITIONS (ESP32-C3 SuperMini)
// ==============================================================================
// NRF24 Module 1
#define NRF1_CE_PIN   20
#define NRF1_CSN_PIN  21

// NRF24 Module 2
#define NRF2_CE_PIN   0
#define NRF2_CSN_PIN  1

// SPI Pins (fixed for ESP32-C3)
#define SPI_SCK_PIN   6
#define SPI_MISO_PIN  5
#define SPI_MOSI_PIN  7

// Status LED (Active LOW - LED on when pin is LOW)
#define STATUS_LED_PIN 8

// Mode selection button (BOOT button, internal pull-up)
#define MODE_BUTTON_PIN 9

// ==============================================================================
// CONFIGURATION CONSTANTS
// ==============================================================================
// Timing constants
#define MODE_DURATION_MS       120000    // 120 seconds per mode
#define BUTTON_DEBOUNCE_MS     50        // Button debounce time
#define BUTTON_LONG_PRESS_MS   3000      // Long press = 3 seconds
#define PACKET_DELAY_US        250       // 250µs between packets (optimized)
#define MODULE_RETRY_COUNT     3         // Module detection retries
#define MODULE_RETRY_DELAY_MS  1000      // 1 second between retries

// Radio configuration
#define PAYLOAD_SIZE           5         // 5-byte jam packets
#define ADDRESS_WIDTH          3         // Shortest address width
#define DATA_RATE              RF24_2MBPS// Maximum data rate
#define POWER_LEVEL            RF24_PA_MAX// Maximum power output

// Channel counts
#define BLE_CHANNEL_COUNT      80        // Bluetooth channels (2-80)
#define WIFI_CHANNEL_COUNT     14        // WiFi channels (1-14)
#define FULL_CHANNEL_COUNT     126       // Full spectrum (0-125)

// ==============================================================================
// GLOBAL VARIABLES & STRUCTURES
// ==============================================================================

// ------------------------------------------------------------------------------
// Mode enumeration
// ------------------------------------------------------------------------------
enum JammerMode {
  MODE_STANDBY = 0,      // 0: Power saving mode, waiting for command
  MODE_BLUETOOTH = 1,    // 1: Jamming all 80 Bluetooth channels
  MODE_WIFI = 2,         // 2: Jamming all 14 WiFi channels
  MODE_FULL_SPECTRUM = 3,// 3: Jamming entire 2.4GHz spectrum (126 channels)
  MODE_COUNT             // Total number of modes
};

// ------------------------------------------------------------------------------
// Module status structure
// ------------------------------------------------------------------------------
struct ModuleStatus {
  bool isActive;          // Whether module is responding
  uint32_t packetCount;   // Packets sent by this module
  uint32_t errorCount;    // Transmission errors
  uint8_t lastChannel;    // Last channel used (for optimization)
};

// ------------------------------------------------------------------------------
// Global state variables
// ------------------------------------------------------------------------------
JammerMode currentMode = MODE_STANDBY;        // Current active mode
JammerMode previousMode = MODE_STANDBY;       // Previous mode (for logging)
unsigned long modeStartTime = 0;              // When current mode started
unsigned long lastButtonCheck = 0;            // Last button check time
unsigned long lastLogTime = 0;                // Last log output time
unsigned long packetCounter = 0;              // Total packets sent

// Module management
ModuleStatus modules[2];                      // Status for both modules
uint8_t activeModuleCount = 0;                // Number of active modules

// Button state tracking
bool lastButtonState = HIGH;                  // Last button reading
unsigned long buttonPressStart = 0;           // When button was pressed
bool buttonHandled = false;                   // Prevent multiple triggers

// Radio objects (dynamically allocated)
RF24* radio1 = nullptr;
RF24* radio2 = nullptr;
SPIClass* hspi = nullptr;                     // Hardware SPI instance

// ==============================================================================
// JAMMING DATA & CHANNEL DEFINITIONS
// ==============================================================================

// ------------------------------------------------------------------------------
// Jam data payload (5 bytes - optimized for speed)
// ------------------------------------------------------------------------------
uint8_t jamPayload[5] = {0xAA, 0x55, 0xAA, 0x55, 0xAA};

// ------------------------------------------------------------------------------
// Bluetooth channel mapping (80 channels from 2402MHz to 2480MHz)
// ------------------------------------------------------------------------------
const uint8_t bleChannels[BLE_CHANNEL_COUNT] = {
  // Data channels (0-36)
  2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,
  21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,
  
  // Advertising channels (37-39)
  37,38,39,
  
  // Extended channels (40-80)
  40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,
  56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,
  72,73,74,75,76,77,78,79,80
};

// ------------------------------------------------------------------------------
// WiFi channel to NRF24 channel mapping
// WiFi Channel -> Center Frequency -> NRF24 Channel (2400MHz + channel)
// ------------------------------------------------------------------------------
const uint8_t wifiChannels[WIFI_CHANNEL_COUNT] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14};
const uint8_t wifiToNRF[WIFI_CHANNEL_COUNT] = {
  12,   // Ch1: 2412MHz
  17,   // Ch2: 2417MHz
  22,   // Ch3: 2422MHz
  27,   // Ch4: 2427MHz
  32,   // Ch5: 2432MHz
  37,   // Ch6: 2437MHz
  42,   // Ch7: 2442MHz
  47,   // Ch8: 2447MHz
  52,   // Ch9: 2452MHz
  57,   // Ch10: 2457MHz
  62,   // Ch11: 2462MHz
  67,   // Ch12: 2467MHz
  72,   // Ch13: 2472MHz
  77    // Ch14: 2484MHz (NRF24 channel 77 = 2477MHz, close enough)
};

// ==============================================================================
// FUNCTION PROTOTYPES
// ==============================================================================
// Initialization functions
void initializeHardware();
bool detectNRF24Modules();
bool initializeRadioModule(RF24* radio, uint8_t cePin, uint8_t csnPin, uint8_t moduleIndex);

// Mode management functions
void switchToMode(JammerMode newMode);
void runCurrentMode();
void executeStandbyMode();
void executeBluetoothMode();
void executeWifiMode();
void executeFullSpectrumMode();

// Radio operation functions
void sendJamPacket(RF24* radio, uint8_t channel, uint8_t moduleIndex);

// Utility functions
void handleModeButton();
void checkModeTimeout();
void updateStatusLED();
void logMessage(const char* level, const char* message);
void printSystemInfo();
void printModuleStatus();

// ==============================================================================
// INITIALIZATION FUNCTIONS
// ==============================================================================

// ------------------------------------------------------------------------------
// setup(): Main initialization function
// ------------------------------------------------------------------------------
void setup() {
  // Initialize hardware pins
  initializeHardware();
  
  // Wait for serial connection (for debugging)
  delay(2000);
  
  // Print project info
  logMessage("INFO", "==========================================");
  logMessage("INFO", "C3mini_jammer_basic_vn_v1.0");
  logMessage("INFO", "Author: NhanMinhz 🇻🇳");
  logMessage("INFO", "Discord: nhan2k91221");
  logMessage("INFO", "==========================================");
  
  // Initialize SPI for NRF24 modules
  logMessage("INFO", "Initializing SPI bus...");
  hspi = new SPIClass(FSPI);
  hspi->begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  
  // Create radio objects
  logMessage("INFO", "Creating NRF24 radio objects...");
  radio1 = new RF24(NRF1_CE_PIN, NRF1_CSN_PIN);
  radio2 = new RF24(NRF2_CE_PIN, NRF2_CSN_PIN);
  
  // Detect and initialize NRF24 modules
  if (!detectNRF24Modules()) {
    logMessage("ERROR", "Failed to initialize any NRF24 modules!");
    logMessage("ERROR", "Check wiring and power supply.");
    
    // Blink LED rapidly to indicate error
    while (true) {
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(100);
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(100);
    }
  }
  
  // Print system information
  printSystemInfo();
  
  // Start in standby mode
  switchToMode(MODE_STANDBY);
  
  logMessage("INFO", "Initialization complete!");
  logMessage("INFO", "Press BOOT button to change modes.");
}

// ------------------------------------------------------------------------------
// initializeHardware(): Configure all GPIO pins
// ------------------------------------------------------------------------------
void initializeHardware() {
  // Configure status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);  // Start with LED off (active low)
  
  // Configure mode button (internal pull-up enabled)
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize serial for debugging
  Serial.begin(115200);
  
  logMessage("INFO", "Hardware pins initialized");
}

// ------------------------------------------------------------------------------
// detectNRF24Modules(): Detect and initialize all NRF24 modules
// ------------------------------------------------------------------------------
bool detectNRF24Modules() {
  logMessage("INFO", "Detecting NRF24 modules...");
  
  // Reset module status
  for (int i = 0; i < 2; i++) {
    modules[i].isActive = false;
    modules[i].packetCount = 0;
    modules[i].errorCount = 0;
    modules[i].lastChannel = 255;
  }
  activeModuleCount = 0;
  
  // Try to initialize each module with retries
  for (int retry = 0; retry < MODULE_RETRY_COUNT; retry++) {
    logMessage("DEBUG", "Detection attempt " + String(retry + 1));
    
    // Try module 1
    if (!modules[0].isActive) {
      if (initializeRadioModule(radio1, NRF1_CE_PIN, NRF1_CSN_PIN, 0)) {
        modules[0].isActive = true;
        activeModuleCount++;
        logMessage("OK", "Module 1 initialized successfully");
      }
    }
    
    // Try module 2
    if (!modules[1].isActive) {
      if (initializeRadioModule(radio2, NRF2_CE_PIN, NRF2_CSN_PIN, 1)) {
        modules[1].isActive = true;
        activeModuleCount++;
        logMessage("OK", "Module 2 initialized successfully");
      }
    }
    
    // If at least one module is active, we're done
    if (activeModuleCount > 0) {
      break;
    }
    
    // Wait before retry (except on last attempt)
    if (retry < MODULE_RETRY_COUNT - 1) {
      logMessage("WARN", "No modules found, retrying in " + String(MODULE_RETRY_DELAY_MS) + "ms...");
      delay(MODULE_RETRY_DELAY_MS);
    }
  }
  
  // Print final status
  if (activeModuleCount == 0) {
    logMessage("ERROR", "No NRF24 modules detected!");
    return false;
  } else {
    char msg[50];
    snprintf(msg, sizeof(msg), "Active modules: %d/2", activeModuleCount);
    logMessage("INFO", msg);
    return true;
  }
}

// ------------------------------------------------------------------------------
// initializeRadioModule(): Configure a single NRF24 module
// ------------------------------------------------------------------------------
bool initializeRadioModule(RF24* radio, uint8_t cePin, uint8_t csnPin, uint8_t moduleIndex) {
  if (!radio) {
    logMessage("ERROR", "Radio object is null!");
    return false;
  }
  
  // Attempt to begin SPI communication
  if (!radio->begin(hspi)) {
    char errorMsg[60];
    snprintf(errorMsg, sizeof(errorMsg), "Module %d failed to begin (check wiring)", moduleIndex + 1);
    logMessage("ERROR", errorMsg);
    return false;
  }
  
  // Configure for maximum jamming performance
  radio->setAutoAck(false);                 // Disable auto acknowledgment
  radio->stopListening();                   // Put in TX mode
  radio->setRetries(0, 0);                  // No retries (max speed)
  radio->setPayloadSize(PAYLOAD_SIZE);      // Fixed payload size
  radio->setAddressWidth(ADDRESS_WIDTH);    // Shortest address width
  radio->setPALevel(POWER_LEVEL);           // Maximum power output
  radio->setDataRate(DATA_RATE);            // Maximum data rate
  radio->setCRCLength(RF24_CRC_DISABLED);   // Disable CRC (max speed)
  
  // Set unique address for each module
  uint8_t address[3] = {0x4A, 0x41, 0x4D + moduleIndex}; // "JAM" + index
  radio->openWritingPipe(address);
  
  // Power up the radio
  radio->powerUp();
  delayMicroseconds(150);  // Required stabilization time
  
  char successMsg[50];
  snprintf(successMsg, sizeof(successMsg), "Module %d configured successfully", moduleIndex + 1);
  logMessage("DEBUG", successMsg);
  
  return true;
}

// ==============================================================================
// MODE MANAGEMENT FUNCTIONS
// ==============================================================================

// ------------------------------------------------------------------------------
// switchToMode(): Change to a new jamming mode
// ------------------------------------------------------------------------------
void switchToMode(JammerMode newMode) {
  // Don't switch if already in this mode
  if (currentMode == newMode) {
    return;
  }
  
  // Update mode tracking
  previousMode = currentMode;
  currentMode = newMode;
  modeStartTime = millis();
  packetCounter = 0;
  
  // Reset packet counters for modules
  for (int i = 0; i < 2; i++) {
    if (modules[i].isActive) {
      modules[i].packetCount = 0;
    }
  }
  
  // Log mode change
  const char* modeNames[] = {"STANDBY", "BLUETOOTH", "WIFI", "FULL SPECTRUM"};
  char logMsg[60];
  snprintf(logMsg, sizeof(logMsg), "Mode changed: %s (for %d seconds)", 
           modeNames[currentMode], MODE_DURATION_MS / 1000);
  logMessage("INFO", logMsg);
  
  // Provide LED feedback (blink LED number of times = mode number)
  digitalWrite(STATUS_LED_PIN, HIGH);  // Ensure LED is off first
  delay(300);
  
  for (int i = 0; i <= currentMode; i++) {
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(150);
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(150);
  }
  
  // Power management based on mode
  if (currentMode == MODE_STANDBY) {
    // Power down radios to save energy and reduce heat
    if (modules[0].isActive && radio1) radio1->powerDown();
    if (modules[1].isActive && radio2) radio2->powerDown();
    logMessage("DEBUG", "Radios powered down for standby");
  } else {
    // Power up radios for active jamming
    if (modules[0].isActive && radio1) radio1->powerUp();
    if (modules[1].isActive && radio2) radio2->powerUp();
    delayMicroseconds(150);  // Stabilization time
    logMessage("DEBUG", "Radios powered up for jamming");
  }
}

// ------------------------------------------------------------------------------
// runCurrentMode(): Execute the current mode's logic
// ------------------------------------------------------------------------------
void runCurrentMode() {
  switch (currentMode) {
    case MODE_STANDBY:
      executeStandbyMode();
      break;
    case MODE_BLUETOOTH:
      executeBluetoothMode();
      break;
    case MODE_WIFI:
      executeWifiMode();
      break;
    case MODE_FULL_SPECTRUM:
      executeFullSpectrumMode();
      break;
    default:
      // Should never reach here
      logMessage("ERROR", "Unknown mode! Resetting to standby.");
      switchToMode(MODE_STANDBY);
      break;
  }
}

// ==============================================================================
// MODE IMPLEMENTATIONS
// ==============================================================================

// ------------------------------------------------------------------------------
// executeStandbyMode(): Standby mode - wait for user input
// ------------------------------------------------------------------------------
void executeStandbyMode() {
  static unsigned long lastBlinkTime = 0;
  unsigned long currentTime = millis();
  
  // Slow blink (1 second interval) to indicate standby
  if (currentTime - lastBlinkTime >= 1000) {
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    lastBlinkTime = currentTime;
  }
  
  // Minimal processing in standby
  delay(100);
}

// ------------------------------------------------------------------------------
// executeBluetoothMode(): Jam all 80 Bluetooth channels
// ------------------------------------------------------------------------------
void executeBluetoothMode() {
  static uint8_t channelIndex = 0;
  
  // Calculate how many channels each module should handle
  uint8_t channelsPerModule = BLE_CHANNEL_COUNT / activeModuleCount;
  
  // Send packets on all active modules
  for (uint8_t moduleIdx = 0; moduleIdx < 2; moduleIdx++) {
    if (!modules[moduleIdx].isActive) continue;
    
    // Calculate which channel this module should use
    uint8_t channelOffset = (channelIndex + moduleIdx * channelsPerModule) % BLE_CHANNEL_COUNT;
    uint8_t targetChannel = bleChannels[channelOffset];
    
    // Select the correct radio object
    RF24* currentRadio = (moduleIdx == 0) ? radio1 : radio2;
    
    // Send jam packet
    sendJamPacket(currentRadio, targetChannel, moduleIdx);
  }
  
  // Move to next channel for next iteration
  channelIndex = (channelIndex + 1) % BLE_CHANNEL_COUNT;
  
  // Log progress every 1000 packets
  static uint32_t lastLogPackets = 0;
  if (packetCounter - lastLogPackets >= 1000) {
    char progressMsg[80];
    snprintf(progressMsg, sizeof(progressMsg), 
             "BLE Mode: %lu packets, Channel %d/%d", 
             packetCounter, channelIndex, BLE_CHANNEL_COUNT);
    logMessage("STAT", progressMsg);
    lastLogPackets = packetCounter;
  }
}

// ------------------------------------------------------------------------------
// executeWifiMode(): Jam all 14 WiFi channels
// ------------------------------------------------------------------------------
void executeWifiMode() {
  static uint8_t channelIndex = 0;
  
  // Send packets on all active modules
  for (uint8_t moduleIdx = 0; moduleIdx < 2; moduleIdx++) {
    if (!modules[moduleIdx].isActive) continue;
    
    // Calculate which WiFi channel this module should target
    // Module 1 handles even indices, Module 2 handles odd indices (if both active)
    uint8_t wifiChannelIdx;
    if (activeModuleCount == 2) {
      wifiChannelIdx = (channelIndex * 2 + moduleIdx) % WIFI_CHANNEL_COUNT;
    } else {
      wifiChannelIdx = channelIndex % WIFI_CHANNEL_COUNT;
    }
    
    uint8_t wifiChannel = wifiChannels[wifiChannelIdx];
    uint8_t nrfChannel = wifiToNRF[wifiChannelIdx];
    
    // Select the correct radio object
    RF24* currentRadio = (moduleIdx == 0) ? radio1 : radio2;
    
    // Send on primary channel
    sendJamPacket(currentRadio, nrfChannel, moduleIdx);
    
    // Also send on ±1MHz for better coverage
    if (nrfChannel > 0) {
      sendJamPacket(currentRadio, nrfChannel - 1, moduleIdx);
    }
    if (nrfChannel < 125) {
      sendJamPacket(currentRadio, nrfChannel + 1, moduleIdx);
    }
  }
  
  // Move to next channel set
  channelIndex = (channelIndex + 1) % WIFI_CHANNEL_COUNT;
  
  // Log progress every 500 packets
  static uint32_t lastLogPackets = 0;
  if (packetCounter - lastLogPackets >= 500) {
    char progressMsg[80];
    snprintf(progressMsg, sizeof(progressMsg), 
             "WiFi Mode: %lu packets, Channel %d/%d", 
             packetCounter, channelIndex + 1, WIFI_CHANNEL_COUNT);
    logMessage("STAT", progressMsg);
    lastLogPackets = packetCounter;
  }
}

// ------------------------------------------------------------------------------
// executeFullSpectrumMode(): Jam entire 2.4GHz spectrum (126 channels)
// ------------------------------------------------------------------------------
void executeFullSpectrumMode() {
  static uint8_t channel = 0;
  
  // Send packets on all active modules
  for (uint8_t moduleIdx = 0; moduleIdx < 2; moduleIdx++) {
    if (!modules[moduleIdx].isActive) continue;
    
    // Select the correct radio object
    RF24* currentRadio = (moduleIdx == 0) ? radio1 : radio2;
    
    // Module 0 scans from low to high, Module 1 scans from high to low
    uint8_t targetChannel;
    if (moduleIdx == 0) {
      targetChannel = channel;
    } else {
      targetChannel = FULL_CHANNEL_COUNT - 1 - channel;
    }
    
    // Send jam packet
    sendJamPacket(currentRadio, targetChannel, moduleIdx);
  }
  
  // Move to next channel
  channel = (channel + 1) % FULL_CHANNEL_COUNT;
  
  // Log progress every 2000 packets
  static uint32_t lastLogPackets = 0;
  if (packetCounter - lastLogPackets >= 2000) {
    char progressMsg[80];
    snprintf(progressMsg, sizeof(progressMsg), 
             "Full Spectrum: %lu packets, Channel %d/%d", 
             packetCounter, channel, FULL_CHANNEL_COUNT);
    logMessage("STAT", progressMsg);
    lastLogPackets = packetCounter;
  }
}

// ==============================================================================
// RADIO OPERATION FUNCTIONS
// ==============================================================================

// ------------------------------------------------------------------------------
// sendJamPacket(): Send a jam packet on specified channel
// ------------------------------------------------------------------------------
void sendJamPacket(RF24* radio, uint8_t channel, uint8_t moduleIndex) {
  // Safety check
  if (!radio || moduleIndex >= 2 || !modules[moduleIndex].isActive) {
    return;
  }
  
  // Optimize: Only set channel if it changed
  if (modules[moduleIndex].lastChannel != channel) {
    radio->setChannel(channel);
    modules[moduleIndex].lastChannel = channel;
  }
  
  // Send the jam packet
  bool success = radio->writeFast(jamPayload, PAYLOAD_SIZE);
  
  // Update counters
  if (success) {
    modules[moduleIndex].packetCount++;
    packetCounter++;
  } else {
    modules[moduleIndex].errorCount++;
  }
  
  // Delay between packets (optimized for performance)
  delayMicroseconds(PACKET_DELAY_US);
}

// ==============================================================================
// UTILITY FUNCTIONS
// ==============================================================================

// ------------------------------------------------------------------------------
// handleModeButton(): Check and handle button presses
// ------------------------------------------------------------------------------
void handleModeButton() {
  unsigned long currentTime = millis();
  
  // Read current button state (active LOW)
  bool currentButtonState = digitalRead(MODE_BUTTON_PIN);
  
  // Detect button press (falling edge)
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    buttonPressStart = currentTime;
    buttonHandled = false;
  }
  
  // Detect button release (rising edge)
  if (lastButtonState == LOW && currentButtonState == HIGH) {
    unsigned long pressDuration = currentTime - buttonPressStart;
    
    // Only handle if not already handled
    if (!buttonHandled) {
      // Short press: change to next mode
      if (pressDuration < BUTTON_LONG_PRESS_MS) {
        JammerMode nextMode = static_cast<JammerMode>((currentMode + 1) % MODE_COUNT);
        switchToMode(nextMode);
        logMessage("DEBUG", "Button short press detected");
      }
      // Long press: reset to standby
      else {
        switchToMode(MODE_STANDBY);
        logMessage("DEBUG", "Button long press detected (reset to standby)");
      }
      
      buttonHandled = true;
    }
  }
  
  // Update last button state
  lastButtonState = currentButtonState;
}

// ------------------------------------------------------------------------------
// checkModeTimeout(): Check if current mode has timed out (120 seconds)
// ------------------------------------------------------------------------------
void checkModeTimeout() {
  // Don't timeout standby mode
  if (currentMode == MODE_STANDBY) {
    return;
  }
  
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - modeStartTime;
  
  // Check if 120 seconds have passed
  if (elapsedTime >= MODE_DURATION_MS) {
    char timeoutMsg[60];
    snprintf(timeoutMsg, sizeof(timeoutMsg), 
             "Mode timeout after %d seconds, returning to standby", 
             MODE_DURATION_MS / 1000);
    logMessage("INFO", timeoutMsg);
    switchToMode(MODE_STANDBY);
  }
  
  // Log time remaining every 30 seconds
  static unsigned long lastTimeLog = 0;
  if (currentTime - lastTimeLog >= 30000) {  // Every 30 seconds
    unsigned long timeRemaining = (MODE_DURATION_MS - elapsedTime) / 1000;
    if (timeRemaining > 0) {
      char timeMsg[50];
      snprintf(timeMsg, sizeof(timeMsg), "Time remaining: %lu seconds", timeRemaining);
      logMessage("INFO", timeMsg);
    }
    lastTimeLog = currentTime;
  }
}

// ------------------------------------------------------------------------------
// updateStatusLED(): Update LED based on current mode
// ------------------------------------------------------------------------------
void updateStatusLED() {
  static unsigned long lastLEDUpdate = 0;
  unsigned long currentTime = millis();
  
  // Update LED at different rates based on mode
  switch (currentMode) {
    case MODE_STANDBY:
      // Blink at 1Hz (already handled in executeStandbyMode)
      break;
      
    case MODE_BLUETOOTH:
      // Fast blink (10Hz) during active jamming
      if (currentTime - lastLEDUpdate >= 50) {
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        lastLEDUpdate = currentTime;
      }
      break;
      
    case MODE_WIFI:
      // Medium blink (5Hz)
      if (currentTime - lastLEDUpdate >= 100) {
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        lastLEDUpdate = currentTime;
      }
      break;
      
    case MODE_FULL_SPECTRUM:
      // Solid on (100% duty cycle)
      digitalWrite(STATUS_LED_PIN, LOW);
      break;
  }
}

// ------------------------------------------------------------------------------
// logMessage(): Output log message to serial console
// ------------------------------------------------------------------------------
void logMessage(const char* level, const char* message) {
  unsigned long timestamp = millis();
  Serial.printf("[%lu] [%s] %s\n", timestamp, level, message);
}

// Overloaded version for String messages
void logMessage(const char* level, String message) {
  logMessage(level, message.c_str());
}

// ------------------------------------------------------------------------------
// printSystemInfo(): Print system information to serial
// ------------------------------------------------------------------------------
void printSystemInfo() {
  logMessage("INFO", "==========================================");
  logMessage("INFO", "SYSTEM INFORMATION");
  logMessage("INFO", "==========================================");
  
  // Hardware info
  logMessage("INFO", "Board: ESP32-C3 SuperMini");
  logMessage("INFO", "Flash: 16MB");
  logMessage("INFO", "CPU Frequency: 160MHz");
  
  // Radio configuration
  char configMsg[100];
  snprintf(configMsg, sizeof(configMsg), 
           "Radio Config: %d bytes payload, %dµs delay, %d Mbps",
           PAYLOAD_SIZE, PACKET_DELAY_US, 2);
  logMessage("INFO", configMsg);
  
  // Mode timing
  snprintf(configMsg, sizeof(configMsg), 
           "Mode Duration: %d seconds per mode", MODE_DURATION_MS / 1000);
  logMessage("INFO", configMsg);
  
  // Channel information
  logMessage("INFO", "Channels: BLE=80, WiFi=14, Full=126");
  
  printModuleStatus();
  logMessage("INFO", "==========================================");
}

// ------------------------------------------------------------------------------
// printModuleStatus(): Print NRF24 module status
// ------------------------------------------------------------------------------
void printModuleStatus() {
  logMessage("INFO", "MODULE STATUS:");
  
  for (int i = 0; i < 2; i++) {
    char moduleMsg[80];
    const char* status = modules[i].isActive ? "ACTIVE" : "INACTIVE";
    snprintf(moduleMsg, sizeof(moduleMsg), 
             "  Module %d: %s", i + 1, status);
    logMessage("INFO", moduleMsg);
  }
  
  char countMsg[40];
  snprintf(countMsg, sizeof(countMsg), "Active modules: %d/2", activeModuleCount);
  logMessage("INFO", countMsg);
}

// ==============================================================================
// MAIN LOOP
// ==============================================================================

// ------------------------------------------------------------------------------
// loop(): Main program loop
// ------------------------------------------------------------------------------
void loop() {
  unsigned long currentTime = millis();
  
  // 1. Check mode button (every 50ms for debouncing)
  if (currentTime - lastButtonCheck >= 50) {
    handleModeButton();
    lastButtonCheck = currentTime;
  }
  
  // 2. Check for mode timeout (120 seconds)
  checkModeTimeout();
  
  // 3. Run current mode
  runCurrentMode();
  
  // 4. Update status LED
  updateStatusLED();
  
  // 5. Log statistics every 5 seconds
  if (currentTime - lastLogTime >= 5000) {
    lastLogTime = currentTime;
    
    // Only log during active modes (not standby)
    if (currentMode != MODE_STANDBY) {
      char statsMsg[100];
      unsigned long elapsedSeconds = (currentTime - modeStartTime) / 1000;
      
      snprintf(statsMsg, sizeof(statsMsg),
               "Mode %d: %lu seconds, %lu packets total",
               currentMode, elapsedSeconds, packetCounter);
      logMessage("STAT", statsMsg);
      
      // Log per-module stats if debugging
      #ifdef DEBUG_MODULES
      for (int i = 0; i < 2; i++) {
        if (modules[i].isActive) {
          snprintf(statsMsg, sizeof(statsMsg),
                   "  Module %d: %lu packets, %lu errors",
                   i + 1, modules[i].packetCount, modules[i].errorCount);
          logMessage("DEBUG", statsMsg);
        }
      }
      #endif
    }
  }
  
  // 6. Handle module errors (recovery attempt)
  static unsigned long lastRecoveryCheck = 0;
  if (currentTime - lastRecoveryCheck >= 10000) {  // Every 10 seconds
    lastRecoveryCheck = currentTime;
    
    // Check if any module has excessive errors
    for (int i = 0; i < 2; i++) {
      if (modules[i].isActive && modules[i].errorCount > 1000) {
        char errorMsg[60];
        snprintf(errorMsg, sizeof(errorMsg),
                 "Module %d has %lu errors, attempting recovery",
                 i + 1, modules[i].errorCount);
        logMessage("WARN", errorMsg);
        
        // Reset error count
        modules[i].errorCount = 0;
        
        // Brief pause to let module recover
        delay(100);
      }
    }
  }
}

// ==============================================================================
// END OF FILE
// ==============================================================================
