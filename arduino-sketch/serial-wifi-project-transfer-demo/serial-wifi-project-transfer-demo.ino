#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <HardwareSerial.h> // Required for Serial2
#include "LumenProtocol.h"  // Lumen Protocol Library

// --- Hardware Configurations ---
#define RXD2 16 // RX Pin for display communication
#define TXD2 17 // TX Pin for display communication

// --- Wi-Fi Network Configurations ---


const char* ssid = "YOUR_WIFI_NETWORK_NAME";    // Replace with your Wi-Fi network name (SSID)
const char* password = "YOUR_WIFI_PASSWORD"; // Replace with your Wi-Fi password

// --- Lumen Project Configurations ---
// Define your project links here.
// To select a project, change the value of SELECTED_PROJECT_INDEX.
const char* PROJECT_URLS[] = {
  // --- Projects for P10600-070T-L0101 Display (7.0 inch) ---
  "https://github.com/victorvision/serial-project-transfer-demo/raw/915f6d5d74e98f917a6f02f2aeee89c1bfaa0499/compiled-projects/P10600-070T-L0101/simple-led-compiled-project/internal_project.lumen",    // 0: Simple LED Demo
  "https://github.com/victorvision/serial-project-transfer-demo/raw/915f6d5d74e98f917a6f02f2aeee89c1bfaa0499/compiled-projects/P10600-070T-L0101/led-rgb-compiled-project/internal_project.lumen",        // 1: LED RGB Demo
  "https://github.com/victorvision/serial-project-transfer-demo/raw/915f6d5d74e98f917a6f02f2aeee89c1bfaa0499/compiled-projects/P10600-070T-L0101/distance-sensor-compiled-project/internal_project.lumen", // 2: Distance Sensor Demo
  "https://github.com/victorvision/serial-project-transfer-demo/raw/915f6d5d74e98f917a6f02f2aeee89c1bfaa0499/compiled-projects/P10600-070T-L0101/servo-motor-compiled-project/internal_project.lumen",    // 3: Servo Motor Demo
  "https://github.com/victorvision/serial-project-transfer-demo/raw/915f6d5d74e98f917a6f02f2aeee89c1bfaa0499/compiled-projects/P10600-070T-L0101/smart-home-compiled-project/internal_project.lumen",    // 4: Smart Home Demo

  // --- Projects for P48272-043T-L0101 Display (4.3 inch) ---
  "http://example.com/P48272-043T-L0101/demo1.lumen", // 5: Generic Demo 1 (Placeholder)
  "http://example.com/P48272-043T-L0101/demo2.lumen", // 6: Generic Demo 2 (Placeholder)
  "http://example.com/P48272-043T-L0101/demo3.lumen", // 7: Generic Demo 3 (Placeholder)
  "http://example.com/P48272-043T-L0101/demo4.lumen", // 8: Generic Demo 4 (Placeholder)
  "http://example.com/P48272-043T-L0101/demo5.lumen"  // 9: Generic Demo 5 (Placeholder)
};
const int NUM_PROJECT_URLS = sizeof(PROJECT_URLS) / sizeof(PROJECT_URLS[0]);

// Change this index to select which project will be transferred (0 to NUM_PROJECT_URLS - 1)
const int SELECTED_PROJECT_INDEX = 0; // Selects the first project by default

// The active project URL is defined based on the selected index
const char* projectFileUrl = PROJECT_URLS[SELECTED_PROJECT_INDEX];

const uint16_t REBOOT_ADDRESS = 106; // Address for Lumen display reboot command

// --- Buffer Configurations (Double Buffering) ---
const int BUFFER_SIZE = 16384; // Size of each buffer (16KB) for transfer optimization
uint8_t bufferA[BUFFER_SIZE];
uint8_t bufferB[BUFFER_SIZE];

// Pointers and flags to manage read and send buffers
uint8_t* currentReadBuffer = nullptr;
uint8_t* currentSendBuffer = nullptr;
size_t currentReadBufferLength = 0;
size_t currentSendBufferLength = 0;
bool bufferA_is_filled = false;
bool bufferB_is_filled = false;

// --- State and Time Control Variables ---
unsigned long lastLumenTickTime = 0;
const unsigned long LUMEN_TICK_INTERVAL_MS = 10; // Interval for Lumen protocol tick update

bool projectTransferStarted = false;
bool projectTransferComplete = false;
size_t bytesReadTotal = 0; // Moved to global scope for accessibility

// --- Lumen Protocol Interface Function Implementations ---
// These functions are called internally by the LumenProtocol library
// to send and receive bytes via Serial.
extern "C" void lumen_write_bytes(uint8_t* data, uint32_t length) {
  Serial2.write(data, length);
}

extern "C" uint16_t lumen_get_byte() {
  if (Serial2.available()) {
    return Serial2.read();
  }
  return DATA_NULL;
}

// --- Auxiliary Functions ---

// Initializes serial communications (USB and Serial2 for the display)
void initializeSerialCommunications() {
  Serial.begin(115200);
  delay(100);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("\n--- Lumen Project Transfer Demo Start ---");
  Serial.printf("Free RAM at start: %lu bytes\n", ESP.getFreeHeap());
}

// Connects the ESP32 to the configured Wi-Fi network
void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true); // Ensures disconnection from previous networks
  delay(1000); // Waits for disconnection

  Serial.print("Connecting to Wi-Fi..."); // Initial message appears only once
  WiFi.begin(ssid, password);

  unsigned long wifiConnectStartTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("."); // Dots continue to appear every 500ms
    if (millis() - wifiConnectStartTime > 30000) {
      Serial.println("\nError: Wi-Fi connection timeout. Restarting ESP...");
      ESP.restart(); // Restarts if Wi-Fi connection fails
    }
  }
  Serial.println(); // Adds a new line after the progress dots
  Serial.println("Wi-Fi connected successfully!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// Reboots the display by sending the appropriate Lumen command
void rebootDisplay() {
  Serial.println("Rebooting display before transfer...");
  lumen_packet_t rebootPacket = { REBOOT_ADDRESS, kBool };
  rebootPacket.data._bool = true;
  lumen_write_packet(&rebootPacket); // Sends the reboot command
  Serial.println("Reboot command sent. Starting data transfer...");
  delay(200);
}

// Manages project file download via HTTP
int downloadProjectFile(HTTPClient& http, WiFiClient& client) {
  http.begin(projectFileUrl);
  Serial.printf("Starting download of: %s\n", projectFileUrl);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    int totalFileSize = http.getSize();
    Serial.printf("Download OK. Total file size: %d bytes.\n", totalFileSize);
    Serial.println("Reading file in chunks and sending via Lumen Protocol (Double Buffer)...");
    return totalFileSize;
  } else {
    Serial.printf("HTTP download ERROR: HTTP Code: %d. Details: %s.\n", httpCode, http.errorToString(httpCode).c_str());
    Serial.printf("Attempted URL: %s\n", projectFileUrl);
    return -1; // Returns -1 on download error
  }
}

// Manages project data transfer to the display using double buffering
void transferProjectData(WiFiClient& client, int totalFileSize) {
  projectTransferStarted = true;

  // Initialize the first buffer for reading (Buffer A)
  currentReadBuffer = bufferA; 
  currentSendBuffer = nullptr; // No buffer to send yet

  while (!projectTransferComplete) {
    // Update Lumen Protocol tick to process asynchronous responses
    unsigned long currentTime = millis();
    if (currentTime - lastLumenTickTime >= LUMEN_TICK_INTERVAL_MS) {
      lumen_project_update_tick(currentTime - lastLumenTickTime);
      lastLumenTickTime = currentTime;
    }

    // Step 1: Attempt to read data from the internet to the read buffer
    // Reading occurs if the current buffer is empty AND there is still data to download.
    if (currentReadBuffer != nullptr && ((currentReadBuffer == bufferA && !bufferA_is_filled) || (currentReadBuffer == bufferB && !bufferB_is_filled))) {
      if (client.connected() && (totalFileSize == -1 || bytesReadTotal < totalFileSize)) {
        size_t bytesReadThisChunk = client.readBytes(currentReadBuffer, BUFFER_SIZE);
        if (bytesReadThisChunk > 0) {
          currentReadBufferLength = bytesReadThisChunk;
          bytesReadTotal += bytesReadThisChunk;
          if (currentReadBuffer == bufferA) bufferA_is_filled = true;
          else bufferB_is_filled = true;
          Serial.printf("Read %d bytes from HTTP. Total: %zu/%d bytes.\n", bytesReadThisChunk, bytesReadTotal, totalFileSize);
        } else if (bytesReadThisChunk == 0) {
          // End of HTTP stream or temporarily no data
          if ((totalFileSize != -1 && bytesReadTotal >= totalFileSize) || !client.connected()) {
            Serial.println("HTTP stream ended. No more data to read.");
            currentReadBuffer = nullptr; // Signals that there are no more buffers to fill
          }
        }
      } else {
        // No more HTTP data or connection lost
        Serial.println("No more HTTP data to read or connection ended.");
        currentReadBuffer = nullptr; // Signals that there are no more buffers to fill
      }
    }

    // Step 2: If there's a filled buffer and no buffer is currently being sent, prepare it for sending
    if (currentSendBuffer == nullptr) {
      if (bufferA_is_filled) {
        currentSendBuffer = bufferA;
        currentSendBufferLength = currentReadBufferLength;
        currentReadBuffer = bufferB; // Next read will go to the other buffer
        Serial.println("Buffer A ready for sending. Next read will go to Buffer B.");
      } else if (bufferB_is_filled) {
        currentSendBuffer = bufferB;
        currentSendBufferLength = currentReadBufferLength;
        currentReadBuffer = bufferA; // Next read will go to the other buffer
        Serial.println("Buffer B ready for sending. Next read will go to Buffer A.");
      }
    }

    // Step 3: Attempt to send the current buffer to Lumen
    if (currentSendBuffer != nullptr) {
      bool lumen_send_success = lumen_project_update_send_data(currentSendBuffer, currentSendBufferLength);
      if (lumen_send_success) {
        Serial.printf("Buffer of %d bytes sent successfully to Lumen.\n", currentSendBufferLength);
        // The buffer that was just sent is now empty and ready to be read into again
        if (currentSendBuffer == bufferA) bufferA_is_filled = false;
        else bufferB_is_filled = false;
        currentSendBuffer = nullptr; // Signals that no buffer is currently being sent
      }
    }

    // Step 4: Check transfer completion condition
    // Transfer is complete if all data has been read and all buffers have been emptied.
    if ((totalFileSize != -1 && bytesReadTotal >= totalFileSize) &&
        !bufferA_is_filled && !bufferB_is_filled &&
        currentSendBuffer == nullptr && currentReadBuffer == nullptr) {
        projectTransferComplete = true;
        Serial.println("All file data processed and buffers emptied.");
    }
  }
  Serial.printf("Total of %zu project bytes received and processed.\n", bytesReadTotal);
}

// Finalizes the Lumen project update process
void finalizeProjectTransfer() {
  bool finalLumenSendSuccess = false;
  while (!finalLumenSendSuccess) {
      unsigned long currentTime = millis();
      if (currentTime - lastLumenTickTime >= LUMEN_TICK_INTERVAL_MS) {
          lumen_project_update_tick(currentTime - lastLumenTickTime);
          lastLumenTickTime = currentTime;
      }
      finalLumenSendSuccess = lumen_project_update_send_data(nullptr, 0); // Signals end to Lumen
  }
  lumen_project_update_finish(); 
  Serial.println("Lumen project transfer completed successfully!");
  projectTransferComplete = true;
  projectTransferStarted = false;
}

// --- Main Arduino Functions ---

void setup() {
  initializeSerialCommunications();
  connectToWiFi();

  // --- Step 1: Get total file size (pre-download) ---
  HTTPClient httpPreDownload;
  httpPreDownload.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  httpPreDownload.begin(projectFileUrl);
  Serial.printf("Starting pre-download to get file size: %s\n", projectFileUrl);
  int httpCodePreDownload = httpPreDownload.GET();
  int totalFileSize = -1;

  if (httpCodePreDownload == HTTP_CODE_OK) {
    totalFileSize = httpPreDownload.getSize();
    Serial.printf("Pre-download OK. Total file size: %d bytes.\n", totalFileSize);
  } else {
    Serial.printf("HTTP pre-download ERROR: HTTP Code: %d. Details: %s.\n", httpCodePreDownload, httpPreDownload.errorToString(httpCodePreDownload).c_str());
    Serial.printf("Attempted URL: %s\n", projectFileUrl);
    Serial.println("--- Demo End (Pre-download Error) ---");
    httpPreDownload.end(); // Close pre-download connection
    return; // Exit setup if pre-download fails
  }
  httpPreDownload.end(); // Close pre-download connection

  // --- Step 2: Reboot the display ---
  rebootDisplay();

  // --- Step 3: Reopen HTTP connection for actual data transfer ---
  HTTPClient httpTransfer; // New HTTPClient instance for transfer
  httpTransfer.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  httpTransfer.begin(projectFileUrl);
  Serial.printf("Reopening HTTP connection for data transfer: %s\n", projectFileUrl);
  int httpCodeTransfer = httpTransfer.GET(); // Get the file again for a fresh stream

  if (httpCodeTransfer == HTTP_CODE_OK) {
    WiFiClient& clientForTransfer = httpTransfer.getStream(); // Get reference to the new stream
    Serial.println("HTTP connection reopened successfully for transfer.");
    
    // --- Step 4: Transfer project data to the display ---
    transferProjectData(clientForTransfer, totalFileSize);
    
    // --- Step 5: Finalize Lumen project transfer ---
    finalizeProjectTransfer();
  } else {
    Serial.printf("ERROR reopening HTTP connection for transfer: HTTP Code: %d. Details: %s.\n", httpCodeTransfer, httpTransfer.errorToString(httpCodeTransfer).c_str());
    Serial.printf("Attempted URL: %s\n", projectFileUrl);
  }
  httpTransfer.end(); // Ensures the HTTP transfer connection is closed

  Serial.printf("Free RAM at end of Setup: %lu bytes\n", ESP.getFreeHeap());
  Serial.println("--- Demo End ---");
}

void loop() {
  // Continuously update Lumen Protocol tick in the main loop
  unsigned long currentTime = millis();
  if (currentTime - lastLumenTickTime >= LUMEN_TICK_INTERVAL_MS) {
    lumen_project_update_tick(currentTime - lastLumenTickTime);
    lastLumenTickTime = currentTime;
  }
}
