/** NimBLE_Server Demo:
 *
This is working to broadcast Power and Cadence under the Cycling Power Service Profile
Data tested against Edge and Phone
 * 
*/
#include <Arduino.h>
#include <NimBLEDevice.h>

#ifndef LED_PIN
#define LED_PIN 22  // GPIO 22 for LoLin32 LED (can be overridden by build flags)
#endif

#ifndef LED_ACTIVE_LOW
#define LED_ACTIVE_LOW 0
#endif

#define LED_ON  (LED_ACTIVE_LOW ? LOW : HIGH)
#define LED_OFF (LED_ACTIVE_LOW ? HIGH : LOW)

short powerInstantaneous = 0;
short cadenceInstantaneous = 0;
short speedInstantaneous = 0;
float powerScale = 1.28; // incoming power is multiplied by this value for correction
short resistance = 0; // Used for virtual gear ratio
unsigned long lastYesoulDataTime = 0;
const unsigned long YESOUL_DATA_TIMEOUT = 3000; // 3 seconds timeout for stale data
bool notify = false;

// Distance and speed tracking
float estimatedSpeed = 0.0; // km/h
float totalDistance = 0.0; // in meters

// LED status variables
enum LEDState {
  LED_CONNECTING_YESOUL,     // Fast blink - connecting to Yesoul
  LED_YESOUL_CONNECTED,      // Double blink - Yesoul connected
  LED_WAITING_CLIENT,        // Slow blink - waiting for iPhone/Apple Watch
  LED_CLIENT_CONNECTED       // Solid on - iPhone/Apple Watch connected
};
LEDState currentLEDState = LED_CONNECTING_YESOUL;
unsigned long lastLEDBlink = 0;
int blinkCount = 0;
bool ledState = false;

// Define stuff for the Client that will receive data from Fitness Machine
// The remote service we wish to connect to.
static BLEUUID serviceUUID("1826"); // Fitness Machine
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("2ad2"); // Indoor Bike (Fitness Machine)

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = true;  // Start with scan enabled
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *myDevice;

// Reconnection logic - exponential backoff with retry limits
const unsigned short MAX_RECONNECT_RETRIES = 10;     // Maximum retry attempts
const unsigned long INITIAL_BACKOFF_MS = 1000;       // 1 second initial backoff
const unsigned long MAX_BACKOFF_MS = 60000;          // 60 seconds maximum backoff
const float BACKOFF_MULTIPLIER = 1.5;                // Exponential backoff multiplier
unsigned short reconnectRetryCount = 0;              // Current retry attempt
unsigned long lastReconnectAttempt = 0;              // Timestamp of last reconnect attempt
unsigned long currentBackoffMs = INITIAL_BACKOFF_MS; // Current backoff duration
bool maxRetriesReached = false;                      // Flag when max retries exceeded

/* 
 * Server Stuff
 */
static NimBLEServer *pServer;
/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class ServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer)
  {
    Serial.println("Client connected");
    Serial.println("Multi-connect support: start advertising");
    currentLEDState = LED_CLIENT_CONNECTED;  // Switch to solid LED
    NimBLEDevice::startAdvertising();
  };
  /** Alternative onConnect() method to extract details of the connection. 
     *  See: src/ble_gap.h for the details of the ble_gap_conn_desc struct.
     */
  void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc)
  {
    Serial.print("Client address: ");
    Serial.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
    /** We can use the connection handle here to ask for different connection parameters.
         *  Args: connection handle, min connection interval, max connection interval
         *  latency, supervision timeout.
         *  Units; Min/Max Intervals: 1.25 millisecond increments.
         *  Latency: number of intervals allowed to skip.
         *  Timeout: 10 millisecond increments, try for 5x interval time for best results.  
         */
    pServer->updateConnParams(desc->conn_handle, 24, 48, 0, 60);
  };
  void onDisconnect(NimBLEServer *pServer)
  {
    Serial.println("Client disconnected - start advertising");
    currentLEDState = LED_WAITING_CLIENT;  // Back to slow blink
    NimBLEDevice::startAdvertising();
  };
  void onMTUChange(uint16_t MTU, ble_gap_conn_desc *desc)
  {
    Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);
  };
};

/** Handler class for characteristic actions */
class CharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
  void onRead(NimBLECharacteristic *pCharacteristic)
  {
    Serial.print(pCharacteristic->getUUID().toString().c_str());
    Serial.print(": onRead(), value: ");
    Serial.println(pCharacteristic->getValue().c_str());
  };

  void onWrite(NimBLECharacteristic *pCharacteristic)
  {
    Serial.print(pCharacteristic->getUUID().toString().c_str());
    Serial.print(": onWrite(), value: ");
    Serial.println(pCharacteristic->getValue().c_str());
  };
  /** Called before notification or indication is sent, 
     *  the value can be changed here before sending if desired.
     */
  void onNotify(NimBLECharacteristic *pCharacteristic)
  {
    Serial.println("Sending notification to clients");
  };

  /** The status returned in status is defined in NimBLECharacteristic.h.
     *  The value returned in code is the NimBLE host return code.
     */
  void onStatus(NimBLECharacteristic *pCharacteristic, Status status, int code)
  {
    String str = ("Notification/Indication status code: ");
    str += status;
    str += ", return code: ";
    str += code;
    str += ", ";
    str += NimBLEUtils::returnCodeToString(code);
    Serial.println(str);
  };

  void onSubscribe(NimBLECharacteristic *pCharacteristic, ble_gap_conn_desc *desc, uint16_t subValue)
  {
    String str = "Client ID: ";
    str += desc->conn_handle;
    str += " Address: ";
    str += std::string(NimBLEAddress(desc->peer_ota_addr)).c_str();
    if (subValue == 0)
    {
      str += " Unsubscribed to ";
    }
    else if (subValue == 1)
    {
      str += " Subscribed to notifications for ";
    }
    else if (subValue == 2)
    {
      str += " Subscribed to indications for ";
    }
    else if (subValue == 3)
    {
      str += " Subscribed to notifications and indications for ";
    }
    str += std::string(pCharacteristic->getUUID()).c_str();

    Serial.println(str);
  };
};

/** Handler class for descriptor actions */
class DescriptorCallbacks : public NimBLEDescriptorCallbacks
{
  void onWrite(NimBLEDescriptor *pDescriptor)
  {
    std::string dscVal = pDescriptor->getValue();
    Serial.print("Descriptor witten value:");
    Serial.println(dscVal.c_str());
  };

  void onRead(NimBLEDescriptor *pDescriptor)
  {
    Serial.print(pDescriptor->getUUID().toString().c_str());
    Serial.println(" Descriptor read");
  };
};
/* 
 * Client Stuff
 */
// This callback is for when data is received from Server
static void notifyCallback(
    BLERemoteCharacteristic *pBLERemoteCharacteristic,
    uint8_t *pData,
    size_t length,
    bool isNotify)
{
  powerInstantaneous = pData[11] | pData[12] << 8;       // 2 bytes of power
  // Serial.printf("Power = %d\n", powerInstantaneous);
  // powerInstantaneous = powerInstantaneous * powerScale;  //power value correction
  cadenceInstantaneous = (pData[4] | pData[5] << 8) / 2; // 2 bytes of power in 0.5 resolution RPM, convert to RPM
  resistance = pData[9];                                 // 1 byte of resistance
  lastYesoulDataTime = millis();                         // Track for stale data timeout
  Serial.printf("Power = %d | Cadence = %d | Resistance = %d | Speed = %.1f km/h | Distance = %.2f m\n", powerInstantaneous, cadenceInstantaneous, resistance, estimatedSpeed, totalDistance);
}

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient *pclient)
  {
  }

  void onDisconnect(BLEClient *pclient)
  {
    connected = false;
    currentLEDState = LED_CONNECTING_YESOUL;  // Back to fast blink
    
    // Implement exponential backoff on disconnect
    if (maxRetriesReached) {
      Serial.println("onDisconnect - Max retries reached. Manual reset required.");
      doScan = false;
      return;
    }
    
    reconnectRetryCount++;
    lastReconnectAttempt = millis();
    
    // Calculate next backoff: cap at MAX_BACKOFF_MS
    currentBackoffMs = (unsigned long)(INITIAL_BACKOFF_MS * pow(BACKOFF_MULTIPLIER, reconnectRetryCount - 1));
    if (currentBackoffMs > MAX_BACKOFF_MS) {
      currentBackoffMs = MAX_BACKOFF_MS;
    }
    
    Serial.printf("onDisconnect - Yesoul disconnected. Retry %u/%u, backoff: %lums\n", 
                  reconnectRetryCount, MAX_RECONNECT_RETRIES, currentBackoffMs);
    
    doScan = false;  // Don't scan yet, wait for backoff
  }
};

bool connectToServer()
{
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());
  
  currentLEDState = LED_CONNECTING_YESOUL;  // Fast blink during connection

  BLEClient *pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(myDevice); // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr)
  {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr)
  {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  // Read the value of the characteristic.
  if (pRemoteCharacteristic->canRead())
  {
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  connected = true;
  currentLEDState = LED_YESOUL_CONNECTED;  // Double blink - connection successful
  blinkCount = 0;
  return true;
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  /**
   * Called for each advertising BLE server.
   */

  /*** Only a reference to the advertised device is passed now
  void onResult(BLEAdvertisedDevice advertisedDevice) { **/
  void onResult(BLEAdvertisedDevice *advertisedDevice)
  {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice->toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    /********************************************************************************
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
********************************************************************************/
    if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(serviceUUID))
    {

      BLEDevice::getScan()->stop();
      /*******************************************************************
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
*******************************************************************/
      myDevice = advertisedDevice; /** Just save the reference now, no need to copy the object */
      doConnect = true;
      doScan = true;

    } // Found our server
  }   // onResult
};    // MyAdvertisedDeviceCallbacks

//delays for X ms, should not block execution
void softDelay(unsigned long delayTime)
{
  unsigned long startTime = millis();
  while ((millis() - startTime) < delayTime)
  {
    //wait
  }
}

// LED control function
void updateLED() {
  unsigned long currentTime = millis();
  
  switch (currentLEDState) {
    case LED_CONNECTING_YESOUL:
      // Fast blink - 200ms interval
      if (currentTime - lastLEDBlink >= 200) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? LED_ON : LED_OFF);
        lastLEDBlink = currentTime;
      }
      break;
      
    case LED_YESOUL_CONNECTED:
      // Double blink - slow pattern with pause
      if (blinkCount < 4) {
        // Blink pattern: ON-OFF-ON-OFF (4 state changes)
        if (currentTime - lastLEDBlink >= 200) {
          ledState = !ledState;
          digitalWrite(LED_PIN, ledState ? LED_ON : LED_OFF);
          blinkCount++;
          lastLEDBlink = currentTime;
        }
      } else if (blinkCount == 4) {
        // Pause after double blink
        if (currentTime - lastLEDBlink >= 500) {
          digitalWrite(LED_PIN, LED_OFF);
          currentLEDState = LED_WAITING_CLIENT;
          blinkCount = 0;
          ledState = false;
          lastLEDBlink = currentTime;
        }
      }
      break;
      
    case LED_WAITING_CLIENT:
      // Slow blink - 1000ms interval
      if (currentTime - lastLEDBlink >= 1000) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? LED_ON : LED_OFF);
        lastLEDBlink = currentTime;
      }
      break;
      
    case LED_CLIENT_CONNECTED:
      // Solid on
      digitalWrite(LED_PIN, LED_ON);
      break;
  }
}

/** Define callback instances globally to use for multiple Charateristics \ Descriptors */
// This section is for the Server that will broadcast the data as Cycling Power
static DescriptorCallbacks dscCallbacks;
static CharacteristicCallbacks chrCallbacks;
// Cycling Power Service characteristics
NimBLECharacteristic *CyclingPowerFeature = NULL;
NimBLECharacteristic *CyclingPowerMeasurement = NULL;
NimBLECharacteristic *CyclingPowerSensorLocation = NULL;

// Cycling Speed and Cadence Service characteristics  
NimBLECharacteristic *CSCFeature = NULL;
NimBLECharacteristic *CSCMeasurement = NULL;
NimBLECharacteristic *CSCSensorLocation = NULL;

unsigned char bleBuffer[8];
unsigned char cscBuffer[11]; // Increased size for wheel + crank data
unsigned char slBuffer[1];
unsigned char fBuffer[4];
unsigned char cscFeatureBuffer[2];
unsigned short revolutions = 0;
unsigned short timestamp = 0;
unsigned short flags = 0x20; // Crank revolution data present flag
byte sensorlocation = 0x0D;
long lastNotify = 0;
long lastRevolution = 0;

// Distance tracking variables
unsigned long wheelRevolutions = 0;
unsigned short wheelTimestamp = 0;
float wheelCircumference = 2.096; // Standard road bike wheel circumference in meters (700x25c)
long lastWheelRevolution = 0;

// Function to reset reconnection state (can be called after max retries)
void resetReconnectionState() {
  reconnectRetryCount = 0;
  currentBackoffMs = INITIAL_BACKOFF_MS;
  maxRetriesReached = false;
  doScan = true;
  lastReconnectAttempt = 0;
  Serial.println("\n*** Reconnection state reset. Restarting scan...\n");
}

// Techno Gym calibrated speed curve based on power
// Calibration points: 90W→21.6km/h, 105W→25.2km/h, 170W→28.8km/h
float getPowerBasedSpeed(short power) {
  if (power <= 90) {
    // Below 90W - linear extrapolation from origin
    return 21.6 * (power / 90.0);
  } else if (power <= 105) {
    // Interpolate between 90W (21.6 km/h) and 105W (25.2 km/h)
    return 21.6 + (power - 90) * (25.2 - 21.6) / (105.0 - 90.0);
  } else if (power <= 170) {
    // Interpolate between 105W (25.2 km/h) and 170W (28.8 km/h)
    return 25.2 + (power - 105) * (28.8 - 25.2) / (170.0 - 105.0);
  } else {
    // Above 170W - continue the trend
    return 28.8 + (power - 170) * (28.8 - 25.2) / (170.0 - 105.0);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting NimBLE Server");
  
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);
  currentLEDState = LED_CONNECTING_YESOUL;

  /** sets device name */
  NimBLEDevice::init("Yesoul_CP");
  /** Optional: set the transmit power, default is 3db */
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
  
  // Print Bluetooth MAC Address for NFC tag
  Serial.println("========================================");
  Serial.print("Bluetooth MAC Address: ");
  Serial.println(NimBLEDevice::getAddress().toString().c_str());
  Serial.println("========================================");

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Cycling Power Service setup
  fBuffer[0] = 0x00;
  fBuffer[1] = 0x00;
  fBuffer[2] = 0x00;
  fBuffer[3] = 0x08;

  slBuffer[0] = sensorlocation & 0xff;

  // Create Cycling Power Service (0x1818)
  NimBLEService *pPowerService = pServer->createService("1818");
  CyclingPowerFeature = pPowerService->createCharacteristic(
      "2A65",
      NIMBLE_PROPERTY::READ);
  CyclingPowerSensorLocation = pPowerService->createCharacteristic(
      "2A5D",
      NIMBLE_PROPERTY::READ);
  CyclingPowerMeasurement = pPowerService->createCharacteristic(
      "2A63",
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  CyclingPowerFeature->setValue(fBuffer, 4);
  CyclingPowerSensorLocation->setValue(slBuffer, 1);
  CyclingPowerMeasurement->setValue(slBuffer, 1);

  // Create Cycling Speed and Cadence Service (0x1816) for better Apple Watch compatibility
  NimBLEService *pCSCService = pServer->createService("1816");
  
  // CSC Feature: bit 0 = Wheel Revolution Data Supported, bit 1 = Crank Revolution Data Supported
  cscFeatureBuffer[0] = 0x03; // Both wheel and crank revolution data supported
  cscFeatureBuffer[1] = 0x00;
  
  CSCFeature = pCSCService->createCharacteristic(
      "2A5C",  // CSC Feature characteristic
      NIMBLE_PROPERTY::READ);
  CSCSensorLocation = pCSCService->createCharacteristic(
      "2A5D",  // Sensor Location characteristic  
      NIMBLE_PROPERTY::READ);
  CSCMeasurement = pCSCService->createCharacteristic(
      "2A5B",  // CSC Measurement characteristic
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  CSCFeature->setValue(cscFeatureBuffer, 2);
  CSCSensorLocation->setValue(slBuffer, 1);
  CSCMeasurement->setValue(slBuffer, 1);

  /** Start the services when finished creating all Characteristics and Descriptors */
  pPowerService->start();
  pCSCService->start();

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  /** Add the services to the advertisment data **/
  pAdvertising->addServiceUUID(pPowerService->getUUID());
  pAdvertising->addServiceUUID(pCSCService->getUUID());
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("Advertising Started");

  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run continuously until device is found.
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  // Optimized scan parameters for responsiveness (power not a concern)
  // Shorter intervals = faster device discovery and reconnection
  pBLEScan->setInterval(500);   // Reduced from 1349 (500ms scan interval)
  pBLEScan->setWindow(250);     // Reduced from 449 (250ms scan window)
  pBLEScan->setActiveScan(true);
  pBLEScan->start(0, false);  // 0 = continuous scanning
}

void loop()
{
  // Update LED status
  updateLED();
  
  // Handle serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "reset") {
      resetReconnectionState();
    } else if (command == "status") {
      Serial.println("\n=== Connection Status ===");
      Serial.printf("Connected to Yesoul: %s\n", connected ? "YES" : "NO");
      Serial.printf("Max retries reached: %s\n", maxRetriesReached ? "YES" : "NO");
      Serial.printf("Retry count: %u/%u\n", reconnectRetryCount, MAX_RECONNECT_RETRIES);
      if (!connected && !maxRetriesReached && reconnectRetryCount > 0) {
        unsigned long elapsed = millis() - lastReconnectAttempt;
        Serial.printf("Next retry in: %lu ms\n", (elapsed < currentBackoffMs) ? (currentBackoffMs - elapsed) : 0);
      }
      Serial.println("=========================\n");
    } else if (command == "help") {
      Serial.println("\n=== Available Commands ===");
      Serial.println("  reset   - Reset reconnection state and restart scan");
      Serial.println("  status  - Show connection status");
      Serial.println("  help    - Show this help message");
      Serial.println("===========================\n");
    }
  }
  
  // Stale data timeout (zero out values if no recent packets)
  if (millis() - lastYesoulDataTime > YESOUL_DATA_TIMEOUT) {
    powerInstantaneous = 0;
    cadenceInstantaneous = 0;
  }

  // Handle exponential backoff before retrying
  if (!doScan && !connected && !maxRetriesReached) {
    unsigned long timeSinceDisconnect = millis() - lastReconnectAttempt;
    if (timeSinceDisconnect >= currentBackoffMs) {
      doScan = true;  // Proceed with scan after backoff period
      Serial.printf("Backoff complete. Attempting reconnect (try %u/%u)...\n", 
                    reconnectRetryCount, MAX_RECONNECT_RETRIES);
    }
  }
  
  // Check if max retries exceeded
  if (reconnectRetryCount >= MAX_RECONNECT_RETRIES && !connected) {
    if (!maxRetriesReached) {
      maxRetriesReached = true;
      doScan = false;
      Serial.println("\n*** MAX RECONNECTION RETRIES REACHED ***");
      Serial.println("Manual reset (power cycle) required to restart connection attempts.");
      currentLEDState = LED_CONNECTING_YESOUL;  // LED will blink to indicate error state
    }
  }
  
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true)
  {
    if (connectToServer())
    {
      Serial.println("We are now connected to the BLE Server.");
      // Reset reconnection counters on successful connection
      reconnectRetryCount = 0;
      currentBackoffMs = INITIAL_BACKOFF_MS;
      maxRetriesReached = false;
      doScan = false;  // Stop scanning when connected
    }
    else
    {
      Serial.println("Connection attempt failed; waiting for backoff...");
      doScan = false;  // Don't scan yet, let backoff timer handle it
      reconnectRetryCount++;
      
      if (reconnectRetryCount >= MAX_RECONNECT_RETRIES) {
        maxRetriesReached = true;
        Serial.println("\n*** MAX RECONNECTION RETRIES REACHED ***");
        Serial.println("Manual reset (power cycle) required to restart connection attempts.");
      } else {
        lastReconnectAttempt = millis();
        currentBackoffMs = (unsigned long)(INITIAL_BACKOFF_MS * pow(BACKOFF_MULTIPLIER, reconnectRetryCount - 1));
        if (currentBackoffMs > MAX_BACKOFF_MS) {
          currentBackoffMs = MAX_BACKOFF_MS;
        }
        Serial.printf("Next retry in %lums (attempt %u/%u)\n", currentBackoffMs, reconnectRetryCount, MAX_RECONNECT_RETRIES);
      }
    }
    doConnect = false;
  }
  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected)
  {
    //Stuff to do when connected to Client
  }
  else if (doScan && !maxRetriesReached)
  {
    // Restart scan if not currently scanning
    if (!BLEDevice::getScan()->isScanning()) {
      Serial.println("Restarting scan for Yesoul device...");
      BLEDevice::getScan()->start(0, false);  // Continuous scan with faster intervals
    }
  }

  // convert RPM to timestamp
  if (cadenceInstantaneous != 0 && (millis()) >= (lastRevolution + (60000 / cadenceInstantaneous)))
  {
    revolutions++;                                  // One crank revolution should have passed, add one revolution
    timestamp = (unsigned short)(((millis() * 1024) / 1000) % 65536); // create timestamp and format
    lastRevolution = millis();
  }
  // Calculate speed from power using Techno Gym calibration curve
  if (powerInstantaneous > 0) {
    estimatedSpeed = getPowerBasedSpeed(powerInstantaneous);
  } else {
    estimatedSpeed = 0;
  }
  
  if (cadenceInstantaneous > 0) {
    
    // Calculate wheel revolutions based on realistic speed from power
    unsigned long currentTime = millis();
    if (currentTime >= (lastWheelRevolution + 100)) { // Update every 100ms when moving
      float revolutionsPerSecond = (estimatedSpeed * 1000) / (wheelCircumference * 3600); // rev/sec
      float timeDeltaSeconds = (currentTime - lastWheelRevolution) / 1000.0;
      float newRevolutions = revolutionsPerSecond * timeDeltaSeconds;
      
      if (newRevolutions >= 1.0) {
        wheelRevolutions += (unsigned long)newRevolutions;
        wheelTimestamp = (unsigned short)(((millis() * 1024) / 1000) % 65536);
        totalDistance += newRevolutions * wheelCircumference;
        lastWheelRevolution = currentTime;
      }
    }
  }

  if (millis() - lastNotify >= 1000) // do this every second
  {
    if (pServer->getConnectedCount() > 0)
    {
      // Apply power correction before sending
      // short correctedPower = powerInstantaneous * powerScale;
      
      // Cycling Power Measurement
      bleBuffer[0] = flags & 0xff;
      bleBuffer[1] = (flags >> 8) & 0xff;
      bleBuffer[2] = powerInstantaneous & 0xff;
      bleBuffer[3] = (powerInstantaneous >> 8) & 0xff;
      bleBuffer[4] = revolutions & 0xff;
      bleBuffer[5] = (revolutions >> 8) & 0xff;
      bleBuffer[6] = timestamp & 0xff;
      bleBuffer[7] = (timestamp >> 8) & 0xff;
      CyclingPowerMeasurement->setValue(bleBuffer, 8);
      CyclingPowerMeasurement->notify();
      
      // CSC Measurement - send both wheel and crank revolution data for distance and cadence
      // Flags: bit 0 = Wheel Revolution Data Present, bit 1 = Crank Revolution Data Present
      cscBuffer[0] = 0x03; // Both wheel and crank revolution data present
      
      // Wheel Revolution Data (for distance)
      cscBuffer[1] = wheelRevolutions & 0xff;         // Cumulative Wheel Revolutions (LSB)
      cscBuffer[2] = (wheelRevolutions >> 8) & 0xff;  // Cumulative Wheel Revolutions (byte 1)
      cscBuffer[3] = (wheelRevolutions >> 16) & 0xff; // Cumulative Wheel Revolutions (byte 2)
      cscBuffer[4] = (wheelRevolutions >> 24) & 0xff; // Cumulative Wheel Revolutions (MSB)
      cscBuffer[5] = wheelTimestamp & 0xff;           // Last Wheel Event Time (LSB)
      cscBuffer[6] = (wheelTimestamp >> 8) & 0xff;    // Last Wheel Event Time (MSB)
      
      // Crank Revolution Data (for cadence)
      cscBuffer[7] = revolutions & 0xff;        // Cumulative Crank Revolutions (LSB)
      cscBuffer[8] = (revolutions >> 8) & 0xff; // Cumulative Crank Revolutions (MSB)
      cscBuffer[9] = timestamp & 0xff;          // Last Crank Event Time (LSB)
      cscBuffer[10] = (timestamp >> 8) & 0xff;  // Last Crank Event Time (MSB)
      
      CSCMeasurement->setValue(cscBuffer, 11);
      CSCMeasurement->notify();
      
      lastNotify = millis();
      
      Serial.printf("Sent - Power: %dW, Cadence: %dRPM, Speed: %.1fkm/h, Distance: %.2fm\n", 
                    powerInstantaneous, cadenceInstantaneous, estimatedSpeed, totalDistance);
    }
  }
  if (pServer->getConnectedCount() == 0)
  {
    powerInstantaneous = 0;
  }
}
