/** NimBLE_Server Demo:
 *
This is working to broadcast Power and Cadence under the Cycling Power Service Profile
Data tested against Edge and Phone
 * 
*/
#include <Arduino.h>
#include <NimBLEDevice.h>

short powerInstantaneous = 0;
short cadenceInstantaneous = 0;
short speedInstantaneous = 0;
float powerScale = 1.28; // incoming power is multiplied by this value for correction
short resistance = 0; //Not currently doing anything with this value after receiving it
bool notify = false;

// Define stuff for the Client that will receive data from Fitness Machine
// The remote service we wish to connect to.
static BLEUUID serviceUUID("1826"); // Fitness Machine
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("2ad2"); // Indoor Bike (Fitness Machine)

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *myDevice;
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
  Serial.printf("Power = %d | Cadence = %d | Resistance = %d\n", powerInstantaneous, cadenceInstantaneous, resistance);
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
    Serial.println("onDisconnect");
  }
};

bool connectToServer()
{
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

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
float totalDistance = 0.0; // in meters
float wheelCircumference = 2.096; // Standard road bike wheel circumference in meters (700x25c)
long lastWheelRevolution = 0;
float estimatedSpeed = 0.0; // km/h

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting NimBLE Server");

  /** sets device name */
  NimBLEDevice::init("Yesoul_CP");
  /** Optional: set the transmit power, default is 3db */
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */

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
  // scan to run for 5 seconds.
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

void loop()
{
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true)
  {
    if (connectToServer())
    {
      Serial.println("We are now connected to the BLE Server.");
    }
    else
    {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }
  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected)
  {
    //Stuff to do when connected to Client
  }
  else if (doScan)
  {
    BLEDevice::getScan()->start(0); // this is just sample to start scan after disconnect, most likely there is better way to do it in arduino
  }

  // convert RPM to timestamp
  if (cadenceInstantaneous != 0 && (millis()) >= (lastRevolution + (60000 / cadenceInstantaneous)))
  {
    revolutions++;                                  // One crank revolution should have passed, add one revolution
    timestamp = (unsigned short)(((millis() * 1024) / 1000) % 65536); // create timestamp and format
    lastRevolution = millis();
  }

  // Calculate distance from cadence (indoor cycling estimation)
  // Estimate speed based on cadence and typical gear ratio for indoor cycling
  if (cadenceInstantaneous > 0) {
    // Rough estimation: cadence * gear ratio factor * wheel circumference
    // For indoor cycling, we'll use a moderate gear ratio equivalent
    float gearRatio = 2.5; // Typical indoor cycling gear ratio
    estimatedSpeed = (cadenceInstantaneous * gearRatio * wheelCircumference * 60) / 1000; // km/h
    
    // Calculate wheel revolutions based on estimated speed
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
