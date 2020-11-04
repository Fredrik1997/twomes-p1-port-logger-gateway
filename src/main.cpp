#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>

#define macArrayLength 6

#define testDevices 6
#define receiveInterval 1005

byte lastAddedNode = 0; //last node which registered at this gateway

typedef struct esp_now_message
{
  byte macID[macArrayLength];
  int temperature1;
  int temperature2;
} esp_now_message;

esp_now_message node[testDevices];

// esp_now_message node0, node1, node2, node3, node4, node5;

// esp_now_message *nodes[testDevices] = {&node0, &node1, &node2, &node3, &node4, &node5};

struct communicationTests
{
  unsigned int missedCalls;
  unsigned long lastContact;
  unsigned long firstRegisterTime;
  unsigned long averageInterval;
};

struct communicationTests nodeTests[testDevices];

typedef struct struct_message
{
  int temperature1;
  int temperature2;
} struct_message;

struct_message myData;

//prototype functions
byte checkExistingMac(byte *pntToMac);
void putMac(byte *pntToMac, byte selectedRow);

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  digitalWrite(LED_BUILTIN, HIGH);
  // char macStr[18];
  // Serial.print("Packet received from: ");
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // Serial.println(macStr);
  byte tempMac[macArrayLength] = {mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]};

  //check if this mac matches with a existing macID
  byte nodeNumber = checkExistingMac(tempMac);
  if (nodeNumber == 255)
  {
    Serial.println("mac not in database yet");
    putMac(tempMac, lastAddedNode);
    nodeTests[lastAddedNode].firstRegisterTime = millis();
    lastAddedNode++;
  }
  else
  {
    long logTime = millis();

    // Serial.printf("Node: %x is known\n", nodeNumber);
    memcpy(&myData, incomingData, sizeof(myData));
    // Serial.print("temperature1 = ");
    // Serial.println(myData.temperature1);
    // Serial.print("temperature2 = ");
    // Serial.println(myData.temperature2);
    if (myData.temperature1 != (node[nodeNumber].temperature1 + 1)) //if we missed a call
    {
      int difference = myData.temperature1 - node[nodeNumber].temperature1;
      #define thresholdDifference 60
      if (difference > thresholdDifference || difference < thresholdDifference)
      {
        Serial.printf("Differene = %i", difference);
        Serial.println("difference to big, so not saved");
      }
      else
      {
        Serial.printf("missed calls: %i\n", difference);
        nodeTests[nodeNumber].missedCalls += difference;
      }
    }
    else
    {
      long interval = logTime - nodeTests[nodeNumber].lastContact;
      nodeTests[nodeNumber].averageInterval += interval;
      nodeTests[nodeNumber].averageInterval = nodeTests[nodeNumber].averageInterval / 2;    //average not calculated on a right way
    }
    nodeTests[nodeNumber].lastContact = logTime;
    node[nodeNumber].temperature1 = myData.temperature1;
    node[nodeNumber].temperature2 = myData.temperature2;
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void putMac(byte *pntToMac, byte selectedRow)
{
  for (byte macTestFill = 0; macTestFill < macArrayLength; macTestFill++)
  {
    node[selectedRow].macID[macTestFill] = (*pntToMac + macTestFill);
  }
}

//input:  pointer to mac address, byte array with 'macArrayLength' positions
//output: 255: did not find existing node, 0-254 found existing node
//effect: return existing node number if this exist, if not return 255
byte checkExistingMac(byte *pntToMac)
{
  for (byte currentNode = 0; currentNode < testDevices; currentNode++)
  {
    //Serial.printf("currentNode = %i\n", currentNode);
    for (byte macTestFill = 0; macTestFill < macArrayLength; macTestFill++)
    {
      // Serial.printf("macTestFill = %i\n", macTestFill);
      // Serial.print("incoming: ");
      // Serial.println(*pntToMac + macTestFill);
      // Serial.printf("database: %i\n", nodes[currentNode]->macID[macTestFill]);
      if (node[currentNode].macID[macTestFill] != (*pntToMac + macTestFill))
      {
        //Serial.println("wrong");
        break;
      }
      else if (macTestFill == 5)
      {
        //Serial.println("good in last test");
        return currentNode;
      }
    }
  }
  return 255;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Serial active");
  delay(10);
  // for (byte macSearch = 0; macSearch < testDevices; macSearch++) //print macs from memory to monitor
  // {
  //   //Serial.print(node[macSearch].macID[1]);
  //   Serial.printf("Node: %i has mac: ", macSearch);
  //   Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n", node[macSearch].macID[0], node[macSearch].macID[1], node[macSearch].macID[2], node[macSearch].macID[3], node[macSearch].macID[4], node[macSearch].macID[5]);
  // }

  // uint8_t macToTest[macArrayLength] = {4, 5, 6, 7, 8, 9};    //test for fucntino checkExistingMac
  // Serial.printf("matching node : %x", checkExistingMac(macToTest));

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  WiFi.mode(WIFI_STA); //Set device as a Wi-Fi Station

  if (esp_now_init() != ESP_OK) //Init ESP-NOW
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv); // Once ESPNow is successfully Init, we will register for recv CB to, get recv packer info
}

void loop()
{
  for (byte testCount = 0; testCount < lastAddedNode; testCount++)
  {
    Serial.printf("Node %i:\n", testCount);
    Serial.printf("Last contact %lu microseconds ago\n", (millis() - nodeTests[testCount].lastContact));
    Serial.printf("Missed calls %i\n", nodeTests[testCount].missedCalls);
    Serial.printf("Registered %lu seconds ago\n", (millis() - nodeTests[testCount].firstRegisterTime) / 1000);
    Serial.printf("Average interval time is %lu microseconds\n", nodeTests[testCount].averageInterval);
    Serial.printf("----\n");
  }
  Serial.println("-----------------------------");
  delay(5000);
}