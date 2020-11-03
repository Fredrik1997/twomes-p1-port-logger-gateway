/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-many-to-one-esp32/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <esp_now.h>
#include <WiFi.h>
//#include <ArduinoJson.h>
//#include <AsyncTCP/src/AsyncTCP.h>
//#include <ESPAsyncWebServer.h>


// Structure example to receive data
// Must match the sender structure
typedef struct struct_message
{
  int id;
  int x;
  int y;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;
struct_message board3;

// Create an array with all the structures
struct_message boardsStruct[3] = {board1, board2, board3};

#define testDevices 5

struct timing_communication
{
  unsigned long lastContact;
  unsigned long firstRegisterTime;
  unsigned long downTime;
};

//JSONVar board;

//AsyncWebServer server(80);

//AsyncEventSource events("/events");

struct timing_communication node[testDevices];

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  char macStr[18];
  //Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));

  if (node[myData.id].firstRegisterTime == 0) //(firstRegisterTime[myData.id] == 0)
  {
    node[myData.id].firstRegisterTime = millis();               //firstRegisterTime[myData.id]
    node[myData.id].lastContact =node[myData.id].firstRegisterTime; //lastContact[myData.id] = firstRegisterTime[myData.id];
  }
#define receiveInterval 1005
  if (millis() - node[myData.id].lastContact > receiveInterval)
  {
    Serial.print("Last receive interval was: ");
    Serial.println((millis() - node[myData.id].lastContact)); /// 1000
    node[myData.id].downTime += (millis() - node[myData.id].lastContact) - 1000; //downTime[myData.id]
  }
  node[myData.id].lastContact = millis();

  //Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  boardsStruct[myData.id - 1].x = myData.x;
  boardsStruct[myData.id - 1].y = myData.y;
  Serial.printf("x value: %d \n", boardsStruct[myData.id - 1].x);
  Serial.printf("y value: %d \n", boardsStruct[myData.id - 1].y);
  Serial.println();
}

void setup()
{
  //Initialize //Serial Monitor
  Serial.begin(115200);
  Serial.println("active");

  pinMode(LED_BUILTIN, OUTPUT);
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
  // Acess the variables for each board
  /*int board1X = boardsStruct[0].x;
  int board1Y = boardsStruct[0].y;
  int board2X = boardsStruct[1].x;
  int board2Y = boardsStruct[1].y;
  int board3X = boardsStruct[2].x;
  int board3Y = boardsStruct[2].y;*/
  for (byte testCount = 0; testCount < testDevices; testCount++)
  {
    if (node[testCount].firstRegisterTime != 0) //firstRegisterTime[testCount]
    {
      Serial.print(testCount);
      Serial.print(". downtime = ");
      Serial.print(node[testCount].downTime); //downTime[testCount]
      Serial.print(", uptime vs. downtime = ");
      Serial.print(float(100 - ((100 * node[testCount].downTime) / (millis() - node[testCount].firstRegisterTime)))); //firstRegisterTime[testCount]
      Serial.println(" %");
    }

    if (node[testCount].lastContact == 0)//lastContact[testCount]
    {
      //Serial.print("Has never received something from device ");
      //Serial.print(testCount);
      //Serial.println("");
    }
    else if ((millis() - node[testCount].lastContact) > 1000)
    {
      //Serial.print("Last contact with device ");
      //Serial.print(testCount);
      //Serial.print(" was ");
      //Serial.print((millis() - lastContact[testCount]) / 1000);
      //Serial.println(" seconds ago.");
      // if (digitalRead(LED_BUILTIN) == HIGH)
      // {
      //   digitalWrite(LED_BUILTIN, LOW);
      // }
    }
    else
    {
      //Serial.print("Connection with device ");
      //Serial.print(testCount);
      //Serial.println(" stable");
    }
    // else if (digitalRead(LED_BUILTIN) == LOW)
    // {
    //   digitalWrite(LED_BUILTIN, HIGH);
    // }
    // if (testCount == testDevices)
    // {
    //   //Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    // }
  }

  delay(5000);
}