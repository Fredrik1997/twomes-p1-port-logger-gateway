#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h> 
#include <util.h>

const char* url = "http://192.168.178.129:8000/slimmemeter";
int i = 0;
void makePostRequest();

void setup() {
  //Initialize //Serial Monitor
  Serial.begin(115200);
  WiFi.mode(WIFI_AP_STA);

  // Connecting to Wi-Fi
  WiFi.begin(SSID, PASS);
  Serial.print("Connecting to WiFi");
  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nConnected to WiFi");
}

void loop() {
  makePostRequest();
  i++;
  delay(5000);
}

void makePostRequest() {
  if(WiFi.status() == WL_CONNECTED){
    Serial.println("Creating HTTP client");
    HTTPClient httpClient;

    String postData = String("{\"dsmrVersion\": ") +i +
    ", \"elecUsedT1\": " +0 +
    ", \"elecUsedT2\": " +0 +
    ", \"elecDeliveredT1\": " +0 +
    ", \"elecDeliveredT2\": " +0 +
    ", \"currentTarrif\": " +0 +
    ", \"elecCurrentUsage\": " +0 +
    ", \"elecCurrentDeliver\": " +0 +
    ", \"gasUsage\": " +0 + 
    ", \"timeGasMeasurement\": " +"0" +
    ", \"timeRead\": " +"0" +
    "}";
    //Serial.println(postData);

    Serial.println("Starting url connection");
    httpClient.begin(url);
    httpClient.addHeader("Content-Type", "application/json");
    
    Serial.println("Creating POST req");
    int httpCode = httpClient.POST(postData);

    if(httpCode > 0){
      String payload = httpClient.getString();
      Serial.println("\nStatuscode: " + String(httpCode));
      Serial.println(payload);
      httpClient.end();
    } else {
        Serial.println("Error has occurred on HTTP request");
      }
  } else {
      Serial.println("Connection lost");
    }
}