#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <Wire.h>
#include <DHT20.h>
#include "config.h"

DHT20 dht20;
int sensorPin=D9;
int buzzerPin=D7;

AsyncWebServer server(httpPort);
AsyncWebSocket ws("/ws");

bool buttonStates[4] = {false, false, false, false};

void TaskTemperatureHumidity(void *pvParameters);
void TaskSwitchRelay(void *pvParameters);

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    if (type == WS_EVT_CONNECT)
    {
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    }
    else if (type == WS_EVT_DISCONNECT)
    {
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
    }
    else if (type == WS_EVT_DATA)
    {
        String message = String((char *)data);
        if (message.startsWith("toggle"))
        {
            int buttonIndex = message.substring(6).toInt();
            Serial.println(buttonIndex);
            buttonStates[buttonIndex] = !buttonStates[buttonIndex];
            String data = String("{\"relay_id\":") + String(buttonIndex+1) + ",\"state\":\"" + (buttonStates[buttonIndex] ? "ON" : "OFF") + "\"}";
            ws.textAll(data);
        }
        // if(message=="getstate")
        // {
        //     Serial.println("getstate");
        //     String data = "{";
        //     data += "\"getState1\":\"" + String(buttonStates[0] ? "ON" : "OFF") + "\",";
        //     data += "\"getState2\":\"" + String(buttonStates[1] ? "ON" : "OFF") + "\",";
        //     data += "\"getState3\":\"" + String(buttonStates[2] ? "ON" : "OFF") + "\",";
        //     data += "\"getState4\":\"" + String(buttonStates[3] ? "ON" : "OFF") + "\"";
        //     data += "}";
        //     Serial.println(data);
        //     ws.textAll(data);
        // }
    }
}

void setup()
{
    Serial.begin(115200);

    if (!LittleFS.begin())
    {
        Serial.println("An Error has occurred while mounting LittleFS");
        return;
    }
    if (!WiFi.config(local_IP, gateway, subnet))
    {
        Serial.println("STA Failed to configure");
    }

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    Serial.println("Connected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    dht20.begin();
    pinMode(sensorPin,INPUT);
    pinMode(buzzerPin, OUTPUT);

    ws.onEvent(onEvent);
    server.addHandler(&ws);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(LittleFS, "/index.html", "text/html"); });
    server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(LittleFS, "/script.js", "application/javascript"); });
    server.on("/styles.css", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(LittleFS, "/styles.css", "text/css"); });

    server.begin();

    xTaskCreate(TaskTemperatureHumidity, "TaskTemperatureHumidity", 2048, NULL, 2, NULL);
    xTaskCreate(TaskSwitchRelay, "TaskSwitchRelay", 2048, NULL, 2, NULL);
    Serial.println("Task started");
}

void loop()
{
  digitalWrite(buzzerPin, digitalRead(sensorPin)==HIGH ? LOW : HIGH);
  ws.cleanupClients();
}

void TaskTemperatureHumidity(void *pvParameters)
{
    while (1)
    {
        dht20.read();
        float temperature = dht20.getTemperature();
        float humidity = dht20.getHumidity();
        if (ws.count() > 0)
        {
          String data = "{\"temperature\":" + String(temperature) + ",\"humidity\":" + String(humidity) +  ",\"flame\":\"" + String(digitalRead(sensorPin) == HIGH ? "No fire detected" : "Fire detected") +  "\"}";
          ws.textAll(data);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS); 
    }
}

void TaskSwitchRelay(void *pvParameters)
{
    int relayPins[4] = {1, 2, 3, 4};

    for (int i = 0; i < 4; i++)
    {
        pinMode(relayPins[i], OUTPUT);
    }
    // pinMode(D3, OUTPUT);

    while (1)
    {
        for (int i = 0; i < 4; i++)
        {
            if (buttonStates[i])
            {
                digitalWrite(relayPins[i], HIGH);
            }
            else
            {
                digitalWrite(relayPins[i], LOW);
            }
        }
        delay(100);
    }
}