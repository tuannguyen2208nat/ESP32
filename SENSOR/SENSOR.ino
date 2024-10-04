#include <Wire.h>
#include <DHT20.h>
#include "colors.h"
#include <TinyGPS++.h>
#include <Ultrasonic.h>
#include <ESP32Servo.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <LiquidCrystal_I2C.h>

#define FULL_ANGLE 300
#define ADC_REF 3.3
#define GROVE_VCC 3.3
int sensorPin = D9;

TinyGPSPlus gps;
SoftwareSerial ss(6, 7); // TX,RX
HardwareSerial RS485Serial(1);

void TaskFS(void *pvParameters); // Flame sensor
void TaskBM(void *pvParameters); // Buzzer module
void TaskAS(void *pvParameters); // Audio sensor
void TaskLCD(void *pvParameters);
void TaskGPS(void *pvParameters);
void TaskMFM(void *pvParameters);  // Mini fan motor
void TaskSMS(void *pvParameters);  // Soil moisture
void TaskASM(void *pvParameters);  // Angle sensor module
void TaskPIR(void *pvParameters);  // Passive infrared sensor
void TaskUSM(void *pvParameters);  // Ultrasonic sensor module
void TaskIOS(void *pvParameters);  // Infrared obstacle sensor
void Task2CSM(void *pvParameters); // 2-Channel switching module
void TaskServo(void *pvParameters);
void TaskRelay(void *pvParameters);
void TaskLight(void *pvParameters);
void TaskLedRGB(void *pvParameters);
void TaskTemperatureHumidity(void *pvParameters);
void TaskTemperatureHumidity_ES35(void *pvParameters);

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  ss.begin(9600);

  xTaskCreate(TaskFS, "TaskFS", 2048, NULL, 2, NULL);
  xTaskCreate(TaskBM, "TaskBM", 2048, NULL, 2, NULL);
  xTaskCreate(TaskAS, "TaskAS", 2048, NULL, 2, NULL);
  xTaskCreate(TaskLCD, "TaskLCD", 2048, NULL, 2, NULL);
  xTaskCreate(TaskGPS, "TaskGPS", 2048, NULL, 2, NULL);
  xTaskCreate(TaskMFM, "TaskMFM", 2048, NULL, 2, NULL);
  xTaskCreate(TaskSMS, "TaskSMS", 2048, NULL, 2, NULL);
  xTaskCreate(TaskASM, "TaskASM", 2048, NULL, 2, NULL);
  xTaskCreate(TaskPIR, "TaskPir", 2048, NULL, 2, NULL);
  xTaskCreate(TaskUSM, "TaskUSM", 2048, NULL, 2, NULL);
  xTaskCreate(TaskIOS, "TaskIOS", 2048, NULL, 2, NULL);
  xTaskCreate(Task2CSM, "Task2CSM", 2048, NULL, 2, NULL);
  xTaskCreate(TaskServo, "TaskServo", 2048, NULL, 2, NULL);
  xTaskCreate(TaskRelay, "TaskRelay", 2048, NULL, 2, NULL);
  xTaskCreate(TaskLight, "TaskLight", 2048, NULL, 2, NULL);
  xTaskCreate(TaskLedRGB, "TaskLedRGB", 2048, NULL, 2, NULL);
  xTaskCreate(TaskTemperatureHumidity, "TaskTemperatureHumidity", 2048, NULL, 2, NULL);
   xTaskCreate(TaskTemperatureHumidity_ES35, "TaskTemperatureHumidity_ES35", 2048, NULL, 2, NULL);

  Serial.println("Start");
}

void loop()
{
  Serial.println();
  while (ss.available())
    gps.encode(ss.read());
}

void TaskFS(void *pvParameters)
{
  pinMode(sensorPin, INPUT);
  while (1)
  {
    Serial.println("Flame sensor : " + String(digitalRead(sensorPin) == HIGH ? "No fire detected" : "Fire detected"));
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void TaskBM(void *pvParameters)
{
  pinMode(sensorPin, OUTPUT);
  bool check = true;

  while (1)
  {
    digitalWrite(sensorPin, check ? HIGH : LOW);
    Serial.println("Buzzer module : " + String(check ? "ON" : "OFF"));
    check = !check;
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void TaskAS(void *pvParameters)
{
  pinMode(sensorPin, INPUT);
  while (1)
  {
    Serial.println("Audio sensor : " + String(digitalRead(sensorPin) == HIGH ? "No sound detected" : "Sound detected"));
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void TaskLCD(void *pvParameters)
{
  LiquidCrystal_I2C lcd(33, 16, 2);
  lcd.begin();

  while (1)
  {
    lcd.setCursor((16 - 5) / 2, 0);
    lcd.print("Hello");
    lcd.setCursor(0, 1);
    lcd.print("My name is Tuan");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    lcd.clear();
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void TaskGPS(void *pvParameters)
{

  while (1)
  {
    float X = gps.location.lat();
    float Y = gps.location.lng();
    String xStr = String(X, 7);
    String yStr = String(Y, 7);

    if (gps.location.isValid())
    {
      Serial.print("X: ");
      Serial.print(xStr);
      Serial.print(" Y: ");
      Serial.println(yStr);
    }
    else
    {
      Serial.println("***");
    }

    if (millis() > 5000 && gps.charsProcessed() < 10)
      Serial.println(F("No GPS data received: check wiring"));

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void TaskMFM(void *pvParameters)
{
  pinMode(sensorPin, OUTPUT);
  bool check = true;

  while (1)
  {
    digitalWrite(sensorPin, check ? HIGH : LOW);
    Serial.println("MFM : " + String(check ? "ON" : "OFF"));
    check = !check;
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void TaskSMS(void *pvParameters)
{
  while (1)
  {
    Serial.println("Soil moisture : " + String(analogRead(sensorPin)) + " %");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void TaskASM(void *pvParameters)
{
  pinMode(sensorPin, INPUT);
  int led = 1;

  while (1)
  {
    float voltage;
    int sensor_value = analogRead(sensorPin);                 // Read the value from the sensor
    voltage = (float)sensor_value * ADC_REF / 4095;           // Convert the analog value to voltage
    float degrees = (voltage * FULL_ANGLE) / GROVE_VCC;       // Calculate the rotation angle based on the read voltage value
    int brightness = map(degrees, 0, FULL_ANGLE, 0, 1023);    // Convert the rotation angle value to LED brightness value
    float brightnessPercentage = (brightness / 1023.0) * 100; // Convert brightness to percentage
    Serial.println("LED brightness: " + String(brightnessPercentage) + "%");
    analogWrite(led, brightness);          // Control the LED brightness via PWM connection
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay for 5 seconds
  }
}

void TaskPIR(void *pvParameters)
{
  pinMode(sensorPin, INPUT);

  while (1)
  {
    Serial.println("Infrared obstacle sensor : " + String(digitalRead(sensorPin) == LOW ? "No object detected" : "Object detected"));
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void TaskUSM(void *pvParameters)
{
  int sensorPin_Left = D9;
  int sensorPin_Right = D10;
  Ultrasonic ultrasonic(sensorPin_Left, sensorPin_Right);

  while (1)
  {
    Serial.println("Distance : " + String(ultrasonic.read()) + " cm");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void TaskIOS(void *pvParameters)
{
  pinMode(sensorPin, INPUT);

  while (1)
  {

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void Task2CSM(void *pvParameters)
{
  pinMode(sensorPin, OUTPUT);
  bool check = true;

  while (1)
  {
    digitalWrite(sensorPin, check ? HIGH : LOW);
    Serial.println("2CSM : " + String(check ? "ON" : "OFF"));
    check = !check;
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void TaskServo(void *pvParameters)
{
  int pos = 0;
  Servo myservo;
  myservo.attach(sensorPin);
  myservo.write(90);
  bool check = true;

  while (1)
  {
    if (check)
    {
      for (pos = 0; pos <= 180; pos++)
      {
        myservo.write(pos);
      }
    }
    else
    {
      for (pos = 180; pos >= 0; pos--)
      {
        myservo.write(pos);
      }
    }
    check = !check;
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void TaskRelay(void *pvParameters)
{
  pinMode(sensorPin, OUTPUT);
  bool check = true;

  while (1)
  {
    digitalWrite(sensorPin, check ? HIGH : LOW);
    Serial.println("Relay : " + String(check ? "ON" : "OFF"));
    check = !check;
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void TaskLight(void *pvParameters)
{
  while (1)
  {
    Serial.println("Light : " + String(analogRead(sensorPin)) + " lux");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void TaskLedRGB(void *pvParameters)
{
  Adafruit_NeoPixel rgb(4, sensorPin, NEO_GRB + NEO_KHZ800);
  uint32_t colors[] = {RED, ORANGE, YELLOW, GREEN, BLUE, INDIGO, PURPLE, WHITE, BLACK, OFF};
  const char *colorNames[] = {"RED", "ORANGE", "YELLOW", "GREEN", "BLUE", "INDIGO", "PURPLE", "WHITE", "BLACK", "OFF"};
  int numColors = sizeof(colors) / sizeof(colors[0]);
  rgb.begin();

  while (1)
  {
    for (int i = 0; i < numColors; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        rgb.setPixelColor(j, colors[i]);
      }
      rgb.show();
      Serial.println("Led RGB : " + String(colorNames[i]));
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }
}

void TaskTemperatureHumidity(void *pvParameters)
{
  DHT20 dht20;
  dht20.begin();

  while (1)
  {
    dht20.read();
    Serial.println("Temperature : " + String(dht20.getTemperature()) + " - " + "Humidity : " + String(dht20.getHumidity()));
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void sendRS485Command(byte *command, int commandSize, byte *response, int responseSize)
{
    RS485Serial.write(command, commandSize);
    RS485Serial.flush();
    delay(100); 
    if (RS485Serial.available() >= responseSize)
    {
        RS485Serial.readBytes(response, responseSize); 
    }
}

void ES35_sensor()
{
  byte temperatureRequest[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
  byte humidityRequest[] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xD5, 0xCA};
  byte response[7];
    sendRS485Command(temperatureRequest, sizeof(temperatureRequest), response, sizeof(response));
    if (response[1] == 0x03)  
    {
        int temperature = (response[3] << 8) | response[4]; 
        float temperatureVal = temperature / 10.0; 
        Serial.print("Temperature: ");
        Serial.println(temperatureVal, 2);
    }
    delay(100);

    memset(response, 0, sizeof(response)); 

    sendRS485Command(humidityRequest, sizeof(humidityRequest), response, sizeof(response));
    if (response[1] == 0x03) 
    {
        int humidity = (response[3] << 8) | response[4]; 
        float humidityVal = humidity / 10.0; 
        Serial.print("Humidity: ");
        Serial.println(humidityVal, 2);
    }
}

void TaskTemperatureHumidity_ES35(void *pvParameters)
{
  RS485Serial.begin(9600, SERIAL_8N1, 18, 17); 
  while (1)
  {
    ES35_sensor();
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}
