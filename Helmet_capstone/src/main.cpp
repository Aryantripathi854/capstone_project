#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <broadcast_lib.cpp>

#define Switch1 5
#define Switch2 17
#define alcohol 16
#define crash 4

int crash_state = 0;

void ESP_Now_Init_cb()
{
  // Initialize ESP-NOW
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESP-NOW Init Success");
    // esp_now_register_recv_cb(receiveCallback);
    esp_now_register_send_cb(sentCallback);
  }
  else
  {
    Serial.println("ESP-NOW Init Failed");
    delay(3000);
    ESP.restart();
  }
}

void WIFI_Init()
{
  // Set ESP32 in STA mode to begin with, Print MAC address, Disconnect from WiFi
  WiFi.mode(WIFI_STA);
  Serial.println("ESP-NOW Broadcast Demo");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  WiFi.disconnect();
}

void setup()
{
  Serial.begin(115200);
  MAC_ADD_Print();
  WIFI_Init();
  ESP_Now_Init_cb();
  
  pinMode(Switch1, INPUT_PULLUP);
  pinMode(Switch2, INPUT_PULLUP);
  pinMode(alcohol, INPUT);
  pinMode(crash, INPUT);
}

void loop()
{
  Serial.print(!digitalRead(Switch1));
  Serial.print("  ");
  Serial.print(!digitalRead(Switch2));
  Serial.print("  ");
  Serial.print(!digitalRead(alcohol));
  Serial.print("  ");
  Serial.print(digitalRead(crash));
  Serial.print("  ");
  Serial.print(crash_state);
  
  if (digitalRead(crash) == 1)
  {
    crash_state = 1; // Crash detected, set state to 1

    // Broadcast crash state immediately
    char buffer[100];
    sprintf(buffer, "Switch1 = %d, Switch2 = %d, Alcohol = %d, Crash = %d", !digitalRead(Switch1), !digitalRead(Switch2), !digitalRead(alcohol), crash_state);
    broadcast(buffer);

    // Wait for 30 seconds (30000 milliseconds)
    delay(3000);

    // After the delay, re-read the crash sensor
    if (digitalRead(crash) == 1)
    {
      crash_state = 1; // Still detecting crash
    }
    else
    {
      crash_state = 0; // Crash condition cleared
    }
  }

  Serial.println("  ");

  // Update broadcast with the current sensor states including updated crash_state
  char buffer[100];
  sprintf(buffer, "Switch1 = %d, Switch2 = %d, Alcohol = %d, Crash = %d", !digitalRead(Switch1), !digitalRead(Switch2), !digitalRead(alcohol), crash_state);
  broadcast(buffer);

  delay(100);
}
