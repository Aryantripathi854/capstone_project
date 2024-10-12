#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <broadcast_lib.cpp>
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>  // Use SoftwareSerial for GPS and GSM

#define relayPIN 19
#define buttonPIN 15
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
#define GPS_RXPin 16  // GPS RX pin
#define GPS_TXPin 17  // GPS TX pin
#define GSM_RXPin 4   // GSM RX pin (D4)
#define GSM_TXPin 5   // GSM TX pin (D5)

TinyGPSPlus gps;                   // Create GPS object
SoftwareSerial gpsSerial(GPS_RXPin, GPS_TXPin);  // GPS software serial
SoftwareSerial gsmSerial(GSM_RXPin, GSM_TXPin);  // GSM software serial for GSM

int state[4] = {0, 0, 0, 0};
unsigned long startTime = 0;
bool timerRunning = false;
bool buttonPressed = false;
bool relayForcedOff = false;  // New flag to track relay off after timer
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// Array of recipient phone numbers
const char* phoneNumbers[] = {
  "+919582123595",  // Replace with real phone numbers
  "+917206016422",  // Replace with real phone numbers
  "+919815769312",  // Replace with real phone numbers
  "+918299794940"   // Replace with real phone numbers
};

// Total number of recipients
const int totalRecipients = 4;

// Function prototypes
void getCoordinatesAndGenerateLink();
void sendSMS(String message);

void ESP_Now_Init_cb()
{
  // Initialize ESP-NOW
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESP-NOW Init Success");
    esp_now_register_recv_cb(receiveCallback);
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

void relay_on()
{
  pinMode(relayPIN, OUTPUT); // relay on
  digitalWrite(relayPIN, LOW);
}

void relay_off()
{
  pinMode(relayPIN, INPUT); // relay off
}

void update_state()
{
  state[0] = mes_signal[10] - '0';
  state[1] = mes_signal[23] - '0';
  state[2] = mes_signal[36] - '0';
  state[3] = mes_signal[47] - '0';
}

void debug_mes_signal()
{
  Serial.print("mes_signal  ");
  Serial.print(mes_signal[10]);
  Serial.print(",");
  Serial.print(mes_signal[23]);
  Serial.print(",");
  Serial.print(mes_signal[36]);
  Serial.print(",");
  Serial.println(mes_signal[47]);
}

void debug_state()
{
  Serial.print("state  ");
  Serial.print(state[0]);
  Serial.print(",");
  Serial.print(state[1]);
  Serial.print(",");
  Serial.print(state[2]);
  Serial.print(",");
  Serial.println(state[3]);
}

void startTimer()
{
  if (!timerRunning)
  {
    startTime = millis();
    timerRunning = true;
    buttonPressed = false; // Reset button press state
    relayForcedOff = false; // Reset forced relay off flag
  }
}

void checkTimer()
{
  if (timerRunning)
  {
    unsigned long elapsedTime = millis() - startTime;
    int remainingTime = 30 - elapsedTime / 1000;

    lcd.setCursor(0, 0);
    lcd.print("Timer: ");
    lcd.print(remainingTime);
    lcd.print(" sec");

    // Check if button is pressed
    if (digitalRead(buttonPIN) == LOW)
    {
      buttonPressed = true;
      lcd.setCursor(0, 1);
      lcd.print("Button Pressed");
      timerRunning = false; // Stop the timer
    }

    // If timer reaches 30 seconds and button is not pressed
    if (elapsedTime >= 30000 && !buttonPressed)
    {
      relay_off();           // Turn off relay if button was not pressed within 30 seconds
      timerRunning = false;  // Stop the timer
      relayForcedOff = true; // Mark the relay as forced off
      lcd.clear();
      getCoordinatesAndGenerateLink();  // Call function to get GPS and generate link
    }
  }
}

void getCoordinatesAndGenerateLink()
{
  lcd.setCursor(0, 1);
  lcd.print("Getting GPS...");

  // Wait for GPS data to be available
  while (gpsSerial.available() > 0)
  {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated())
    {
      float latitude = gps.location.lat();
      float longitude = gps.location.lng();

      // Generate Google Maps link
      String googleMapsLink = "https://maps.google.com/?q=" + String(latitude, 6) + "," + String(longitude, 6);

      // Print GPS coordinates and Google Maps link to Serial Monitor
      Serial.println("Coordinates Retrieved:");
      Serial.print("Latitude: ");
      Serial.println(latitude, 6);
      Serial.print("Longitude: ");
      Serial.println(longitude, 6);
      Serial.print("Google Maps Link: ");
      Serial.println(googleMapsLink);

      // Show on LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Lat:");
      lcd.print(latitude, 6);
      lcd.setCursor(0, 1);
      lcd.print("Lon:");
      lcd.print(longitude, 6);

      // Send the Google Maps link via SMS to all recipients
      sendSMS(googleMapsLink);

      break;  // Exit the loop once coordinates are updated
    }
  }
}

void sendSMS(String message)
{
  for (int i = 0; i < totalRecipients; i++)
  {
    gsmSerial.println("AT+CMGF=1");  // Set the GSM module to text mode
    delay(1000);

    gsmSerial.println("AT+CMGS=\"" + String(phoneNumbers[i]) + "\"");  // Send SMS to the mobile number
    delay(1000);

    gsmSerial.println(message);  // Send the Google Maps link
    delay(1000);

    gsmSerial.write(26);  // Send Ctrl+Z to send the SMS
    delay(3000);  // Longer delay to ensure SMS is sent before sending another one

    Serial.println("SMS Sent to: " + String(phoneNumbers[i]));  // Print to Serial Monitor for confirmation
  }
}

void setup()
{
  Serial.begin(115200);
  delay(100);
  MAC_ADD_Print();
  WIFI_Init();
  ESP_Now_Init_cb();
  pinMode(relayPIN, OUTPUT);
  pinMode(buttonPIN, INPUT_PULLUP); // Set up the button pin
  relay_off();

  lcd.init();
  lcd.backlight();
  lcd.clear();

  gpsSerial.begin(9600);  // Start software serial for GPS
  gsmSerial.begin(9600);  // Start software serial for GSM
}

void loop()
{
  update_state();
  debug_state();

  delay(400);

  // Check conditions to start the timer
  if (state[3] && !relayForcedOff)
  {
    startTimer(); // Start the timer when state[3] is high
  }

  // Check if the timer is running and handle button press
  checkTimer();

  // Determine relay action based on state[0], state[1], state[2], and relayForcedOff
  if (!relayForcedOff && state[0] && state[1] && !state[2] && !state[3])
  {
    relay_on();
  }

  // If states don't meet the condition, turn off the relay
  if (!state[0] || !state[1] || state[2] || relayForcedOff)
  {
    relay_off();
  }

  // Continuously parse GPS data
  while (gpsSerial.available() > 0)
  {
    gps.encode(gpsSerial.read());
  }
}