// =========Blynk authontication========
#define BLYNK_AUTH_TOKEN ""   // add your Blynk token here
#define BLYNK_TEMPLATE_ID ""  // add your Blynk template ID here
#define BLYNK_TEMPLATE_NAME ""  // add your Blynk template name here

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <WiFiManager.h> // for runtime WiFi configuration

#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

// IR Receiver
const uint16_t RECV_PIN = D5;
IRrecv irrecv(RECV_PIN);
decode_results results;

// IR codes
uint32_t BUTTON_1 = 0xFF30CF;
uint32_t BUTTON_2 = 0xFF18E7;
uint32_t BUTTON_3 = 0xFF7A85;
uint32_t BUTTON_4 = 0xFF10EF;
uint32_t POWER_BUTTON = 0xFFA25D;

// Relay states
bool relayState[4] = {false, false, false, false};

void checkSerialFromSTM32() 
{
  while (Serial.available()) 
  {
    //Serial.println("I"); // Send command to STM32
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("BTN1")) 
    {
      updateButtonUI(0,!relayState[0]);
    } 
    else if (line.startsWith("BTN2")) 
    {
      updateButtonUI(1,!relayState[1]);
    }
    else if (line.startsWith("BTN3")) 
    {
      updateButtonUI(2,!relayState[2]);
    }
    else if (line.startsWith("BTN4")) 
    {
      updateButtonUI(3,!relayState[3]);
    }
    else if (line.startsWith("PUMP"))  // if water level exceed thresh hold it simply turn it off
    {
      updateButtonUI(2,false);
    }
    else if (line.startsWith("TEMP:")) 
    {
      int temp = line.substring(5).toInt();
      Blynk.virtualWrite(V5, temp);
    } 
    else if (line.startsWith("HUM:")) 
    {
      int hum = line.substring(4).toInt();
      Blynk.virtualWrite(V6, hum);
    } 
    else if (line.startsWith("DIST:")) 
    {
      int dist = line.substring(5).toInt();
      Blynk.virtualWrite(V4, dist);
    } 
  }
}

// on press of button fron stm32 it update UI of Blynk and relay state array
void updateButtonUI(int relayIndex, bool state)
{
  relayState[relayIndex] = state;
  
  int vpinValue;

  switch (relayIndex)
  {
  case 0:
    vpinValue = state ? 1 : 0;
    Blynk.virtualWrite(V0, vpinValue);
    break;
  case 1:
    vpinValue = state ? 11 : 10;
    Blynk.virtualWrite(V1, vpinValue);
    break;
  case 2:
    vpinValue = state ? 21 : 20;
    Blynk.virtualWrite(V2, vpinValue);
    break;
  case 3:
    vpinValue = state ? 31 : 30;
    Blynk.virtualWrite(V3, vpinValue);
    break;
  }
}

// Send relay state to STM32 and update Blynk UI
void updateRelay(int relayIndex, bool state)
{
  relayState[relayIndex] = state;

  String command;
  int vpinValue;

  switch (relayIndex)
  {
  case 0:
    command = state ? "RELAY1_ON" : "RELAY1_OFF";
    vpinValue = state ? 1 : 0;
    Blynk.virtualWrite(V0, vpinValue);
    break;
  case 1:
    command = state ? "RELAY2_ON" : "RELAY2_OFF";
    vpinValue = state ? 11 : 10;
    Blynk.virtualWrite(V1, vpinValue);
    break;
  case 2:
    command = state ? "RELAY3_ON" : "RELAY3_OFF";
    vpinValue = state ? 21 : 20;
    Blynk.virtualWrite(V2, vpinValue);
    break;
  case 3:
    command = state ? "RELAY4_ON" : "RELAY4_OFF";
    vpinValue = state ? 31 : 30;
    Blynk.virtualWrite(V3, vpinValue);
    break;
  }

  Serial.println(command); // Send command to STM32
}

// Blynk button handlers
BLYNK_WRITE(V0)
{
  int state = param.asInt(); // 0 or 1
  updateRelay(0, state == 1);
}

BLYNK_WRITE(V1)
{
  int state = param.asInt(); // 10 or 11
  updateRelay(1, state == 11);
}

BLYNK_WRITE(V2)
{
  int state = param.asInt(); // 20 or 21
  updateRelay(2, state == 21);
}

BLYNK_WRITE(V3)
{
  int state = param.asInt(); // 30 or 31
  updateRelay(3, state == 31);
}

void setup()
{
  Serial.begin(9600);

  // Initialize WiFiManager
  WiFiManager wifiManager;

  // Auto connect or start AP with name "AutoConnectAP"
  wifiManager.autoConnect("AutoConnectAP");

  // Wi-Fi connected, now start Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, WiFi.SSID().c_str(), WiFi.psk().c_str());

  irrecv.enableIRIn(); // Start IR receiver
}

// Handle IR inputs
void checkIR()
{
  if (irrecv.decode(&results))
  {
    uint32_t code = results.value;
    if (code == BUTTON_1)
    {
      //Serial.println("btn1"); // Send command to STM32
      updateRelay(0, !relayState[0]);
    }
    else if (code == BUTTON_2)
    {
      updateRelay(1, !relayState[1]);
    }
    else if (code == BUTTON_3)
    {
      updateRelay(2, !relayState[2]);
    }
    else if (code == BUTTON_4)
    {
      updateRelay(3, !relayState[3]);
    }
    else if (code == POWER_BUTTON)
    {
      bool newState = !relayState[0]; // Toggle all based on relay1
      for (int i = 0; i < 4; i++)
      {
        updateRelay(i, newState);
      }
    }

    irrecv.resume();
  }
}

void loop()
{
  Blynk.run();
  checkIR();
  checkSerialFromSTM32();
}
