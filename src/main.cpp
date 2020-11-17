/*
Proofer
Proofing cabinet control software
*/

#include <ESP8266WiFi.h>       //ESP8266 Core WiFi Library
#include <DNSServer.h>         //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>  //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>       //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <PubSubClient.h>      //https://github.com/knolleary/pubsubclient/ ESP8266 WiFi Connection manager with fallback web configuration portal
#include <OneWire.h>           //https://github.com/PaulStoffregen/OneWire Control 1-Wire protocol (DS18S20, DS18B20, DS2408 and etc)
#include <DallasTemperature.h> //https://github.com/milesburton/Arduino-Temperature-Control-Library Arduino Library for Maxim Temperature Integrated Circuits
#include <FS.h>                //Arduino file system library
#include <ArduinoJson.h>       //https://github.com/bblanchon/ArduinoJson ArduinoJson is a C++ JSON library for Arduino and IoT (Internet Of Things).

// <!-- Update these variables with values suitable for your setup.
IPAddress mqttBroker(192, 168, 95, 11);                  // IP-Address of your MQTT-Broker
char *topicPrefix = "myHome/basement/hobbyRoom/proofer"; // Your MQTT-Topic prefix
const int oneWireBus = 2;                                // Number of the digital input the thermometer is connected to
const uint8_t relayPin = 0;                              // Number of the digital output the relay is connect to
// -->

float actualTemperature = 0;

WiFiClient wifiClient;
PubSubClient pubSubClient(wifiClient);

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// Delays
const long reconnectDelay = 5000;
unsigned long reconnectDelayStarted;
const long actualTemperatureDelay = 5000;
unsigned long actualTemperatureDelayStarted;

// Topics
char *topicActualTemperature = "/actualTemperature";
char *topicSetpointTemperature = "/setpointTemperature";
char *topicSwitchingDeviation = "/switchingDeviation";

// Configuration
struct Config
{
  float setpointTemperature;
  float switchingDeviation;
};

const char *filename = "/config.json";
Config config;

char *scat(char *s, char *t)
{
  int sLength = strlen(s);
  int tLength = strlen(t);
  char *p = new char[sLength + tLength + 1];
  int m = 0, n = 0;

  while (s[n] != '\0')
  {
    p[m++] = s[n++];
  }
  n = 0;
  while (t[n] != '\0')
  {
    p[m++] = t[n++];
  }
  p[m++] = '\0';
  return p;
}

float btof(byte *s, unsigned int t)
{
  // Converting byte payload to string
  char p[t];
  for (int i = 0; i < t; i++)
  {
    p[i] = (char)s[i];
  }

  // Parsing float value from string
  return strtof(p, NULL);
}

void loadConfiguration(const char *filename, Config &config)
{
  File _file = SPIFFS.open(filename, "r");
  StaticJsonDocument<512> _doc;
  DeserializationError _error = deserializeJson(_doc, _file);

  config.setpointTemperature = _doc["setpointTemperature"] | 0;
  config.switchingDeviation = _doc["switchingDeviation"] | 1;

  _file.close();
}

void saveConfiguration(const char *filename, const Config &config)
{
  SPIFFS.remove(filename);

  File _file = SPIFFS.open(filename, "w");
  if (!_file)
  {

    return;
  }

  StaticJsonDocument<512> _doc;
  _doc["setpointTemperature"] = config.setpointTemperature;
  _doc["switchingDeviation"] = config.switchingDeviation;

  serializeJson(_doc, _file);

  _file.close();
}

void publish(char *topic, float value)
{
  char msg[32];

  // Converting float value to string
  dtostrf(value, 6, 2, msg);

  // Building MQTT-Topic
  char *fullTopic = scat(topicPrefix, topic);

  // Publishing
  pubSubClient.publish(fullTopic, msg);
}

void handleMQTTInit()
{
  if (pubSubClient.connected())
  {
    publish(topicSetpointTemperature, config.setpointTemperature);
    publish(topicSwitchingDeviation, config.switchingDeviation);
  }
}

void reconnect(unsigned long currentMillis)
{
  // Loop until we're reconnected
  while (!pubSubClient.connected())
  {

    // Attempt to connect
    if (pubSubClient.connect("arduinoClient"))
    {
      // Once connected, publish an announcement...
      pubSubClient.publish("outTopic", "hello world");
      // ... and resubscribe
      char *fullTopic = scat(topicPrefix, "/#");
      pubSubClient.subscribe(fullTopic);
    }
    else
    {
      // Reset reconnect delay
      reconnectDelayStarted = currentMillis;
    }
  }
}

void handleActualTemperatureChanged(unsigned long currentMillis)
{
  if (pubSubClient.connected() && currentMillis > actualTemperatureDelayStarted + actualTemperatureDelay)
  {
    // Getting actual temperature
    sensors.requestTemperatures();
    float value = sensors.getTempCByIndex(0);

    // Check if temperature has changed
    if (value != actualTemperature)
    {
      // Setting new actual temperature
      actualTemperature = value;

      // Publish
      publish(topicActualTemperature, actualTemperature);
    }

    // Reset reconnect delay
    actualTemperatureDelayStarted = currentMillis;
  }
}

void handleSetpointTemperatureChanged(byte *payload, unsigned int length)
{
  /* Parsing float value from byte
       and setting new setpoint temperature */
  config.setpointTemperature = btof(payload, length);

  // Saving new setpoint temperature to flash
  saveConfiguration(filename, config);
}

void handleSwitchingDeviationChanged(byte *payload, unsigned int length)
{
  /* Parsing float value from byte
       and setting new  switching deviation */
  config.switchingDeviation = btof(payload, length);

  // Saving new switching deviation to flash
  saveConfiguration(filename, config);
}

void callback(char *topic, byte *payload, unsigned int length)
{
  if (strcmp(topic, scat(topicPrefix, topicSetpointTemperature)) == 0)
  {
    handleSetpointTemperatureChanged(payload, length);
  }
  if (strcmp(topic, scat(topicPrefix, topicSwitchingDeviation)) == 0)
  {
    handleSwitchingDeviationChanged(payload, length);
  }
}

void setup()
{
  Serial.begin(115200);

  /* Initializing file system 
       and load configuration from flash */
  SPIFFS.begin();
  loadConfiguration(filename, config);

  // Initializing the MQTT publish/subscribe library
  pubSubClient.setServer(mqttBroker, 1883);
  pubSubClient.setCallback(callback);

  // Initializing WiFi and the WiFi-Manager
  WiFiManager wifiManager;
  wifiManager.autoConnect();

  // Initializing the thermometer
  sensors.begin();

  // Setting up the relay pin
  pinMode(relayPin, OUTPUT);

  // Allow the hardware to sort itself out
  delay(1500);
}

void loop()
{
  unsigned long currentMillis = millis();

  // If MQTT is not connected, try to reconnect
  if (!pubSubClient.connected() && currentMillis > reconnectDelayStarted + reconnectDelay)
  {
    reconnect(currentMillis);
    // Publish initial state
    handleMQTTInit();
  }

  if (pubSubClient.connected())
  {
    // Handle MQTT subscriptions
    pubSubClient.loop();

    /* Check if the actual temperature has changed 
           and publish it if so. */
    handleActualTemperatureChanged(currentMillis);
  }

  // Switching the relay based on temperature
  if (actualTemperature <= config.setpointTemperature - config.switchingDeviation)
  {
    digitalWrite(relayPin, HIGH);
  }
  if (actualTemperature >= config.setpointTemperature + config.switchingDeviation)
  {
    digitalWrite(relayPin, LOW);
  }
}