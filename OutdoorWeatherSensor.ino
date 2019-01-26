/************************************************************************/
/*                                                                      */
/*      Temperatura, wilgotność, ciśnienie BME280 na MQTT 	            */
/*              Hardware: ESP8266 (ESP12E)                              */
/*                                                                      */
/*              Author: Dariusz Wulkiewicz                              */
/*                      d.wulkiewicz@gmail.com                          */
/*                                                                      */
/*              Date: 01.2019                                           */
/************************************************************************/

//Na podstawie
//https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_esp8266/mqtt_esp8266.ino
//http://www.esp8266.com/viewtopic.php?f=29&t=8746#
//https://home-assistant.io/blog/2015/10/11/measure-temperature-with-esp8266-and-report-to-mqtt/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <FS.h>
#include <Wire.h>
#include <math.h>
#include "Constants.h"

String wifiSSID;
String wifiPassword;
String mqttServer;

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme; // I2C

float humidity, temp_c;  // Values read from sensor
// Generally, you should use "unsigned long" for variables that hold time
unsigned long previousMillis = 0;        // will store last temp was read
const long interval = 2000;              // interval at which to read sensor

char msg[50];

// Set Hostname.
String esp2866_hostname(HOSTNAME_PREFIX);
//----------------------------------------------------------------------------------------
bool loadConfig() {
	File file = SPIFFS.open("/config.json", "r");
	if (!file) {
		Serial.println("Failed to open config file");
		return false;
	}

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));
  else
  {
    JsonObject root = doc.as<JsonObject>();  
    wifiSSID = root["ssid"].as<String>();
    wifiPassword = root["password"].as<String>();
    mqttServer = root["mqtt_server"].as<String>();  
  }
  file.close();
 
	Serial.printf("Loaded wifiSSID: %s\r\n", wifiSSID.c_str());
	Serial.printf("Loaded wifiPassword: %s\r\n", wifiPassword.c_str());
	Serial.printf("Loaded mqttServer: %s\r\n", mqttServer.c_str());
	return true;
}
//----------------------------------------------------------------------------------------
void setup_wifi() {
	delay(10);
	// We start by connecting to a WiFi network
	Serial.printf("\r\nConnecting to %s\r\n", wifiSSID.c_str());

	WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());

	// Wait for connection
	while (WiFi.status() != WL_CONNECTED) {
		digitalWrite(BUILT_LED, LED_ON);
		delay(250);
		digitalWrite(BUILT_LED, LED_OFF);
		delay(250);
		Serial.print(".");
	}

	// Set Hostname.
	esp2866_hostname += String(ESP.getChipId(), HEX);
	WiFi.hostname(esp2866_hostname);

	Serial.println("");
	Serial.println("WiFi connected");
	Serial.printf("IP address: %s Hostname: %s\r\n", WiFi.localIP().toString().c_str(), esp2866_hostname.c_str());
}
//----------------------------------------------------------------------------------------
void reconnect() {
	digitalWrite(BUILT_LED, LED_OFF);
	// Loop until we're reconnected
	while (!client.connected()) {
		Serial.println("Attempting MQTT connection...");
		// Attempt to connect
		char buf[20];
		sprintf(buf, "%s_%08X", prefixClientID, ESP.getChipId());
		if (client.connect(&buf[0])) {
			digitalWrite(BUILT_LED, LED_ON);
			Serial.printf("connected as %s\r\n", &buf[0]);

			// ... and resubscribe
			// można subskrybować wiele topiców
			client.subscribe(sensorsBME280CommandTopic);
		}
		else {
			Serial.print("failed, rc=");
			Serial.print(client.state());
			Serial.println(" try again in 5 seconds");

			// Wait 5 seconds before retrying
			delay(5000);
		}
	}
}
//----------------------------------------------------------------------------------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
	// Conver the incoming byte array to a string
	payload[length] = '\0'; // Null terminator used to terminate the char array
	String mqttTopic = topic;
	String mqttMessage = (char*)payload;

	Serial.printf("MQTT received topic:[%s], msg: %s\r\n", topic, mqttMessage.c_str());

	/*BME280*/
	if (mqttTopic.equals(sensorsBME280CommandTopic) && mqttMessage.equals(sensorsBME280CommandTemp)) {		
		dtostrf(round(bmeTempCorection(bme.readTemperature())*10.0) / 10.0, 2, 1/*2*/, msg);
		Serial.printf("MQTT send topic:[%s], msg: %s\r\n", sensorsBME280TemperatureTopic, msg);
		client.publish(sensorsBME280TemperatureTopic, msg);
	}
	else if (mqttTopic.equals(sensorsBME280CommandTopic) && mqttMessage.equals(sensorsBME280CommandPress)) {
		dtostrf(round(bme.readPressure()/10.0)/10.0, 3, 1/*2*/, msg);
		Serial.printf("MQTT send topic:[%s], msg: %s\r\n", sensorsBME280PressureTopic, msg);
		client.publish(sensorsBME280PressureTopic, msg);
	}
	else if (mqttTopic.equals(sensorsBME280CommandTopic) && mqttMessage.equals(sensorsBME280CommandHum)) {
		dtostrf(round(bme.readHumidity()*10.0) / 10.0, 2, 1/*2*/, msg);
		Serial.printf("MQTT send topic:[%s], msg: %s\r\n", sensorsBME280HumidityTopic, msg);
		client.publish(sensorsBME280HumidityTopic, msg);
	}
}
//----------------------------------------------------------------------------------------
float bmeTempCorection(float temp) {
   return temp; // - 1/*korekta*/;
}
//----------------------------------------------------------------------------------------
void setup() {
	pinMode(BUILT_LED, OUTPUT);
	Serial.begin(115200);

	digitalWrite(BUILT_LED, LED_ON); // Turn the LED on
	delay(1000);
	digitalWrite(BUILT_LED, LED_OFF); // Turn the LED off by making the voltage HIGH
  Serial.println("");
    
	Serial.println("****************************");
	Serial.println("* BME280 by MQTT           *");
	Serial.println("****************************");

	while (!SPIFFS.begin()) {
		Serial.println("Failed to mount file system");
    for (uint8_t i=0; i<=5; i++){
      digitalWrite(BUILT_LED, LED_ON); // Turn the LED on
      delay(100);
      digitalWrite(BUILT_LED, LED_OFF); // Turn the LED off by making the voltage HIGH
      delay(100);
    }
		delay(5000);    
	}

	while (!loadConfig()) {
		Serial.println("Failed load config");
    for (uint8_t i=0; i<=5; i++){
      digitalWrite(BUILT_LED, LED_ON); // Turn the LED on
      delay(100);
      digitalWrite(BUILT_LED, LED_OFF); // Turn the LED off by making the voltage HIGH
      delay(100);
    }   
		delay(5000);
	}

    // default settings
	// (you can also pass in a Wire library object like &Wire2)
	bool status = bme.begin(BME280_I2C_ADDRESS);
	if (!status) {
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
	}

	setup_wifi();                   // Connect to wifi   

  client.setServer(mqttServer.c_str(), 1883);
	client.setCallback(mqttCallback);

	// Start OTA server.
	ArduinoOTA.setHostname((const char *)esp2866_hostname.c_str());
	ArduinoOTA.begin();
}
//----------------------------------------------------------------------------------------
void loop() {
	if (!client.connected()) {
		reconnect();
	}

	// Handle OTA server.
	ArduinoOTA.handle();

	client.loop();
}
