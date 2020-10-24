#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_CCS811.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "env_config.h"

#define SEALEVELPRESSURE_HPA (1013.25)

//#define I2C_SDA 21
//#define I2C_SCL 22


//Wire wire = Wire();
Adafruit_BME280 bme;
Adafruit_CCS811 ccs;

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);


void scan_i2c() {
  Serial.println();
  Serial.println("Start I2C scanner ...");
  Serial.print("\r\n");
  byte count = 0;
  
  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0)
      {
      Serial.print("Found I2C Device: ");
      Serial.print(" (0x");
      Serial.print(i, HEX);
      Serial.println(")");
      count++;
      delay(1);
      }
  }
  Serial.print("\r\n");
  Serial.println("Finish I2C scanner");
  Serial.print("Found ");
  Serial.print(count, HEX);
  Serial.println(" Device(s).");
}

void mqtt_connect() {
  while (!mqtt_client.connected()) {
    Serial.println("Connecting to MQTT...");
    mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
    if (mqtt_client.connect("ESP32Client", MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(mqtt_client.state());
      delay(2000);
    }
  }  
}

void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  delay(5000);
  
  Serial.begin(115200);
  while(!Serial){}

  retry_setup:
  scan_i2c();

  bool status;
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    goto retry_setup;
  }
  status = ccs.begin(0x5B);  
  if (!status) {
    Serial.println("Could not find a valid CCS811 sensor, check wiring!");
    goto retry_setup;
  }

  WiFi.begin(WIFI_SSID, WIFI_PSK);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());

  mqtt_connect();

  digitalWrite(2, LOW);
}

void loop() {
  Serial.print("Temperature = ");
  double temperature = bme.readTemperature();
  Serial.print(temperature);
  Serial.println(" *C");
  
  Serial.print("Pressure = ");
  float pressure = bme.readPressure() / 100.0F;
  Serial.print(pressure);
  Serial.println(" hPa");

  //Serial.print("Approx. Altitude = ");
  //Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  //Serial.println(" m");

  Serial.print("Humidity = ");
  float humidity = bme.readHumidity();
  Serial.print(humidity);
  Serial.println(" %");

  Serial.println();

  uint16_t eCO2 = 400;
  uint16_t TVOC = 0;
  if(ccs.available()){
    ccs.setEnvironmentalData(humidity, temperature);
    if(!ccs.readData()){
      eCO2 = ccs.geteCO2();
      Serial.print("eCO2: ");
      Serial.print(eCO2);
      Serial.print("ppm, TVOC: ");
      TVOC = ccs.getTVOC();
      Serial.println(TVOC);
      Serial.println();
    }
  }

  String json = "{";
  json += "\"temperature\":" + String(temperature) + ",";
  json += "\"pressure\":" + String(pressure) + ",";
  json += "\"humidity\":" + String(humidity) + ",";
  json += "\"eCO2\":" + String(eCO2) + ",";
  json += "\"TVOC\":" + String(TVOC);
  json += "}";

  Serial.println(json);

  mqtt_connect();

  if (mqtt_client.publish(MQTT_TOPIC, json.c_str()) == true) {
    Serial.println("Success sending message");
  } else {
    Serial.println("Error sending message");
  }

  delay(MEASUREMENT_INTERVAL);
}
