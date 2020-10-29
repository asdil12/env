#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_CCS811.h>
#include <Adafruit_SGP30.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include "env_config.h"

#define SEALEVELPRESSURE_HPA (1013.25)

//#define I2C_SDA 21
//#define I2C_SCL 22


//Wire wire = Wire();
Adafruit_BME280 bme;
//Adafruit_CCS811 ccs;
Adafruit_SGP30 sgp;

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);

uint8_t send_nth_measurement = 0;
uint32_t save_baseline_nth_measurement = 0;

/* return absolute humidity [mg/m^3] with approximation formula
* @param temperature [°C]
* @param humidity [%RH]
*/
uint32_t calc_absolute_humidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

uint16_t eeprom_read_16bit(uint16_t address) {
  uint16_t value = 0;
  value |= (EEPROM.read(address+0) & 0xFF) << 0;
  value |= (EEPROM.read(address+1) & 0xFF) << 8;
}

void eeprom_write_16bit(uint16_t address, uint16_t value) {
  EEPROM.write(address+0, (value >> 0) & 0xFF);
  EEPROM.write(address+1, (value >> 8) & 0xFF);
  EEPROM.commit();
}

void scan_i2c() {
  Serial.println();
  Serial.println("Start I2C scanner ...");
  Serial.print("\r\n");
  byte count = 0;

  EEPROM.begin(4);
  
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
  
  /*status = ccs.begin(0x5B);  
  if (!status) {
    Serial.println("Could not find a valid CCS811 sensor, check wiring!");
    goto retry_setup;
  }*/
  
  status = sgp.begin();  
  if (!status) {
    Serial.println("Could not find a valid SGP30 sensor, check wiring!");
    goto retry_setup;
  }
  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);
  uint16_t eco2_base = eeprom_read_16bit(0);
  uint16_t tvoc_base = eeprom_read_16bit(2);
  if (eco2_base && tvoc_base) {
    Serial.print("Using SGP30 Baseline values to EEPROM: ");
    Serial.print("eCO2: 0x"); Serial.print(eco2_base, HEX);
    Serial.print(" & TVOC: 0x"); Serial.println(tvoc_base, HEX);
    sgp.setIAQBaseline(eco2_base, tvoc_base);
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
  bool got_eco2_or_tvoc = false;
  
  /*if(ccs.available()){
    ccs.setEnvironmentalData(humidity, temperature);
    if(!ccs.readData()){
      eCO2 = ccs.geteCO2();
      TVOC = ccs.getTVOC();
      got_eco2_or_tvoc = true;
    }
  }*/

  sgp.setHumidity(calc_absolute_humidity(temperature, humidity));
  if (sgp.IAQmeasure()) {
    eCO2 = sgp.eCO2;
    TVOC = sgp.TVOC;
    got_eco2_or_tvoc = true;
    if (++save_baseline_nth_measurement == SAVE_BASELINE_EVERY_NTH_MEASUREMENT) {
      save_baseline_nth_measurement = 0;
      uint16_t eco2_base;
      uint16_t tvoc_base;
      sgp.getIAQBaseline(&eco2_base, &tvoc_base);
      Serial.print("Writing SGP30 Baseline values to EEPROM: ");
      Serial.print("eCO2: 0x"); Serial.print(eco2_base, HEX);
      Serial.print(" & TVOC: 0x"); Serial.println(tvoc_base, HEX);
      eeprom_write_16bit(0, eco2_base);
      eeprom_write_16bit(2, tvoc_base);
    }
  }

  if (got_eco2_or_tvoc) {
    Serial.print("TVOC "); Serial.print(TVOC); Serial.print(" ppb\t");
    Serial.print("eCO2 "); Serial.print(eCO2); Serial.println(" ppm");
  }

  if (++send_nth_measurement == SEND_EVERY_NTH_MEASUREMENT) {
    send_nth_measurement = 0;

    mqtt_connect();
    
    String json = "{";
    json += "\"temperature\":" + String(temperature) + ",";
    json += "\"pressure\":" + String(pressure) + ",";
    json += "\"humidity\":" + String(humidity) + ",";
    json += "\"eCO2\":" + String(eCO2) + ",";
    json += "\"TVOC\":" + String(TVOC);
    json += "}";
    Serial.print("\n");
    Serial.println(json);
  
    if (mqtt_client.publish(MQTT_TOPIC, json.c_str()) == true) {
      Serial.println("Success sending message\n");
    } else {
      Serial.println("Error sending message\n");
    }
  }

  Serial.println("\n");
  delay(MEASUREMENT_INTERVAL);
}
