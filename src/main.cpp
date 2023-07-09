#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <BH1750.h>
#include "config.h"
#include "DHT.h"
#include <InfluxDbClient.h>
#include <WiFi.h>

// Config
#define DEEP_SLEEP_DURATION_US 60LL * 60 * 1000000
#define OPERATION_TIMEOUT_MS 10 * 1000

// Pins
#define POWER_CTRL (4)

#define I2C_SDA (25)
#define I2C_SCL (26)

#define DHT1x_PIN (16)
#define BAT_ADC (33)
#define SALT_PIN (34)
#define SOIL_PIN (32)

// Addresses
#define OB_BH1750_ADDRESS (0x23)

typedef struct
{
  float temperature_c;
  float light_lux;
  float humidity_pct;
  float voltage_mv;
  uint8_t soil_moisture_pct;
  uint8_t salt_pct;
} measurement_t;

DHT airSensor(DHT1x_PIN, DHT11);
BH1750 lightSensor(OB_BH1750_ADDRESS);
InfluxDBClient dbClient(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
Point measurement_point("plant_condition");

uint16_t last_loop_ms;
uint16_t loop_interval_ms = 500;
measurement_t measurement = {0};

void connectToWiFi();
void goToDeepSleep();
bool measure(measurement_t &measurement);
bool measureAirTemperatureAndHumidity(measurement_t &measurement);
bool measureLight(measurement_t &measurement);
bool measureSoilHumidity(measurement_t &measurement);
bool measureSoilSalt(measurement_t &measurement);
bool measureBatteryVoltage(measurement_t &measurement);
void upload(measurement_t &measurement);

void setup()
{
  Serial.begin(115200);

  // Powers the on-board light sensor
  pinMode(POWER_CTRL, OUTPUT);
  digitalWrite(POWER_CTRL, HIGH);
  Wire.begin(I2C_SDA, I2C_SCL);

  airSensor.begin();
  lightSensor.begin();

  connectToWiFi();

  measurement_point.addTag("sensor_name", SENSOR_NAME);
}

void loop()
{
  if (last_loop_ms > 0 && millis() - last_loop_ms < loop_interval_ms)
  {
    return;
  }
  else
  {
    last_loop_ms = millis();
  }

  if (WiFi.isConnected() && measure(measurement))
  {
    upload(measurement);
    goToDeepSleep();
  }
  else if (millis() >= OPERATION_TIMEOUT_MS)
  {
    Serial.println("Timed out waiting for WiFi or a measurement");
    goToDeepSleep();
  }
}

void connectToWiFi()
{
  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);
}

void goToDeepSleep()
{
  Serial.println("Entering deep sleep");
  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION_US);
  esp_deep_sleep_start();
}

bool measure(measurement_t &measurement)
{
  return measureAirTemperatureAndHumidity(measurement) &&
         measureLight(measurement) &&
         measureSoilHumidity(measurement) &&
         measureSoilSalt(measurement) &&
         measureBatteryVoltage(measurement);
}

bool measureAirTemperatureAndHumidity(measurement_t &measurement)
{
  measurement.temperature_c = airSensor.readTemperature();
  measurement.humidity_pct = airSensor.readHumidity();

  return !isnan(measurement.temperature_c) && !isnan(measurement.humidity_pct);
}

bool measureLight(measurement_t &measurement)
{
  measurement.light_lux = lightSensor.readLightLevel();
  return !isnan(measurement.light_lux);
}

bool measureSoilHumidity(measurement_t &measurement)
{
  uint16_t soil = analogRead(SOIL_PIN);
  measurement.soil_moisture_pct = map(soil, 0, 4095, 100, 0);
  return true;
}

bool measureSoilSalt(measurement_t &measurement)
{
  uint8_t samples = 120;
  uint32_t humi = 0;
  uint16_t array[120];
  for (int i = 0; i < samples; i++)
  {
    array[i] = analogRead(SALT_PIN);
    delay(2);
  }
  std::sort(array, array + samples);
  for (int i = 1; i < samples - 1; i++)
  {
    humi += array[i];
  }
  humi /= samples - 2;
  measurement.salt_pct = humi;
  return true;
}

bool measureBatteryVoltage(measurement_t &measurement)
{
  int vref = 1100;
  uint16_t volt = analogRead(BAT_ADC);
  measurement.voltage_mv = ((float)volt / 4095.0) * 6.6 * (vref);
  return true;
}

void upload(measurement_t &measurement)
{
  measurement_point.addField("air_humidity", measurement.humidity_pct);
  measurement_point.addField("air_temperature", measurement.temperature_c);
  measurement_point.addField("soil_moisture", measurement.soil_moisture_pct);
  measurement_point.addField("soil_salt", measurement.salt_pct);
  measurement_point.addField("light", measurement.light_lux);
  measurement_point.addField("voltage", measurement.voltage_mv);
  measurement_point.addField("uptime_ms", millis());

  Serial.println("Uploading measurement...");
  Serial.println(measurement_point.toLineProtocol());
  dbClient.writePoint(measurement_point);
}