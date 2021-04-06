#include <Wire.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <avr/dtostrf.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_PM25AQI.h>
#include <Adafruit_seesaw.h>
// This is a custom library that exports `SSID` and `WIFI_PASS` variables
// Do something similar (to avoid checking in credentials) or just define
// `SSID` and `WIFI_PASS` locally.
#include <wificreds.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define UDP_SRC_PORT (1111)

struct BME280_Data {
  float temperature;
  float pressure;
  float altitude;
  float humidity;
};

struct SGP30_Data {
  bool warm = false;
  uint16_t tvoc;
  uint16_t eco2;
  uint16_t h2;
  uint16_t ethanol;
};

struct SoilSensor_Data {
  float temperature;
  uint16_t capacitance;
};

int radioStatus = WL_IDLE_STATUS;
int measurementCount = 0;

Adafruit_SGP30 sgp;
Adafruit_BME280 bme;
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
Adafruit_seesaw soilSensor;
BME280_Data bmeEnv;
SGP30_Data sgpEnv;
PM25_AQI_Data pm25Env;
SoilSensor_Data soilEnv;

WiFiUDP Udp;

float celsiusToFahrenheit(float celsius) {
  return (celsius * 9.0 / 5.0) + 32.0;
}

void connectToWiFi(int* wifiStatus, const char* ssid, const char* password) {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi module not present");
    while(true);
  } else {
    Serial.println("WiFi detected.");
  }
  String fv = WiFi.firmwareVersion();
  Serial.print("WiFi firmware version is: ");
  Serial.println(fv);

  while (*wifiStatus != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    *wifiStatus = WiFi.begin(ssid, password);

    // wait 7 seconds for connection:
    delay(7000);
  }

  Serial.print("Connected to network ");
  Serial.println(WiFi.SSID());
  Serial.print("IP address is ");
  Serial.println(WiFi.localIP());
}

void reportError(WiFiUDP* udp, const char* message) {
  logMessage(udp, message);
  Serial.println(message);
}

void reportValue(WiFiUDP* udp, const char* message) {
  udp->beginPacket(METRICS_IP, METRICS_PORT);
  udp->write(message);
  udp->endPacket();
}

void logMessage(WiFiUDP* udp, const char* message) {
  udp->beginPacket(METRICS_IP, LOGS_PORT);
  udp->write(message);
  udp->endPacket();
}

void readBME280(Adafruit_BME280* bme, BME280_Data* env) {
  env->temperature = bme->readTemperature();
  env->pressure = bme->readPressure();
  env->altitude = bme->readAltitude(SEALEVELPRESSURE_HPA);
  env->humidity = bme->readHumidity();
}

void readSGP30(Adafruit_SGP30* sgp, SGP30_Data* env, WiFiUDP* udp) {
  if (! sgp->IAQmeasure()) {
    reportError(udp, "SGP30 measurement failed");
    reportValue(udp, "sgp30_failed:1|c");
    return;
  }
  if (sgp->TVOC == 0 && sgp->eCO2 == 400) {
    return;
  }
  env->warm = true;
  env->tvoc = sgp->TVOC;
  env->eco2 = sgp->eCO2;
  if (! sgp->IAQmeasureRaw()) {
    reportError(udp, "SGP30 raw measurement failed");
    reportValue(udp, "sgp30_raw_failed:1|c");
    return;
  }
  env->h2 = sgp->rawH2;
  env->ethanol = sgp->rawEthanol;
}

bool readPM25(Adafruit_PM25AQI* aqi, PM25_AQI_Data* env, WiFiUDP* udp) {
  if (! aqi->read(env)) {
    reportError(udp, "PM 2.5 measurement failed. Trying to reset serial connection...");
    reportValue(udp, "pm25_failed:1|c");
    Serial1.begin(9600);
    aqi->begin_UART(&Serial1);
    return false;
  }

  // Check the returned values and make sure they seem reasonable
  // Sometimes the sensor returns momentary spikes of extremely high values
  if (env->pm10_standard > 3000 && env->pm25_standard > 1500 && env->pm100_standard > 1500) {
    reportValue(&Udp, "pm25_extreme_reading:1|c");
    return false;
  }
  return true;
}

void readSoilSensor(Adafruit_seesaw* ss, SoilSensor_Data* env) {
  env->temperature = ss->getTemp();
  env->capacitance = ss->touchRead(0);
}

void getSGP30Baseline(WiFiUDP* udp, Adafruit_SGP30* sgp, uint16_t* eCO2_base, uint16_t* TVOC_base) {
  if (! sgp->getIAQBaseline(eCO2_base, TVOC_base)) {
    reportError(udp, "Failed to get SGP30 baseline readings");
  }
}

void sendBME280(WiFiUDP* udp, BME280_Data* data) {
  char tempString[8];
  char tempFString[8];
  char pressureString[12];
  char humidityString[8];
  float temperatureF = celsiusToFahrenheit(data->temperature);
  dtostrf(data->temperature, 0, 2, tempString);
  dtostrf(temperatureF, 0, 2, tempFString);
  dtostrf(data->pressure, 0, 2, pressureString);
  dtostrf(data->humidity, 0, 2, humidityString);
  char packet[100];
  snprintf(packet, 100, "temperature_c:%s|g\ntemperature_f:%s|g\npressure:%s|g\nhumidity:%s|g", tempString, tempFString, pressureString, humidityString);
  reportValue(udp, packet);
}

void sendSGP30(WiFiUDP* udp, SGP30_Data* data) {
  char packet[100];
  snprintf(packet, 100, "TVOC:%u|g\neCO2:%u|g\nh2:%u|g\nethanol:%u|g", data->tvoc, data->eco2, data->h2, data->ethanol);
  reportValue(udp, packet);
}

void sendPM25(WiFiUDP* udp, PM25_AQI_Data* data) {
  char standardPacket[100];
  snprintf(standardPacket, 100, "st_pm10:%u|g\nst_pm25:%u|g\nst_pm100:%u|g", data->pm10_standard, data->pm25_standard, data->pm100_standard);
  reportValue(udp, standardPacket);
  char envPacket[100];
  snprintf(envPacket, 100, "env_pm10:%u|g\nenv_pm25:%u|g\nenv_pm100:%u|g", data->pm10_env, data->pm25_env, data->pm100_env);
  reportValue(udp, envPacket);
  char particlesPacket[100];
  snprintf(particlesPacket, 100, "particles_03um:%u|g\nparticles_05um:%u|g\nparticles_10um:%u|g", data->particles_03um, data->particles_05um, data->particles_10um);
  reportValue(udp, particlesPacket);
  char particlesPacket2[100];
  snprintf(particlesPacket2, 100, "particles_25um:%u|g\nparticles_50um:%u|g\nparticles_100um:%u|g", data->particles_25um, data->particles_50um, data->particles_100um);
  reportValue(udp, particlesPacket2);
}

void sendSoilSensor(WiFiUDP* udp, SoilSensor_Data* data) {
  char packet[100];
  float temperatureF = celsiusToFahrenheit(data->temperature);
  char tempString[8];
  char tempFString[8];
  dtostrf(data->temperature, 0, 2, tempString);
  dtostrf(temperatureF, 0, 2, tempFString);
  snprintf(packet, 100, "temperature_soil_c:%s|g\ntemperature_soil_f:%s|g\nsoil_capacitance:%u|g", tempString, tempFString, data->capacitance);
  reportValue(udp, packet);
}

void setup() {
  Serial.begin(9600);
  delay(100);
  if (Serial) {
    Serial.println("Serial ready!");
  }

  connectToWiFi(&radioStatus, SSID, WIFI_PASS);
  Udp.begin(UDP_SRC_PORT);

  if (! sgp.begin()){
    reportError(&Udp, "SGP30 sensor failed to initialize!");
    while (1) delay(1000);
  }
  // Readings taken 2021-04-03
  // In the future, these will be saved to flash storage automatically
  if (! sgp.setIAQBaseline(35675, 37324)) {
    reportError(&Udp, "Failed to set SGP30 baseline readings");
  }

  if (! bme.begin()){
    reportError(&Udp, "BME280 sensor failed to initialize!");
    while (1) delay(1000);
  }

  // This is for the PM2.5 sensor, connected over UART
  Serial1.begin(9600);
  if (! aqi.begin_UART(&Serial1)) {
    reportError(&Udp, "PM 2.5 sensor failed to initialize!");
    while (1) delay(1000);
  }

  if (!soilSensor.begin(0x36)) {
    reportError(&Udp, "Soil sensor failed to initialize!");
    while (1) delay(1000);
  }
}

void loop() {
  int wifiStatus = WiFi.status();
  if (wifiStatus != WL_CONNECTED) {
    connectToWiFi(&radioStatus, SSID, WIFI_PASS);
  }

  // SGP30 measurements
  readSGP30(&sgp, &sgpEnv, &Udp);
  if (sgpEnv.warm) {
    sendSGP30(&Udp, &sgpEnv);
  } else {
    reportValue(&Udp, "sgp30_warming:1|c");
    Serial.println("SGP30 is still warming up...");
  }

  // BME280 measurements
  readBME280(&bme, &bmeEnv);
  sendBME280(&Udp, &bmeEnv);

  // PM 2.5 measurements
  if (readPM25(&aqi, &pm25Env, &Udp)) {
    sendPM25(&Udp, &pm25Env);
  }

  // Soil measurements
  readSoilSensor(&soilSensor, &soilEnv);
  sendSoilSensor(&Udp, &soilEnv);

  // Report baseline values every 2 hours
  if (measurementCount == 240) {
    uint16_t eCO2_base = 0;
    uint16_t TVOC_base = 0;
    getSGP30Baseline(&Udp, &sgp, &eCO2_base, &TVOC_base);
    char baselineReport[100];
    snprintf(baselineReport, 100, "eCO2 Base: %u, TVOC Base: %u\n", eCO2_base, TVOC_base);
    logMessage(&Udp, baselineReport);
    // Reset counter
    measurementCount = 0;
  }

  measurementCount++;
  // Report values every 30 seconds
  delay(30000);
}
