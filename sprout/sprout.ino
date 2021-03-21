#include <Wire.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <avr/dtostrf.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_PM25AQI.h>
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

int radioStatus = WL_IDLE_STATUS;
int readingCount = 0;

Adafruit_SGP30 sgp;
Adafruit_BME280 bme;
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
BME280_Data bmeEnv;
SGP30_Data sgpEnv;
PM25_AQI_Data pm25Env;
WiFiUDP udp;

void connectToWiFi(int* wifiStatus, const char* ssid, const char* password) {
  if (WiFi.status() == WL_NO_MODULE) {
    reportError("WiFi module not present");
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

void reportError(const char* message) {
  Serial.println(message);
}

void reportValue(WiFiUDP* udp, const char* message) {
  udp->beginPacket(METRICS_IP, METRICS_PORT);
  udp->write(message);
  udp->endPacket();
}

void readBME280(Adafruit_BME280* bme, BME280_Data* env) {
  env->temperature = bme->readTemperature();
  env->pressure = bme->readPressure();
  env->altitude = bme->readAltitude(SEALEVELPRESSURE_HPA);
  env->humidity = bme->readHumidity();
}

void readSGP30(Adafruit_SGP30* sgp, SGP30_Data* env) {
  if (! sgp->IAQmeasure()) {
    reportError("SGP30 measurement failed");
    return;
  }
  if (sgp->TVOC == 0 && sgp->eCO2 == 400) {
    return;
  }
  env->warm = true;
  env->tvoc = sgp->TVOC;
  env->eco2 = sgp->eCO2;
  if (! sgp->IAQmeasureRaw()) {
    reportError("SGP30 raw measurement failed");
    return;
  }
  env->h2 = sgp->rawH2;
  env->ethanol = sgp->rawEthanol;
}

bool readPM25(Adafruit_PM25AQI* aqi, PM25_AQI_Data* env) {
  if (! aqi->read(env)) {
    reportError("PM 2.5 measurement failed. Trying to reset serial connection...");
    Serial1.begin(9600);
    aqi->begin_UART(&Serial1);
    return false;
  }
  return true;
}

void getSGP30Baseline(Adafruit_SGP30* sgp, uint16_t* eCO2_base, uint16_t* TVOC_base) {
  if (! sgp->getIAQBaseline(eCO2_base, TVOC_base)) {
    reportError("Failed to get SGP30 baseline readings");
  }
}

void setSGP30Baseline(Adafruit_SGP30* sgp, uint16_t eCO2_base, uint16_t TVOC_base) {
  if (! sgp->setIAQBaseline(eCO2_base, TVOC_base)) {
    reportError("Failed to set SGP30 baseline readings");
  }
}

void sendBME280(WiFiUDP* udp, BME280_Data* data) {
  char tempString[8];
  char pressureString[12];
  char humidityString[8];
  dtostrf(data->temperature, 0, 2, tempString);
  dtostrf(data->pressure, 0, 2, pressureString);
  dtostrf(data->humidity, 0, 2, humidityString);
  char packet[100];
  snprintf(packet, 100, "temperature:%s|g\npressure:%s|g\nhumidity:%s|g", tempString, pressureString, humidityString);
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

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  Serial.println("Serial ready!");

  connectToWiFi(&radioStatus, SSID, WIFI_PASS);
  udp.begin(UDP_SRC_PORT);

  if (! sgp.begin()){
    reportError("SGP30 sensor failed to initialize!");
    while (1) delay(1000);
  }
  // Readings taken 2021-03-21
  // In the future, these will be saved to flash storage automatically
  if (! sgp.setIAQBaseline(35496, 36315)) {
    reportError("Failed to set SGP30 baseline readings");
  }

  if (! bme.begin()){
    reportError("BME280 sensor failed to initialize!");
    while (1) delay(1000);
  }

  // This is for the PM2.5 sensor, connected over UART
  Serial1.begin(9600);
  if (! aqi.begin_UART(&Serial1)) {
    Serial.println("PM 2.5 sensor failed to initialize!");
    while (1) delay(1000);
  }
}

void loop() {
  // SGP30 measurements
  readSGP30(&sgp, &sgpEnv);
  if (sgpEnv.warm) {
    sendSGP30(&udp, &sgpEnv);
    /*
    if (readingCount % 60 == 0) {
      uint16_t eCO2_base = 0;
      uint16_t TVOC_base = 0;
      getSGP30Baseline(&sgp, &eCO2_base, &TVOC_base);
      Serial.print("Baseline values: eC02 ");
      Serial.print(eCO2_base);
      Serial.print("; TVOC");
      Serial.println(TVOC_base);
    }
    */
  } else {
    Serial.println("SGP30 is still warming up...");
  }

  // BME280 measurements
  readBME280(&bme, &bmeEnv);
  sendBME280(&udp, &bmeEnv);

  // PM 2.5 measurements
  if (readPM25(&aqi, &pm25Env)) {
    sendPM25(&udp, &pm25Env);
  }
  readingCount++;
  delay(10000);
}
