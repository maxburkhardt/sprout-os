#include <Wire.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <avr/dtostrf.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SGP30.h>
// This is a custom library that exports `ssid` and `pass` variables
// Do something similar (to avoid checking in credentials) or just define
// `ssid` and `pass` locally.
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
BME280_Data bmeEnv;
SGP30_Data sgpEnv;
WiFiUDP udp;

void connectToWiFi(int* wifiStatus, const char* ssid, const char* password) {
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
  Serial.print("Sending packet: ");
  Serial.println(message);
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

  if (WiFi.status() == WL_NO_MODULE) {
    reportError("WiFi module not present");
    while(true);
  } else {
    Serial.println("WiFi detected.");
  }

  String fv = WiFi.firmwareVersion();
  Serial.print("Firmware version is: ");
  Serial.println(fv);
}

void loop() {
  // SGP30 measurements
  readSGP30(&sgp, &sgpEnv);
  if (sgpEnv.warm) {
    Serial.print("TVOC ");
    Serial.print(sgpEnv.tvoc);
    Serial.println(" ppb");
    Serial.print("eCO2 ");
    Serial.print(sgpEnv.eco2);
    Serial.println(" ppm");
    Serial.print("Raw H2 ");
    Serial.println(sgpEnv.h2);
    Serial.print("Raw Ethanol ");
    Serial.println(sgpEnv.ethanol);
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
  Serial.print("Temperature = ");
  Serial.print(bmeEnv.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");

  Serial.print(bmeEnv.pressure / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmeEnv.altitude);
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bmeEnv.humidity);
  Serial.println(" %");
  sendBME280(&udp, &bmeEnv);

  Serial.println();
  readingCount++;
  delay(10000);
}
