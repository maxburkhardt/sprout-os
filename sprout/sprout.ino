#include <Wire.h>
#include <WiFiNINA.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_SGP30.h"
// This is a custom library that exports `ssid` and `pass` variables
// Do something similar (to avoid checking in credentials) or just define
// `ssid` and `pass` locally.
#include <wificreds.h>

#define SEALEVELPRESSURE_HPA (1013.25)

int radioStatus = WL_IDLE_STATUS;

Adafruit_SGP30 sgp;
Adafruit_BME280 bme;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  Serial.println("Serial ready!");

  if (! sgp.begin()){
    Serial.println("SGP30 sensor failed to initialize!");
    while (1) delay(1000);
  }

  if (! bme.begin()){
    Serial.println("BME280 sensor failed to initialize!");
    while (1) delay(1000);
  }

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi module not present");
    while(true);
  } else {
    Serial.println("WiFi detected.");
  }

  String fv = WiFi.firmwareVersion();
  Serial.print("Firmware version is: ");
  Serial.println(fv);

  /*
  while (radioStatus != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    radioStatus = WiFi.begin(ssid, pass);

    // wait 7 seconds for connection:
    delay(7000);
  }
  
  Serial.print("Connected to network ");
  Serial.println(WiFi.SSID());
  Serial.print("IP address is ");
  Serial.println(WiFi.localIP());
  */
}

void loop() {
  // SGP30 measurements
  if (! sgp.IAQmeasure()) {
    Serial.println("SGP30 measurement failed");
  } else {
    Serial.print("TVOC ");
    Serial.print(sgp.TVOC);
    Serial.println(" ppb");
    Serial.print("eCO2 ");
    Serial.print(sgp.eCO2);
    Serial.println(" ppm");
    if (! sgp.IAQmeasureRaw()) {
      Serial.println("SGP30 raw measurement failed");
    } else {
      Serial.print("Raw H2 ");
      Serial.println(sgp.rawH2);
      Serial.print("Raw Ethanol ");
      Serial.println(sgp.rawEthanol);
    }
  }

  // BME280 measurements
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
  delay(5000);
}
