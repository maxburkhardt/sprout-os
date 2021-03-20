#include <WiFiNINA.h>
#include <WiFiUdp.h>

char ssid[] = "";
char pass[] = "";
int radioStatus = WL_IDLE_STATUS;
unsigned int localPort = 2390;
char packetBuffer[255];
WiFiUDP Udp;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  Serial.println("Serial ready!");

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi module not present");
    while(true);
  } else {
    Serial.println("WiFi detected.");
  }

  String fv = WiFi.firmwareVersion();
  Serial.print("Firmware version is: ");
  Serial.println(fv);

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
  Serial.println("Starting UDP server.");
  Udp.begin(localPort);
}

void loop() {

}
