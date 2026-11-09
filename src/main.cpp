#include <ETH.h>
#include <SPI.h>

#define W5500_MISO_PIN  13
#define W5500_MOSI_PIN  12
#define W5500_SCLK_PIN  14
#define W5500_CS_PIN    48
#define W5500_RST_PIN   21
#define W5500_INT_PIN   -1   // set to your actual INT pin if wired; -1 disables interrupt mode

NetworkServer server(80);   // yes, WiFiServer — it works over ETH too, same LwIP stack
bool eth_connected = false;

void onEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.print("ETH IP: ");
      Serial.println(ETH.localIP());
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(W5500_RST_PIN, OUTPUT);
  digitalWrite(W5500_RST_PIN, LOW);
  delay(100);
  digitalWrite(W5500_RST_PIN, HIGH);
  delay(200);

  Network.onEvent(onEvent);

  SPI.begin(W5500_SCLK_PIN, W5500_MISO_PIN, W5500_MOSI_PIN, W5500_CS_PIN);
  ETH.begin(ETH_PHY_W5500, 1, W5500_CS_PIN, W5500_INT_PIN, W5500_RST_PIN, SPI);

  server.begin();
}

void loop() {
  if (!eth_connected) return;

  NetworkClient client = server.available();
  if (client) {
    unsigned long startTime = millis();
    while (!client.available() && client.connected()) {
      if (millis() - startTime > 1000) { client.stop(); return; }
    }
    if (client.connected()) {
      while (client.available()) client.read();
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/html");
      client.println("Connection: close");
      client.println();
      client.println("<h1>Hello from ESP32 + W5500</h1>");
    }
    client.stop();
  }
}