#include <SPI.h>
#include <Ethernet.h>

// Define your custom pin mappings
#define W5500_MISO_PIN  21
#define W5500_MOSI_PIN  33
#define W5500_SCLK_PIN  18
#define W5500_CS_PIN     5
#define W5500_RST_PIN   17

// Set a unique MAC address
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

void setup() {
  Serial.begin(115200);
 // while (!Serial);
  delay(10000); // Allow time for Serial Monitor to open
  Serial.println("Initializing Custom SPI Pins for W5500...");

  // 1. Hardware reset the W5500 module using your custom RST pin (GPIO 17)
  pinMode(W5500_RST_PIN, OUTPUT);
  digitalWrite(W5500_RST_PIN, LOW);
  delay(100);
  digitalWrite(W5500_RST_PIN, HIGH);
  delay(200);

  // 2. FORCE the ESP32 SPI bus to use your specific pins
  // SPI.begin(SCLK, MISO, MOSI, SS/CS)
  SPI.begin(W5500_SCLK_PIN, W5500_MISO_PIN, W5500_MOSI_PIN, W5500_CS_PIN);

  // 3. Configure Ethernet library to use your custom Chip Select pin (GPIO 5)
  Ethernet.init(W5500_CS_PIN);

  // 4. Start Ethernet with DHCP
  Serial.println("Requesting IP from DHCP server...");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    
    // Diagnostic checks
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Error: W5500 hardware not found. Check SPI pin assignments.");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Error: Ethernet cable is unplugged.");
    }
    while (true) { delay(1); } // Halt
  }

  // Success!
  Serial.println("Successfully connected!");
  Serial.print("Assigned IP Address: ");
  Serial.println(Ethernet.localIP());
  Serial.print("Gateway Address:     ");
  Serial.println(Ethernet.gatewayIP());
}

void loop() {
  // Maintain DHCP lease automatically
  Ethernet.maintain();
  
  delay(1000);
}
