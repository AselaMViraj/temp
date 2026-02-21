// =============================================================
// Transparent RS-485 Bridge for Xiao ESP32-S3 + MAX485
// Bridges USB-CDC Serial ↔ UART ↔ MAX485 ↔ RS-485 (ZLAC8015D)
// =============================================================
//
// Wiring:
//   Xiao S3 D6 (GPIO43) → MAX485 DI
//   Xiao S3 D7 (GPIO44) → MAX485 RO
//   Xiao S3 D2 (GPIO3)  → MAX485 DE + RE (tied together)
//   Xiao S3 3V3         → MAX485 VCC
//   Xiao S3 GND         → MAX485 GND
//   MAX485 A             → ZLAC8015D eRS A+
//   MAX485 B             → ZLAC8015D eRS B-
//
// Board: XIAO_ESP32S3 in Arduino IDE
// =============================================================

#define RS485_TX_PIN  43    // Xiao S3 D6 → MAX485 DI
#define RS485_RX_PIN  44    // Xiao S3 D7 → MAX485 RO
#define RS485_DE_PIN  3     // Xiao S3 D2 → MAX485 DE+RE (direction control)

#define BAUD_RATE     115200  // ZLAC8015D default baud rate

// Use Hardware UART1 for RS-485
HardwareSerial RS485Serial(1);

// Buffer for more efficient transfers
#define BUF_SIZE 256
uint8_t buf[BUF_SIZE];

void setup() {
  // USB-CDC serial (to Jetson Orin Nano)
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    delay(10);  // Wait for USB connection
  }
  
  // Hardware UART to MAX485
  RS485Serial.begin(BAUD_RATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  
  // Direction control: HIGH = transmit, LOW = receive
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);  // Start in receive mode
  
  // Indicate ready
  Serial.println("RS485 Bridge Ready");
}

void loop() {
  // ---- Jetson → ZLAC8015D (USB-CDC → RS-485) ----
  int usbAvailable = Serial.available();
  if (usbAvailable > 0) {
    // Switch to transmit mode
    digitalWrite(RS485_DE_PIN, HIGH);
    delayMicroseconds(50);  // Allow transceiver to stabilize
    
    // Read USB data and forward to RS-485
    while (usbAvailable > 0) {
      int bytesToRead = min(usbAvailable, BUF_SIZE);
      int bytesRead = Serial.readBytes(buf, bytesToRead);
      RS485Serial.write(buf, bytesRead);
      usbAvailable = Serial.available();
    }
    
    // Wait for all bytes to be transmitted
    RS485Serial.flush();
    delayMicroseconds(50);
    
    // Switch back to receive mode
    digitalWrite(RS485_DE_PIN, LOW);
  }
  
  // ---- ZLAC8015D → Jetson (RS-485 → USB-CDC) ----
  int rs485Available = RS485Serial.available();
  if (rs485Available > 0) {
    while (rs485Available > 0) {
      int bytesToRead = min(rs485Available, BUF_SIZE);
      int bytesRead = RS485Serial.readBytes(buf, bytesToRead);
      Serial.write(buf, bytesRead);
      rs485Available = RS485Serial.available();
    }
  }
}
