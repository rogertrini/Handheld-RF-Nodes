#include <SPI.h>
#include <RF24.h>

// nRF24L01 wiring for Arduino Nano/Uno:
// CE  -> D9
// CSN -> D10
// SCK -> D13
// MOSI -> D11
// MISO -> D12
// VCC -> 3.3V only
// GND -> GND
const byte CE_PIN = 9;
const byte CSN_PIN = 10;
const byte BUZZER_PIN = 3;

RF24 radio(CE_PIN, CSN_PIN);

const byte RADIO_ADDRESS[6] = "RF001";

struct Packet {
  uint8_t type;
  uint32_t count;
};

const uint8_t BUTTON_PRESS = 1;
const unsigned int BUZZ_HZ = 2200;
const unsigned int BUZZ_MS = 250;
const unsigned long STATUS_MS = 500;

unsigned long lastStatusMs = 0;

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  Serial.begin(9600);
  while (!Serial) {
    ; // Needed only for boards with native USB.
  }

  if (!radio.begin()) {
    Serial.println("nRF24L01 radio not responding");
    while (true) {
      delay(1000);
    }
  }

  radio.setChannel(108);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, RADIO_ADDRESS);
  radio.startListening();

  Serial.println("RX ready. Waiting for button packets.");
}

void loop() {
  bool received = false;

  while (radio.available()) {
    Packet packet;
    radio.read(&packet, sizeof(packet));
    received = true;

    Serial.print("Received packet type ");
    Serial.print(packet.type);
    Serial.print(", count ");
    Serial.print(packet.count);
    Serial.println(" received=1");

    if (packet.type == BUTTON_PRESS) {
      buzz();
    }
  }

  if (!received && millis() - lastStatusMs >= STATUS_MS) {
    Serial.println("received=0");
    lastStatusMs = millis();
  } else if (received) {
    lastStatusMs = millis();
  }
}

void buzz() {
  tone(BUZZER_PIN, BUZZ_HZ, BUZZ_MS);
  delay(BUZZ_MS + 20);
  noTone(BUZZER_PIN);
}
