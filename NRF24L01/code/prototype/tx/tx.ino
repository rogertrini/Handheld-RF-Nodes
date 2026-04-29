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
const byte BUTTON_PIN = 2;

RF24 radio(CE_PIN, CSN_PIN);

const byte RADIO_ADDRESS[6] = "RF001";

struct Packet {
  uint8_t type;
  uint32_t count;
};

uint32_t pressCount = 0;
bool lastButtonState = HIGH;
bool lastPrintedButtonPressed = false;
unsigned long lastDebounceMs = 0;
unsigned long lastStatusMs = 0;

const unsigned long DEBOUNCE_MS = 40;
const unsigned long STATUS_MS = 500;
const uint8_t BUTTON_PRESS = 1;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);

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
  radio.setRetries(5, 15);
  radio.openWritingPipe(RADIO_ADDRESS);
  radio.stopListening();

  Serial.println("TX ready. Press D2 button to transmit.");
}

void loop() {
  bool buttonState = digitalRead(BUTTON_PIN);
  bool buttonPressed = buttonState == LOW;

  if (buttonPressed != lastPrintedButtonPressed || millis() - lastStatusMs >= STATUS_MS) {
    Serial.print("button=");
    Serial.print(buttonPressed ? 1 : 0);
    Serial.println(" sent=0");
    lastPrintedButtonPressed = buttonPressed;
    lastStatusMs = millis();
  }

  if (buttonState != lastButtonState) {
    lastDebounceMs = millis();
    lastButtonState = buttonState;
  }

  if ((millis() - lastDebounceMs) > DEBOUNCE_MS && buttonState == LOW) {
    sendButtonPress();

    while (digitalRead(BUTTON_PIN) == LOW) {
      delay(5);
    }

    lastButtonState = HIGH;
    lastDebounceMs = millis();
  }
}

void sendButtonPress() {
  Packet packet = {BUTTON_PRESS, ++pressCount};
  bool ok = radio.write(&packet, sizeof(packet));

  Serial.print("Sent button press #");
  Serial.print(packet.count);
  Serial.print(" button=1 sent=");
  Serial.println(ok ? 1 : 0);
}
