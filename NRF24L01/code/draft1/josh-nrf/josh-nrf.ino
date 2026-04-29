#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Josh device: 1:1 with schematic, except using only one red LED on D8.
// Pinout:
//   Button 1 -> D2  -> previous TX message
//   Button 2 -> D3  -> next TX message
//   Button 3 -> D4  -> send selected TX message
//   Button 4 -> D5  -> quick-send SOS
//   Red LED  -> D8
//   Buzzer   -> A0
//   OLED SDA -> A4
//   OLED SCL -> A5
//   nRF CE   -> D9
//   nRF CSN  -> D10
//   nRF MOSI -> D11
//   nRF MISO -> D12
//   nRF SCK  -> D13
//   nRF VCC  -> 3.3V only
//   All GNDs connected together
const byte CE_PIN = 9;
const byte CSN_PIN = 10;
const byte BUTTON_1_PIN = 2;
const byte BUTTON_2_PIN = 3;
const byte BUTTON_3_PIN = 4;
const byte BUTTON_4_PIN = 5;
const byte LED_PIN = 8;
const byte BUZZER_PIN = A0;

const byte THIS_ADDRESS[6] = "JOSH1";
const byte PEER_ADDRESS[6] = "ROGER";

const int SCREEN_WIDTH = 128;
const int SCREEN_HEIGHT = 64;
const int OLED_RESET = -1;
const byte SCREEN_ADDRESS = 0x3C;

RF24 radio(CE_PIN, CSN_PIN);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const char *messages[] = {
  "HI",
  "BYE",
  "YES",
  "NO",
  "CLEAR",
  "SOS"
};

const byte MESSAGE_COUNT = sizeof(messages) / sizeof(messages[0]);
const byte SOS_INDEX = 5;
const unsigned long DEBOUNCE_MS = 160;
const unsigned long RX_INDICATOR_MS = 350;
const unsigned long SOS_FLASH_MS = 3000;
const unsigned long SOS_FLASH_STEP_MS = 120;
const unsigned int BUZZ_MS = 180;

struct Packet {
  uint8_t messageIndex;
  uint8_t senderId;
  uint32_t counter;
};

byte currentIndex = 0;
int receivedIndex = -1;
int lastSentIndex = -1;
uint32_t txCounter = 0;
unsigned long lastButtonMs = 0;
unsigned long rxIndicatorUntil = 0;
unsigned long buzzerUntil = 0;
unsigned long sosFlashUntil = 0;
unsigned long lastSosFlashMs = 0;
bool sosLedState = LOW;

bool lastB1 = HIGH;
bool lastB2 = HIGH;
bool lastB3 = HIGH;
bool lastB4 = HIGH;

void setup() {
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);
  pinMode(BUTTON_4_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.begin(9600);
  Serial.println("=== JOSH NRF ===");

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED failed");
    while (true) {
      delay(1000);
    }
  }

  if (!radio.begin()) {
    Serial.println("nRF24L01 failed");
    drawStatus("RADIO FAIL");
    while (true) {
      delay(1000);
    }
  }

  radio.setChannel(108);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setRetries(5, 15);
  radio.openWritingPipe(PEER_ADDRESS);
  radio.openReadingPipe(1, THIS_ADDRESS);
  radio.startListening();

  drawScreen();
  Serial.println("Ready");
}

void loop() {
  handleButtons();
  handleRadio();

  handleBuzzer();
  handleSosFlash();

  if (sosFlashUntil == 0 && rxIndicatorUntil != 0 && millis() > rxIndicatorUntil) {
    digitalWrite(LED_PIN, LOW);
    rxIndicatorUntil = 0;
  }
}

void handleButtons() {
  bool b1 = digitalRead(BUTTON_1_PIN);
  bool b2 = digitalRead(BUTTON_2_PIN);
  bool b3 = digitalRead(BUTTON_3_PIN);
  bool b4 = digitalRead(BUTTON_4_PIN);

  if (millis() - lastButtonMs > DEBOUNCE_MS) {
    if (lastB1 == HIGH && b1 == LOW) {
      previousMessage();
      printButtons(b1, b2, b3, b4);
    }

    if (lastB2 == HIGH && b2 == LOW) {
      nextMessage();
      printButtons(b1, b2, b3, b4);
    }

    if (lastB3 == HIGH && b3 == LOW) {
      sendMessage(currentIndex);
      printButtons(b1, b2, b3, b4);
    }

    if (lastB4 == HIGH && b4 == LOW) {
      sendMessage(SOS_INDEX);
      printButtons(b1, b2, b3, b4);
    }
  }

  lastB1 = b1;
  lastB2 = b2;
  lastB3 = b3;
  lastB4 = b4;
}

void handleRadio() {
  while (radio.available()) {
    Packet packet;
    radio.read(&packet, sizeof(packet));

    if (packet.messageIndex < MESSAGE_COUNT) {
      receivedIndex = packet.messageIndex;
      drawScreen();
      if (packet.messageIndex == SOS_INDEX) {
        startSosAlert();
      } else {
        digitalWrite(LED_PIN, HIGH);
        rxIndicatorUntil = millis() + RX_INDICATOR_MS;
        startBuzzer(BUZZ_MS);
      }

      Serial.print("received=1 msg=");
      Serial.print(messages[packet.messageIndex]);
      Serial.print(" count=");
      Serial.println(packet.counter);
    }
  }
}

void sendMessage(byte messageIndex) {
  if (messageIndex >= MESSAGE_COUNT) {
    return;
  }

  Packet packet = {messageIndex, 1, ++txCounter};

  radio.stopListening();
  bool ok = radio.write(&packet, sizeof(packet));
  radio.startListening();
  lastSentIndex = messageIndex;
  drawScreen();

  Serial.print("sent=");
  Serial.print(ok ? 1 : 0);
  Serial.print(" msg=");
  Serial.print(messages[messageIndex]);
  Serial.print(" count=");
  Serial.println(packet.counter);

  digitalWrite(LED_PIN, ok ? HIGH : LOW);
  rxIndicatorUntil = millis() + 120;
  lastButtonMs = millis();
}

void previousMessage() {
  currentIndex = currentIndex == 0 ? MESSAGE_COUNT - 1 : currentIndex - 1;
  drawScreen();
  lastButtonMs = millis();
}

void nextMessage() {
  currentIndex++;
  if (currentIndex >= MESSAGE_COUNT) {
    currentIndex = 0;
  }
  drawScreen();
  lastButtonMs = millis();
}

void drawScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("JOSH NRF");
  display.setCursor(0, 10);
  display.print("TX:");
  display.print(messages[currentIndex]);
  if (lastSentIndex >= 0) {
    display.setCursor(58, 10);
    display.print("S:");
    display.print(messages[lastSentIndex]);
  }
  display.drawLine(0, 21, 127, 21, SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 25);
  display.println("RECEIVED");

  const char *rxText = receivedIndex < 0 ? "NONE" : messages[receivedIndex];
  drawText(rxText, 42, strlen(rxText) > 5 ? 2 : 3);
  display.display();
}

void drawStatus(const char *status) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(status);
  display.display();
}

void drawText(const char *text, int y, byte size) {
  display.setTextSize(size);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  int x = (SCREEN_WIDTH - w) / 2;
  if (x < 0) {
    x = 0;
  }
  display.setCursor(x, y);
  display.println(text);
}

void printButtons(bool b1, bool b2, bool b3, bool b4) {
  Serial.print("buttons=");
  Serial.print(b1 == LOW ? 1 : 0);
  Serial.print(",");
  Serial.print(b2 == LOW ? 1 : 0);
  Serial.print(",");
  Serial.print(b3 == LOW ? 1 : 0);
  Serial.print(",");
  Serial.println(b4 == LOW ? 1 : 0);
}

void startSosAlert() {
  sosFlashUntil = millis() + SOS_FLASH_MS;
  lastSosFlashMs = 0;
  sosLedState = LOW;
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
}

void handleSosFlash() {
  if (sosFlashUntil == 0) {
    return;
  }

  if (millis() >= sosFlashUntil) {
    sosFlashUntil = 0;
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  if (millis() - lastSosFlashMs >= SOS_FLASH_STEP_MS) {
    lastSosFlashMs = millis();
    sosLedState = !sosLedState;
    digitalWrite(LED_PIN, sosLedState);
    digitalWrite(BUZZER_PIN, sosLedState);
  }
}

void startBuzzer(unsigned long durationMs) {
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerUntil = millis() + durationMs;
}

void handleBuzzer() {
  if (buzzerUntil != 0 && millis() >= buzzerUntil) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerUntil = 0;
  }
}
