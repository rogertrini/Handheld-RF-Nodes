// need to implement the recieved message terminal.. currently only sends fake message


#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define BTN_UP     2
#define BTN_DOWN   3
#define BTN_SEND   4
#define BTN_EXTRA  5

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const char* messages[] = {
  "HI",
  "BYE",
  "SOS",
  "HELP",
  "COME HERE"
};

const int numMessages = 5;

int currentIndex = 0;
int receivedIndex = -1;

bool lastUpState = HIGH;
bool lastDownState = HIGH;
bool lastSendState = HIGH;
bool lastExtraState = HIGH;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 180;

/*
  drawCenteredText
*/
void drawCenteredText(const char* msg, int y, int textSize) {
  display.setTextSize(textSize);
  display.setTextColor(SSD1306_WHITE);

  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);

  int x = (SCREEN_WIDTH - w) / 2;
  if (x < 0) x = 0;

  display.setCursor(x, y);
  display.println(msg);
}

/*
  showScreen
*/
void showScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Top section: RECEIVED
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("RECEIVED");

  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  if (receivedIndex == -1) {
    drawCenteredText("NONE", 16, 2);
  } else {
    if (receivedIndex == 4) {
      drawCenteredText(messages[receivedIndex], 18, 1);
    } else {
      drawCenteredText(messages[receivedIndex], 14, 2);
    }
  }

  // Divider
  display.drawLine(0, 36, 127, 36, SSD1306_WHITE);

  // Bottom section: SEND
  display.setTextSize(1);
  display.setCursor(0, 40);
  display.println("SEND");

  if (currentIndex == 4) {
    display.setTextSize(1);
    display.setCursor(10, 54);
    display.print("> ");
    display.println(messages[currentIndex]);
  } else {
    display.setTextSize(2);
    display.setCursor(0, 48);
    display.print("> ");
    display.println(messages[currentIndex]);
  }

  display.display();
}

void setup() {
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SEND, INPUT_PULLUP);
  pinMode(BTN_EXTRA, INPUT_PULLUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for (;;) {
    }
  }

  display.clearDisplay();
  display.display();

  showScreen();
}

void loop() {
  bool upState = digitalRead(BTN_UP);
  bool downState = digitalRead(BTN_DOWN);
  bool sendState = digitalRead(BTN_SEND);
  bool extraState = digitalRead(BTN_EXTRA);

  if (millis() - lastDebounceTime > debounceDelay) {
    if (lastUpState == HIGH && upState == LOW) {
      currentIndex--;
      if (currentIndex < 0) {
        currentIndex = numMessages - 1;
      }
      showScreen();
      lastDebounceTime = millis();
    }

    if (lastDownState == HIGH && downState == LOW) {
      currentIndex++;
      if (currentIndex >= numMessages) {
        currentIndex = 0;
      }
      showScreen();
      lastDebounceTime = millis();
    }

    if (lastSendState == HIGH && sendState == LOW) {
      receivedIndex = currentIndex;   // fake receive for UI testing
      showScreen();
      lastDebounceTime = millis();
    }

    if (lastExtraState == HIGH && extraState == LOW) {
      receivedIndex = -1;
      showScreen();
      lastDebounceTime = millis();
    }
  }

  lastUpState = upState;
  lastDownState = downState;
  lastSendState = sendState;
  lastExtraState = extraState;
}
