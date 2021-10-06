#include <RoxMux.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET  3
#define OLED_DC1     2
#define OLED_DC2     38
#define OLED_CS     5

#define OLED2_CS     35

Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT,
                          &SPI, OLED_DC1, OLED_RESET, OLED_CS);

Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT,
                          &SPI, OLED_DC2, -1, OLED2_CS);

#define SELECT1 27
#define SELECT2 28
#define SELECT3 29
#define SELECT4 30

uint8_t pinStates[1];
uint8_t digit;
uint8_t digit1;
uint8_t digit2;
uint8_t digit3;
uint8_t digit4;
uint8_t prevdigit1;
uint8_t prevdigit2;
uint8_t prevdigit3;
uint8_t prevdigit4;
uint8_t oldbank;
uint8_t data1;
uint8_t data2;
uint8_t data3;
uint8_t data4;
uint16_t screenType1;
uint16_t screenType2;
uint16_t patchScreen;
uint16_t midiPushed;
int paramMode = 0;
int messageMode = 0;
uint8_t charstable[4];

// define segment truth table for each digit
static const int digit_array[41][8] = {
  {1, 1, 1, 1, 1, 1, 0, 0},  // 0
  {0, 1, 1, 0, 0, 0, 0, 0},  // 1
  {1, 1, 0, 1, 1, 0, 1, 0},  // 2
  {1, 1, 1, 1, 0, 0, 1, 0},  // 3
  {0, 1, 1, 0, 0, 1, 1, 0},  // 4
  {1, 0, 1, 1, 0, 1, 1, 0},  // 5
  {1, 0, 1, 1, 1, 1, 1, 0},  // 6
  {1, 1, 1, 0, 0, 0, 0, 0},  // 7
  {1, 1, 1, 1, 1, 1, 1, 0},  // 8
  {1, 1, 1, 0, 0, 1, 1, 0},  // 9 - 10
  {0, 0, 0, 0, 0, 0, 1, 0},  // -
  {0, 0, 0, 0, 0, 0, 0, 0},  // blank
  {1, 0, 0, 1, 1, 1, 0, 0},  // C
  {0, 1, 1, 0, 1, 1, 1, 0},  // H
  {0, 1, 1, 0, 0, 0, 0, 1},  // 1.
  {1, 1, 0, 1, 1, 0, 1, 1},  // 2.
  {1, 1, 1, 1, 0, 0, 1, 1},  // 3.
  {0, 1, 1, 0, 0, 1, 1, 1},  // 4.
  {1, 0, 1, 1, 0, 1, 1, 1},  // 5.
  {1, 0, 1, 1, 1, 1, 1, 1},  // 6. - 20
  {1, 1, 1, 0, 0, 0, 0, 1},  // 7.
  {1, 1, 1, 1, 1, 1, 1, 1},  // 8.
  {1, 1, 1, 0, 0, 1, 1, 1},  // 9.
  {1, 1, 0, 0, 1, 1, 1, 0},  // P
  {0, 0, 1, 0, 1, 0, 1, 0},  // n
  {0, 0, 1, 0, 0, 0, 0, 0},  // |
  {0, 0, 1, 1, 1, 0, 1, 0},  // o
  {1, 1, 1, 0, 1, 1, 0, 0},  // M
  {0, 0, 1, 1, 1, 1, 1, 0},  // b
  {0, 0, 1, 0, 1, 1, 0, 0},  // value 52
  {0, 0, 0, 0, 1, 0, 0, 0},  // value 16
  {0, 1, 1, 0, 1, 0, 0, 0},  // value 22
  {0, 0, 1, 0, 1, 0, 0, 0},  // value 20
  {0, 0, 0, 0, 1, 1, 0, 0},  // value 48
  {1, 1, 0, 0, 0, 1, 1, 0},  // value 99
  {0, 0, 0, 1, 1, 1, 1, 0},   // t
  {1, 0, 0, 1, 0, 0, 0, 0},   // =
  {0, 0, 0, 0, 1, 0, 1, 0},   // r
  {1, 0, 0, 1, 1, 1, 1, 0},   // E
  {1, 0, 1, 1, 1, 1, 0, 0},   // G
  {0, 1, 1, 1, 1, 0, 1, 0},   // d

};

Rox74HC165 <1> mux;
#define PIN_DATA  24 // pin 9 on 74HC165 (DATA)
#define PIN_LOAD  25 // pin 1 on 74HC165 (LOAD)
#define PIN_CLK   26 // pin 2 on 74HC165 (CLK))

void setup()
{

  pinMode(SELECT1, INPUT_PULLDOWN);
  pinMode(SELECT2, INPUT_PULLDOWN);
  pinMode(SELECT3, INPUT_PULLDOWN);
  pinMode(SELECT4, INPUT_PULLDOWN);

  attachInterrupt(digitalPinToInterrupt(SELECT1), data1_pin, RISING);
  attachInterrupt(digitalPinToInterrupt(SELECT2), data2_pin, RISING);
  attachInterrupt(digitalPinToInterrupt(SELECT3), data3_pin, RISING);
  attachInterrupt(digitalPinToInterrupt(SELECT4), data4_pin, RISING);

  Serial.begin(115200);

  mux.begin(PIN_DATA, PIN_LOAD, PIN_CLK);

  display1.begin(SSD1306_SWITCHCAPVCC);  // initialize with the I2C addr 0x3D (for the 128x64)
  display1.clearDisplay();   // Clear the buffer.

  //display1.setFont(&FreeSerif9pt7b);

  // func params: render_digits(pox_x, pos_y, spacing, WHITE)
  render_digits_left(8, 4, 24, 88, WHITE);
  display1.display();
  display2.begin(SSD1306_SWITCHCAPVCC);  // initialize with the I2C addr 0x3D (for the 128x64)
  display2.clearDisplay();   // Clear the buffer.
  // func params: render_digits(pox_x, pos_y, spacing, WHITE)
  render_digits_right(8, 4, 24, 88, WHITE);
  display2.display();
  delay(500);
  display1.clearDisplay();   // Clear the buffer.
  display1.display();
  display2.clearDisplay();   // Clear the buffer.
  display2.display();
}

void data1_pin()
{
  mux.update();

  for (uint8_t i = 0; i < 1; i++) {
    uint8_t data = mux.read(i);
    charstable[0] = data;
  }
}

void data2_pin()
{
  mux.update();

  for (uint8_t i = 0; i < 1; i++) {
    uint8_t data = mux.read(i);
    charstable[1] = data;
  }
}

void data3_pin()
{
  mux.update();

  for (uint8_t i = 0; i < 1; i++) {
    uint8_t data = mux.read(i);

    charstable[2] = data;
  }
}

void data4_pin()
{
  mux.update();

  for (uint8_t i = 0; i < 1; i++) {
    uint8_t data = mux.read(i);
    charstable[3] = data;
  }
}

void loop()
{

  convertData(charstable[0]);
  digit1 = digit;
  convertData(charstable[1]);
  digit2 = digit;
  displayLeftScreen(digit1, digit2, prevdigit1, prevdigit2);
  display1.display();
  prevdigit1 = digit1;
  prevdigit2 = digit2;

  convertData(charstable[2]);
  digit3 = digit;
  convertData(charstable[3]);
  digit4 = digit;
  displayRightScreen(digit3, digit4, prevdigit3, prevdigit4);
  prevdigit3 = digit3;
  prevdigit4 = digit3;
  display2.display();
}

void clearLeftScreenValues()
{
  display1.clearDisplay();
  render_digit_left(18, 22, prevdigit1, BLACK);
  render_digit_left(78, 22, prevdigit2, BLACK);
}

void clearRightScreenValues()
{
  display2.clearDisplay();
  render_digit_right(18, 22, prevdigit3, BLACK);
  render_digit_right(78, 22, prevdigit4, BLACK);
}

void drawRectLeft()
{
  display1.drawRect( 0,  0,  128,  64,  WHITE);
}

void drawRectRight()
{
  display2.drawRect( 0,  0,  128,  64,  WHITE);
}

void displayLeftScreen(uint8_t digit1, uint8_t digit2, uint8_t prevdigit1, uint8_t prevdigit2)
{
  patchScreen = (charstable[0] + charstable[2]);

  switch (patchScreen)
  {
    case 64:
      clearLeftScreenValues();
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(2);
      display1.setCursor(36, 0);
      display1.println(F("BANK"));
      display2.clearDisplay();
      display2.setTextColor(WHITE);
      display2.setTextSize(2);
      display2.setCursor(38, 0);
      display2.println(F("PATCH"));
      render_digit_left(18, 22, prevdigit1, BLACK);
      render_digit_left(18, 22, digit1, WHITE);
      render_digit_left(78, 22, prevdigit2, BLACK);
      render_digit_left(78, 22, digit2, WHITE);
      return;

    case 179:
      clearLeftScreenValues();
      paramMode = 0;
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(2);
      display1.setCursor(28, 0);
      display1.println(F("PRESET"));
      display2.clearDisplay();
      display2.setTextColor(WHITE);
      display2.setTextSize(2);
      display2.setCursor(38, 0);
      display2.println(F("PATCH"));
      render_digit_left(18, 22, prevdigit1, BLACK);
      render_digit_left(18, 22, digit1, WHITE);
      render_digit_left(78, 22, prevdigit2, BLACK);
      render_digit_left(78, 22, digit2, WHITE);
      return;

    case 121:
      clearLeftScreenValues();
      paramMode = 0;
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(2);
      display1.setCursor(36, 0);
      display1.println(F("CART"));
      display2.clearDisplay();
      display2.setTextColor(WHITE);
      display2.setTextSize(2);
      display2.setCursor(38, 0);
      display2.println(F("PATCH"));
      render_digit_left(18, 22, prevdigit1, BLACK);
      render_digit_left(18, 22, digit1, WHITE);
      render_digit_left(78, 22, prevdigit2, BLACK);
      render_digit_left(78, 22, digit2, WHITE);
      return;
  }

  screenType1 = (charstable[0] + charstable[1]);

  switch (screenType1)
  {
    case 239:
      paramMode = 1;
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(1);
      display1.setCursor(14, 0);
      display1.println(F(" Global Parameter "));
      display1.setTextSize(3);
      display1.setCursor(22, 17);
      display1.println(F("PITCH"));
      display1.setCursor(32, 43);
      display1.println(F("BEND"));
      break;

    case 182:
      paramMode = 1;
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 22 "));
      display1.setTextSize(3);
      display1.setCursor(22, 17);
      display1.println(F("DCO-1"));
      display1.setCursor(28, 43);
      display1.println(F("WAVE"));
      break;

    case 310:
      paramMode = 1;
      display1.setTextColor(BLACK, WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 22 "));
      display1.setTextColor(WHITE);
      display1.setTextSize(3);
      display1.setCursor(22, 17);
      display1.println(F("DCO-1"));
      display1.setCursor(28, 43);
      display1.println(F("WAVE"));
      break;


    case 193:
      paramMode = 1;
      switch (charstable[0])
      {
        case 91:
          if ((charstable[0] == 91 && charstable[1] == 102) || (charstable[0] == 91 && charstable[1] == 230 ))
          {
            display1.clearDisplay();
            display1.setTextColor(WHITE);
            display1.setTextSize(1);
            display1.setCursor(20, 0);
            display1.println(F(" Parameter 24 "));
            display1.setTextSize(3);
            display1.setCursor(22, 17);
            display1.println(F("DCO-2"));
            display1.setCursor(28, 43);
            display1.println(F("WAVE"));
          }
          break;

        case 102:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 42 "));
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("ENV"));
          display1.setCursor(22, 43);
          display1.println(F("DECAY"));
          break;
      }
      break;

    case 321:
      paramMode = 1;
      switch (charstable[0])
      {
        case 91:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 24 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(22, 17);
          display1.println(F("DCO-2"));
          display1.setCursor(28, 43);
          display1.println(F("WAVE"));
          break;

        case 102:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 42 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("ENV"));
          display1.setCursor(22, 43);
          display1.println(F("DECAY"));
          break;
      }
      break;

    case 344:
      paramMode = 1;
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 26 "));
      display1.setTextSize(3);
      display1.setCursor(22, 17);
      display1.println(F("DCO-1"));
      display1.setCursor(12, 43);
      display1.println(F("ENV FM"));
      break;

    case 216:
      paramMode = 1;
      display1.setTextColor(BLACK, WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 26 "));
      display1.setTextColor(WHITE);
      display1.setTextSize(3);
      display1.setCursor(22, 17);
      display1.println(F("DCO-1"));
      display1.setCursor(12, 43);
      display1.println(F("ENV FM"));
      break;

    case 200:
      paramMode = 1;
      switch (charstable[0])
      {
        case 91:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 25 "));
          display1.setTextSize(3);
          display1.setCursor(22, 17);
          display1.println(F("DCO-1"));
          display1.setCursor(12, 43);
          display1.println(F("LFO FM"));
          break;

        case 109:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 52 "));
          display1.setTextSize(3);
          display1.setCursor(18, 17);
          display1.println(F("SPLIT"));
          display1.setCursor(18, 43);
          display1.println(F("POINT"));
          break;
      }
      break;

    case 328:
      paramMode = 1;
      switch (charstable[0])
      {
        case 91:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 25 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(22, 17);
          display1.println(F("DCO-1"));
          display1.setCursor(12, 43);
          display1.println(F("LFO FM"));
          break;

        case 109:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 52 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(18, 17);
          display1.println(F("SPLIT"));
          display1.setCursor(18, 43);
          display1.println(F("POINT"));
          break;
      }
      break;

    case 98:
      paramMode = 1;
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 27 "));
      display1.setTextSize(3);
      display1.setCursor(22, 17);
      display1.println(F("DCO-2"));
      display1.setCursor(12, 43);
      display1.println(F("LFO FM"));
      break;

    case 226:
      paramMode = 1;
      display1.setTextColor(BLACK, WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 27 "));
      display1.setTextColor(WHITE);
      display1.setTextSize(3);
      display1.setCursor(22, 17);
      display1.println(F("DCO-2"));
      display1.setCursor(12, 43);
      display1.println(F("LFO FM"));
      break;

    case 218:
      paramMode = 1;
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 28 "));
      display1.setTextSize(3);
      display1.setCursor(22, 17);
      display1.println(F("DCO-2"));
      display1.setCursor(12, 43);
      display1.println(F("ENV FM"));
      break;

    case 346:
      paramMode = 1;
      display1.setTextColor(BLACK, WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 28 "));
      display1.setTextColor(WHITE);
      display1.setTextSize(3);
      display1.setCursor(22, 17);
      display1.println(F("DCO-2"));
      display1.setCursor(12, 43);
      display1.println(F("ENV FM"));
      break;

    case 158:
      paramMode = 1;
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 33 "));
      display1.setTextSize(3);
      display1.setCursor(36, 17);
      display1.println(F("VCF"));
      display1.setCursor(0, 43);
      display1.println(F("KEY TRK"));
      break;

    case 286:
      paramMode = 1;
      display1.setTextColor(BLACK, WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 33 "));
      display1.setTextColor(WHITE);
      display1.setTextSize(3);
      display1.setCursor(36, 17);
      display1.println(F("VCF"));
      display1.setCursor(0, 43);
      display1.println(F("KEY TRK"));
      break;

    case 181:
      paramMode = 1;
      switch (charstable[0])
      {
        case 79:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 34 "));
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("VCF"));
          display1.setCursor(0, 43);
          display1.println(F("LFO MOD"));
          break;

        case 102:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 43 "));
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("ENV"));
          display1.setCursor(0, 43);
          display1.println(F("SUSTAIN"));
          break;
      }
      break;

    case 309:
      paramMode = 1;
      switch (charstable[0])
      {
        case 79:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 34 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("VCF"));
          display1.setCursor(0, 43);
          display1.println(F("LFO MOD"));
          break;

        case 102:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 43 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("ENV"));
          display1.setCursor(0, 43);
          display1.println(F("SUSTAIN"));
          break;
      }
      break;

    case 188:
      paramMode = 1;
      switch (charstable[0])
      {
        case 79:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 35 "));
          display1.setTextSize(3);
          display1.setCursor(12, 17);
          display1.println(F("CHORUS"));
          display1.setCursor(20, 43);
          display1.println(F("SPEED"));
          break;

        case 109:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 53 "));
          display1.setTextSize(3);
          display1.setCursor(18, 17);
          display1.println(F("SPLIT"));
          display1.setCursor(12, 43);
          display1.println(F("UP/DWN"));
          break;
      }
      break;

    case 316:
      switch (charstable[0])
      {
        case 79:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 35 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(12, 17);
          display1.println(F("CHORUS"));
          display1.setCursor(20, 43);
          display1.println(F("SPEED"));
          break;

        case 109:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 53 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(18, 17);
          display1.println(F("SPLIT"));
          display1.setCursor(12, 43);
          display1.println(F("UP/DWN"));
          break;
      }
      break;

    case 204:
      paramMode = 1;
      switch (charstable[0])
      {
        case 79:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 36 "));
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("VCF"));
          display1.setCursor(0, 43);
          display1.println(F("ENV INV"));
          break;

        case 102:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 44 "));
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("ENV"));
          display1.setCursor(18, 43);
          display1.println(F("DECAY"));
          break;
      }
      break;

    case 332:
      paramMode = 1;
      switch (charstable[0])
      {
        case 79:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 36 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("VCF"));
          display1.setCursor(0, 43);
          display1.println(F("ENV INV"));
          break;
          break;

        case 102:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 44 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("ENV"));
          display1.setCursor(18, 43);
          display1.println(F("DECAY"));
          break;
      }
      break;

    case 86:
      paramMode = 1;
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 37 "));
      display1.setTextSize(3);
      display1.setCursor(36, 17);
      display1.println(F("VCA"));
      display1.setCursor(22, 43);
      display1.println(F("LEVEL"));
      break;

    case 214:
      paramMode = 1;
      display1.setTextColor(BLACK, WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 37 "));
      display1.setTextColor(WHITE);
      display1.setTextSize(3);
      display1.setCursor(36, 17);
      display1.println(F("VCA"));
      display1.setCursor(22, 43);
      display1.println(F("LEVEL"));
      break;

    case 206:
      paramMode = 1;
      switch (charstable[0])
      {
        case 79:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 38 "));
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("VCA"));
          display1.setCursor(28, 43);
          display1.println(F("MODE"));
      }
      break;

    case 334:
      paramMode = 1;
      display1.setTextColor(BLACK, WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 38 "));
      display1.setTextColor(WHITE);
      display1.setTextSize(3);
      display1.setCursor(36, 17);
      display1.println(F("VCA"));
      display1.setCursor(28, 43);
      display1.println(F("MODE"));
      break;

    case 211:
      switch (charstable[0])
      {
        case 102:
          paramMode = 1;
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 45 "));
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("LFO"));
          display1.setCursor(28, 43);
          display1.println(F("WAVE"));
          break;

        case 109:
          paramMode = 1;
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 54 "));
          display1.setTextSize(3);
          display1.setCursor(16, 17);
          display1.println(F("SYNTH"));
          display1.setCursor(2, 43);
          display1.println(F("DYNAMIC"));
          break;
      }
      break;

    case 339:
      switch (charstable[0])
      {
        case 102:
          paramMode = 1;
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 45 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("LFO"));
          display1.setCursor(28, 43);
          display1.println(F("WAVE"));
          break;

        case 109:
          paramMode = 1;
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 54 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(16, 17);
          display1.println(F("SYNTH"));
          display1.setCursor(2, 43);
          display1.println(F("DYNAMIC"));
          break;
      }
      break;

    case 227:
      paramMode = 1;
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 46 "));
      display1.setTextSize(3);
      display1.setCursor(36, 17);
      display1.println(F("LFO"));
      display1.setCursor(18, 43);
      display1.println(F("DELAY"));
      break;

    case 355:
      paramMode = 1;
      display1.setTextColor(BLACK, WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 46 "));
      display1.setTextColor(WHITE);
      display1.setTextSize(3);
      display1.setCursor(36, 17);
      display1.println(F("LFO"));
      display1.setCursor(18, 43);
      display1.println(F("DELAY"));
      break;

    case 109:
      switch (charstable[0])
      {
        case 102:
          paramMode = 1;
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 47 "));
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("LFO"));
          display1.setCursor(26, 43);
          display1.println(F("RATE"));
      }
      break;

    case 237:
      paramMode = 1;
      display1.setTextColor(BLACK, WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 47 "));
      display1.setTextColor(WHITE);
      display1.setTextSize(3);
      display1.setCursor(36, 17);
      display1.println(F("LFO"));
      display1.setCursor(26, 43);
      display1.println(F("RATE"));
      break;

    case 229:
      paramMode = 1;
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 48 "));
      display1.setTextSize(3);
      display1.setCursor(8, 17);
      display1.println(F("CHORUS"));
      display1.setCursor(8, 43);
      display1.println(F("SWITCH"));
      break;

    case 357:
      paramMode = 1;
      display1.setTextColor(BLACK, WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 48 "));
      display1.setTextColor(WHITE);
      display1.setTextSize(3);
      display1.setCursor(8, 17);
      display1.println(F("CHORUS"));
      display1.setCursor(8, 43);
      display1.println(F("SWITCH"));
      break;

    case 12:
      paramMode = 1;
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 11 "));
      display1.setTextSize(3);
      display1.setCursor(36, 17);
      display1.println(F("VCF"));
      display1.setCursor(12, 43);
      display1.println(F("CUTOFF"));
      break;

    case 140:
      paramMode = 1;
      display1.setTextColor(BLACK, WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 11 "));
      display1.setTextColor(WHITE);
      display1.setTextSize(3);
      display1.setCursor(36, 17);
      display1.println(F("VCF"));
      display1.setCursor(12, 43);
      display1.println(F("CUTOFF"));
      break;

    case 97:
      paramMode = 1;
      switch (charstable[0])
      {
        case 6:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 12 "));
          display1.setTextSize(3);
          display1.setCursor(22, 17);
          display1.println(F("DCO-2"));
          display1.setCursor(14, 43);
          display1.println(F("F-TUNE"));
          break;

        case 91:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 21 "));
          display1.setTextSize(3);
          display1.setCursor(22, 17);
          display1.println(F("DCO-1"));
          display1.setCursor(20, 43);
          display1.println(F("RANGE"));
          break;
      }
      break;

    case 225:
      paramMode = 1;
      switch (charstable[0])
      {
        case 6:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 12 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(22, 17);
          display1.println(F("DCO-2"));
          display1.setCursor(14, 43);
          display1.println(F("F-TUNE"));
          break;

        case 91:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 21 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(22, 17);
          display1.println(F("DCO-1"));
          display1.setCursor(20, 43);
          display1.println(F("RANGE"));
          break;
      }
      break;

    case 170:
      paramMode = 1;
      switch (charstable[0])
      {
        case 91:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 23 "));
          display1.setTextSize(3);
          display1.setCursor(22, 17);
          display1.println(F("DCO-2"));
          display1.setCursor(20, 43);
          display1.println(F("RANGE"));
          break;
        case 79:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 32 "));
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("VCF"));
          display1.setCursor(0, 43);
          display1.println(F("ENV MOD"));
          break;
      }
      break;

    case 298:
      paramMode = 1;
      switch (charstable[0])
      {
        case 91:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 23 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(22, 17);
          display1.println(F("DCO-2"));
          display1.setCursor(20, 43);
          display1.println(F("RANGE"));
          break;
        case 79:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 32 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("VCF"));
          display1.setCursor(0, 43);
          display1.println(F("ENV MOD"));
          break;
      }
      break;

    case 85:
      paramMode = 1;
      switch (charstable[0])
      {
        case 6:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 13 "));
          display1.setTextSize(3);
          display1.setCursor(22, 17);
          display1.println(F("DCO-2"));
          display1.setCursor(14, 43);
          display1.println(F("C-TUNE"));
          break;

        case 79:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 31 "));
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("VCF"));
          display1.setCursor(36, 43);
          display1.println(F("RES"));
          break;
      }
      break;

    case 213:
      paramMode = 1;
      switch (charstable[0])
      {
        case 6:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 13 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(22, 17);
          display1.println(F("DCO-2"));
          display1.setCursor(14, 43);
          display1.println(F("C-TUNE"));
          break;
        case 79:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 31 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(36, 17);
          display1.println(F("VCF"));
          display1.setCursor(36, 43);
          display1.println(F("RES"));
          break;
      }
      break;

    case 108:
      paramMode = 1;
      switch (charstable[0])
      {
        case 6:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 14 "));
          display1.setTextSize(3);
          display1.setCursor(22, 17);
          display1.println(F("DCO-2"));
          display1.setCursor(22, 43);
          display1.println(F("X-MOD"));
          break;

        case 102:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 41 "));
          display1.setTextSize(3);
          display1.setCursor(34, 17);
          display1.println(F("ENV"));
          display1.setCursor(10, 43);
          display1.println(F("ATTACK"));
          break;
      }
      break;

    case 236:
      paramMode = 1;
      switch (charstable[0])
      {
        case 6:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 14 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(22, 17);
          display1.println(F("DCO-2"));
          display1.setCursor(22, 43);
          display1.println(F("X-MOD"));
          break;

        case 102:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 41 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(34, 17);
          display1.println(F("ENV"));
          display1.setCursor(10, 43);
          display1.println(F("ATTACK"));
          break;
      }
      break;

    case 115:
      paramMode = 1;
      switch (charstable[0])
      {
        case 6:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 15 "));
          display1.setTextSize(3);
          display1.setCursor(34, 17);
          display1.println(F("LFO"));
          display1.setCursor(16, 43);
          display1.println(F("DEPTH"));
          break;

        case 109:
          display1.clearDisplay();
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 51 "));
          display1.setTextSize(3);
          display1.setCursor(24, 17);
          display1.println(F("PEDAL"));
          display1.setCursor(32, 43);
          display1.println(F("MODE"));
          break;
      }
      break;

    case 243:
      paramMode = 1;
      switch (charstable[0])
      {
        case 6:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 15 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(34, 17);
          display1.println(F("LFO"));
          display1.setCursor(16, 43);
          display1.println(F("DEPTH"));
          break;

        case 109:
          display1.setTextColor(BLACK, WHITE);
          display1.setTextSize(1);
          display1.setCursor(20, 0);
          display1.println(F(" Parameter 51 "));
          display1.setTextColor(WHITE);
          display1.setTextSize(3);
          display1.setCursor(24, 17);
          display1.println(F("PEDAL"));
          display1.setCursor(32, 43);
          display1.println(F("MODE"));
          break;
      }
      break;

    case 131:
      paramMode = 1;
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 16 "));
      display1.setTextSize(3);
      display1.setCursor(34, 17);
      display1.println(F("ENV"));
      display1.setCursor(16, 43);
      display1.println(F("DEPTH"));
      break;

    case 259:
      paramMode = 1;
      display1.setTextColor(BLACK, WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 16 "));
      display1.setTextColor(WHITE);
      display1.setTextSize(3);
      display1.setCursor(34, 17);
      display1.println(F("ENV"));
      display1.setCursor(16, 43);
      display1.println(F("DEPTH"));
      break;

    case 13:
      paramMode = 1;
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 17 "));
      display1.setTextSize(3);
      display1.setCursor(34, 17);
      display1.println(F("ENV"));
      display1.setCursor(10, 43);
      display1.println(F("INVERT"));
      break;

    case 141:
      paramMode = 1;
      display1.setTextColor(BLACK, WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 17 "));
      display1.setTextColor(WHITE);
      display1.setTextSize(3);
      display1.setCursor(34, 17);
      display1.println(F("ENV"));
      display1.setCursor(10, 43);
      display1.println(F("INVERT"));
      break;

    case 133:
      paramMode = 1;
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 18 "));
      display1.setTextSize(3);
      display1.setCursor(12, 17);
      display1.println(F("SOURCE"));
      display1.setCursor(38, 43);
      display1.println(F("MIX"));
      break;

    case 261:
      paramMode = 1;
      display1.setTextColor(BLACK, WHITE);
      display1.setTextSize(1);
      display1.setCursor(20, 0);
      display1.println(F(" Parameter 18 "));
      display1.setTextColor(WHITE);
      display1.setTextSize(3);
      display1.setCursor(12, 17);
      display1.println(F("SOURCE"));
      display1.setCursor(38, 43);
      display1.println(F("MIX"));
      break;

    case 175:
    case 147:
      paramMode = 1;
      display1.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(1);
      display1.setCursor(14, 0);
      display1.println(F(" System Parameter "));
      display1.setTextSize(3);
      display1.setCursor(30, 17);
      display1.println(F("MIDI"));
      display1.setCursor(30, 43);
      display1.println(F("CHAN"));
      break;

  }

}

//
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//


void displayRightScreen(uint8_t digit3, uint8_t digit4, uint8_t prevdigit3, uint8_t prevdigit4)
{


  screenType2 = (charstable[2] + charstable[3]);
  switch (screenType2)
  {
    case 184:
    case 191:
    case 198:
    case 0:
      switch (charstable[2])
      {
        case 92:
        case 99:
          messageMode = 1;

          display1.clearDisplay(); // Clear display buffer
          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(4, 0);
          display1.println(F("Global Midi Settings"));
          display1.setTextColor(WHITE);
          display1.setTextSize(2);
          display1.setCursor(4, 14);
          display1.println(F("PGRM"));
          display1.setCursor(4, 30);
          display1.println(F("CHNG"));
          display1.setCursor(68, 14);
          display1.println(F("HOLD"));
          display1.setCursor(64, 30);
          display1.println(F("PEDAL"));
          display2.clearDisplay(); // Clear display buffer
          display2.setTextColor(WHITE);
          display2.setTextSize(1);
          display2.setCursor(4, 0);
          display2.println(F("Global Midi Settings"));
          display2.setTextColor(WHITE);
          display2.setTextSize(2);
          display2.setCursor(0, 14);
          display2.println(F("PITCH"));
          display2.setCursor(4, 30);
          display2.println(F("BEND"));
          display2.setCursor(76, 14);
          display2.println(F("MOD"));
          display2.setCursor(64, 30);
          display2.println(F("WHEEL"));
          if (charstable[0] == 92)
          {
            display1.setCursor(10, 48);
            display1.println(F("OFF"));
          }
          if (charstable[0] == 99)
          {
            display1.setCursor(16, 48);
            display1.println(F("ON"));
          }
          if (charstable[1] == 92)
          {
            display1.setCursor(76, 48);
            display1.println(F("OFF"));
          }
          if (charstable[1] == 99)
          {
            display1.setCursor(82, 48);
            display1.println(F("ON"));
          }

          if (charstable[2] == 92)
          {
            display2.setCursor(10, 48);
            display2.println(F("OFF"));
          }
          if (charstable[2] == 99)
          {
            display2.setCursor(16, 48);
            display2.println(F("ON"));
          }
          if (charstable[3] == 92)
          {
            display2.setCursor(76, 48);
            display2.println(F("OFF"));
          }
          if (charstable[3] == 99)
          {
            display2.setCursor(82, 48);
            display2.println(F("ON"));
          }
          display1.display();
          display2.display();
          break;

        case 0:
          messageMode = 1;
          display1.clearDisplay(); // Clear display buffer

          display2.clearDisplay();

          display1.setTextColor(WHITE);
          display1.setTextSize(1);
          display1.setCursor(4, 0);
          display1.println(F("Global Midi Settings"));
          display1.setTextColor(WHITE);
          display1.setTextSize(2);
          display1.setCursor(4, 14);
          display1.println(F("MIDI"));
          display1.setCursor(4, 30);
          display1.println(F("CTRL"));
          display1.setCursor(68, 14);
          display1.println(F("PG200"));
          display1.setCursor(72, 30);
          display1.println(F("MIDI"));

          if (charstable[0] == 92)
          {
            display1.setCursor(10, 48);
            display1.println(F("OFF"));
          }
          if (charstable[0] == 99)
          {
            display1.setCursor(16, 48);
            display1.println(F("ON"));
          }
          if (charstable[1] == 92)
          {
            display1.setCursor(76, 48);
            display1.println(F("OFF"));
          }
          if (charstable[1] == 99)
          {
            display1.setCursor(82, 48);
            display1.println(F("ON"));
          }
          display1.display();
          display2.display();
          break;

        default:
          render_digit_right(18, 22, prevdigit3, BLACK);
          render_digit_right(18, 22, digit3, WHITE);
          render_digit_right(78, 22, prevdigit4, BLACK);
          render_digit_right(78, 22, digit4, WHITE);
          break;
      }
      return;

    case 128: // -- clear the right screen
      messageMode = 1;
      display1.clearDisplay();
      display2.clearDisplay();
      display1.setTextColor(WHITE);
      display1.setTextSize(1);
      display1.setCursor(14, 0);
      display1.println(F(" System Message "));
      display1.setTextSize(3);
      display1.setCursor(8, 17);
      display1.println(F("SYSTEM"));
      display1.setCursor(2, 43);
      display1.println(F("MESSAGE"));
      display1.display();
      display2.display();
      break;

    case 121: // -C

      clearRightScreenValues();
      display2.setTextColor(WHITE);
      display2.setTextSize(2);
      display2.setCursor(32, 0);
      display2.println(F("ACTION"));
      display2.setTextSize(4);
      display2.setCursor(16, 21);
      display2.println(F("COPY"));
      display2.display();
      delay(2500);
      break;

    case 179: // -P
      clearRightScreenValues();
      display2.setTextColor(WHITE);
      display2.setTextSize(2);
      display2.setCursor(32, 0);
      display2.println(F("ACTION"));
      display2.setTextSize(4);
      display2.setCursor(16, 21);
      display2.println(F("PROT"));
      display2.display();
      delay(2500);
      break;

    case 125: // -G
      if ((charstable[0]) == 64 && (charstable[1]) == 64 )
      {
        clearRightScreenValues();
        display2.setTextColor(WHITE);
        display2.setTextSize(2);
        display2.setCursor(32, 0);
        display2.println(F("ACTION"));
        display2.setTextSize(4);
        display2.setCursor(16, 21);
        display2.println(F("GOOD"));
        display2.display();
        delay(2500);
      }
      else
      {
        render_digit_right(18, 22, prevdigit3, BLACK);
        render_digit_right(18, 22, digit3, WHITE);
        render_digit_right(78, 22, prevdigit4, BLACK);
        render_digit_right(78, 22, digit4, WHITE);
      }
      break;

    case 185: // -E
      clearRightScreenValues();
      display2.setTextColor(WHITE);
      display2.setTextSize(2);
      display2.setCursor(32, 0);
      display2.println(F("ACTION"));
      display2.setTextSize(4);
      display2.setCursor(12, 21);
      display2.println(F("ERROR"));
      display2.display();
      delay(2500);
      break;

    case 88:
      clearRightScreenValues();
      if (charstable[2] == 84 && charstable[3] == 4)
      {
        setValueDefaults();
        display2.setTextSize(4);
        display2.setCursor(22, 21);
        display2.println(F("OMNI"));
      }
      if (charstable[2] == 9 && charstable[3] == 79)
      {
        display2.clearDisplay();
        display2.setTextColor(WHITE);
        display2.setTextSize(2);
        display2.setCursor(38, 0);
        display2.println(F("PATCH"));
        render_digit_right(18, 22, prevdigit3, BLACK);
        render_digit_right(18, 22, digit3, WHITE);
        render_digit_right(78, 22, prevdigit4, BLACK);
        render_digit_right(78, 22, digit4, WHITE);
      }
      break;

    case 99:
      switch (screenType1)
      {
        case 13:
        case 141:
        case 204:
        case 332:
          clearRightScreenValues();
          drawPositiveEnv();
          break;

        case 200:
        case 328:
        case 216:
        case 344:
        case 98:
        case 226:
        case 218:
        case 346:
        case 229:
        case 357:
        case 337:
        case 211:
        case 339:
          clearRightScreenValues();
          setValueDefaults();
          display2.setTextSize(4);
          display2.setCursor(42, 21);
          display2.println(F("ON"));
          break;

        case 206:
        case 334:
          clearRightScreenValues();
          setValueDefaults();
          clearRightScreenValues();
          drawEnvVCA();
          break;

        case 115:
        case 243:
          clearRightScreenValues();
          setValueDefaults();
          display2.setTextSize(4);
          display2.setCursor(4, 21);
          display2.println(F("SPEED"));
          break;

        case 188:
        case 316:
          clearRightScreenValues();
          setValueDefaults();
          display2.setTextSize(4);
          display2.setCursor(4, 21);
          display2.println(F("ABOVE"));
          break;
      }
      break;

    case 92:
      switch (screenType1)
      {
        case 13:
        case 141:
        case 204:
        case 332:
          clearRightScreenValues();
          drawNegativeEnv();
          break;

        case 200:
        case 328:
        case 216:
        case 344:
        case 98:
        case 226:
        case 218:
        case 346:
        case 229:
        case 357:
        case 337:
        case 211:
        case 339:
          clearRightScreenValues();
          setValueDefaults();
          display2.setTextSize(4);
          display2.setCursor(30, 21);
          display2.println(F("OFF"));
          break;

        case 206:
        case 334:
          clearRightScreenValues();
          setValueDefaults();
          clearRightScreenValues();
          drawGate();
          break;

        case 115:
        case 243:
          clearRightScreenValues();
          setValueDefaults();
          display2.setTextSize(4);
          display2.setCursor(20, 21);
          display2.println(F("HOLD"));
          break;

        case 188:
        case 316:
          clearRightScreenValues();
          setValueDefaults();
          display2.setTextSize(4);
          display2.setCursor(4, 21);
          display2.println(F("BELOW"));
          break;
      }
      break;
    case 72: // Iiii DCO2 wave
      switch (charstable[2])
      {
        case 52:
          clearRightScreenValues();
          drawSaw();
          break;

        case 20:
          clearRightScreenValues();
          drawSquareWave();
          break;
      }
      break;

    case 42:
      switch (charstable[2])
      {
        case 22: // iIii
          if (screenType1 == 193 || screenType1 == 321)
          {
            clearRightScreenValues();
            drawPWM2();
          }
          else
          {
            clearRightScreenValues();
            drawPWM();
          }
          break;

        case 20: // iiiI
          clearRightScreenValues();
          setValueDefaults();
          display2.setTextSize(4);
          display2.setCursor(6, 21);
          display2.println(F("NOISE"));
          break;
      }
      break;

    case 68: // Iii
      switch (charstable[2])
      {
        case 52: //first char Ii
          switch (screenType1)
          {
            case 182: //waveform
            case 310:
              clearRightScreenValues();
              drawSaw();
              break;

            case 108: //sync
            case 236:
              clearRightScreenValues();
              setValueDefaults();
              display2.setTextSize(4);
              display2.setCursor(30, 21);
              display2.println(F("OFF"));
              break;

            case 97: // 16'
            case 225:
            case 170:
            case 298:
              clearRightScreenValues();
              setValueDefaults();
              display2.setTextSize(4);
              display2.setCursor(38, 21);
              display2.println(F("16'"));
              break;

            case 339:
            case 211:
              clearRightScreenValues();
              drawSine();
              break;
          }
          break;

        case 20: //first char ii
          switch (screenType1)
          {
            case 182: // waveform
            case 310:
              clearRightScreenValues();
              drawSquareWave();
              break;

            case 108: // sync
            case 236:
              clearRightScreenValues();
              setValueDefaults();
              display2.setTextSize(4);
              display2.setCursor(8, 21);
              display2.println(F("METAL"));
              break;

            case 97: // 8'
            case 225:
            case 170:
            case 298:
              clearRightScreenValues();
              setValueDefaults();
              display2.setTextSize(4);
              display2.setCursor(52, 21);
              display2.println(F("4'"));
              break;

            case 339:
            case 211:
              clearRightScreenValues();
              setValueDefaults();
              display2.setTextSize(4);
              display2.setCursor(18, 21);
              display2.println(F("RAND"));
              break;
          }
          break;
      }
      break;

    case 38: // iIi
      switch (screenType1)
      {
        case 182: //waveform
        case 310:
          clearRightScreenValues();
          drawPWM();
          break;

        case 108: //sync
        case 236:
          clearRightScreenValues();
          setValueDefaults();
          display2.setTextSize(4);
          display2.setCursor(12, 21);
          display2.println(F("SYNC"));
          break;

        case 97: // 8'
        case 225:
        case 170:
        case 298:
          clearRightScreenValues();
          setValueDefaults();
          display2.setTextSize(4);
          display2.setCursor(52, 21);
          display2.println(F("8'"));
          break;

        case 339:
        case 211:
          clearRightScreenValues();
          drawSquareWave();
          break;
      }
      break;

    default:
      if (paramMode == 1)
      {
        display2.clearDisplay();
        setValueDefaults();
      }
      else
      {
        display2.clearDisplay();
        display2.setTextColor(WHITE);
        display2.setTextSize(2);
        display2.setCursor(38, 0);
        display2.println(F("PATCH"));
      }
      render_digit_right(18, 22, prevdigit3, BLACK);
      render_digit_right(18, 22, digit3, WHITE);
      render_digit_right(78, 22, prevdigit4, BLACK);
      render_digit_right(78, 22, digit4, WHITE);
      break;
  }

}


void convertData(uint8_t data)
{

  switch (data)
  {
    case 6:
      digit = 1;
      break;
    case 91:
      digit = 2;
      break;
    case 79:
      digit = 3;
      break;
    case 102:
      digit = 4;
      break;
    case 109:
      digit = 5;
      break;
    case 125:
      digit = 6;
      break;
    case 7:
      digit = 7;
      break;
    case 127:
      digit = 8;
      break;
    case 111:
      digit = 9;
      break;
    case 63:
      digit = 0;
      break;
    case 64: // -
      digit = 10;
      break;
    case 0:
      digit = 11;  // blank
      break;
    case 57:
      digit = 12; // C
      break;
    case 118:
      digit = 13; // H
      break;
    case 134:  // 1.
      digit = 14;
      break;
    case 219:  // 2.
      digit = 15;
      break;
    case 207: // 3.
      digit = 16;
      break;
    case 230: // 4.
      digit = 17;
      break;
    case 237: // 5.
      digit = 18;
      break;
    case 253: // 6.
      digit = 19;
      break;
    case 135: // 7.
      digit = 20;
      break;
    case 255: // 8.
      digit = 21;
      break;
    case 254: // 9.
      digit = 22;
      break;
    case 115: // P
      digit = 23;
      break;
    case 84: // n
      digit = 24;
      break;
    case 4: // |
      digit = 25;
      break;
    case 92: // o
      digit = 26;
      break;
    case 55: // M
      digit = 27;
      break;
    case 124: // b
      digit = 28;
      break;
    case 52:
      digit = 29;
      break;
    case 16:
      digit = 30;
      break;
    case 22:
      digit = 31;
      break;
    case 20:
      digit = 32;
      break;
    case 48: //
      digit = 33;
      break;
    case 99: //
      digit = 34;
      break;
    case 120: // t
      digit = 35;
      break;
    case 9: // =
      digit = 36;
      break;
    case 80: // =
      digit = 37; // r
      break;
    case 121: // E
      digit = 38;
      break;
    case 61: // G
      digit = 39;
      break;
    case 94: // d
      digit = 40;
      break;
  }
}

void printStates(uint8_t n) {
  Serial.print("Mux: ");
  Serial.println(n);

  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(bitRead(pinStates[n], i));
  }
  Serial.println("");
  Serial.println("------------");
}

void render_digits_left(uint8_t pos_x, uint8_t pos_y,
                        uint8_t spacing_x, uint16_t digit, uint8_t color) {
  uint8_t digit_array[] = {digit / 100 % 10, digit / 10 % 10, digit % 10};
  if (digit > 99) {
    render_digit_left(pos_x, pos_y, digit_array[0], color);
  }
  if (digit > 9) {
    render_digit_left(pos_x + spacing_x, pos_y, digit_array[1], color);
  }
  render_digit_left(pos_x + (spacing_x * 4), pos_y, digit_array[2], color);
}

void render_digits_right(uint8_t pos_x, uint8_t pos_y,
                         uint8_t spacing_x, uint16_t digit, uint8_t color) {
  uint8_t digit_array[] = {digit / 100 % 10, digit / 10 % 10, digit % 10};
  if (digit > 99) {
    render_digit_right(pos_x, pos_y, digit_array[0], color);
  }
  if (digit > 9) {
    render_digit_right(pos_x + spacing_x, pos_y, digit_array[1], color);
  }
  render_digit_right(pos_x + (spacing_x * 4), pos_y, digit_array[2], color);
}


void render_digit_left(uint8_t pos_x, uint8_t pos_y,
                       uint8_t digit, uint8_t color) {
  // loop through 7 segments
  for (uint8_t i = 0; i < 8; i++) {
    bool seg_on = digit_array[digit][i];
    // if seg_on is true draw segment
    if (seg_on) {
      switch (i) {
        case 0:
          display1.fillRoundRect(2 + pos_x, 0 + pos_y, 22, 5, 2, color); // SEG a
          break;
        case 1:
          display1.fillRoundRect(22 + pos_x, 2 + pos_y, 5, 14, 2, color); // SEG b
          break;
        case 2:
          display1.fillRoundRect(22 + pos_x, 17 + pos_y, 5, 14, 2, color); // SEG c
          break;
        case 3:
          display1.fillRoundRect(2 + pos_x, 30 + pos_y, 22, 5, 2, color); // SEG d
          break;
        case 4:
          display1.fillRoundRect(0 + pos_x, 17 + pos_y, 5, 14, 2, color); // SEG e
          break;
        case 5:
          display1.fillRoundRect(0 + pos_x, 2 + pos_y, 5, 14, 2, color); // SEG f
          break;
        case 6:
          display1.fillRoundRect(2 + pos_x, 15 + pos_y, 22, 5, 2, color); // SEG g
          break;
        case 7:
          display1.fillRoundRect(32 + pos_x, 28 + pos_y, 7, 7, 5, color); // DP
      }
      seg_on = false;
    }
  }
}

void render_digit_right(uint8_t pos_x, uint8_t pos_y,
                        uint8_t digit, uint8_t color) {
  // loop through 7 segments
  for (uint8_t i = 0; i < 8; i++) {
    bool seg_on = digit_array[digit][i];
    // if seg_on is true draw segment
    if (seg_on) {
      switch (i) {
        case 0:
          display2.fillRoundRect(2 + pos_x, 0 + pos_y, 22, 5, 2, color); // SEG a
          break;
        case 1:
          display2.fillRoundRect(22 + pos_x, 2 + pos_y, 5, 14, 2, color); // SEG b
          break;
        case 2:
          display2.fillRoundRect(22 + pos_x, 17 + pos_y, 5, 14, 2, color); // SEG c
          break;
        case 3:
          display2.fillRoundRect(2 + pos_x, 30 + pos_y, 22, 5, 2, color); // SEG d
          break;
        case 4:
          display2.fillRoundRect(0 + pos_x, 17 + pos_y, 5, 14, 2, color); // SEG e
          break;
        case 5:
          display2.fillRoundRect(0 + pos_x, 2 + pos_y, 5, 14, 2, color); // SEG f
          break;
        case 6:
          display2.fillRoundRect(2 + pos_x, 15 + pos_y, 22, 5, 2, color); // SEG g
          break;
        case 7:
          display2.fillRoundRect(30 + pos_x, 18 + pos_y, 5, 3, 4, color); // DP
          break;
      }
      seg_on = false;
    }
  }
}

void setValueDefaults()
{
  display2.setTextColor(WHITE);
  display2.setTextSize(2);
  display2.setCursor(36, 0);
  display2.println(F("VALUE"));
}

void drawSine()
{
  display2.clearDisplay(); // Clear display buffer
  setValueDefaults();
  int hCenter = 33;                               // horizontal center of 64 / 2 = 32
  int Radius = 11;                                // radius of circle
  const float Pi = 3.14159265359;
  for (int i = 24; i < 110; i++) {                              // draws 120 pixels per loop

    float Angle = i * 15;                                       // 120 X 3 = 360°
    int a = (hCenter + (sin(Angle * (Pi / 180)) * Radius));    // Pi/180 converts degrees to radians


    display2.drawPixel(i, a, WHITE);
    display2.drawPixel(i, a + 1, WHITE);
    display2.drawPixel(i, a + 2, WHITE);
    display2.drawPixel(i, a - 1, WHITE);
    display2.drawPixel(i, a - 2, WHITE);
    display2.drawPixel(i + 1, a, WHITE);
    display2.drawPixel(i + 2, a, WHITE);
    display2.drawPixel(i - 1, a, WHITE);
    display2.drawPixel(i - 2, a, WHITE);
  }

  display2.setTextSize(1);
  display2.setCursor(52, 52);
  display2.println(F("Sine"));
}

void drawSaw()
{
  display2.clearDisplay(); // Clear display buffer
  setValueDefaults();


  display2.fillRoundRect(22, 44, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(24, 42, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(26, 40, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(28, 38, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(30, 36, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(32, 34, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(34, 32, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(36, 30, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(38, 28, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(40, 26, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(42, 24, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(44, 22, 5, 26, 2, WHITE);
  display2.fillRoundRect(22 + 22, 44, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(24 + 22, 42, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(26 + 22, 40, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(28 + 22, 38, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(30 + 22, 36, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(32 + 22, 34, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(34 + 22, 32, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(36 + 22, 30, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(38 + 22, 28, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(40 + 22, 26, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(42 + 22, 24, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(66, 22, 5, 26, 2, WHITE);
  display2.fillRoundRect(22 + 44, 44, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(24 + 44, 42, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(26 + 44, 40, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(28 + 44, 38, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(30 + 44, 36, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(32 + 44, 34, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(34 + 44, 32, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(36 + 44, 30, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(38 + 44, 28, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(40 + 44, 26, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(42 + 44, 24, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(88, 22, 5, 26, 2, WHITE);
  display2.fillRoundRect(22 + 66, 44, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(24 + 66, 42, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(26 + 66, 40, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(28 + 66, 38, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(30 + 66, 36, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(32 + 66, 34, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(34 + 66, 32, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(36 + 66, 30, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(38 + 66, 28, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(40 + 66, 26, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(42 + 66, 24, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(110, 22, 5, 26, 2, WHITE);
  display2.setTextSize(1);
  display2.setCursor(42, 52);
  display2.println(F("Sawtooth"));
}

void drawPositiveEnv()
{
  display2.clearDisplay(); // Clear display buffer
  setValueDefaults();


  display2.fillRoundRect(22, 44, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(24, 42, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(26, 40, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(28, 38, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(30, 36, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(32, 34, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(34, 32, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(36, 30, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(38, 28, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(40, 26, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(42, 24, 5, 5, 4, WHITE); // DOT

  display2.fillRoundRect(44, 26, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(46, 28, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(48, 30, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(50, 32, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(52, 34, 5, 5, 4, WHITE); // DOT

  display2.fillRoundRect(52, 34, 28, 5, 2, WHITE);

  display2.fillRoundRect(78, 34, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(80, 36, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(82, 38, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(84, 40, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(86, 42, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(88, 44, 5, 5, 4, WHITE); // DOT

  display2.setTextSize(1);
  display2.setCursor(28, 52);
  display2.println(F("Positive Env"));
}

void drawEnvVCA()
{
  display2.clearDisplay(); // Clear display buffer
  setValueDefaults();


  display2.fillRoundRect(22, 44, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(24, 42, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(26, 40, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(28, 38, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(30, 36, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(32, 34, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(34, 32, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(36, 30, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(38, 28, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(40, 26, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(42, 24, 5, 5, 4, WHITE); // DOT

  display2.fillRoundRect(44, 26, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(46, 28, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(48, 30, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(50, 32, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(52, 34, 5, 5, 4, WHITE); // DOT

  display2.fillRoundRect(52, 34, 28, 5, 2, WHITE);

  display2.fillRoundRect(78, 34, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(80, 36, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(82, 38, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(84, 40, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(86, 42, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(88, 44, 5, 5, 4, WHITE); // DOT

  display2.setTextSize(1);
  display2.setCursor(28, 52);
  display2.println(F("Envelope VCA"));
}

void drawGate()
{
  display2.clearDisplay(); // Clear display buffer
  setValueDefaults();


  display2.fillRoundRect(30, 44, 26, 5, 2, WHITE);
  display2.fillRoundRect(52, 22, 26, 5, 2, WHITE);
  display2.fillRoundRect(52, 22, 5, 26, 2, WHITE);
  display2.fillRoundRect(74, 44, 26, 5, 2, WHITE);
  display2.fillRoundRect(74, 22, 5, 26, 2, WHITE);
  display2.setTextSize(1);
  display2.setCursor(38, 52);
  display2.println(F("Gated VCA"));
}

void drawNegativeEnv()
{
  display2.clearDisplay(); // Clear display buffer
  setValueDefaults();


  display2.fillRoundRect(22, 22, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(24, 24, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(26, 26, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(28, 28, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(30, 30, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(32, 32, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(34, 34, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(36, 36, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(38, 38, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(40, 40, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(42, 42, 5, 5, 4, WHITE); // DOT

  display2.fillRoundRect(44, 42, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(46, 40, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(48, 38, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(50, 36, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(52, 34, 5, 5, 4, WHITE); // DOT

  display2.fillRoundRect(52, 34, 28, 5, 2, WHITE);

  display2.fillRoundRect(78, 34, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(80, 32, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(82, 30, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(84, 28, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(86, 26, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(88, 24, 5, 5, 4, WHITE); // DOT

  display2.setTextSize(1);
  display2.setCursor(28, 52);
  display2.println(F("Negative Env"));
}

void drawSquareWave()
{
  display2.clearDisplay(); // Clear display buffer
  setValueDefaults();
  display2.fillRoundRect(22, 22, 26, 5, 2, WHITE);
  display2.fillRoundRect(22, 22, 5, 28, 2, WHITE);
  display2.fillRoundRect(44, 44, 26, 5, 2, WHITE);
  display2.fillRoundRect(44, 22, 5, 26, 2, WHITE);
  display2.fillRoundRect(66, 22, 26, 5, 2, WHITE);
  display2.fillRoundRect(66, 22, 5, 26, 2, WHITE);
  display2.fillRoundRect(88, 44, 26, 5, 2, WHITE);
  display2.fillRoundRect(88, 22, 5, 26, 2, WHITE);
  display2.setTextSize(1);
  display2.setCursor(38, 52);
  display2.println(F("Squarewave"));
}

void drawPWM()
{
  display2.clearDisplay(); // Clear display buffer
  setValueDefaults();
  display2.fillRoundRect(22, 22, 13, 5, 2, WHITE);
  display2.fillRoundRect(22, 22, 5, 28, 2, WHITE);
  display2.fillRoundRect(33, 44, 38, 5, 2, WHITE);
  display2.fillRoundRect(33, 22, 5, 26, 2, WHITE);
  display2.fillRoundRect(66, 22, 13, 5, 2, WHITE);
  display2.fillRoundRect(66, 22, 5, 26, 2, WHITE);
  display2.fillRoundRect(77, 44, 38, 5, 2, WHITE);
  display2.fillRoundRect(77, 22, 5, 26, 2, WHITE);
  display2.setTextSize(1);
  display2.setCursor(40, 52);
  display2.println(F("Pulse Wave"));
}

void drawPWM2()
{
  display2.clearDisplay(); // Clear display buffer
  setValueDefaults();
  display2.fillRoundRect(22, 22, 13, 5, 2, WHITE);
  display2.fillRoundRect(22, 22, 5, 28, 2, WHITE);
  display2.fillRoundRect(33, 44, 38, 5, 2, WHITE);
  display2.fillRoundRect(33, 22, 5, 26, 2, WHITE);
  display2.fillRoundRect(66, 22, 13, 5, 2, WHITE);
  display2.fillRoundRect(66, 22, 5, 26, 2, WHITE);
  display2.fillRoundRect(77, 44, 38, 5, 2, WHITE);
  display2.fillRoundRect(77, 22, 5, 26, 2, WHITE);
  display2.fillRoundRect(39, 22, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(44, 22, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(49, 22, 5, 20, 4, WHITE); // DOT
  display2.fillRoundRect(39 + 44, 22, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(44 + 44, 22, 5, 5, 4, WHITE); // DOT
  display2.fillRoundRect(49 + 44, 22, 5, 20, 4, WHITE); // DOT
  display2.setTextSize(1);
  display2.setCursor(32, 52);
  display2.println(F("PW Modulation"));
}
