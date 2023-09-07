/*
 * Arduino Oscilloscope using a graphic LCD Version 1.13
 * The max real time sampling rates are 15ksps with 2 channels and 308ksps with a channel.
 * The max equivalent time sampling rates is 16Msps with single channel.
 * Copyright (c) 2009-2014, Noriaki Mitsunaga
 * Copyright (c) 2020-2022, Modified by Siliconvalley4066 on 2022.3.29
 */

//#define TFTSHIELD_ID  0x9341
//#define TFTSHIELDROT  3
//#define TFTCOLOR_BGR
#define TFTSHIELD_ID  0x4747
#define TFTSHIELDROT  1
//#define ADAFRUIT_GFX
#define SAVE_EEPROM
//#define START_SCREEN
//#define DOT_GRID
#if defined(ARDUINO_AVR_PRO) || defined(ARDUINO_AVR_NANO)
#define DISPLAY_AC
#define CALIB_PULSE
#endif

#ifdef ADAFRUIT_GFX
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#else
#include <MCUFRIEND_kbv.h>
#endif

#include <TouchScreen.h>
#include <EEPROM.h>
#include <fix_fft.h>
#define FFT_N 256

#ifdef ADAFRUIT_GFX
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
#else
MCUFRIEND_kbv tft;       // hard-wired for UNO shields anyway.
#endif

#if TFTSHIELD_ID != 0x4747
#define YP A1  // must be an analog pin, use "An" notation!  A1 for shield
#define XM A2  // must be an analog pin, use "An" notation!  A2 for shield
#define YM 7   // can be a digital pin-----uno=9 mega=23      7 for shield
#define XP 6   // can be a digital pin-----uno=8 mega=22      6 for shield
#define TS_MINX 930
#define TS_MINY 940
#define TS_MAXX 150
#define TS_MAXY 210
#else
const int XP=8,XM=A2,YP=A3,YM=9; //240x320 ID=0x4747
const int TS_MINX=201,TS_MAXX=923,TS_MAXY=914,TS_MINY=186;
#endif

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);   

#define MINPRESSURE 200
#define MAXPRESSURE 1000

float waveFreq;                // frequency (Hz)
float waveDuty;                // duty ratio (%)
int dataMin;                   // buffer minimum value (smallest=0)
int dataMax;                   //        maximum value (largest=1023)
int dataAve;                   // 10 x average value (use 10x value to keep accuracy. so, max=10230)
int saveTimer;                 // remaining time for saving EEPROM
int timeExec = 100;            // approx. execution time of current range setting (ms)

const int LCD_WIDTH = 320;
const int LCD_HEIGHT = 240;
const int LCD_YMAX = LCD_HEIGHT - 1;
const int SAMPLES = 300;
const int DOTS_DIV = 30;

const int ad_ch0 = 6;                   // Analog 6 pin for channel 0
const int ad_ch1 = 7;                   // Analog 7 pin for channel 1
const long VREF[] = {150, 300, 750, 1500, 3000}; // reference voltage 5.0V ->  150 :   1V/div range (100mV/dot)
                                        // It means 5.0 * DOTS_DIV = 150. Use 4.9 if reference voltage is 4.9[V]
                                        //                        -> 300 : 0.5V/div
                                        //                        -> 750 : 0.2V/div
                                        //                        ->1500 : 100mV/div
                                        //                       -> 3000 :  50mV/div
//const int MILLIVOL_per_dot[] = {33, 17, 6, 3, 2}; // mV/dot
#define CALPIN 10
#ifdef DISPLAY_AC
#define CH1DCSW 11
#define CH2DCSW 12
const int ac_offset[] = {307, -102, -348, -430, -470};
#endif
const int MODE_ON = 0;
const int MODE_INV = 1;
const int MODE_OFF = 2;
const char ModesN[3][5] PROGMEM = {"NORM", "INV", "OFF"};
const char * const Modes[3] PROGMEM = {ModesN[0], ModesN[1], ModesN[2]};
#define TRIG_AUTO 0
#define TRIG_NORM 1
#define TRIG_SCAN 2
#define TRIG_ONE  3
const char TRIG_ModesN[4][5] PROGMEM = {"Auto", "Norm", "Scan", "One"};
const char * const TRIG_Modes[4] PROGMEM = {TRIG_ModesN[0], TRIG_ModesN[1], TRIG_ModesN[2], TRIG_ModesN[3]};
#define TRIG_E_UP 0
#define TRIG_E_DN 1
#define RATE_MIN 0
#define RATE_MAX 23
const char RN[24][5] PROGMEM = {"97us", "177u", "200u", "500u", "1ms",  "2ms",  "5ms", "10ms", "20ms", "50ms", "0.1s", "0.2s", "0.5s", "  1s", "  2s", "  5s", " 10s",
                                "1.9u", "3.8u", "9.4u", "19us", "38us", "94us", "188u"};
const char * const Rates[24] PROGMEM = {RN[0], RN[1], RN[2], RN[3], RN[4], RN[5], RN[6], RN[7], RN[8], RN[9], RN[10], RN[11], RN[12], RN[13], RN[14], RN[15], RN[16],
                                        RN[17], RN[18], RN[19], RN[20], RN[21], RN[22], RN[23]};
const unsigned long HREF[] PROGMEM = {97, 177, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000, 2000000, 5000000, 10000000};
#define RANGE_MIN 0
#define RANGE_MAX 4
const char RangesN[5][5] PROGMEM = {" 1V ", "0.5V", "0.2V", "0.1V", "50mV"};
const char * const Ranges[5] PROGMEM = {RangesN[0], RangesN[1], RangesN[2], RangesN[3], RangesN[4]};
byte data[4][SAMPLES];                   // keep twice of the number of channels to make it a double buffer
byte sample=0;                           // index for double buffer

#define SEL_NONE    0
#define SEL_RANGE1  1
#define SEL_RANGE2  2
#define SEL_RATE    3
#define SEL_TGSRC   4
#define SEL_EDGE    5
#define SEL_TGLVL   6
#define SEL_TGMODE  7
#define SEL_OFST1   8
#define SEL_OFST2   9

///////////////////////////////////////////////////////////////////////////////////////////////
// Define colors here
// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define GREEN   0x07E0
#define MAGENTA 0xF81F
#define WHITE   0xFFFF
#define GRAY    0x8410
#ifndef TFTCOLOR_BGR
#define BLUE    0x001F
#define RED     0xF800
#define CYAN    0x07FF
#define YELLOW  0xFFE0
#else
#define RED     0x001F
#define BLUE    0xF800
#define YELLOW  0x07FF
#define CYAN    0xFFE0
#endif

#define	BGCOLOR  BLACK
#define	GRIDCOLOR WHITE
#define CH1COLOR  GREEN
#define CH2COLOR  YELLOW
#define FRMCOLOR  GRAY
#define VRF 5

// Declare variables and set defaults here
// Note: only ch0 is available with the 320x240 TFT LCD Shield
byte range0 = RANGE_MIN, ch0_mode = MODE_ON;  // CH0
short ch0_off = 0;
byte range1 = RANGE_MIN, ch1_mode = MODE_ON;  // CH1
short ch1_off = 0;
byte rate = 5;                                // sampling rate
byte trig_mode = TRIG_AUTO, trig_lv = 120, trig_edge = TRIG_E_UP, trig_ch = ad_ch0; // trigger settings
bool Start = true;  // Start sampling
bool fft_mode = false;
byte menu = 0;  // Default menu
byte info_mode = 1; // Text information display mode
///////////////////////////////////////////////////////////////////////////////////////////////

void setup(){
  tft.reset();
//  uint16_t identifier = tft.readID();
//  tft.begin(identifier);
  tft.begin(TFTSHIELD_ID);
  tft.setRotation(TFTSHIELDROT);
  tft.fillScreen(BGCOLOR);

#ifdef START_SCREEN
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(30, 100);
  tft.print(F("Arduino Oscillo Scope"));
  tft.setCursor(30, 120);
  tft.print(F("with TFT LCD (320x240)"));
  tft.setCursor(30, 140);
  tft.print(F("(c) 2009-2014 non"));
  tft.setCursor(30, 160);
  tft.print(F("(c) 2020-2022"));
  delay(2000);
  tft.fillScreen(BGCOLOR);
#endif

//  Serial.begin(115200);
#ifdef SAVE_EEPROM
  loadEEPROM();                         // read last settings from EEPROM
#endif
  DrawGrid();
  draw_hold_button();
  draw_updown_button();
  DrawText();
#ifdef DISPLAY_AC
  pinMode(CH1DCSW, INPUT_PULLUP);            // CH1 DC/AC sense SW
  pinMode(CH2DCSW, INPUT_PULLUP);            // CH2 DC/AC sense SW
#endif
#ifdef CALIB_PULSE
  pulse();                              // calibration pulse output
#endif
}

//void CheckSW() {
//}

//void SendData() {
//  Serial.print(Rates[rate]);
//  Serial.println(F("/div (30 samples)"));
//  for (int i=0; i<SAMPLES; i ++) {
//      Serial.print(data[sample + 0][i]*MILLIVOL_per_dot[range0]);
//      Serial.print(" ");
//      Serial.println(data[sample + 1][i]*MILLIVOL_per_dot[range1]);
//   } 
//}

#ifdef DOT_GRID
void DrawGrid() {
    for (int x=0; x<=SAMPLES; x += 2) { // Horizontal Line
      for (int y=0; y<LCD_HEIGHT; y += DOTS_DIV) {
        tft.drawPixel(x, y, GRIDCOLOR);
//        CheckSW();
      }
      tft.drawPixel(x, LCD_YMAX, GRIDCOLOR);
    }
    for (int x=0; x<=SAMPLES; x += DOTS_DIV ) { // Vertical Line
      for (int y=0; y<=LCD_HEIGHT; y += 2) {
        tft.drawPixel(x, y, GRIDCOLOR);
//        CheckSW();
      }
    }
}
#else
void DrawGrid() {
  tft.drawFastVLine(0, 0, LCD_YMAX, FRMCOLOR);       // left vertical line
  tft.drawFastVLine(SAMPLES, 0, LCD_YMAX, FRMCOLOR); // right vertical line
  tft.drawFastHLine(0, 0, SAMPLES, FRMCOLOR);        // top horizontal line
  tft.drawFastHLine(0, LCD_YMAX, SAMPLES, FRMCOLOR); // bottom horizontal line

  for (int y = 0; y < LCD_HEIGHT; y += LCD_HEIGHT / 8) {
    if (y > 0){
      tft.drawFastHLine(1, y, SAMPLES - 1, GRAY); // Draw 7 horizontal lines
    }
    for (int i = 6; i < 30; i += 6) {
      tft.drawFastHLine(SAMPLES / 2 - 3, y + i, 7, GRAY); // Draw the vertical center line ticks
    }
  }
  for (int x = 0; x < SAMPLES; x += SAMPLES / 10) {
    if (x > 0){
      tft.drawFastVLine(x, 1, LCD_HEIGHT - 2, GRAY); // Draw 9 vertical lines
    }
    for (int i = 6; i < 30; i += 6) {
      tft.drawFastVLine(x + i, LCD_HEIGHT / 2 -3, 7, GRAY); // Draw the horizontal center line ticks
    }
  }
}
#endif

void DrawText_small() {
  char str[5];
  byte line = 1;

  tft.setTextSize(1);

  TextBG(&line, 302, 3);  // HALT
  if (Start == false) {
    tft.setTextColor(RED);
    tft.print(F("HLT"));
  }

  disp_ch0(302, line);    // CH1
  line += 10;

  TextBG(&line, 296, 4);  // CH1 Range
  disp_ch0_range();

  TextBG(&line, 296, 4);  // CH1 Mode
  strcpy_P(str, (char*)pgm_read_word(&(Modes[ch0_mode])));
  if (strlen(str) < 4) {
    tft.setCursor(302, line - 10);
  }
  tft.print(str);

  disp_ch1(302, line);    // CH2
  line += 10;

  TextBG(&line, 296, 4);  // CH2 Range
  disp_ch1_range();

  TextBG(&line, 296, 4);  // CH2 Mode
  strcpy_P(str, (char*)pgm_read_word(&(Modes[ch1_mode])));
  if (strlen(str) < 4) {
    tft.setCursor(302, line - 10);
  }
  tft.print(str);

  tft.setTextColor(WHITE);
  TextBG(&line, 296, 4);  // Sweep Rate
  disp_sweep_rate();
  TextBG(&line, 296, 4);  // Trigger Mode
  disp_trig_mode();
  TextBG(&line, 308, 1);  // Trigger Edge
  disp_trig_edge();
  TextBG(&line, 302, 3);  // Trigger Source
  disp_trig_source(); 
  TextBG(&line, 302, 3);  // Trigger Level
  tft.print(trig_lv);
}

void disp_ch0(int x, int y) {
  tft.setCursor(x, y);
  if (ch0_mode == MODE_OFF)
    tft.setTextColor(GRAY);
  else
    tft.setTextColor(CH1COLOR);
  tft.print(F("CH1"));
}

void disp_ch1(int x, int y) {
  tft.setCursor(x, y);
//  if (ch1_mode == MODE_OFF || rate < 5 || 16 < rate)
  if (ch1_mode == MODE_OFF || rate < 5)
    tft.setTextColor(GRAY);
  else
    tft.setTextColor(CH2COLOR);
  tft.print(F("CH2"));
}

void disp_ch0_range() {
  char str[5];
  strcpy_P(str, (char*)pgm_read_word(&(Ranges[range0])));
  tft.print(str);
}

void disp_ch1_range() {
  char str[5];
  strcpy_P(str, (char*)pgm_read_word(&(Ranges[range1])));
  if (ch1_mode != MODE_OFF) tft.setTextColor(WHITE);  // omit this saves 28bytes code size
  tft.print(str);
}

void disp_sweep_rate() {
  char str[5];
  strcpy_P(str, (char*)pgm_read_word(&(Rates[rate])));
  tft.print(str);
}

void disp_trig_edge() {
  tft.print(trig_edge == TRIG_E_UP ? char(0x18) : char(0x19));  // up or down arrow
}

void disp_trig_source() {
  tft.print(trig_ch == ad_ch0 ? F("TG1") : F("TG2")); 
}

void disp_trig_mode() {
  char str[5];
  strcpy_P(str, (char*)pgm_read_word(&(TRIG_Modes[trig_mode])));
  tft.print(str);
}

void TextBG(byte *y, int x, byte chrs) {
  int yinc, wid, hi;
  if (info_mode == 1) {
    yinc = 20, wid = 12, hi = 16;
  } else {
    yinc = 10, wid = 6, hi = 8;
  }
  tft.fillRect(x, *y, wid * chrs - 1, hi, BGCOLOR);
  tft.setCursor(x, *y);
  *y += yinc;
}

void DrawText() {
  if (info_mode > 0 && info_mode < 5) {         // 1, 2, 3, 4
    DrawText_big();
  } else if (info_mode > 4 && info_mode < 7) {  // 5, 6
    DrawText_small();
  } else {                                      // 0 : no text
    return;
  }
  if (info_mode > 0 && (info_mode & 1) > 0 && Start && !fft_mode) {
    dataAnalize();
    measure_frequency();
    measure_voltage();
  }
}

#define BOTTOM_LINE 224

void DrawText_big() {
  char str[5];
  byte y;

  if (info_mode == 1 || info_mode == 2) {
    tft.setTextSize(2);
    y = BOTTOM_LINE;
  } else if (info_mode == 3 || info_mode == 4) {
    tft.setTextSize(1);
    y = BOTTOM_LINE + 7;
  } else {
    return;
  }
  disp_ch0(1, 1);         // CH1
#ifdef DISPLAY_AC
  display_ac_inv(1, CH1DCSW);
#endif
  if (ch0_mode == MODE_INV) tft.print(char(0x19));  // down arrow
  set_pos_color(60, 1, WHITE);      // CH1 range
  disp_ch0_range();
  tft.setCursor(132, 1);  // Rate
  set_menu_color(SEL_RATE);
  disp_sweep_rate();
  if (fft_mode)
    return;
  tft.setCursor(192, 1);  // Trigger level
  set_menu_color(SEL_TGLVL);
  if (16 < rate) {
    tft.print(F("EQIV"));
  } else if (menu == SEL_TGLVL) {
    tft.print(trig_lv);
  } else {
    tft.print(F("TGLV"));
  }
  tft.setCursor(252, 1);  // Position/Halt
  if (Start == false) {
    tft.setTextColor(RED);
    tft.print(F("HALT"));
  } else if (menu == SEL_OFST1) {
    set_menu_color(SEL_OFST1);
    tft.print(F("POS1"));
  } else if (menu == SEL_OFST2) {
    set_menu_color(SEL_OFST2);
    tft.print(F("POS2"));
  } else {
    tft.setTextColor(GRAY);
    tft.print(F("VPOS"));
  }

  disp_ch1(1, y);         // CH2
#ifdef DISPLAY_AC
  display_ac_inv(y, CH2DCSW);
#endif
  if (ch1_mode == MODE_INV) tft.print(char(0x19));  // down arrow
  tft.setCursor(60, y);   // CH2 range
  disp_ch1_range();
  set_pos_color(132, y, WHITE); // Trigger souce
  disp_trig_source(); 
  tft.setCursor(192, y);  // Trigger edge
  disp_trig_edge();
  tft.setCursor(252, y);  // Trigger mode
  disp_trig_mode();
}

void display_ac_inv(byte y, byte sw) {
  byte h, x;
  if (info_mode == 3 || info_mode == 4) {
    h = 7, x = 18;
  } else {
    h = 15, x = 37;
  }
  tft.fillRect(x, y, 12, h, BGCOLOR); // clear AC/DC Inv
  if (digitalRead(sw) == LOW) {
    tft.print('~');
    if (info_mode == 1 || info_mode == 2) {
      tft.setCursor(37, y); // back space
    }
  }
}

void set_pos_color(int x, int y, int color) {
  tft.setTextColor(color);
  tft.setCursor(x, y);
}

void set_menu_color(byte sel) {
  if (menu == sel) {
    tft.setTextColor(CYAN);
  } else {
    tft.setTextColor(WHITE);
  }
}

#ifdef DOT_GRID
void DrawGrid(int x) {
  if ((x % 2) == 0) {
    for (int y=0; y<LCD_HEIGHT; y += DOTS_DIV)
      tft.drawPixel(x, y, GRIDCOLOR);
    tft.drawPixel(x, LCD_YMAX, GRIDCOLOR);
  }
  if ((x % DOTS_DIV) == 0)
    for (int y=0; y<LCD_HEIGHT; y += 2)
      tft.drawPixel(x, y, GRIDCOLOR);
}
#else
void DrawGrid(int x) {
//  if (x == SAMPLES)
//    tft.drawFastVLine(x, 0, LCD_YMAX, FRMCOLOR);  // right vertical line
  tft.drawPixel(x, 0, FRMCOLOR);                    // top horizontal line
  tft.drawPixel(x, LCD_YMAX, FRMCOLOR);             // bottom horizontal line

  if (x > 0)
    for (int y = LCD_HEIGHT / 8; y < LCD_YMAX; y += LCD_HEIGHT / 8)
      tft.drawPixel(x, y, GRAY);                        // Draw 7 horizontal lines
  if (x > SAMPLES / 2 - 4 && x < SAMPLES / 2 + 4 && x != (SAMPLES / 2)) {
    for (int y = 0; y < LCD_YMAX; y += LCD_HEIGHT / 8)
      for (int i = 1; i < 5; ++i)
        tft.drawPixel(x, y + i * 6, GRAY);            // Draw the vertical center line ticks
  }
  if (x == 0) {
    tft.drawFastVLine(x, 0, LCD_YMAX, FRMCOLOR);      // left vertical line
  } else if (x % (SAMPLES / 10) == 0){
    tft.drawFastVLine(x, 1, LCD_HEIGHT - 2, GRAY);    // Draw 9 vertical lines
  } else if((x % (SAMPLES / 10)) % 6 == 0) {
    tft.drawFastVLine(x, LCD_HEIGHT / 2 -3, 7, GRAY); // Draw the horizontal center line ticks
  }
}
#endif

void ClearAndDrawGraph() {
  int clear;
  byte *p1, *p2, *p3, *p4, *p5, *p6, *p7, *p8;

  if (sample == 0)
    clear = 2;
  else
    clear = 0;
  p1 = data[clear+0];
  p2 = p1 + 1;
  p3 = data[sample+0];
  p4 = p3 + 1;
  p5 = data[clear+1];
  p6 = p5 + 1;
  p7 = data[sample+1];
  p8 = p7 + 1;
  for (int x=0; x<(SAMPLES-1); x++) {
    DrawGrid(x);
    if (ch0_mode != MODE_OFF) {
      tft.drawLine(x, LCD_YMAX-*p1++, x+1, LCD_YMAX-*p2++, BGCOLOR);
      tft.drawLine(x, LCD_YMAX-*p3++, x+1, LCD_YMAX-*p4++, CH1COLOR);
    }
    if (ch1_mode != MODE_OFF && rate > 4) {
      tft.drawLine(x, LCD_YMAX-*p5++, x+1, LCD_YMAX-*p6++, BGCOLOR);
      tft.drawLine(x, LCD_YMAX-*p7++, x+1, LCD_YMAX-*p8++, CH2COLOR);
    }
//    CheckSW();
  }
}

void ClearAndDrawGraphMag(int mag) {
  int clear;
  byte *p1, *p2, *p3, *p4;

  if (sample == 0)
    clear = 2;
  else
    clear = 0;
  p1 = data[clear+0];
  p2 = p1 + 1;
  p3 = data[sample+0];
  p4 = p3 + 1;
  for (int x=0; x<(SAMPLES-mag); x+=mag) {
    DrawGrid(x);
    tft.drawLine(x, LCD_YMAX-*p1++, x+mag, LCD_YMAX-*p2++, BGCOLOR);
    tft.drawLine(x, LCD_YMAX-*p3++, x+mag, LCD_YMAX-*p4++, CH1COLOR);
//    CheckSW();
  }
}

void ClearAndDrawDot(int i) {
  int clear = 0;

  if (i <= 1)
    return;
  if (sample == 0)
    clear = 2;
  DrawGrid(i);
  if (ch0_mode != MODE_OFF) {
    tft.drawLine(i-1, LCD_YMAX-data[clear+0][i-1], i, LCD_YMAX-data[clear+0][i], BGCOLOR);
    tft.drawLine(i-1, LCD_YMAX-data[sample+0][i-1], i, LCD_YMAX-data[sample+0][i], CH1COLOR);
  }
  if (ch1_mode != MODE_OFF) {
    tft.drawLine(i-1, LCD_YMAX-data[clear+1][i-1], i, LCD_YMAX-data[clear+1][i], BGCOLOR);
    tft.drawLine(i-1, LCD_YMAX-data[sample+1][i-1], i, LCD_YMAX-data[sample+1][i], CH2COLOR);
  }
}

//void DrawGraph() {
//   for (int x=0; x<SAMPLES; x++) {
//     tft.drawPixel(x, LCD_HEIGHT-data[sample+0][x], CH1COLOR);
//     tft.drawPixel(x, LCD_HEIGHT-data[sample+1][x], CH2COLOR);
//  }
//}
//
//void ClearGraph() {
//  int clear = 0;
//  
//  if (sample == 0)
//    clear = 2;
//  for (int x=0; x<SAMPLES; x++) {
//     tft.drawPixel(x, LCD_HEIGHT-data[clear+0][x], BGCOLOR);
//     tft.drawPixel(x, LCD_HEIGHT-data[clear+1][x], BGCOLOR);
//  }
//}

void scaleDataArray()
{
  byte *pdata;
  int *idata;
  long a;

  idata = (int *) data[sample+0];
  pdata = data[sample+0];
  for (int i = 0; i < SAMPLES; i++) {
    a = ((*idata++ + ch0_off) * VREF[range0] + 512) >> 10;
    if (a > LCD_YMAX) a = LCD_YMAX;
    else if (a < 0) a = 0;
    if (ch0_mode == MODE_INV)
      a = LCD_YMAX - a;
    *pdata++ = (byte) a;
  }
}

byte adRead(byte ch, byte mode, int off)
{
  long a = analogRead(ch);
  a = ((a+off)*VREF[ch == ad_ch0 ? range0 : range1]+512) >> 10;
  if (a > LCD_YMAX) a = LCD_YMAX;
  else if (a < 0) a = 0;
  if (mode == MODE_INV)
    return LCD_YMAX - a;
  return a;
}

int advalue(int value, long vref, byte mode, int offs) {
  if (mode == MODE_INV)
    value = LCD_YMAX - value;
  return (((long)value << 10) - 512L) / vref - offs;
}

void  loop() {
  int oad, ad;
  unsigned int auto_time;
  TSPoint p;
  int trigger_ad;

  if (trig_ch == ad_ch0) {
    trigger_ad = advalue(trig_lv, VREF[range0], ch0_mode, ch0_off);
  } else {
    trigger_ad = advalue(trig_lv, VREF[range1], ch1_mode, ch1_off);
  }
  delayMicroseconds(500);
  for (int i = 0; i < 50; i++) {          // wait until the A/D becomes stable
    (void) analogRead(trig_ch);           // read and neglect
    delayMicroseconds(20);
  }
  if (rate < 17) {
  ADCSRA = (ADCSRA & 0x07)| _BV(ADEN) | _BV(ADIF);  // Auto Trigger disable
  ADCSRB &= 0xf8;   // Auto Trigger source free run
  auto_time = 2 * pow(10, rate / 3);
  if (trig_mode != TRIG_SCAN) {
      unsigned long st = millis();
      oad = analogRead(trig_ch);
      for (;;) {
        ad = analogRead(trig_ch);

        if (trig_edge == TRIG_E_UP) {
          if (ad >= trigger_ad && trigger_ad > oad)
            break;
        } else {
          if (ad <= trigger_ad && trigger_ad < oad)
            break;
        }
        oad = ad;

//        if (rate > 9)       // This is usefull when timing accuracy is not so important
//          check_button();
//        if (trig_mode == TRIG_SCAN)   // This is usefull if mode can be changed by interrupt
//          break;
        if (trig_mode == TRIG_AUTO && (millis() - st) > auto_time)
          break;
      }
  }
  }

  // sample and draw depending on the sampling rate
  if (rate <= 9 && Start) {
    // change the index for the double buffer
    if (sample == 0)
      sample = 2;
    else
      sample = 0;

    if (rate == 0) {        // full speed, channel 0 only ADC free run. 97.4us/div 308ksps
      sample_97us();
    } else if (rate == 1) { // full speed, channel 0 only analogRead(). 177us/div 176ksps
      sample_177us();
    } else if (rate == 2) { // analogRead() channel 0 only. 200us/div 150ksps
      sample_200us();
    } else if (rate == 3) { // analogRead() channel 0 only. 500us/div 60ksps
      sample_500us();
    } else if (rate == 4) { // analogRead() channel 0 only. 1ms/div 30ksps
      sample_1ms();
    } else if (rate >= 5 && rate <= 9) {
      sample_5ms();   // dual channel 2ms/div 5ms/div 10ms/div 20ms/div 50ms/div
    }
//    DrawGrid();
    if (fft_mode) {
      plotFFT();
    } else {
      if (rate < 5) {   // channel 0 only
        ClearAndDrawGraphMag(1);
      } else {
        ClearAndDrawGraph();
      }
      tft.drawFastVLine(SAMPLES, 0, LCD_YMAX, FRMCOLOR); // right vertical line
    }
//    check_button();
//    DrawText();
  } else if (rate <= 16 && Start) { // 100ms/div - 10s/div
    ADCSRA = ADCSRA | 0x07; // dividing ratio = 128 (default of Arduino）
  // copy currently showing data to another
    if (sample == 0) {
      for (int i=0; i<SAMPLES; i ++) {
        data[2][i] = data[0][i];
        data[3][i] = data[1][i];
      }
    } else {
      for (int i=0; i<SAMPLES; i ++) {
        data[0][i] = data[2][i];
        data[1][i] = data[3][i];
      }      
    }
//    DrawGrid();

    const unsigned long r_[] = {50000/DOTS_DIV, 100000/DOTS_DIV, 200000/DOTS_DIV,
                      500000/DOTS_DIV, 1000000/DOTS_DIV, 2000000/DOTS_DIV, 
                      5000000/DOTS_DIV, 10000000/DOTS_DIV};
    unsigned long st0 = millis();
    unsigned long st = micros();
    for (int i=0; i<SAMPLES; i ++) {
      while((st - micros())<r_[rate-9]) {
//        CheckSW();
        if (rate<9)
          break;
      }
//      if (rate<9) { // sampling rate has been changed.  This is usefull if rate can be changed by interrupt
//        tft.fillScreen(BGCOLOR);
//        break;
//      }
      st += r_[rate-9];
      if (st - micros()>r_[rate-9])
          st = micros(); // sampling rate has been changed to shorter interval
      if (!Start) {
         i --;
         continue;
      }
      if (ch0_mode!=MODE_OFF) data[sample+0][i] = adRead(ad_ch0, ch0_mode, ch0_off);
      if (ch1_mode!=MODE_OFF) data[sample+1][i] = adRead(ad_ch1, ch1_mode, ch1_off);
      ClearAndDrawDot(i);     
      p = ts.getPoint();    // check if any touch to break the sampling loop
      pinMode(XM, OUTPUT);
      pinMode(YP, OUTPUT);
      if (p.z > MINPRESSURE && i > DOTS_DIV / 2)  // do not break while first half div.
        break;
    }
    // Serial.println(millis()-st0);
    tft.drawFastVLine(SAMPLES, 0, LCD_YMAX, FRMCOLOR); // right vertical line
//    check_button();
//    DrawText();
  } else if (Start) { // Equivalent Time sampling
    extern byte oscspeed;
    // change the index for the double buffer
    if (sample == 0)
      sample = 2;
    else
      sample = 0;
    oscspeed = rate - 17; // 17...23 -> 0...6
    modeequiv();
    if (fft_mode) {
      plotFFT();
    } else {      // channel 0 only
      ClearAndDrawGraphMag(1);
      tft.drawFastVLine(SAMPLES, 0, LCD_YMAX, FRMCOLOR); // right vertical line
    }
  } else {
//    check_button();
  }
  DrawText();
  if (trig_mode == TRIG_ONE)
    Start = false;
  check_button();
#ifdef SAVE_EEPROM
  saveEEPROM();                         // save settings to EEPROM if necessary
#endif
}

void measure_frequency() {
  int x1, x2;
  byte y;
  freqDuty();
  if (info_mode == 1) {
    x1 = 205, x2 = 217;
    tft.setTextSize(2);
  } else {
    x1 = 242, x2 = 254;
    tft.setTextSize(1);
  }
  y = 21;
  TextBG(&y, x1, 8);
  if (waveFreq < 999.5)
    tft.print(waveFreq);
  else if (waveFreq < 999999.5)
    tft.print(waveFreq, 0);
  else {
    tft.print(waveFreq/1000.0, 0);
    tft.print('k');
  }
  tft.print(F("Hz"));
  TextBG(&y, x2, 7);
  tft.print(waveDuty);  tft.print('%');
}

void measure_voltage() {
  int x, dave, dmax, dmin;
  byte y;
  if (info_mode == 1) {
    x = 205, y = 61;
  } else {
    x = 241, y = 41;
  }
  if (ch0_mode == MODE_INV) {
    dave = (LCD_YMAX) * 10 - dataAve;
    dmax = dataMin;
    dmin = dataMax;
  } else {
    dave = dataAve;
    dmax = dataMax;
    dmin = dataMin;
  }
  float vavr = VRF * (((dave * 102.4) - 512.0) / VREF[range0] - ch0_off) / 1023.0;
  float vmax = VRF * advalue(dmax, VREF[range0], ch0_mode, ch0_off) / 1023.0;
  float vmin = VRF * advalue(dmin, VREF[range0], ch0_mode, ch0_off) / 1023.0;
  TextBG(&y, x, 8);
  tft.print("max");  tft.print(vmax); if (vmax >= 0.0) tft.print('V');
  TextBG(&y, x, 8);
  tft.print("avr");  tft.print(vavr); if (vavr >= 0.0) tft.print('V');
  TextBG(&y, x, 8);
  tft.print("min");  tft.print(vmin); if (vmin >= 0.0) tft.print('V');
}

void sample_97us() {  // full speed, channel 0 only ADC free run. 97.4us/div 308ksps
  byte *pdata;
  pdata = data[sample+0];
  ADCSRA = (ADCSRA & 0xf8)| 0x62; // Auto Trigger Enable. dividing ratio = 4(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  for (int i=0; i<SAMPLES; i ++) {
    while ((ADCSRA&0x10)==0) ;  // polling until adif==1
    ADCSRA |= 0x10;             // clear adif
    *pdata++ = ADCL;            // must read adch low byte first
    *pdata++ = ADCH;            // read adch high byte
  }
  ADCSRA = ADCSRA & 0x9f;       // stop ADC free run ADSC=0 ADATE=0
  scaleDataArray();
}

void sample_177us() { // analogRead() full speed, channel 0 only. 177us/div 176ksps
  int *idata;
  ADCSRA = (ADCSRA & 0xf8) | 0x02;  // dividing ratio = 4(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  idata = (int *) data[sample+0];
  for (int i=0; i<SAMPLES; i ++) {
    *idata++ = analogRead(ad_ch0);
  }
  scaleDataArray();             // 2.46ms
}

void sample_200us() { // analogRead(), channel 0 only. 200us/div 150ksps
  int *idata;
  ADCSRA = (ADCSRA & 0xf8) | 0x02;  // dividing ratio = 4(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  idata = (int *) data[sample+0];
  for (int i=0; i<SAMPLES; i ++) {
    *idata++ = analogRead(ad_ch0);
    // time fine adjustment 0.0625 x 11 = 0.6875us（nop=0.0625us @16MHz)
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop");
  }
  scaleDataArray();
}

void sample_500us() { // analogRead(), channel 0 only. 500us/div 60ksps
  int *idata;
  ADCSRA = (ADCSRA & 0xf8) | 0x03;  // dividing ratio = 8(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  idata = (int *) data[sample+0];
  for (int i=0; i<SAMPLES; i ++) {
    *idata++ = analogRead(ad_ch0);
    delayMicroseconds(8);     // timing adjustmet tuned
    // time fine adjustment 0.0625 x 9 = 0.5625us（nop=0.0625us @16MHz)
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop");
  }
  scaleDataArray();
}

void sample_600us() { // analogRead() with timing, channel 0 only. 600us/div 50ksps
  int *idata;
  ADCSRA = (ADCSRA & 0xf8) | 0x03;  // dividing ratio = 8(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  idata = (int *) data[sample+0];
  unsigned long r = 600/DOTS_DIV; // 600us/div
  unsigned long st = micros();
  st += r;
  for (int i=0; i<SAMPLES; i ++) {
    while(micros()<st) ;
    *idata++ = analogRead(ad_ch0);
    st += r;
  }
  scaleDataArray();
}

void sample_1ms() { // analogRead(), channel 0 only. 1ms/div 30ksps
  int *idata;
  ADCSRA = (ADCSRA & 0xf8) | 0x04;  // dividing ratio = 16(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  idata = (int *) data[sample+0];
  for (int i=0; i<SAMPLES; i ++) {
    *idata++ = analogRead(ad_ch0);
    delayMicroseconds(18);     // timing adjustmet tuned
    // time fine adjustment 0.0625 x 10 = 0.625us（nop=0.0625us @16MHz)
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
  scaleDataArray();
}

void sample_1890us() { // full speed, dual channel. 1.89ms/div 15.9ksps
  ADCSRA = (ADCSRA & 0xf8) | 0x04;  // dividing ratio = 16(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  for (int i=0; i<SAMPLES; i ++) {
    data[sample+0][i] = adRead(ad_ch0, ch0_mode, ch0_off);
    data[sample+1][i] = adRead(ad_ch1, ch1_mode, ch1_off);
  }
}

void sample_2ms() { // dual channel. 2ms/div 15ksps
  ADCSRA = (ADCSRA & 0xf8) | 0x04;  // dividing ratio = 16(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
//  unsigned long st = micros();
  for (int i=0; i<SAMPLES; i ++) {
    data[sample+0][i] = adRead(ad_ch0, ch0_mode, ch0_off);
    data[sample+1][i] = adRead(ad_ch1, ch1_mode, ch1_off);
    delayMicroseconds(11);   // timing adjustmet tuned
    // time fine adjustment 0.0625 x 2 = 0.125us（nop=0.0625us @16MHz)
    asm("nop"); asm("nop");
  }
//  Serial.println(micros()-st);
}

#ifdef CONSTANT_SAMPLING_RATE
// The total time may be a little bit smaller due to the integer division by DOTS_DIV.
// However, sampling period shall be stable.
// Usefull when there is no remainder in division.
void sample_5ms() { // dual channel. 5ms, 10ms, 20ms
  byte *p0, *p1;
  p0 = data[sample+0]; p1 = data[sample+1];
  ADCSRA = (ADCSRA & 0xf8) | 0x04;  // dividing ratio = 16(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  const unsigned long r_[] = {2000/DOTS_DIV, 5000/DOTS_DIV, 10000/DOTS_DIV, 20000/DOTS_DIV, 50000/DOTS_DIV};
  unsigned long r = r_[rate - 5];
  unsigned long st = micros();
  for (int i=0; i<SAMPLES; i ++) {
    while(micros()<st) ;
    *p0++ = adRead(ad_ch0, ch0_mode, ch0_off);
    *p1++ = adRead(ad_ch1, ch1_mode, ch1_off);
    st += r;
  }
}
#else
// The total time becomes accurate than above, even if there is remainder in division.
// However, sampling period may fluctuates arround 1 micro second.
void sample_5ms() { // dual channel. 2ms, 5ms, 10ms, 20ms, 50ms
  byte *p0, *p1;
  p0 = data[sample+0]; p1 = data[sample+1];
  ADCSRA = (ADCSRA & 0xf8) | 0x04;  // dividing ratio = 16(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  const unsigned long r_[] = {2000, 5000, 10000, 20000, 50000};
  unsigned int r = r_[rate - 5];
  unsigned int rr = r / DOTS_DIV;
  unsigned int f = r % DOTS_DIV;
  unsigned int ff = 0;
  unsigned long st = micros();
  for (int i=0; i<SAMPLES; i ++) {
    while(micros()<st) ;
    if (ch0_mode!=MODE_OFF) *p0++ = adRead(ad_ch0, ch0_mode, ch0_off);
    if (ch1_mode!=MODE_OFF) *p1++ = adRead(ad_ch1, ch1_mode, ch1_off);
    st += rr;
    if ((ff += f) >= DOTS_DIV) {
      ++st;
      ff -= DOTS_DIV;
    }
  }
}
#endif

void sample_us1(unsigned long r) {   // register direct with timing, channel 0 only. 2000 =< r
  byte *pdata;
  ADMUX = (ADMUX & 0xf8) + ad_ch0;
  ADCSRA = (ADCSRA & 0xf8) | 0x04;  // dividing ratio = 16(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  pdata = data[sample+0];
  unsigned long st1 = r + r;
  unsigned long st0 = micros();
  unsigned long st = st0 + r / DOTS_DIV;
  for (int i=0; i<SAMPLES; i ++) {
    while(micros()<st) ;
    ADCSRA |= 0x40;  // start the conversion(1 << ADSC)
    while (ADCSRA & 0x40); // ADSC is cleared when the conversion finishes
    *pdata++ = ADCL;            // must read adch low byte first
    *pdata++ = ADCH;            // read adch high byte
    st = st0 + st1/DOTS_DIV;
    st1 += r;
  }
  scaleDataArray();
}

void sample_us(unsigned long r) {   // analogRead() with timing, channel 0 only. 2000 =< r
  int *idata;
  ADCSRA = (ADCSRA & 0xf8) | 0x04;  // dividing ratio = 16(0x1=2, 0x2=4, 0x3=8, 0x4=16, 0x5=32, 0x6=64, 0x7=128)
  idata = (int *) data[sample+0];
  unsigned long st1 = r + r;
  unsigned long st0 = micros();
  unsigned long st = st0 + r / DOTS_DIV;
  for (int i=0; i<SAMPLES; i ++) {
    while(micros()<st) ;
    *idata++ = analogRead(ad_ch0);
    st = st0 + st1/DOTS_DIV;
    st1 += r;
  }
  scaleDataArray();
}

unsigned long cbtime = 0;
int up_down_level = 0;

void check_button(void) {
  TSPoint p;

  if (millis() - cbtime < 300) {
    return;  // neglect within 300ms
  } else {
    cbtime = millis();
  }
  byte sra;
  sra = ADCSRA;             // save current setting
  ADCSRA = ADCSRA | 0x07;   // dividing ratio = 128 (default of Arduino）
  for (int i = 0; i < 2; i++) { // wait until the A/D becomes stable
    p = ts.getPoint();
  }
  ADCSRA = sra;             // recover current setting
  //pinMode(XP, OUTPUT);
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  //pinMode(YM, OUTPUT);
  if (p.z < MINPRESSURE) {
    up_down_level = 0;
    return;
  }

  saveTimer = 5000;     // set EEPROM save timer to 5 secnd
  int py = map(p.x, TS_MINX, TS_MAXX, 0, LCD_HEIGHT);
  int px = map(p.y, TS_MINY, TS_MAXY, 0, LCD_WIDTH);

  // (1st button)
  if (px > 300 && py > 16 && py < 48) {
#ifndef DISPLAY_AC
    if (menu == SEL_OFST1) {
      ch0_off = 0;
    } else if (menu == SEL_OFST2) {
      ch1_off = 0;
#else DISPLAY_AC
    if (menu == SEL_OFST1) {
      if (digitalRead(CH1DCSW) == LOW) {
        ch0_off = ac_offset[range0];  // AC
      } else {
        ch0_off = 0;                  // DC
      }
    } else if (menu == SEL_OFST2) {
      if (digitalRead(CH2DCSW) == LOW) {
        ch1_off = ac_offset[range1];  // AC
      } else {
        ch1_off = 0;                  // DC
      }
#endif
    } else {
      if (Start) {
         Start = false;
      } else {
         Start = true;
      }
    }
    tft.fillRect(252, 1, 48, 16, BGCOLOR);  // clear position area
  }

  // Text information mode (2nd button)
  else if (px > 300 && py > 61 && py < 93) {
    if (++info_mode > 6) {
      info_mode = 0;
      tft.fillRect(296, 1, 24, 137, BGCOLOR);       // clear small text lines
    }
    if (info_mode == 2)
      tft.fillRect(181, 21, 119, 96, BGCOLOR);      // clear big frequency area
    else if ((info_mode & 1) < 1)
      tft.fillRect(230, 21, 59, 48, BGCOLOR);       // clear small frequency area
    tft.fillRect(1, 1, SAMPLES - 1, 15, BGCOLOR);   // clear top line
    tft.fillRect(1, BOTTOM_LINE, SAMPLES - 1, 15, BGCOLOR); // clear bottom line
    if (info_mode == 5)
      tft.fillRect(302, 16, 18, 122, BGCOLOR);      // clear hold button
    else if (info_mode < 2)
      draw_hold_button();
    if (Start == false)
      DrawText();
  }
   
  // FFT (3rd button)
  else if (px > 300 && py > 106 && py < 138) {
    clear_wave_area();
    fft_mode = !fft_mode;
  }

  // up (4th button)
  else if (px > 300 && py > 151 && py < 183) {
    if (menu == SEL_RATE) {
      rateup();       // s/div up 
    } else if (menu == SEL_OFST1) {
      ch0offsetup();  // CH0 OFF +
    } else if (menu == SEL_OFST2) {
      ch1offsetup();  // CH1 OFF +
    } else if (menu == SEL_TGLVL) {
      trig_lv_up();
    }
  }

  // down (5th button)
  else if (px > 300 && py > 196 && py < 232) {
    if (menu == SEL_RATE) {
      ratedown();     // s/div down
    } else if (menu == SEL_OFST1) {
      ch0offsetdn();  // CH0 OFF -
    } else if (menu == SEL_OFST2) {
      ch1offsetdn();  // CH1 OFF -
    } else if (menu == SEL_TGLVL) {
      trig_lv_dn();
    }
  }

  // Rotate ch0 range
  else if (px > 60 && px < 108 && py < 32) {
    if (range0 < RANGE_MAX)
      ++range0;
    else
      range0 = RANGE_MIN;
    tft.fillRect(60, 1, 48, 16, BGCOLOR); // clear CH1 range
  }

  // ch0 on/inv/off
  else if (px > 0 && px < 48 && py < 32) {
    if (ch0_mode == MODE_ON) {
      ch0_mode = MODE_INV;
    } else if (ch0_mode == MODE_INV && ch1_mode != MODE_OFF && 4 < rate && rate < 17) {
      ch0_mode = MODE_OFF;
      clear_wave_area();
    } else {
      ch0_mode = MODE_ON;
    }
    tft.fillRect(1, 1, 48, 16, BGCOLOR);  // clear CH1
  }

  // ch1 on/inv/off
  else if (px > 0 && px < 48 && py > 208) {
    extern byte oscinput;
    if (ch1_mode == MODE_ON) {
      ch1_mode = MODE_INV;
    } else if (ch1_mode == MODE_INV && ch0_mode != MODE_OFF) {
      ch1_mode = MODE_OFF;
      oscinput = 0;
      clear_wave_area();
    } else {
      ch1_mode = MODE_ON;
      oscinput = 1;
    }
    tft.fillRect(1, 224, 48, 16, BGCOLOR);  // clear CH2
  }

  // Rotate ch1 range
  else if (px > 60 && px < 108 && py > 208) {
    if (range1 < RANGE_MAX)
      ++range1;
    else
      range1 = RANGE_MIN;
    tft.fillRect(60, 224, 48, 16, BGCOLOR); // clear CH2 range
  }

  // sweep rate on/off
  else if (px > 132 && px < 180 && py < 32) {
    if (menu == SEL_RATE) {
      menu = SEL_NONE;
    } else {
      menu = SEL_RATE;
      tft.fillRect(192, 1, 48, 16, BGCOLOR);  // clear trigger level area
      tft.fillRect(252, 1, 48, 16, BGCOLOR);  // clear position area
    }
  }

  // Trigger level
  else if (px > 192 && px < 240 && py < 32) {
    if (menu == SEL_TGLVL) {
      menu = SEL_NONE;
      tft.drawFastHLine(0, LCD_YMAX - trig_lv, SAMPLES, BGCOLOR); // erase trig_lv line
    } else if (rate < 17) {
      menu = SEL_TGLVL;
      tft.drawFastHLine(0, LCD_YMAX - trig_lv, SAMPLES, CYAN);    // draw trig_lv line
    }
    tft.fillRect(192, 1, 48, 16, BGCOLOR);  // clear trigger level area
    tft.fillRect(252, 1, 48, 16, BGCOLOR);  // clear position area
  }

  // Trigger Level
  else if (menu == SEL_TGLVL) {
    tft.drawFastHLine(0, LCD_YMAX - trig_lv, SAMPLES, BGCOLOR); // erase trig_lv line
    trig_lv = 240 - py;
    tft.drawFastHLine(0, LCD_YMAX - trig_lv, SAMPLES, CYAN);    // draw trig_lv line
    tft.fillRect(192, 1, 48, 16, BGCOLOR);  // clear trigger level area
  }

  // Wave position
  else if (px > 252 && px < 300 && py < 32) {
    if (menu == SEL_OFST2) {
      menu = SEL_NONE;
    } else if (menu == SEL_OFST1) {
      menu = SEL_OFST2;
    } else {
      menu = SEL_OFST1;
    }
    tft.fillRect(192, 1, 48, 16, BGCOLOR);  // clear trigger level area
    tft.fillRect(252, 1, 48, 16, BGCOLOR);  // clear position area
  }

  // Toggle trigger channel
  else if (px > 132 && px < 180 && py > 208) {
    if (trig_ch == ad_ch0)
      trig_ch = ad_ch1;
    else
      trig_ch = ad_ch0;
    tft.fillRect(132, 224, 36, 16, BGCOLOR);  // clear trigger channel area
  }

  // Toggle trigger edge
  else if (px > 192 && px < 240 && py > 208) {
    if (trig_edge == TRIG_E_UP)
      trig_edge = TRIG_E_DN;
    else
      trig_edge = TRIG_E_UP;
    tft.fillRect(192, 224, 12, 16, BGCOLOR);  // clear trigger edge area
  }

  // Rotate trigger mode
  else if (px > 252 && px < 300 && py > 208) {
    // TRIG MODE
    if (trig_mode < TRIG_ONE) {
      trig_mode ++;
    } else {
      trig_mode = TRIG_AUTO;
      Start = true;
    }
    tft.fillRect(252, 224, 48, 16, BGCOLOR);  // clear trigger mode area
    tft.fillRect(252, 1, 48, 16, BGCOLOR);    // clear position area  
    DrawText();
  }
}

void ClearOldData(void) {
  byte *p1, *p2;
  p1 = data[sample+0];
  p2 = data[sample+1];
  for (int x=0; x<SAMPLES; x++) {
    *p1++ = 0;
    *p2++ = 0;
  }
}

void draw_hold_button(void) {
  tft.setTextSize(2);  tft.setTextColor(WHITE);
  tft.fillRect(302, 16, 18, 32, BLUE);
  tft.setCursor(306, 24);  tft.print('\x13');   // !! button
  tft.fillRect(302, 61, 18, 32, BLUE);
  tft.setCursor(306, 69);  tft.print('?');      // ? button
  tft.fillRect(302, 106, 18, 32, BLUE);
  tft.setCursor(306, 114);  tft.print('\x9f');  // f button
}

void draw_updown_button(void) {
  tft.setTextSize(2);  tft.setTextColor(WHITE);
  tft.fillRect(302, 151, 18, 32, BLUE);
  tft.setCursor(306, 159);  tft.print('+');     // + button
  tft.fillRect(302, 196, 18, 32, BLUE);
  tft.setCursor(306, 204);  tft.print('-');     // - button
}

void rateup() {
  draw_updown_button();
  if (rate < RATE_MAX) {
    rate ++;
  } else {
    rate = RATE_MIN;
  }
  clear_wave_area();
  if (ch0_mode == MODE_OFF && (rate < 5 || 16 < rate))
    ch0_mode = MODE_ON;
#ifdef CALIB_PULSE
  if (rate < 17)  // in case realtime sampling
    pulse();      // calibration pulse output
#endif
}

void ratedown() {
  draw_updown_button();
  if (rate > RATE_MIN) {
    rate --;
  } else {
    rate = RATE_MAX;
  }
  clear_wave_area();
  if (ch0_mode == MODE_OFF && (rate < 5 || 16 < rate))
    ch0_mode = MODE_ON;
#ifdef CALIB_PULSE
  if (rate < 17)  // in case realtime sampling
    pulse();      // calibration pulse output
#endif
}

void ch0offsetup() {
  if (++up_down_level > 30)
    up_down_level = 30;   // limit the acceleration
  ch0_off += (up_down_level * 1024)/VREF[range0];
  if (ch0_off > 1023)
    ch0_off = 1023;
}

void ch0offsetdn() {
  if (++up_down_level > 30)
    up_down_level = 30;   // limit the acceleration
  ch0_off -= (up_down_level * 1024)/VREF[range0];
  if (ch0_off < -1023)
    ch0_off = -1023;
}

void ch1offsetup() {
  if (++up_down_level > 30)
    up_down_level = 30;   // limit the acceleration
  ch1_off += (up_down_level * 1024)/VREF[range1];
  if (ch1_off > 1023)
    ch1_off = 1023;
}

void ch1offsetdn() {
  if (++up_down_level > 30)
    up_down_level = 30;   // limit the acceleration
  ch1_off -= (up_down_level * 1024)/VREF[range1];
  if (ch1_off < -1023)
    ch1_off = -1023;
}

void trig_lv_up() {
  ++up_down_level;
  tft.drawFastHLine(0, LCD_YMAX - trig_lv, SAMPLES, BGCOLOR); // erase trig_lv line
  trig_lv += up_down_level;
  tft.drawFastHLine(0, LCD_YMAX - trig_lv, SAMPLES, CYAN);    // draw trig_lv line
  tft.fillRect(192, 1, 48, 16, BGCOLOR);  // clear trigger level area
}

void trig_lv_dn() {
  ++up_down_level;
  tft.drawFastHLine(0, LCD_YMAX - trig_lv, SAMPLES, BGCOLOR); // erase trig_lv line
  trig_lv -= up_down_level;
  tft.drawFastHLine(0, LCD_YMAX - trig_lv, SAMPLES, CYAN);    // draw trig_lv line
  tft.fillRect(192, 1, 48, 16, BGCOLOR);  // clear trigger level area
}

void clear_wave_area(void) {
  tft.fillRect(0, 0, SAMPLES, LCD_HEIGHT, BGCOLOR);
  ClearOldData();
}

#define LEFT_MARGIN 20

void plotFFT() {
  byte *lastplot;
  char *im, *re;
  int x = LEFT_MARGIN;
  int ylim = 218;

  int clear = (sample == 0) ? 2 : 0;
  re = data[sample];
  im = data[1];  // use ch1 buffer for imaginary data
  for (int i = 0; i < FFT_N; i++) {
    int d = *(byte *)re;
    d = d - 120;
    *re++ = constrain(d, -128, 127);
    *im++ = 0;
  }
  re = data[sample];
  im = data[1];  // use ch1 buffer for imaginary data
  lastplot = data[clear];
  fix_fft(re, im, 8, 0);  // full scale 2^8=256, FFT mode
  for (int i = 1; i < FFT_N/2; i++) {
    int dat = sqrt(re[i] * re[i] + im[i] * im[i]);
    dat = dat + dat + dat;
    dat = constrain(dat, 0, ylim);
#ifdef RECT_DISPLAY
    tft.fillRect(i*2 + x, ylim - lastplot[i], 2, lastplot[i], BGCOLOR); // erase old
    tft.fillRect(i*2 + x, ylim - dat, 2, dat, WHITE);
#else
    tft.drawFastVLine(i*2 + x, ylim - lastplot[i], lastplot[i], BGCOLOR); // erase old
    tft.drawFastVLine(i*2 + x, ylim - dat, dat, WHITE);
#endif
    re[i] = dat;
  }
  draw_scale();
}

void draw_scale() {
  int x = LEFT_MARGIN;
  int ylim = 222;
  float fhref, nyquist;
  tft.setTextColor(GREEN);
  tft.setTextSize(1);
  tft.setCursor(x, ylim); tft.print(F("0Hz")); 
  if (rate > 16) {
    fhref = ethref();
  } else {
    fhref = (float)pgm_read_dword_near(HREF + rate);
  }
  nyquist = 15.0e6 / fhref; // Nyquist frequency
  tft.setCursor(x+116, ylim);
  if (nyquist > 999.0) {
    nyquist = nyquist / 1000.0;
    if (nyquist > 99.5) {
      tft.print(nyquist/2,0);tft.print('k');
      tft.setCursor(x+244, ylim); tft.print(nyquist,0);
    } else {
      tft.print(nyquist/2,1);tft.print('k');
      tft.setCursor(x+244, ylim); tft.print(nyquist,1);
    }
    tft.print('k');
  } else {
    tft.print(nyquist/2,0);
    tft.setCursor(x+244, ylim); tft.print(nyquist,0);
  }
  tft.setCursor(x+25, ylim+8); tft.print(F("Frequency (Hz)"));
}

#define EEPROM_START 64
void saveEEPROM() {                   // Save the setting value in EEPROM after waiting a while after the button operation.
  int p = EEPROM_START;
  if (saveTimer > 0) {                // If the timer value is positive
    saveTimer = saveTimer - timeExec; // Timer subtraction
    if (saveTimer <= 0) {             // if time up
      EEPROM.write(p++, range0);      // save current status to EEPROM
      EEPROM.write(p++, ch0_mode);
      EEPROM.write(p++, lowByte(ch0_off));  // save as Little endian
      EEPROM.write(p++, highByte(ch0_off));
      EEPROM.write(p++, range1);
      EEPROM.write(p++, ch1_mode);
      EEPROM.write(p++, lowByte(ch1_off));  // save as Little endian
      EEPROM.write(p++, highByte(ch1_off));
      EEPROM.write(p++, rate);
      EEPROM.write(p++, trig_mode);
      EEPROM.write(p++, trig_lv);
      EEPROM.write(p++, trig_edge);
      EEPROM.write(p++, trig_ch);
      EEPROM.write(p++, fft_mode);
      EEPROM.write(p++, info_mode);
    }
  }
}

void set_default() {
  range0 = 1;
  ch0_mode = MODE_ON;
  ch0_off = 0;
  range1 = 1;
  ch1_mode = MODE_ON;
  ch1_off = 0;
  rate = 5;
  trig_mode = TRIG_AUTO;
  trig_lv = 120;
  trig_edge = TRIG_E_UP;
  trig_ch = ad_ch0;
  fft_mode = false;
  info_mode = 1;
}

void loadEEPROM() { // Read setting values from EEPROM (abnormal values will be corrected to default)
  int p = EEPROM_START, error = 0;

  range0 = EEPROM.read(p++);                // range0
  if ((range0 < RANGE_MIN) || (range0 > RANGE_MAX)) ++error;
  ch0_mode = EEPROM.read(p++);              // ch0_mode
  if (ch0_mode > 2) ++error;
  *((byte *)&ch0_off) = EEPROM.read(p++);     // ch0_off low
  *((byte *)&ch0_off + 1) = EEPROM.read(p++); // ch0_off high
  if ((ch0_off < -1024) || (ch0_off > 1023)) ++error;

  range1 = EEPROM.read(p++);                // range1
  if ((range1 < RANGE_MIN) || (range1 > RANGE_MAX)) ++error;
  ch1_mode = EEPROM.read(p++);              // ch1_mode
  if (ch1_mode > 2) ++error;
  *((byte *)&ch1_off) = EEPROM.read(p++);     // ch1_off low
  *((byte *)&ch1_off + 1) = EEPROM.read(p++); // ch1_off high
  if ((ch1_off < -1024) || (ch1_off > 1023)) ++error;

  rate = EEPROM.read(p++);                  // rate
  if ((rate < RATE_MIN) || (rate > RATE_MAX)) ++error;
//  if (ch0_mode == MODE_OFF && rate < 5) ++error;  // correct ch0_mode
  trig_mode = EEPROM.read(p++);             // trig_mode
  if (trig_mode > 3) ++error;
  trig_lv = EEPROM.read(p++);               // trig_lv
  if (trig_lv > LCD_YMAX) ++error;
  trig_edge = EEPROM.read(p++);             // trig_edge
  if (trig_edge > 1) ++error;
  trig_ch = EEPROM.read(p++);               // trig_ch
  if (trig_ch > 7) ++error;
  fft_mode = EEPROM.read(p++);              // fft_mode
  if (fft_mode > 1) ++error;
  info_mode = EEPROM.read(p++);             // info_mode
  if (info_mode > 6) ++error;
  if (error > 0)
    set_default();
}

#ifdef CALIB_PULSE

void pulse() {
  pinMode(CALPIN, OUTPUT);

// TCCR1A: COM1A1, COM1A0, COM1B1, COM1B0, -, -, WGM11, 1GM10
// OC1A set on compare match, clear at BOTTOM (COM1A = 0b01)
// OC1B clear on compare match, set at BOTTOM (COM1B = 0b10)
// OCR1A Fast PWM Mode, TOP値=OCR1A (WGM11:10 = 0b11)
//  TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10); // Fast PWM mode - compare to OCR1A
  TCCR1A = _BV(COM1B1) | _BV(WGM11) | _BV(WGM10); // Fast PWM mode - compare to OCR1A
// TCCR1B: ICNC1, ICES1, -, WGM13, WGM12, CS12, CS11, CS10
// OCR1A Fast PWM Mode (WGM13:12 = 0b11)
// CS12:10 001:ck/1, 010:ck/8, 011:ck/64, 100:ck/256, 101:ck/1024
  TCNT1 = 0x0000;               // initialize TCNT1
  TCCR1B = 0b00011001;  // ck/1
  // TOP value
  OCR1A = 15999;  // 1kHz = 16000000 / (15999 + 1)
  // Duty ratio
  OCR1B = 7999;   // 50%
}
#endif
