
#include <QMC5883LCompass.h>
QMC5883LCompass compass;

//#include "U8glib.h"
//U8GLIB_SSD1306_128X64 u8g(SCK, MOSI, 10, 9, A0);  // SW SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9

#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif


U8G2_SSD1309_128X64_NONAME2_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ SCK, /* data=*/ MOSI, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ A0);
//


// Beetle board   (32u4 diymore), leonardo
// MO  --> TFT SDA
// SCK --> TFT SCL
// 5V  --> TFT VCC
// A0  --> TFT RES
// D9  --> TFT DC
// D10 --> TFT CS
// D11 --> forced low, compas GND
// SDA --> compass SDA
// SCL --> compas SCL
// GND --> TFT GND
// 3.3V  --> compas VCC

void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_logisoso28_tr);//u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}
String value;
String str_0;
String str_00;

void setup() {
  Serial.begin(9600);
  compass.init();
  pinMode (11, OUTPUT);
  digitalWrite(11, 0);
  u8g2.begin();
  u8g2_prepare();
  str_0 = String ("0");
  str_00 = String ("00");
}

void loop() {
  int a;

  // Read compass values
  compass.read();

  // Return Azimuth reading
  a = compass.getAzimuth();
  char myArray[4];
  compass.getDirection(myArray, a);
  myArray[3] = 0;

  Serial.print("A: ");
  Serial.print(a);
  Serial.println();

  if ( a < 10)
    value = str_00 + String (a);
  else if (a < 100)
    value = str_0 + String (a);
  else
    value = String (a);

  float angle = (float) (a - 90) * 3.14 / 180.0;



  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso28_tr);
  u8g2.drawStr( 0, 0, value.c_str());
  u8g2.setFont(u8g2_font_inb16_mr);
  u8g2.drawStr( 8, 36, myArray);

  u8g2.drawCircle (90, 30, 30);
  u8g2.drawLine (90, 30, x(25, angle), y(25, angle));
  u8g2.drawDisc(x(20, angle), y(20, angle), 3);



  u8g2.sendBuffer();

  delay(100);
}

float x (float _radius, float _angle)
{

  return 90 + _radius * cos(_angle);
}


float y (float _radius, float _angle)
{
  return 30 + _radius * sin(_angle);
}
