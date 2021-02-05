
#include <QMC5883LCompass.h>
QMC5883LCompass compass;

#ifdef USE_STREAMING
#include <Streaming.h>
#endif

#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

float knots, hertz;
bool flipflop;
#define SQUAREWAVE_OUT A1
unsigned long setfreq;

#define OCR1A_1HZ 15624     //value of the timer counter for a 1Hz interrupt call

const float frequency_values[] PROGMEM = {0, 3.6,7,10.2,13.6,17.3,20.7,23.8,27.5,30.9,34.4,37.8,41.2,44.3,47.6,51.1,54.3,57.8,61.3,64.8,71.7,75.1,78.5,82.1};

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
// A1 square wave out signal 

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
  compass.setCalibration(-600,1196,170,2005,-1143,-208);
  compass.setSmoothing (8, true);

  pinMode (SQUAREWAVE_OUT, OUTPUT);

  knots = 1.0;
  flipflop = false;
  hertz = 1;

  cli();//stop interrupts

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 65535;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts

  SetCompareMatchRegisterForHertz(3.2);
  setfreq = millis();
  
}

void loop() {
  int a;

  // Read compass values
  compass.read();
  //xyz();
  //delay (20);
  //return;

  // Return Azimuth reading
  a = compass.getAzimuth();
  char myArray[4];
  compass.getDirection(myArray, a);
  myArray[3] = 0;
/*
  Serial.print("A: ");
  Serial.print(a);
  Serial.println();
*/
  if ( a < 10)
    value = str_00 + String (a);
  else if (a < 100)
    value = str_0 + String (a);
  else
    value = String (a);

  float angle = (float) (a - 90) * 3.14 / 180.0;
  float angle_deg = float (a);



  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso28_tr);
  u8g2.drawStr( 0, 0, value.c_str());
  u8g2.setFont(u8g2_font_inb16_mr);
  u8g2.drawStr( 8, 36, myArray);

  u8g2.drawCircle (90, 30, 30);
  u8g2.drawLine (90, 30, x(25, angle), y(25, angle));
  u8g2.drawDisc(x(20, angle), y(20, angle), 3);

  u8g2.sendBuffer();

  if (millis() - setfreq >= 1000)
  {
    SetCompareMatchRegisterForHertz(computeFrequency2(angle_deg / 360 * 12));
    //Serial << "angle " << angle_deg << "  knots " << angle_deg / 360 * 12 << " freq " << computeFrequency2(angle_deg/ 360.0 * 12.0) <<"\n";
    setfreq = millis();
  }
  
  delay(10);
  
  
}


void xyz()
{
 Serial.print (compass.getX());
 Serial.print (",");
 Serial.print (compass.getY());
 Serial.print (",");
 Serial.println (compass.getZ()); 
}

float x (float _radius, float _angle)
{

  return 90 + _radius * cos(_angle);
}


float y (float _radius, float _angle)
{
  return 30 + _radius * sin(_angle);
}



void SetCompareMatchRegisterForHertz(float hzsetpoint)
{
   hertz =  hzsetpoint;
  //compare match register = [ 16,000,000Hz/ (prescaler * desired interrupt frequency) ] - 1
  // prescaler is 1024

  cli();
  //OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  TCNT1  = 0;

  if (hzsetpoint == -1)
    OCR1A = 65535;
  else
    OCR1A = OCR1A_1HZ / hzsetpoint;
    //Serial << OCR1A;

  sei();

}


ISR(TIMER1_COMPA_vect)
{
  flipflop = !flipflop;
  digitalWrite (SQUAREWAVE_OUT, flipflop);
 // Serial <<".";
}



float computeFrequency2(float _knots)
{
  int i ;
  for (i = 0; i <= 24; i++)
  {
    if ((_knots >= i) && (_knots < i+1))
      return (_knots-i)* (frequency_values [i+1]-frequency_values [i])/(i+1-i) + frequency_values[i];
     
  }
  
}
