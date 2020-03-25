
/*
 * A software I²C slave implementation.
 * 
 * This slave does not send any data. It only receives data from the master.
 * The data received are the 5 float values:
 * In this example the values represent: Temperature, Humidity, UV index, Lux, Pressure.
 * The received values are displayed on the TFT.
 * 
 * The TFT occupies all pins of my Arduino Uno.
 * The Uno's default designated (hardware) I2C pins can not be used,
 * because they connect to the TFT's pins that control RESET & CS.
 * If the hardware I2C would be used, the TFT would behave irratic.
 * I do not use the TFT's SD card, so i can use 2 of those pins for the (software) I²C bus.
 * Arduino pins used: 11 = SCL, 12 = SDA.
 * This software I2C bus operates at a +5V level.
 * Therefore connect the TFT I2C-slave at the high level side of the Logic Level Converter.
*/

// TFT
#include <Adafruit_GFX.h>
#include <UTFTGLUE.h>
UTFTGLUE tft(0x9341,A2,A1,A3,A4,A0);
#if !defined(SmallFont)
extern uint8_t SmallFont[];
#endif

#define BLACK   0x0000
#define BLUE    0x001F
#define WHITE   0xFFFF

// text size 1 = 6x8 pixels char
#define TFT_TEXTSIZE 1
// lineheight 8 * TFT_TEXTSIZE pixels
#define TFT_LINEHEIGHT (8 * TFT_TEXTSIZE)
// values are printed from char.x = 17 => pos.x = 17 * 6 * TFT_TEXTSIZE
#define TFT_VALUES_OFFSETX (17 * 6 * TFT_TEXTSIZE)


// I2C
#define I2C_SLAVE_ADDRESS 73
#define SCL_BIT PB3
#define SDA_BIT PB4
#define I2C_DDR DDRB
#define I2C_OUT PORTB
#define I2C_IN  PINB


extern "C" {
  volatile uint8_t byteN; // receiving byte n (0..39)
  volatile uint8_t I2C_data;
  /*
  f[0] = CACHE_BMP280_TEMP
  f[1] = CACHE_BMP280_PRESSURE
  f[2] = CACHE_BMP280_HEIGHT
  f[3] = CACHE_DHT22_TEMP
  f[4] = CACHE_DHT22_HUM
  f[5] = CACHE_DHT22_HEATINDEX
  f[6] = CACHE_DHT22_DEWPOINT
  f[7] = CACHE_UVM30A_UV
  f[8] = CACHE_BH1750_LUX
  f[9] = CACHE_BATTERY
  #define CACHE_COUNT 10
  */
  volatile union u_floatarray {
    uint8_t b[10*4];
    float f[10];
    //int32_t i[10];
  } I2C_values;
/*
  // buffer
  volatile uint8_t buffer_count = 0;
  volatile uint8_t buffer_ptr = 0;
  volatile uint8_t buffer_values[255];

  void buffer_add(float value) {
    uint8_t v = constrain((int)value, 0,40) * 3;  // convert value to a byte
    buffer_values[buffer_ptr++] = v;
    if (buffer_count<255) buffer_count++;
    buffer_draw();
  }
  
  void buffer_draw() {
    tft.writeFillRect(51,100,255,119,WHITE);
    for (uint8_t i=0; i<buffer_count; i++) {
      uint8_t ptr = buffer_ptr - i - 1;
      tft.writePixel(306-i, 219-buffer_values[ptr], BLUE);
    }
  }
*/
  void InComingData(void) {
    I2C_values.b[byteN++] = I2C_data;
    if (byteN % 4 == 0) {                   // a complete float has been received
      uint8_t v = (byteN >> 2) - 1;         // index into the I2C_values[] array
      uint8_t r = v * TFT_LINEHEIGHT;       // lineheight
      tft.setCursor(TFT_VALUES_OFFSETX, r);
      //if (v==8 || v==9) tft.println(I2C_values.i[v]); else tft.println(I2C_values.f[v]); // print as (int or float)
      tft.println(I2C_values.f[v]);
/*
      // add to buffer for displaying graph
      if (v==0) buffer_add(I2C_values.f[v]);
*/
    }
  }
  void OutGoingData(void) {
    // nothing happens here. This function will never be called
  }
}




void setup() {
  tft.begin();
  tft.InitLCD();
  if (tft.height() > tft.width()) tft.setRotation(1);    //LANDSCAPE
  tft.fillScreen(WHITE);
  tft.setFont(SmallFont);
  tft.setTextColor(BLACK, WHITE);
/*
  // graph
  tft.setTextSize(1);
  tft.drawFastVLine(50,100,120,BLACK);
  tft.drawFastHLine(50,220,256,BLACK);
  tft.setCursor(4, 90);   tft.println("Temperature (C)");
  tft.setCursor(4, 100);  tft.println("     40");
  tft.setCursor(4, 212);  tft.println("      0");
*/
  // current values
  tft.setTextSize(TFT_TEXTSIZE);
  tft.setCursor(0, 0);
  tft.println("    BMP280_TEMP:");
  tft.println("BMP280_PRESSURE:");
  tft.println("  BMP280_HEIGHT:");
  tft.println("     DHT22_TEMP:");
  tft.println("      DHT22_HUM:");
  tft.println("DHT22_HEATINDEX:");
  tft.println(" DHT22_DEWPOINT:");
  tft.println("      UVM30A_UV:");
  tft.println("     BH1750_LUX:");
  tft.println("        BATTERY:");

  TIMSK0 = 0;  // disable the Arduino's Timer 0 (NOTE: delay() and millis() will no longer work)
  __asm__ __volatile__ (
          "cli"                                                                     "\n"
          "cbi %[I2CDDR], %[SCLBIT]"                                                "\n"
          "cbi %[I2COUT], %[SCLBIT]"                                                "\n"
          "cbi %[I2CDDR], %[SDABIT]"                                                "\n"
          "cbi %[I2COUT], %[SDABIT]"                                                "\n"
   :: [I2CDDR] "I" (_SFR_IO_ADDR(I2C_DDR)),
      [I2COUT] "I" (_SFR_IO_ADDR(I2C_OUT)),
      [SCLBIT] "I" (SCL_BIT),
      [SDABIT] "I" (SDA_BIT)
  );
}


void loop() {
  __asm__ __volatile__ (
    "HandleTransaction:"                                                             "\n"
           "rcall I2C_activity"                                                      "\n"
           "brtc HandleTransaction"                                                  "\n"
    "StartCondition:"                                                                "\n"
           "ldi R16, 0"                                                              "\n"
           "sts byteN, R16"                                                          "\n"
           "rcall wait_SCL_low"                                                      "\n"
           "rcall slave_readByte"                                                    "\n"
           "brts StartCondition"                                                     "\n"
           "brhs StopCondition"                                                      "\n"
           "mov R19, R18"                                                            "\n"
           "lsr R19"                                                                 "\n"
           "cpi R19, %[SLAVE_ADDR]"                                                  "\n"
           "brne StopCondition"                                                      "\n"
           "rcall slave_writeACK"                                                    "\n"
           "andi R18, 0b00000001"                                                    "\n"
           "brne MasterRead"                                                         "\n"
    "MasterWrite:"                                                                   "\n"
           "rcall slave_readByte"                                                    "\n"
           "brts StartCondition"                                                     "\n"
           "brhs StopCondition"                                                      "\n"
           "rcall slave_writeACK"                                                    "\n"
           "sbi %[I2CDDR], %[SCLBIT]"                                                "\n"
           "sts I2C_data, R18"                                                       "\n"
           "call InComingData"                                                       "\n"
           "cbi %[I2CDDR], %[SCLBIT]"                                                "\n"
           "rjmp MasterWrite"                                                        "\n"
    "MasterRead:"                                                                    "\n"
           "sbi %[I2CDDR], %[SCLBIT]"                                                "\n"
           "call OutGoingData"                                                       "\n"
           "lds R18, I2C_data"                                                       "\n"
           "cbi %[I2CDDR], %[SCLBIT]"                                                "\n"
           "rcall slave_writeByte"                                                   "\n"
           "brts StartCondition"                                                     "\n"
           "brhs StopCondition"                                                      "\n"
           "rcall slave_readACK"                                                     "\n"
           "breq MasterRead"                                                         "\n"
    "StopCondition:"                                                                 "\n"
           "rjmp DoneTransaction"                                                    "\n"

    "I2C_activity:"                                                                  "\n"
           "in R16, %[I2CIN]"                                                        "\n"
           "andi R16, (1<<%[SCLBIT] | 1<<%[SDABIT])"                                 "\n"
    "ac1:" "in R17, %[I2CIN]"                                                        "\n"
           "andi R17, (1<<%[SCLBIT] | 1<<%[SDABIT])"                                 "\n"
           "cp R16, R17"                                                             "\n"
           "breq ac1"                                                                "\n"
           "clh"                                                                     "\n"
           "clt"                                                                     "\n"
           "sbrs R16, %[SCLBIT]"                                                     "\n"
           "rjmp ac2"                                                                "\n"
           "sbrs R17, %[SCLBIT]"                                                     "\n"
           "rjmp ac2"                                                                "\n"
           "sbrs R17, %[SDABIT]"                                                     "\n"
           "set"                                                                     "\n"
           "sbrc R17, %[SDABIT]"                                                     "\n"
           "seh"                                                                     "\n"
    "ac2:" "ret"                                                                     "\n"

    "slave_readByte:"                                                                "\n"
           "ldi R18, 0b00000001"                                                     "\n"
    "rb1:" "rcall wait_SCL_high"                                                     "\n"
           "in R19, %[I2CIN]"                                                        "\n"
           "rcall I2C_activity"                                                      "\n"
           "brts rb2"                                                                "\n"
           "brhs rb2"                                                                "\n"
           "sec"                                                                     "\n"
           "sbrs R19, %[SDABIT]"                                                     "\n"
           "clc"                                                                     "\n"
           "rol R18"                                                                 "\n"
           "brcc rb1"                                                                "\n"
           "clz"                                                                     "\n"
           "clh"                                                                     "\n"
    "rb2:" "ret"                                                                     "\n"

    "slave_writeByte:"                                                               "\n"
           "ldi R19, 8"                                                              "\n"
    "wb1:" "lsl R18"                                                                 "\n"
           "brcs wb2"                                                                "\n"
           "sbi %[I2CDDR], %[SDABIT]"                                                "\n"
    "wb2:" "brcc wb3"                                                                "\n"
           "cbi %[I2CDDR], %[SDABIT]"                                                "\n"
    "wb3:" "rcall wait_SCL_high"                                                     "\n"
           "rcall I2C_activity"                                                      "\n"
           "brts wb4"                                                                "\n"
           "brhs wb4"                                                                "\n"
           "dec R19"                                                                 "\n"
           "brne wb1"                                                                "\n"
    "wb4:" "cbi %[I2CDDR], %[SDABIT]"                                                "\n"
           "ret"                                                                     "\n"

    "skipPulse:"                                                                     "\n"
           "rcall wait_SCL_high"                                                     "\n"
    "wait_SCL_low:"                                                                  "\n"
           "sbic %[I2CIN], %[SCLBIT]"                                                "\n"
           "rjmp wait_SCL_low"                                                       "\n"
           "ret"                                                                     "\n"

    "wait_SCL_high:"                                                                 "\n"
           "sbis %[I2CIN], %[SCLBIT]"                                                "\n"
           "rjmp wait_SCL_high"                                                      "\n"
           "ret"                                                                     "\n"

    "slave_writeACK:"                                                                "\n"
           "sbi %[I2CDDR], %[SDABIT]"                                                "\n"
           "rcall skipPulse"                                                         "\n"
           "cbi %[I2CDDR], %[SDABIT]"                                                "\n"
           "ret"                                                                     "\n"

    "slave_readACK:"                                                                 "\n"
           "rcall wait_SCL_high"                                                     "\n"
           "sez"                                                                     "\n"
           "sbic %[I2CIN], %[SDABIT]"                                                "\n"
           "clz"                                                                     "\n"
           "rcall wait_SCL_low"                                                      "\n"
           "ret"                                                                     "\n"

    "DoneTransaction:"                                                               "\n"
  :: [I2CIN] "I" (_SFR_IO_ADDR(I2C_IN)),
     [I2COUT] "I" (_SFR_IO_ADDR(I2C_OUT)),
     [I2CDDR] "I" (_SFR_IO_ADDR(I2C_DDR)),
     [SCLBIT] "I" (SCL_BIT),
     [SDABIT] "I" (SDA_BIT),
     [SLAVE_ADDR] "M" (I2C_SLAVE_ADDRESS),
     "e" (InComingData),
     "e" (OutGoingData),
     [I2C_data] "label" (I2C_data),
     [byteN] "label" (byteN)
  );
}

