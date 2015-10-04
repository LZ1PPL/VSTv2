#include <LibAPRS.h>
#include "U8glib.h"
#include <TinyGPS++.h>

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);
TinyGPSPlus gps;

unsigned int txCounter = 0;
unsigned long lastTx = 0;
char nw, wl;
boolean gotPacket = false;
AX25Msg incomingPacket;
uint8_t *packetData;
float lastTxLat = 0.00;
float lastTxLng = 0.00;
float lastTxdistance = 0.0;
int previousHeading = 400;
int lastbearing = 0;
int headingDelta, lastheadingDelta = 0;
byte hour = 0, minute = 0, second = 0;
unsigned long secondtimer = 0;
unsigned long timer = 0;
boolean paket = false;
int buttonState = 0;
const int buttonPin = 8;

#define ADC_REFERENCE REF_3V3
#define OPEN_SQUELCH true

void setup()
{
  pinMode(buttonPin, INPUT);
  Serial.begin(9600);
  Serial.println(F("AT+DMOSETGROUP=0,144.8000,144.8000,0000,0,0000"));
  delay(100);
  Serial.println(F("AT+DMOSETVOLUME=8"));
  delay(100);
  Serial.println(F("AT+SETFILTER=1,1,1"));
  delay(100);
  // Initialise APRS library - This starts the modem
  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  APRS_setCallsign("LZ1PPL", 9);
  APRS_setSymbol('>');
  APRS_setDestination("APMT01", 0);
  APRS_setPath1("WIDE1", 1);
  APRS_setPath2("WIDE2", 1);
  APRS_setPreamble(1000);
  APRS_setTail(150);
}

void aprs_msg_callback(struct AX25Msg *msg)
{
  if (!gotPacket)
  {
    gotPacket = true;
    memcpy(&incomingPacket, msg, sizeof(AX25Msg));
  }
}

void loop()
{
  if (millis() - secondtimer >= 1000)
  {
    second++;
    secondtimer = millis();
    if (second == 60)
    {
      second = 0;
      minute++;
      if (minute == 60)
      {
        minute = 0;
        hour++;
        if (hour == 24)
        {
          hour = 0;
        }
      }
    }
  }

  while (Serial.available() > 0)
  {
    gps.encode(Serial.read());
  }

  if ( gps.time.isUpdated() )
  {
    hour = gps.time.hour();
    minute = gps.time.minute();
    second = gps.time.second();
  }

  ///////////////// Triggered by location updates ///////////////////////
  if ( gps.location.isUpdated() )
  {
    lastTxdistance = TinyGPSPlus::distanceBetween(
                       gps.location.lat(),
                       gps.location.lng(),
                       lastTxLat,
                       lastTxLng);

    lastbearing = (int)TinyGPSPlus::courseTo(
                    lastTxLat,
                    lastTxLng,
                    gps.location.lat(),
                    gps.location.lng());

    nw = gps.location.rawLat().negative ? 'S' : 'N';
    wl = gps.location.rawLng().negative ? 'W' : 'E';
    // Get headings and heading delta

    headingDelta = abs(lastbearing - (int)gps.course.deg());
    if (headingDelta  > 180)
    {
      headingDelta  = 360 - headingDelta;
    }

    lastheadingDelta = abs(lastbearing - previousHeading);
    if (lastheadingDelta > 180)
    {
      lastheadingDelta = 360 - lastheadingDelta;
    }

  } // endof gps.location.isUpdated()

  long latt, lonn;

  lonn = (gps.location.lng() * 100000) + 18000000; // Step 1
  latt = (gps.location.lat() * 100000) + 9000000; // Adjust so Locn AA is at the pole
  char MH[6] = {'A', 'A', '0', '0', 'a', 'a'}; // Initialise our print string
  MH[0] += lonn / 2000000; // Field
  MH[1] += latt / 1000000;
  MH[2] += (lonn % 2000000) / 200000; // Square
  MH[3] += (latt % 1000000) / 100000;
  MH[4] += (lonn % 200000) / 8333; // Subsquare .08333 is 5/60
  MH[5] += (latt % 100000) / 4166; // .04166 is 2.5/60
  String MH_txt = ""; // Build up Maidenhead
  int i = 0; // into a string that's easy to print
  while (i < 6)
  {
    MH_txt += MH[i];
    i++;
  }

  buttonState = digitalRead(buttonPin);
  if (gotPacket)
  {
    paket = true;
  }
  if (paket == true)
  {
    if (gotPacket) {
      gotPacket = false;
      u8g.firstPage();
      do
      {
        u8g.setFont(u8g_font_5x8);
        u8g.setPrintPos(0, 9);
        printTime();
        
        u8g.setPrintPos(60, 9);
        u8g.print(incomingPacket.src.call);
        if (incomingPacket.src.ssid > 0)
        {
          u8g.print(F("-"));
          u8g.print(incomingPacket.src.ssid);
        } 
        u8g.setPrintPos(0, 27);
        for (int i = 1; i < incomingPacket.len; i++)
        {
          u8g.print(char(incomingPacket.info[i]));
          if (i == 23)
          {
            u8g.setPrintPos(0, 36);
          }
          if (i == 46)
          {
            u8g.setPrintPos(0, 45);
          }
          if (i == 69)
          {
            u8g.setPrintPos(0, 54);
          }
          if (i == 92)
          {
            u8g.setPrintPos(0, 63);
          }
        }
      }
      while ( u8g.nextPage() );
    }
  }
  else
  {
    u8g.firstPage();
    do
    {
      u8g.setFont(u8g_font_6x10);
      u8g.setPrintPos(0, 12);
      printTime();
      u8g.setPrintPos(60, 12);
     // u8g.print(F("Volts: "));
     // u8g.print(result1);
      u8g.setPrintPos(0, 24);
      u8g.print(F("SAT's: "));
      u8g.print(gps.satellites.value());
      u8g.setPrintPos(60, 24);
     // u8g.print(gps.speed.kmph(), 2);
     // u8g.print(F(" Km/H"));
      u8g.setPrintPos(0, 36);
      u8g.print(F("LAT: "));
      u8g.print(gps.location.lat(), 6);
      u8g.setPrintPos(0, 48);
      u8g.print(F("LON: "));
      u8g.print(gps.location.lng(), 6);
      u8g.setPrintPos(0, 60);
      u8g.print(F("LOCATOR: "));
      u8g.print(MH_txt);
    }
    while ( u8g.nextPage() );
  }

  if (buttonState == HIGH)
  {
    paket = false;
  }

  ////////////////////////////////////////////////////////////////////////////////////
  // Check for when to Tx packet
  ////////////////////////////////////////////////////////////////////////////////////

  // Only check the below if locked satellites > 3
  if ( gps.satellites.value() >= 3 && (millis() - lastTx) > 15000 && gps.hdop.value() < 500 )
  {
    // Check for heading between 70 and 95degrees in more than 50m and previousHeading is defined (for left or right turn)
    if ( headingDelta > 70 && headingDelta < 95 && lastheadingDelta > 50 && lastTxdistance > 50 && previousHeading != 400)
    {
      TxtoRadio();
    } // endif headingDelta

    // if headingdelta < 95 deg last Tx distance is more than 100m+ 4 x Speed lower than 60 KM/h OR lastdistance is more than 600m
    // if more than 110 deg + 500 meter
    else if ( (headingDelta < 95 && lastTxdistance > ( 100 + (gps.speed.kmph() * 4) + gps.hdop.value()) && gps.speed.kmph() < 60) || (lastTxdistance > 600  && gps.speed.kmph() < 60))
    {
      TxtoRadio();
    }

    // lastTxdistance >= 10 x Speed in Km/H and speed more than 60 Km/H = minimum 600m
    else if ( lastTxdistance > (gps.speed.kmph() * 10) && gps.speed.kmph() >= 60 )
    {
      TxtoRadio();
    }

    else if ( (millis() - lastTx) > 900000) // every 15 minutes
    {
      TxtoRadio();
    } // endif of check for lastTx > txInterval
  } // Endif check for satellites
}

void TxtoRadio()
{
    char latOut[15], lngOutTmp[15];
    float latDegMin, lngDegMin = 0.0;
    latDegMin = convertDegMin(gps.location.lat());
    lngDegMin = convertDegMin(gps.location.lng());
    dtostrf(latDegMin, 2, 2, latOut );
    dtostrf(lngDegMin, 2, 2, lngOutTmp );

    char lngOut[15] = {
    };
    int cur_len = strlen(latOut);
    latOut[cur_len] = nw;
    latOut[cur_len + 1] = '\0';

    if (lngDegMin < 10000)
    {
      int n = strlen(lngOutTmp);
      lngOut[0] = '0';
      for (int i = 1; i < n + 1; i++)
      {
        lngOut[i] = lngOutTmp[i - 1];
      }
      cur_len = strlen(lngOut);
      lngOut[cur_len] = '\0';
    }
    else {
      strncpy(lngOut, lngOutTmp, 15);
    }

    lngOut[cur_len] = wl;
    lngOut[cur_len + 1] = '\0';

    APRS_setLat(latOut);
    APRS_setLon(lngOut);
 
    int iiii = txCounter;
    char strii[10];
    sprintf(strii, "%d", iiii);
    char *comment = strii;

    APRS_setPower(1);
    APRS_setHeight(0);
    APRS_setGain(3);
    APRS_setDirectivity(0);

    APRS_sendLoc(comment, strlen(comment));
    APRS_sendLoc(comment, strlen(comment));

    // Reset the Tx internal
    lastTxdistance = 0;
    lastTxLat = gps.location.lat();
    lastTxLng = gps.location.lng();
    previousHeading = lastbearing;
  lastTx = millis();
  txCounter++;
} // endof TxtoRadio()

float convertDegMin(float decDeg)
{
  float DegMin;
  int intDeg = decDeg;
  decDeg -= intDeg;
  decDeg *= 60;
  DegMin = ( intDeg * 100 ) + decDeg;
  return DegMin;
}

static void printTime()
{
  if (hour < 10)
    u8g.print(F("0"));
  u8g.print(hour);
  u8g.print(F(":"));
  if (minute < 10)
    u8g.print(F("0"));
  u8g.print(minute);
  u8g.print(F(":"));
  if (second < 10)
    u8g.print(F("0"));
  u8g.print(second);
}
