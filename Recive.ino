#include <LibAPRS.h>
#include "U8glib.h"


U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);

#define ADC_REFERENCE REF_3V3
#define OPEN_SQUELCH true

char call[7] = "LZ1PPL";
int call_ssid = 0;
int freq=144.8000;
int volume=8;
unsigned long preamble = 350UL;
unsigned long tail = 50UL;

boolean gotPacket = false;
AX25Msg incomingPacket;
uint8_t *packetData;
void aprs_msg_callback(struct AX25Msg *msg) {
  // If we already have a packet waiting to be
  // processed, we must drop the new one.
  if (!gotPacket) {
    // Set flag to indicate we got a packet
    gotPacket = true;

    // The memory referenced as *msg is volatile
    // and we need to copy all the data to a
    // local variable for later processing.
    memcpy(&incomingPacket, msg, sizeof(AX25Msg));

    // We need to allocate a new buffer for the
    // data payload of the packet. First we check
    // if there is enough free RAM.
    if (freeMemory() > msg->len) {
      packetData = (uint8_t*)malloc(msg->len);
      memcpy(packetData, msg->info, msg->len);
      incomingPacket.info = packetData;
    } else {
      // We did not have enough free RAM to receive
      // this packet, so we drop it.
      gotPacket = false;
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("AT+DMOSETGROUP=0,freq,freq,0000,0,0000");
  delay(100);
  Serial.println("AT+DMOSETVOLUME=volume");
  delay(100);
  Serial.println("AT+SETFILTER=0,0,0");
  delay(100);
  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  APRS_setCallsign(call, call_ssid);  
}

void processPacket() {
  if (gotPacket) {
    gotPacket = false;
      u8g.firstPage();
      do
      {
        u8g.setFont(u8g_font_5x8);
        u8g.setPrintPos(0, 9);
        u8g.print(incomingPacket.src.call);
        if (incomingPacket.src.ssid > 0)
        {
          u8g.print(F("-"));
          u8g.print(incomingPacket.src.ssid);
        } // SSID
        //u8g.print(" >");
        u8g.setPrintPos(0, 18);
        //u8g.print(incomingPacket.dst.call);             // address callsign or MIC-E data
        //u8g.print(":");                                 // :
        for (int i = 1; i < incomingPacket.len; i++)
        {
          u8g.print(char(incomingPacket.info[i]));// data contents
          if (i == 23)
          {
            u8g.setPrintPos(0, 26);
          }
          if (i == 46)
          {
            u8g.setPrintPos(0, 34);
          }
        }
      }
      while( u8g.nextPage() );
    // Remeber to free memory for our buffer!
    free(packetData);
  }
}

void loop() 
{
  processPacket();
}
