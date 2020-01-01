//
// Use one MAX7219 to control 8-digit seven-segment display
// the display module: http://www.icshop.com.tw/product_info.php/products_id/20686
//
// MAX7219 datasheet: https://datasheets.maximintegrated.com/en/ds/MAX7219-MAX7221.pdf
//

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiUdp.h>


#include <BME280I2C.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

#ifndef _BV
  #define _BV(bit) (1<<(bit))
#endif

#define DISPLAY_ADDRESS   0x70

Adafruit_7segment clockDisplay = Adafruit_7segment();

#include <ESP8266HTTPClient.h>

const char* WSSD = "crackhouse";
const char* WPW = "danmotherf4cker";

ESP8266WiFiMulti WiFiMulti;

const int buttonPin = 2;     // the number of the pushbutton pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

IPAddress timeServerIP;
const char* ntpServerName = "pool.ntp.org";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

unsigned int counter = 0;
unsigned int ntp_update = 60000;
unsigned int ntp_counter = ntp_update;
unsigned int digit_base = 10;
unsigned int last_time = 0;
unsigned int now = 0;
unsigned int no_time = 1;
unsigned int envdisplay = 0;
unsigned long epoch = 0;
int tzOffset = 1;
unsigned int pot = 0;
unsigned int previousPot = 0;
unsigned int brightness = 15;

unsigned int localPort = 2390;      // local port to listen for UDP packets


bool blinkColon = false;


void setup()  
{

  Serial.begin(115200);
  Serial.println();

  // Setup the display.
  clockDisplay.begin(DISPLAY_ADDRESS);

  while(!bme.begin()){
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }
  
  pinMode(buttonPin, INPUT);
  
  // bme.chipID(); // Deprecated. See chipModel().
  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }

  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WSSD, WPW);

  Serial.println("WiFi connecting");
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    counter++;
    writeNumber(counter);
  }
  
  Serial.println("");
  
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());
  now = millis();
  last_time=now;
}

void loop()  {
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP); 

  unsigned int number = 0; // numer to be displayed on the 7segment

  if ((ntp_counter == ntp_update) || no_time) { 
    ntp_counter = 0;
    no_time = 1;
    sendNTPpacket(timeServerIP); // send an NTP packet to a time server
    // wait to see if a reply is available
    delay(1000);
    int cb = udp.parsePacket();
    if (!cb) {
      Serial.println("no packet yet");
    }
    else {
      no_time=0;
      Serial.print("packet received, length=");
      Serial.println(cb);
      // We've received a packet, read the data from it
      udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

      //the timestamp starts at byte 40 of the received packet and is four bytes,
      // or two words, long. First, esxtract the two words:

     unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
     unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
     // combine the four bytes (two words) into a long integer
     // this is NTP time (seconds since Jan 1 1900):
     unsigned long secsSince1900 = highWord << 16 | lowWord;
     Serial.print("Seconds since Jan 1 1900 = " );
     Serial.println(secsSince1900);

     // now convert NTP time into everyday time:
     Serial.print("Unix time = ");
     // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
     const unsigned long seventyYears = 2208988800UL;
     // subtract seventy years:
     epoch = secsSince1900 - seventyYears;
     // print Unix time:
     Serial.println(epoch);

     // print the hour, minute and second:
     Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
     Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
     Serial.print(':');
     if ( ((epoch % 3600) / 60) < 10 ) {
       // In the first 10 minutes of each hour, we'll want a leading '0'
       Serial.print('0');
     }
     Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
     Serial.print(':');
     if ( (epoch % 60) < 10 ) {
       // In the first 10 seconds of each minute, we'll want a leading '0'
       Serial.print('0');
     }
     Serial.println(epoch % 60); // print the second
    }
  }
  
  now = millis();

  if ((now-last_time) >= 1000) {
    last_time=now;
    epoch=epoch+1;  
    writeTime(epoch);
    ntp_counter = ntp_counter+1;
  } 

  pot = analogRead(A0);
  if ( (pot > 10) && ((previousPot > (pot+10)) || (previousPot < (pot-10)))) {
    Serial.println("Pottriggered");
    Serial.println(pot);
    Serial.println(previousPot);
    previousPot = pot;
    brightness = pot/45;
    blinkColon = false;
    writeTime(epoch);
  }
}


void writeNumber(unsigned int number) {
  // Now print the time value to the display.
  clockDisplay.print(number, DEC);
    // Now push out to the display the new values that were set above.
  clockDisplay.writeDisplay();
}  

void writeTime(long epoch) {

  float temp(NAN), hum(NAN), pres(NAN);
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  bme.read(pres, temp, hum, tempUnit, presUnit);
  
  buttonState = digitalRead(buttonPin);

  if (buttonState == LOW) {
      if (envdisplay == 0) {
        clockDisplay.print(temp, DEC);
        envdisplay++;
      } else if (envdisplay == 1) {
        clockDisplay.print(hum, DEC);
        envdisplay++;
      } else if (envdisplay == 2) {
        clockDisplay.print(pres/100, DEC);
        envdisplay = 0;
      }
  } else {
    unsigned int hours = (epoch  % 86400L) / 3600;
    unsigned int minutes =  (epoch  % 3600) / 60;
    hours += tzOffset;  
    int displayValue = hours*100 + minutes;
    clockDisplay.print(displayValue, DEC);
    
    //  clockDisplay.setBrightness(1);
    if (hours == 24) {
      // Pad hour 0.
      clockDisplay.writeDigitNum(1, 0);
      clockDisplay.writeDigitNum(0, 0);
    }
    // Also pad when the 10's minute is 0 and should be padded.
    if (hours < 10) {
      clockDisplay.writeDigitNum(0, 0);
    }
    // Also pad when the 10's minute is 0 and should be padded.
    if (minutes < 10) {
    clockDisplay.writeDigitNum(2, 0);
    }
    blinkColon = !blinkColon;
    clockDisplay.drawColon(blinkColon);
  }
  clockDisplay.setBrightness(brightness);
  clockDisplay.writeDisplay();
}



// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}
