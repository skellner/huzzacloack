//
// Use one MAX7219 to control 8-digit seven-segment display
// the display module: http://www.icshop.com.tw/product_info.php/products_id/20686
//
// MAX7219 datasheet: https://datasheets.maximintegrated.com/en/ds/MAX7219-MAX7221.pdf
//

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WiFiUdp.h>
//#include <ArduinoJson.h>

#include <ESP8266HTTPClient.h>

const char* WSSD = "crackhouse";
const char* WPW = "danmotherf4cker";

ESP8266WiFiMulti WiFiMulti;

IPAddress timeServerIP;
const char* ntpServerName = "pool.ntp.org";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

// the MAX7219 address map (datasheet table 2)
#define MAX7219_DECODE_REG      (0x09)
#define MAX7219_INTENSITY_REG   (0x0A)
#define MAX7219_SCANLIMIT_REG   (0x0B)
#define MAX7219_SHUTDOWN_REG    (0X0C)
#define MAX7219_DISPLAYTEST_REG (0x0F)
#define MAX7219_DIGIT_REG(pos)  ((pos) + 1)

// shutdown mode (datasheet table 3)
#define MAX7219_OFF             (0x0)
#define MAX7219_ON              (0x1)

// pin 13 of MAX7219 (CLK)
const int clock_pin = D2;
// pin 12 of MAX7219 (LOAD)
const int data_latch_pin = D1;
// pin 1 of MAX7219 (DIN)
const int data_input_pin = D3;

// digit pattern for a 7-segment display. datasheet table 5
const byte digit_pattern[17] =
{
  B01111110,  // 0
  B00110000,  // 1
  B01101101,  // 2
  B01111001,  // 3
  B00110011,  // 4
  B01011011,  // 5
  B01011111,  // 6
  B01110000,  // 7
  B01111111,  // 8
  B01111011,  // 9
  B01110111,  // A
  B00011111,  // b
  B01001110,  // C
  B00111101,  // d
  B01001111,  // E
  B01000111,  // F
  B00000001   // -
};

#define DP_FLAG       (B10000000)
#define NUM_OF_DIGITS (8)

unsigned int counter = 0;
unsigned int ntp_update = 60000;
unsigned int ntp_counter = ntp_update;
unsigned int digit_base = 10;
unsigned int last_time = 0;
unsigned int now = 0;
unsigned int no_time = 1;
unsigned long epoch = 0;
int tzOffset = 1;

unsigned int localPort = 2390;      // local port to listen for UDP packets

// update the register value of MAX7219
void set_register(byte address, byte value)  
{
  digitalWrite(data_latch_pin, LOW);
  shiftOut(data_input_pin, clock_pin, MSBFIRST, address);
  shiftOut(data_input_pin, clock_pin, MSBFIRST, value);
  digitalWrite(data_latch_pin, HIGH);
}

void init_max7219()
{
  // disable test mode. datasheet table 10
  set_register(MAX7219_DISPLAYTEST_REG, MAX7219_OFF);
  // set medium intensity. datasheet table 7
  set_register(MAX7219_INTENSITY_REG, 0x8);
  // turn off display. datasheet table 3
  set_register(MAX7219_SHUTDOWN_REG, MAX7219_OFF);
  // drive 8 digits. datasheet table 8
  set_register(MAX7219_SCANLIMIT_REG, 7);
  // no decode mode for all positions. datasheet table 4
  set_register(MAX7219_DECODE_REG, B00000000);
}

void setup()  
{
  // init pin states
  pinMode(clock_pin, OUTPUT);
  pinMode(data_latch_pin, OUTPUT);    
  pinMode(data_input_pin, OUTPUT);

  // init MAX2719 states
  init_max7219();

  Serial.begin(115200);
  Serial.println();
  Serial.println();


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
  
  while ((now-last_time) < 1000) {
      now = millis();
  }
  last_time=now;
  epoch=epoch+1;  
  writeTime(epoch);
  ntp_counter = ntp_counter+1;
  
}


void writeNumber(unsigned int number) {
  
  int i;
  unsigned int digit_value;
  byte byte_data;
    
  // turn off display first
  set_register(MAX7219_SHUTDOWN_REG, MAX7219_OFF);

  for (i = 0; i < NUM_OF_DIGITS; i++)
  {
    digit_value = number % digit_base;
    number /= digit_base;
    byte_data = digit_pattern[digit_value];
    set_register(MAX7219_DIGIT_REG(i), byte_data);
  }

  // turn on display
  set_register(MAX7219_SHUTDOWN_REG, MAX7219_ON);

}


void writeString(String text) {
  
  int i;
  byte byte_data;
  unsigned int digit_value;
  String character;
    
  // turn off display first
  set_register(MAX7219_SHUTDOWN_REG, MAX7219_OFF);

  for (i = 0; i < NUM_OF_DIGITS; i++)
  {
    if (text.charAt(i) == '-') {
      byte_data = digit_pattern[16];
    } else {
      character = text.charAt(i);
      digit_value = character.toInt();
      byte_data = digit_pattern[digit_value];
    }
    set_register(MAX7219_DIGIT_REG(NUM_OF_DIGITS-1-i), byte_data);
  }

  // turn on display
  set_register(MAX7219_SHUTDOWN_REG, MAX7219_ON);

}

String addLeadingZero(int number) {  
  String result;
  if (number < 10) {
    result = "0" + String(number);
  } else {
    result = String(number);
  }
  return result;
}
  

void writeTime(long epoch) {
  
  int i;
  byte byte_data;
  unsigned int digit_value;
  String character;
  String text;
  unsigned int hours = (epoch  % 86400L) / 3600;
  unsigned int minutes =  (epoch  % 3600) / 60;
  unsigned int seconds = (epoch % 60);
  
  hours += tzOffset;  
  text = addLeadingZero(hours) + "-" + addLeadingZero(minutes) + "-" + addLeadingZero(seconds);
 
  // turn off display first
  set_register(MAX7219_SHUTDOWN_REG, MAX7219_OFF);

  for (i = 0; i < NUM_OF_DIGITS; i++)
  {
    if (text.charAt(i) == '-') {
      byte_data = digit_pattern[16];
    } else {
      character = text.charAt(i);
      digit_value = character.toInt();
      byte_data = digit_pattern[digit_value];
    }
    set_register(MAX7219_DIGIT_REG(NUM_OF_DIGITS-1-i), byte_data);
  }

  // turn on display
  set_register(MAX7219_SHUTDOWN_REG, MAX7219_ON);

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
