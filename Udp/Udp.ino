/*
  UDPSendReceive.pde:
  This sketch receives UDP message strings, prints them to the serial port
  and sends an "acknowledge" string back to the sender

  A Processing sketch is included at the end of file that can be used to send
  and received messages for testing with a computer.

  created 21 Aug 2010
  by Michael Margolis

  This code is in the public domain.

  adapted from Ethernet library examples
*/


#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <string.h>

#define STR_TAG_CTRL "TAG_CTRL"
#define STR_TAG_CLIENT "TAG_CLIENT"

#ifndef STASSID
#define STASSID "belief"
#define STAPSK  "Dian12345"
#endif

#define STR_SW_ON  "SW_ON"
#define STR_SW_OFF "SW_OFF"
#define STR_MODE_SQUARE "MODE_SQUARE"

#define  TAG_CTRL  0
#define  TAG_CLIENT  1
//#define NODE_TYPE TAG_CTRL
#define NODE_TYPE TAG_CLIENT

unsigned int localPort = 8888;      // local port to listen on

enum {
  HOST_PC,
  TAG_CLIENT_1,
  TAG_CLIENT_2,
  TAG_CLIENT_3,

  MODE_HOLD,
  MODE_SQUARE,

  SW_OFF = 0,
  SW_ON = 1,

  GPIO_SW = 0,
};



// buffers for receiving and sending data
#define UDP_RX_BUFF_SIZE 64
char packetBuffer[UDP_RX_BUFF_SIZE + 1]; //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged\r\n";       // a string to send back

WiFiUDP Udp;

IPAddress IP_PC(192,168,43,88);
IPAddress IP_TAG_1(192,168,43,174);
IPAddress IP_TAG_2(192,168,43,88);
IPAddress IP_TAG_3(192,168,43,88);
unsigned int remotePort_PC = 8888; 
unsigned int remotePort_TAG_1 = 8888; 
unsigned int remotePort_TAG_2 = 8888; 
unsigned int remotePort_TAG_3 = 8888; 


volatile int outputMode = MODE_HOLD;
volatile int swState = LOW;
unsigned long previousMillis = 0;
const long interval = 1000; // 1s


void udp_rec_proc(void)
{
  if(Udp.remoteIP() == IP_PC)
  {
    Serial.print("rec from PC\n");
    if(strstr(packetBuffer, STR_SW_ON))
    {
      Serial.print("rec  STR_SW_ON\n");
      outputMode = MODE_HOLD;
      swState = SW_ON;
    }
    else if(strstr(packetBuffer, STR_SW_OFF))
    {
      Serial.print("rec  STR_SW_OFF\n");
      outputMode = MODE_HOLD;
      swState = SW_OFF;
    }
    else if(strstr(packetBuffer, STR_MODE_SQUARE))
    {
      Serial.print("rec  MODE_SQUARE\n");
      outputMode = MODE_SQUARE;
      swState = SW_OFF;
    }
    else
    {
      Serial.print("rec cmd not support\n");
    }
  }
#if NODE_TYPE == TAG_CTRL
  else if(Udp.remoteIP() == IP_TAG_1)
  {
    Serial.print("rec from TAG 1");
  }
  else if(Udp.remoteIP() == IP_TAG_2)
  {
    Serial.print("rec from TAG 2");
  }
  else if(Udp.remoteIP() == IP_TAG_3)
  {
    Serial.print("rec from TAG 3");
  }
  else
  {
    Serial.print("rec from xxx");
  }
#else
  else
  {
    if(strstr(packetBuffer, STR_SW_ON))
    {
      Serial.print("rec  STR_SW_ON\n");
      digitalWrite(GPIO_SW, LOW);
//      digitalWrite(GPIO_SW, HIGH);
    }
    else if(strstr(packetBuffer, STR_SW_OFF))
    {
      Serial.print("rec  STR_SW_OFF\n");
      digitalWrite(GPIO_SW, HIGH);
//      digitalWrite(GPIO_SW, HIGH);
    }
  }
#endif
}

void udp_sendTag(IPAddress IP, unsigned int remotePort, const char *state)
{
    Udp.beginPacket(IP, remotePort);
    Udp.write(IP.toString().c_str());
    Udp.write("->");
    Udp.write(state);
    Udp.endPacket();
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
  Serial.printf("UDP server on port %d\n", localPort);
  Udp.begin(localPort);

 
  Udp.beginPacket(IP_PC, remotePort_PC);

#if NODE_TYPE == TAG_CTRL
  Udp.write(STR_TAG_CTRL" | ""node get online");
#else
  Udp.write(STR_TAG_CLIENT" | ""node get online");
#endif
  Udp.endPacket();
  Serial.printf("online send \n");

// init gpio
  pinMode(GPIO_SW, OUTPUT);
  digitalWrite(GPIO_SW, LOW);
}
void loop() 
{
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) 
  {
    Serial.printf("Received packet of size %d from %s:%d\n    (to %s:%d, free heap = %d B)\n",
                  packetSize,
                  Udp.remoteIP().toString().c_str(), Udp.remotePort(),
                  Udp.destinationIP().toString().c_str(), Udp.localPort(),
                  ESP.getFreeHeap());

    // read the packet into packetBufffer
    int n = Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[n] = 0;
    Serial.println("Contents:");
    Serial.println(packetBuffer);
    // proc packet here
    udp_rec_proc();

    // ACK
#if NODE_TYPE == TAG_CLIENT
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
#elif NODE_TYPE == TAG_CTRL
    Udp.beginPacket(IP_PC, remotePort_PC);
#endif
    Udp.write("ack> ");
    Udp.write(packetBuffer);
    Udp.endPacket();

  }

#if NODE_TYPE == TAG_CTRL
  unsigned long currentMillis = millis();
  if ((outputMode == MODE_SQUARE) && (currentMillis - previousMillis >= interval)) 
  {
    previousMillis = currentMillis;
    if (swState == LOW) 
    {
      swState = HIGH;  
    } 
    else 
    {
      swState = LOW;  
    }
  }

  // update tag state
//  if(0)
  {
    delay(200);  
    udp_sendTag(IP_TAG_1, remotePort_TAG_1, swState? STR_SW_ON : STR_SW_OFF);
    delay(200);  
    udp_sendTag(IP_TAG_2, remotePort_TAG_2, swState? STR_SW_ON : STR_SW_OFF);
    delay(200);  
    udp_sendTag(IP_TAG_3, remotePort_TAG_3, swState? STR_SW_ON : STR_SW_OFF);
  }
#endif

}
