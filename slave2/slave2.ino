#include <Datagram.h>
#include <SoftwareSerial.h>
#define SSerialRX        5  //Serial Receive pin DI
#define SSerialTX        6  //Serial Transmit pin RO

#define SSerialTxControl 3   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

SoftwareSerial RS485Serial(SSerialRX, SSerialTX); // RX, TX
unsigned long startTime, endTime;
ELECHOUSE_CC1101 cc1102;
Datagram manager(cc1102,0x0101);  // driver, address
byte temp_buf[40];
byte data[10];
uint16_t thisAddress;
int temp;
byte num;
void setup()
{
  Serial.begin(9600);
  manager.init(55);
  manager.SetReceive();
  pinMode(SSerialTxControl, OUTPUT);  
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
  RS485Serial.begin(9600);   // set the data rate 
}
void loop()
{
  manager.SetReceive();
  if(manager.available())
  {
    if(manager.receive(temp_buf) && manager.headerTo() == thisAddress)
    {
      if(manager.headerType() == INSTRUCTION_FROM_GATEWAY && manager.headerFrom() == /*master's address*/)//gateway로부터의 명령
      {
        //manager.send(thisAddress, /*master's address*/, thisAddress, /*master's address*/, ACK, 0, 0, 0, temp_buf, sizeof(temp_buf));//ACK 안보내도 됨
        for(int i = 0;i < 10;i++)
        {
          data[i] = temp_buf[i];
        }
        digitalWrite(SSerialTxControl, RS485Transmit);
        RS485Serial.write(data, sizeof(data));
        digitalWrite(SSerialTxControl, RS485Receive);
      }
      if(manager.headerType() == INSTRUCTION_FROM_RC && manager.headerFrom() == /*roomcon's address*/)//룸콘으로부터의 명령
      {
        manager.send(thisAddress, /*roomcon's address*/, thisAddress, /*roomcon's address*/, ACK, 0, 0, 0, temp_buf, sizeof(temp_buf));
        for(int i = 0;i < 10;i++)
        {
          data[i] = temp_buf[i];
        }
        digitalWrite(SSerialTxControl, RS485Transmit);
        RS485Serial.write(data, sizeof(data));
        digitalWrite(SSerialTxControl, RS485Receive);
      }
      if(manager.headerType() == SCAN_REQUEST_TO_SLAVE && manager.headerFrom() == /*roomcon's address*/)//룸콘으로부터의 스캔 명령
      {
        for(int i = 0;i < 10;i++)
        {
          data[i] = temp_buf[i];
        }
        digitalWrite(SSerialTxControl, RS485Transmit);
        RS485Serial.write(data, sizeof(data));
        digitalWrite(SSerialTxControl, RS485Receive);
        while(1)
        {
          if(RS485Serial.available())
          {
            num = 0;
            while(1)
            {
              temp = RS485Serial.read();
              if(temp != -1)
              {
                data[num] = temp;
                num++;
              }
              if(num == 10)
               break;
            }
          }
        }
        manager.send(thisAddress, /*roomcon's address*/, thisAddress, /*roomcon's address*/, SCAN_RESPONSE_TO_RC, 0, 0, 0, data, sizeof(data));
      }
    }
  }
}
