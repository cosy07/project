//<룸콘>
//하는 일 : 스캔, 사용자가 룸콘 조작시 FCU들에게 명령
#include <SoftwareSerial.h>
#include <Datagram.h>
#define SSerialRX        5  //Serial Receive pin DI
#define SSerialTX        6  //Serial Transmit pin RO

#define SSerialTxControl 3   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

SoftwareSerial RS485Serial(SSerialRX, SSerialTX); // RX, TX
ELECHOUSE_CC1101 cc1102;
Datagram manager(cc1102, 0x0110);
byte data[10];
byte fromController[16][10] = {0};
byte fromRC[16][10] = {0};
byte temp_buf[40];
unsigned long startTime = 0;
byte maxNum;
int8_t receiveSeqNum = -1;
int8_t sendSeqNum = 0;
uint16_t thisAddress;
bool receiveScanAck = false;
int temp;
byte num = 0;
void setup()
{
  Serial.begin(9600);
  pinMode(SSerialTxControl, OUTPUT);  
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
  RS485Serial.begin(9600);   // set the data rate 
  manager.init(55);
  manager.SetReceive();
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
      if(data[2] > maxNum)
        maxNum = data[2];
      else
        break;
    }
  }
  temp_buf[0] = maxNum;
  while(manager.sendToWait(thisAddress, /*master's address*/, thisAddress, /*master's address*/, MAX_NUM_TO_MASTER, 0, 0, 0, sendSeqNum, temp_buf, size(temp_buf)));
    sendSeqNum++;
  //master에게 maxNum보내서 전체개수 알려줌
  attachInterrupt(0, receiveFromRC, FALLING);
}
void receiveFromRC()
{
  if (RS485Serial.available()) 
  {
    data[num] = RS485Serial.read();
    num++;
    if(num == 10)
    {
      if(data[0] == 0xC5)//명령이 오면 slave들에게 바로 보내줌
      {
        manager.send(thisAddress, /*slave's broadcast address*/, thisAddress, /*slave's broadcast address*/, INSTRUCTION_FROM_RC, 0, 0, 0, data, sizeof(data));
      }
      for(int i = 0;i < 10;i++)
      {
        fromRC[data[2]][i] = data[i];
      }
      if(fromController[data[2]][0] != 0)
      {
        digitalWrite(SSerialTxControl, RS485Transmit);
        RS485Serial.write(fromController[data[2]], sizeof(fromController[data[2]]));
        digitalWrite(SSerialTxControl, RS485Receive);
      }
      else
      {
        data[0] = 0xB5;
        data[9] = 0x00;
        for(int i = 0;i < 9;i++)
        {
          data[9] ^= data[i];
        }
        digitalWrite(SSerialTxControl, RS485Transmit);
        RS485Serial.write(data, sizeof(data));
        digitalWrite(SSerialTxControl, RS485Receive);
      }
      num = 0;
    }
  }
}
void loop()
{
  manager.SetReceive();
  if(manager.available())
  {
    if(manager.recvData(temp_buf) && manager.headerTo() == thisAddress && manager.headerFrom() == /*master's address*/)
    {
      seqNum++;
      manager.send(thisAddress, /*master's address*/, thisAddress, /*master's address*/, ACK, 0, 0, 0, temp_buf, sizeof(temp_buf));
      if(manager.headerType() == SCAN_REQUEST_TO_RC_EXTERNAL && manager.headerSeqNum() > receiveSeqNum)
      {
        receiveSeqNum = manager.headerSeqNum();
        for(int i = 0;i < maxNum;i++)
        {
          receiveScanAck = false;
          for(int j = 0;j < 3;j++)//retry
          {
            manager.send(thisAddress, /*slave's address*/, thisAddress, /*slave's address*/, SCAN_REQUEST_TO_SLAVE, 0, 0, 0, fromRC[i], sizeof(fromRC[i]));
            startTime = millis();
            while(millis() - starTime < TIME_TERM)
            {
              manager.SetReceive();
              if(manager.available())
              {
                if(manager.recvData(temp_buf) && manager.headerTo() == thisAddress && manager.headerFrom() == /*slave's address*/ && manager.headerType() == SCAN_RESPONSE_TO_RC && temp_buf[2] == i)
                {
                  receiveScanAck = true;
                  for(int k = 0;k < 10;k++)
                  {
                    fromController[temp_buf[2]][k] = temp_buf[k];
                  }
                  break;
                }
              }
            }
            if(receiveScanAck)
              break;
          }
          if(!receiveScanAck)
          {
            fromController[i][0] = 0xEE;
          }
        }
        if(manager.sendToWait(thisAddress, /*master's address*/, thisAddress, /*master's address*/, SCAN_RESPONSE_TO_MASTER, 0, 0, sendSeqNum, temp_buf, sizeof(temp_buf)))
          sendSeqNum++;
      }        
    }
  }
}
