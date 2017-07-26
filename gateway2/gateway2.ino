#include <SoftwareSerial.h>
#include <Datagram.h>
#define SSerialRX        5  //Serial Receive pin DI
#define SSerialTX        6  //Serial Transmit pin RO

#define SSerialTxControl 3   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

SoftwareSerial RS485Serial(SSerialRX, SSerialTX); // RX, TX
ELECHOUSE_CC1101 cc1102;
Datagram manager(cc1102, 0x0000);
byte data1[10];
byte data2[32][10];
byte maxOP = 0;
byte curOP = 0;
byte fromFCU[32][10] = {0};
byte fromPC[32][10] = {0};
byte temp_buf[40];
unsigned long startTime = 0;
bool receiveScanResponse = false;
bool receiveACK = false;
byte SeqNum[33] = { 0 };//seqNum을 쓰는 이유 : 중복된 데이터 전송 때문에 일어나는 혼란을 막기위해서
                        //SeqNum 변수는 타입에 상관없이 목적지마다 한개씩 갖고있어야 한다.
unsigned long timeLimit = 60000;
uint8_t retry = 3;//재전송횟수
uint16_t thisAddress;
byte maxNum;
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
  manager.FromGatewayToComtroller();
  attachInterrupt(0, receiveFromPC, FALLING);
}
void receiveFromPC()
{
  if (RS485Serial.available()) 
  {
    /*num = 0;
    while(1)
    {
      temp = RS485Serial.read();
      if(temp != -1)
      {
        data1[num] = temp;
        num++;
      }
      if(num == 10)
       break;
    }*/
    data1[num] = RS485Serial.read();
    num++;
    if(num == 10)
    {
      if(data1[0] == 0xBE)
      {
        if(maxOP == curOP)
        {
          for(int i = 0;i < 10;i++)
          {
            data2[maxOP][i] = data1[i];
          }
        }
        else
        {
          maxOP++
          for(int i = 0;i < 10;i++)
          {
            data2[maxOP][i] = data1[i];
          }
        }
        maxOP++;
        if(maxOP >= 32)
          maxOP = 0;
      }
      for(int i = 0;i < 10;i++)
      {
        fromPC[data1[2]][i] = data1[i];
      }
      if(fromFCU[data1[2]][0] != 0)
      {
        digitalWrite(SSerialTxControl, RS485Transmit);
        RS485Serial.write(fromFCU[data1[2]], sizeof(fromFCU[data1[2]]));
        digitalWrite(SSerialTxControl, RS485Receive);
      }
      else
      {
        data1[0] = 0xAD;
        data1[9] = 0x00;
        for(int i = 0;i < 9;i++)
        {
          data1[9] ^= data1[i];
        }
        digitalWrite(SSerialTxControl, RS485Transmit);
        RS485Serial.write(data1, sizeof(data1));
        digitalWrite(SSerialTxControl, RS485Receive);
      }
      num = 0;
    }
  }
}
void loop()
{
  if(maxOP != curOP)
  {
    if(!manager.sendToWait(thisAddress, getRouteTo(/*master's address*/)->next_hop[0], thisAddress, /*master's address*/, INSTRUCTION_FROM_GATEWAY, 0, 0, 0, data2[curOP], sizeof(data2[curOP]), /*time*/) && getRouteTo(/*master's address*/)->next_hop[1] != 0)
      manager.sendToWait(thisAddress, getRouteTo(/*master's address*/)->next_hop[1], thisAddress, /*master's address*/, INSTRUCTION_FROM_GATEWAY, 0, 0, 0, data2[curOP], sizeof(data2[curOP]), /*time*/);
    curOP++;
    if(curOP >= 32)
      curOP = 0;
  }
  for(int i = 0;i < maxNum;i++)
  {
    if(maxOP != curOP)
    {
      if(!manager.sendToWait(thisAddress, getRouteTo(/*master's address*/)->next_hop[0], thisAddress, /*master's address*/, INSTRUCTION_FROM_GATEWAY, 0, 0, 0, data2[curOP], sizeof(data2[curOP]), /*time*/) && getRouteTo(/*master's address*/)->next_hop[1] != 0)
        manager.sendToWait(thisAddress, getRouteTo(/*master's address*/)->next_hop[1], thisAddress, /*master's address*/, INSTRUCTION_FROM_GATEWAY, 0, 0, 0, data2[curOP], sizeof(data2[curOP]), /*time*/);
      curOP++;
      if(curOP >= 32)
        curOP = 0;
    }
    receiveACK = false;
    receiveScanResponse = false;
    if(manager.sendToWait(thisAddress, getRouteTo(/*master's address*/)->next_hop[0], thisAddress, /*master's address*/, INSTRUCTION_FROM_GATEWAY, 0, 0, 0, fromPC[i], sizeof(fromPC[i]), /*time*/))
      receiveACK = true;
    else if(getRouteTo(/*master's address*/)->next_hop[1] != 0)
      receiveACK = manager.sendToWait(thisAddress, getRouteTo(/*master's address*/)->next_hop[1], thisAddress, /*master's address*/, INSTRUCTION_FROM_GATEWAY, 0, 0, 0, fromPC[i], sizeof(fromPC[i]), /*time*/);
    if(receiveACK)
    {
      startTime = millis();
      while(millis() - startTime < timeLimit)
      {
        manager.SetReceive();
        if(manager.available())
        {
          if(recvData(temp_buf) && manager.headerType() == SCAN_RESPONSE_TO_GATEWAY && manager.headerSource() == /*master's address*/)
          {
            receiveScanResponse = true;
            for(int j = 0;j < 10;j++)
            {
              fromFCU[i][j] = temp_buf[j];
            }
            break;
          }
        }
      }
      if(!receiveScanResponse)
      {
        fromFCU[i][0] = 0xEE;
      }
    }
  }
}
