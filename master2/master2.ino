#include <SoftwareSerial.h>
#include <Datagram.h>
#define SSerialRX        5  //Serial Receive pin DI
#define SSerialTX        6  //Serial Transmit pin RO

#define SSerialTxControl 3   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

SoftwareSerial RS485Serial(SSerialRX, SSerialTX); // RX, TX
ELECHOUSE_CC1101 cc1102;
ELECHOUSE_CC1101 cc1103;
Datagram manager(cc1102, 0x0110);//외부통신
Datagram manager2(cc1103, 0x0110);//내부통신
byte gatewaySeqNum = -1;
byte roomconSeqNum = -1;
byte temp_buf[40];
uint16_t thisAddress;
byte maxNum;
byte data[10];
byte fromGateway[10];
unsigned long startTime;
byte retry;
bool recvFromRC = false;
int temp;
byte num;
void setup() 
{
  Serial.begin(9600);
  pinMode(SSerialTxControl, OUTPUT);  
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
  RS485Serial.begin(9600);   // set the data rate 
  manager.init(55);
  manager.SetReceive();
  manager.fromControllerToGateway();
  manager2.init(55);
  manager2.SetReceive();
}
void loop() 
{
  manager.SetReceive();
  if(manager.available())
  {
    if(manager.recvData(temp_buf) && manager.headerTo() == thisAddress)
    {
      manager.send(thisAddress, manager.headerFrom(), thisAddress, manager.headerFrom(), ACK, 0, 0, 0, temp_buf, sizeof(temp_buf));
      if(manager.headerDestination() != manager._thisAddress)
      {
        if(!manager.sendToWait(thisAddress, manager.getRouteTo(manager.headerDestination())->next_hop[0], manager.headerSource(), manager.headerDestination(), manager.headerType(), manager.headerData(), manager.headerFlags(), manager.headerSeqNum(), temp_buf, sizeof(temp_buf), /*time*/) && getRouteTo(manager.headerDestination())->next_hop[1] != 0)
          manager.sendToWait(thisAddress, manager.getRouteTo(manager.headerDestination())->next_hop[1], manager.headerSource(), manager.headerDestination(), manager.headerType(), manager.headerData(), manager.headerFlags(), manager.headerSeqNum(), temp_buf, sizeof(temp_buf), /*time*/)//실목적지에다가 전송해줌
      }
      else
      {
        if(manager.headerType() == INSTRUCTION_FROM_GATEWAY && manager.headerFrom() == GATEWAY_ADDR)//gateway로부터 instruction
        {
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
              break;
            }
          }
          for(int i = 0;i < maxNum;i++)
          {
            manager2.sendToWait(thisAddress, /*slave's address*/, thisAddress, /*slave's address*/, INSTRUCTION_FROM_GATEWAY, 0, 0, 0, tempb_buf, sizeof(temp_buf, /*time*/));
          }
        }
        else if(manager.headerType() == SCAN_REQUEST_TO_MASTER && manager.headerFrom() == GATEWAY_ADDR)//gateway로부터 scan요청
        {
          for(int i = 0;i < 10;i++)
          {
            fromGateway[i] = temp_buf[i];
          }
          recvFromRC = false;
          for(int i = 0;i < retry;i++)
          {
            manager2.send(thisAddress, /*roomcon's address*/, thisAddress, /*roomcon's address*/, SCAN_REQUEST_TO_RC_EXTERNAL, 0, 0, 0, temp_buf, sizeof(temp_buf));
            startTime = millis();
            while(millis() - starTime < TIME_TERM)//master가 내부주소가 0이니까 gateway는 얘한테 제일먼저 요청을 보낸다.
            {
              manager2.SetReceive();
              if(manager2.headerType() == SCAN_REQUEST_TO_SLAVE && manager2.headerFrom() == /*roomcon's address*/ && manager2.headerTo() == thisAddress)//룸콘으로부터 scan요청
              {
                recvFromRC = true;
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
                    manager2.send(thisAddress, /*roomcon's address*/, thisAddress, /*roomcon's address*/, SCAN_RESPONSE_TO_RC, 0, 0, 0, data, sizeof(data));
                    break;
                  }
                }
              }
            }
            if(recvFromRC)
              break;
          }
        }
      }
    }
  }
  manager2.SetReceive();
  if(manager2.available())
  {
    if(manager2.recvData(temp_buf) && manager2.headerTo() == thisAddress)
    {
      if(manager2.headerType() == MAX_NUM_TO_MASTER && manager2.headerFrom() == /*roomcon's address*/)//roomcon으로 부터 slave 개수를 받음
      {
        manager2.send(thisAddress, /*roomcon's address*/, thisAddress, /*roomcon's address*/, ACK, 0, 0, 0, temp_buf, sizeof(temp_buf));
        maxNum = temp_buf[0];
      }
      else if(manager2.headerType() == INSTRUCTION_FROM_RC && manager2.headerFrom() == /*roomcon's address*/)//룸콘으로부터 instruction
      {
        //manager2.send(thisAddress, /*roomcon's address*/, thisAddress, /*roomcon's address*/, ACK, 0, 0, 0, temp_buf, sizeof(temp_buf));
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
            break;
          }
        }
      }
      else if(manager2.headerType() == SCAN_REQUEST_TO_SLAVE && manager2.headerFrom() == /*roomcon's address*/ && manager2.headerTo() == thisAddress)//룸콘으로부터 scan요청
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
            manager2.send(thisAddress, /*roomcon's address*/, thisAddress, /*roomcon's address*/, SCAN_RESPONSE_TO_RC, 0, 0, 0, data, sizeof(data));
            break;
          }
        }
      }
      else if(manager2.headerType() == SCAN_RESPONSE_TO_MASTER && manager2.headerFrom() == /*roomcon's address*/)//룸콘으로부터 scan끝
      {
        manager2.send(thisAddress, /*roomcon's address*/, thisAddress, /*roomcon's address*/, ACK, 0, 0, 0, temp_buf, sizeof(temp_buf));
        digitalWrite(SSerialTxControl, RS485Transmit);
        RS485Serial.write(fromGateway, sizeof(fromGateway));
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
            break;
          }
        }
        if(!manager.sendToWait(thisAddress, manager.getRouteTo(GATEWAY_ADDR)->next_hop[0], thisAddress, GATEWAY_ADDR, SCAN_RESPONSE_TO_GATEWAY, 0, 0, 0, data, sizeof(data)) && getRouteTo(/*roomcon's address*/)->next_hop[1] != 0)
          manager.sendToWait(thisAddress, manager.getRouteTo(GATEWAY_ADDR)->next_hop[1], thisAddress, GATEWAY_ADDR, SCAN_RESPONSE_TO_GATEWAY, 0, 0, 0, data, sizeof(data))
      }
    }
  }
}
