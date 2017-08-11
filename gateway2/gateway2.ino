#include <SoftwareSerial.h>
#include <Datagram.h>
#define SSerialRX        5  //Serial Receive pin DI
#define SSerialTX        6  //Serial Transmit pin RO

#define SSerialTxControl 3   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

SoftwareSerial RS485Serial(SSerialRX, SSerialTX); // RX, TX
ELECHOUSE_CC1120 cc1120;
Datagram manager(cc1120, 0x0000);
byte data1[10];
byte fromPC[32][10];
byte control_message[10][10];
byte maxOP = 0;
byte curOP = 0;
byte master_id = 1;
byte group_id;
byte fromFCU[32][10] = {0};
byte temp_buf[40];
byte unreceiveFirstTime[32] = {0};
byte unreceiveNum[32] = {0};
byte SeqNum[33] = { 0 };//seqNum을 쓰는 이유 : 중복된 데이터 전송 때문에 일어나는 혼란을 막기위해서
                        //SeqNum 변수는 타입에 상관없이 목적지마다 한개씩 갖고있어야 한다.
uint16_t master_address;

uint16_t thisAddress = 0x0400;
byte maxNum;
int temp;
byte num = 0;
void setup()
{
  Serial.begin(9600);
  pinMode(SSerialTxControl, OUTPUT);  
  digitalWrite(SSerialTxControl, RS485Receive);  // Init Transceiver
  RS485Serial.begin(9600);   // set the data rate 
  manager.init(1);
  manager.SetReceive();
  manager.FromGatewayToMaster();
  attachInterrupt(0, receiveFromPC, FALLING);
}
void receiveFromPC()
{
    if (RS485Serial.available()) 
    {  
      data1[num] = RS485Serial.read();
      num++;
      if(num == 10)
      {
        if(data1[0] == 0xBE)  // Control message
        {
            if(maxOP == curOP)
            {
              for(int i = 0;i < 10;i++)
                control_message[maxOP][i] = data1[i];
            }
            else
            {
              maxOP++;
              for(int i = 0;i < 10;i++)
                control_message[maxOP][i] = data1[i];
            }
            maxOP++;
            if(maxOP >= 32)
                maxOP = 0;
        }  // end of if(data1[0] == 0xBE) 
        group_id = data1[2]; //Group ID 
        for(int i = 0;i < 10;i++)
        {
          fromPC[group_id][i] = data1[i];
        }
        if(fromFCU[group_id][0] != 0)
        {
          digitalWrite(SSerialTxControl, RS485Transmit);
          RS485Serial.write(fromFCU[group_id], sizeof(fromFCU[group_id]));
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
      }  // end of  if(num == 10)
    }
}
void loop()
{
    if (master_id >31)
        master_id =1;
    master_address = thisAddress | (uint16_t)(master_id<<5) ;
    
    manager.G_handle_CONTROL_message(&maxOP, &curOP, control_message, fromFCU);  // if there is any control message from PC, handle it.
                                           // the gateway should wait until the transmission of CONTROL message succeeds.
                                                     
    if(!manager.G_handle_SCAN_message(master_address, fromPC[master_id], fromFCU)) // send a SCAN message to a master node.  Q : (master scan) or (slave scan request to master) ?
    {
      if(unreceiveNum[master_id] == 0)
      {
        unreceiveFirstTime[master_id] = millis();
      }
      unreceiveNum[master_id]++;
      if(millis() - unreceiveFirstTime[master_id] < 10000)//10s
      {
        if(unreceiveNum[master_id] >= 5)
        {
          unreceiveNum[master_id] = 0;
          unreceiveFirstTime[master_id] = 0;
          manager.G_discoverNewPath(master_address);
        }
      }
      else
      {
        unreceiveNum[master_id] = 1;
        unreceiveFirstTime[master_id] = millis();
      }
    }
    master_id ++;
}
