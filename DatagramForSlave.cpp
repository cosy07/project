//2017-07-17
// Datagram.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RHDatagram.cpp,v 1.6 2014/05/23 02:20:17 mikem Exp $

#include <DatagramForSlave.h>

DatagramForSlave::DatagramForSlave(ELECHOUSE_CC1120& driver, uint16_t thisAddress)
	:
	_driver(driver),
	_thisAddress(thisAddress),
//	_retransmissions(0),
//	_lastSequenceNumber(0),
//	_timeout(DEFAULT_TIMEOUT),
	_retries(DEFAULT_RETRIES)
{
	// Serial.println(" SendData 2");
	//_retransmissions = 0;
	
}

////////////////////////////////////////////////////////////////////
// Public methods
void DatagramForSlave::init()
{
	setThisAddress(_thisAddress);
	_driver.Init();

}
////////////////////////////////////////////////////////////////////
// Public methods
void DatagramForSlave::init(byte ch)  // Setup Channel number
{

	setThisAddress(_thisAddress);
	_driver.Init(ch);
}

////////////////////////////////////////////////////////////////////
// Public methods
void DatagramForSlave::SetReceive()
{
	_driver.SetReceive();
}

void  DatagramForSlave::setThisAddress(uint16_t thisAddress)
{
	_driver.setThisAddress(thisAddress);
	// Use this address in the transmitted FROM header
	setHeaderFrom(thisAddress);
	_thisAddress = thisAddress;
}

void  DatagramForSlave::setHeaderFrom(uint16_t from)
{
	_driver.setHeaderFrom(from);
}

void DatagramForSlave::setHeaderTo(uint16_t to)
{
	_driver.setHeaderTo(to);
}


void DatagramForSlave::setHeaderSource(uint16_t source)
{
	_driver.setHeaderSource(source);
}

void DatagramForSlave::setHeaderDestination(uint16_t destination)
{
	_driver.setHeaderDestination(destination);
}

void DatagramForSlave::setHeaderType(uint8_t type)
{
	_driver.setHeaderType(type);
}

void DatagramForSlave::setHeaderData(uint8_t data)
{
	_driver.setHeaderData(data);
}
void DatagramForSlave::setHeaderSeqNum(uint8_t seq)
{
	_driver.setHeaderSeqNum(seq);
}
void DatagramForSlave::setHeaderFlags(uint8_t flags)
{
	_driver.setHeaderFlags(flags);
}
void DatagramForSlave::setHeaderHop(uint8_t hop)
{
	_driver.setHeaderHop(hop);
}

uint16_t DatagramForSlave::headerTo()
{
	uint16_t temp = _driver.headerTo();
	return temp;
}
uint16_t DatagramForSlave::headerFrom()
{
	uint16_t temp = _driver.headerFrom();
	return temp;
}

uint16_t DatagramForSlave::headerSource()
{
	return(_driver.headerSource());
}
uint16_t DatagramForSlave::headerDestination()
{
	return(_driver.headerDestination());
}
uint8_t DatagramForSlave::headerType()
{
	return(_driver.headerType());
}
uint8_t DatagramForSlave::headerData()
{
	return(_driver.headerData());
}
uint8_t  DatagramForSlave::headerSeqNum()
{
	return(_driver.headerSeqNum());
}
uint8_t DatagramForSlave::headerFlags()
{
	return(_driver.headerFlags());
}
uint8_t DatagramForSlave::headerHop()
{
	return(_driver.headerHop());
}
/****************************************************************
*FUNCTION NAME:  sendto
*FUNCTION     : sendto
*INPUT        :  buf, size of buf, destination address
*OUTPUT       :
****************************************************************/

/****************************************************************
*FUNCTION NAME:  waitAvailableTimeout
*FUNCTION     : wait until time or there is a packet
*INPUT        :   timeout time
*OUTPUT       : true if there is a packet,
false if  time out
****************************************************************/

bool DatagramForSlave::waitAvailableTimeout(uint16_t timeout)
{
	unsigned long starttime = millis();
	while ((millis() - starttime) < timeout)
	{
		if (_driver.CheckReceiveFlag())
			return true;
		else
			yield();
	}
	return false;
}
bool DatagramForSlave::available()
{   //  Serial.println("111  CHECK"); 
	if (_driver.CheckReceiveFlag()) {
		return true;
	}
	else {
		return false;
	}

}
void DatagramForSlave::waitAvailable()
{
	while (!available())
		yield();
}
////////////////////////////////////////////////////////////////////
/****************************************************************
*FUNCTION NAME:  recvData (OLD recvfrom(buf, len, &_from, &_to, &_id, &_flags)) )
*FUNCTION     : It receives data and stores data into buf,
From, To, Source, Type, Data, Flags, SeqNum are stored.
*INPUT        :none
*OUTPUT       : The size of received data excluding headers
****************************************************************/

byte  DatagramForSlave::recvData(uint8_t* buf)
{
	byte size;
	if (size = _driver.ReceiveData(buf))
	{
		_rxHeaderFrom = headerFrom();
		_rxHeaderTo = headerTo();
		_rxHeaderSource = headerSource();
		_rxHeaderDestination = headerDestination();
		_rxHeaderType = headerType();
		_rxHeaderData = headerData();
		_rxHeaderFlags = headerFlags();
		_rxHeaderSeqNum = headerSeqNum();
		_rxHeaderHop = headerHop();
		return (size - CC1120_HEADER_LEN);
	}
	return 0;
}



////////////////////////////////////////////////////////////////////
void DatagramForSlave::setRetries(uint8_t retries)

{
	_retries = retries;
}

////////////////////////////////////////////////////////////////////
uint8_t DatagramForSlave::retries()
{
	return _retries;
}


void DatagramForSlave::send(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size)
{
	setHeaderFrom(from);
	setHeaderTo(to);
	setHeaderSource(src);
	setHeaderDestination(dst);
	setHeaderType(type);
	setHeaderData(data);
	setHeaderFlags(flags);
	setHeaderSeqNum(seqNum);
	setHeaderHop(hop);
	_driver.SendData(temp_buf, size);
}
bool DatagramForSlave::sendToWait(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size, unsigned long time)
{
	unsigned long startTime = 0;
	int retry = 3;

	setHeaderFrom(from);
	setHeaderTo(to);
	setHeaderSource(src);
	setHeaderDestination(dst);
	setHeaderType(type);
	setHeaderData(data);
	setHeaderSeqNum(seqNum);
	setHeaderHop(hop);
	_driver.SendData(temp_buf, size);
	for (int i = 0; i < retry; i++)
	{
		startTime = millis();
		while (millis() - startTime < TIME_HOP)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf) && _rxHeaderTo == _thisAddress && _rxHeaderType == ACK && _rxHeaderFrom == to)
				{
					return true;
				}
			}
		}
		delay(time);
	}
	return false;
}

bool DatagramForSlave::sendToWaitAck(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size, unsigned long time)
{
	unsigned long startTime = 0;
	int retry = 3;
	setHeaderFrom(from);
	setHeaderTo(to);
	setHeaderSource(src);
	setHeaderDestination(dst);
	setHeaderType(type);
	setHeaderData(data);
	setHeaderSeqNum(seqNum);
	setHeaderHop(hop);

	for (int i = 0; i < retry; i++)
	{
		_driver.SendData(temp_buf, size);
		startTime = millis();
		while (millis() - startTime < time)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf) && _rxHeaderTo == _thisAddress &&  _rxHeaderFrom == to)
				{
					if (_rxHeaderType == ACK || _rxHeaderType == REQUEST_ACK_TYPE || _rxHeaderType == R2_REQUEST_ACK_TYPE || _rxHeaderType == REQUEST_PATH_ONE_BY_ONE_ACK)
					{
						return true;
					}
				}
			}
		}
	}
	return false;
}
void DatagramForSlave::printRecvPacketHeader()
{
	Serial.println("-----------------------------------------------------------------------------");
	Serial.print("_rxHeaderFrom : ");
	Serial.println(_rxHeaderFrom);
	Serial.print("_rxHeaderTo : ");
	Serial.println(_rxHeaderTo);
	Serial.print("_rxHeaderSource : ");
	Serial.println(_rxHeaderSource);
	Serial.print("_rxHeaderDestination : ");
	Serial.println(_rxHeaderDestination);
	Serial.print("_rxHeaderType : ");
	Serial.println(_rxHeaderType);
	Serial.print("_rxHeaderData : ");
	Serial.println(_rxHeaderData);
	Serial.print("_rxHeaderFlags : ");
	Serial.println(_rxHeaderFlags);
	Serial.print("_rxHeaderSeqNum : ");
	Serial.println(_rxHeaderSeqNum);
	Serial.print("_rxHeaderHop : ");
	Serial.println(_rxHeaderHop);
}
/****************************************************************
*FUNCTION NAME:  convertToAddress(uint8_t gatewayNumber, uint8_t masterNumber,uint8_t slaveNumber)
*FUNCTION     :  2 바이트 형식의 주소로 변환
*INPUT        : 게이트 번호, 매스터 노드 번호, 슬레이브 번호
*OUTPUT       : 2바이트  노드 주소
****************************************************************/
uint16_t DatagramForSlave::convertToAddress(uint8_t gatewayNumber, uint8_t masterNumber, uint8_t slaveNumber)
{
	if (gatewayNumber > 64 || masterNumber > 31 || slaveNumber > 16)
		return -1;

	uint16_t address;
	address = gatewayNumber << 5;
	address = address | masterNumber;
	address = address << 5;
	address = address | slaveNumber;
	return address;

}
/****************************************************************
*FUNCTION NAME:  convertToMasterNumber(uint16_t address)
*FUNCTION     :  일반적인 주소로 마스터 번호를 구함
*INPUT        : 2바이트 노도 주소
*OUTPUT       : 마스터 번호
****************************************************************/
uint8_t DatagramForSlave::convertToMasterNumber(uint16_t address)
{
	uint8_t masterNum;
	address = address & 0x03FF;
	masterNum = (uint8_t)address >> 5;
	return masterNum;
}
/****************************************************************
*FUNCTION NAME:  M_handle_ERROR_message
*FUNCTION     :   The master receives the  ERROR_message from the slave
				  The master send ERROR_message t to the room con.				 
*INPUT        :  
*OUTPUT       :  

****************************************************************/	
bool DatagramForSlave::M_handle_ERROR_message( uint16_t slave_address  )
{
	uint8_t send_buf[10] ={0};
	uint16_t roomConAddress = _thisAddress + 1;
	send_buf[0] = 0xEE;
	send_buf[2] = slave_address;
	
	for (int i = 0; i < 9; i++)
	{
		send_buf[9] ^= send_buf[i];
	}
	
	sendToWait(_thisAddress, roomConAddress , _thisAddress, roomConAddress, ERROR_MESSAGE, 0, 0, 0,1 ,send_buf, sizeof(send_buf),TIME_HOP);
	
}



/****************************************************************
*FUNCTION NAME:  M_handle_SCAN_SLAVE_message
*FUNCTION     :   The master receives the SCAN_SLAVE message from the gateway.
				  The master sends the SCAN message to the first slave node.
				  Every one second, the master continues to send the SCAN message to the slave nodes.		  
				 
*INPUT        : 
*OUTPUT       :  

****************************************************************/	
bool DatagramForSlave::M_handle_SCAN_SLAVE_message( int  slave_id, uint8_t * read_buf  )
{
    
 	uint16_t gateway_address = _thisAddress & 0xFC00;
	uint16_t To =   _thisAddress + slave_id  ;
	uint16_t rc_address =  _thisAddress| 0x0001 ;
		
	if (sendToWait(_thisAddress, To , _thisAddress, To , SCAN_MESSAGE , 0, 0, 0,1, read_buf, sizeof(read_buf),TIME_HOP))
	{ 
		return true; 
	
	}
	else		// if there is no response, then consider as the error.
	{
	    read_buf[0] = 0xEE;  
        read_buf[2] = slave_id;
        sendToWait(_thisAddress, rc_address , _thisAddress, rc_address , SCAN_SLAVE_ACK , 0, 0, 0, 1,read_buf, sizeof(read_buf),TIME_HOP);
	}
}



bool DatagramForSlave::sendToWaitBroadcast(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t headerData, uint8_t flags, uint8_t seqNum, byte* temp_buf, byte size, unsigned long time)
{
	unsigned long startTime = 0;
	uint16_t broadcast_address = to | 0x001F;
	int retry = 3;
	setHeaderFrom(from);
	setHeaderTo(to);
	setHeaderSource(src);
	setHeaderDestination(broadcast_address);
	setHeaderType(type);
	setHeaderData(headerData);
	setHeaderSeqNum(seqNum);

	for (int i = 0; i < retry; i++)
	{
		_driver.SendData(temp_buf, size);
		startTime = millis();
		while (millis() - startTime < TIME_HOP)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf) && _rxHeaderTo == broadcast_address && _rxHeaderType == CONTROL_ACK)
				{				
						return true;					
				}
			}
		}
		delay(time); // the first prioty is the control message from the room con, the second is the control from the master to slave.
	}
	return false;
}	

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
