//2017-07-17
// Datagram.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RHDatagram.cpp,v 1.6 2014/05/23 02:20:17 mikem Exp $

#include <Datagram.h>

Datagram::Datagram(ELECHOUSE_CC1120& driver, uint16_t thisAddress)
	:
	_driver(driver),
	_thisAddress(thisAddress),
	_retransmissions(0),
	_lastSequenceNumber(0),
	_timeout(DEFAULT_TIMEOUT),
	_retries(DEFAULT_RETRIES),
	_max_hops(DEFAULT_MAX_HOPS),
	_receivedRequestFlag(false)
{
	// Serial.println(" SendData 2");
	//_retransmissions = 0;
	clearRoutingTable();
}

////////////////////////////////////////////////////////////////////
// Public methods
void Datagram::init()
{
	setThisAddress(_thisAddress);
	_driver.Init();

}
////////////////////////////////////////////////////////////////////
// Public methods
void Datagram::init(byte ch)  // Setup Channel number
{

	setThisAddress(_thisAddress);
	_driver.Init(ch);
}

////////////////////////////////////////////////////////////////////
// Public methods
void Datagram::SetReceive()
{
	_driver.SetReceive();
}

void  Datagram::setThisAddress(uint16_t thisAddress)
{
	_driver.setThisAddress(thisAddress);
	// Use this address in the transmitted FROM header
	setHeaderFrom(thisAddress);
	_thisAddress = thisAddress;
}

void  Datagram::setHeaderFrom(uint16_t from)
{
	_driver.setHeaderFrom(from);
}

void Datagram::setHeaderTo(uint16_t to)
{
	_driver.setHeaderTo(to);
}


void Datagram::setHeaderSource(uint16_t source)
{
	_driver.setHeaderSource(source);
}

void Datagram::setHeaderDestination(uint16_t destination)
{
	_driver.setHeaderDestination(destination);
}

void Datagram::setHeaderType(uint8_t type)
{
	_driver.setHeaderType(type);
}

void Datagram::setHeaderData(uint8_t data)
{
	_driver.setHeaderData(data);
}
void Datagram::setHeaderSeqNum(uint8_t seq)
{
	_driver.setHeaderSeqNum(seq);
}
void Datagram::setHeaderFlags(uint8_t flags)
{
	_driver.setHeaderFlags(flags);
}

uint16_t Datagram::headerTo()
{
	uint16_t temp = _driver.headerTo();
	return temp;
}
uint16_t Datagram::headerFrom()
{
	uint16_t temp = _driver.headerFrom();
	return temp;
}

uint16_t Datagram::headerSource()
{
	return(_driver.headerSource());
}
uint16_t Datagram::headerDestination()
{
	return(_driver.headerDestination());
}
uint8_t Datagram::headerType()
{
	return(_driver.headerType());
}
uint8_t Datagram::headerData()
{
	return(_driver.headerData());
}
uint8_t  Datagram::headerSeqNum()
{
	return(_driver.headerSeqNum());
}
uint8_t Datagram::headerFlags()
{
	return(_driver.headerFlags());
}
/****************************************************************
*FUNCTION NAME:  sendto
*FUNCTION     : sendto
*INPUT        :  buf, size of buf, destination address
*OUTPUT       :
****************************************************************/
void Datagram::sendto(uint8_t* buf, uint8_t len, uint16_t address)
{
	setHeaderTo(address);
	_driver.SendData(buf, len);
}
/****************************************************************
*FUNCTION NAME:  waitAvailableTimeout
*FUNCTION     : wait until time or there is a packet
*INPUT        :   timeout time
*OUTPUT       : true if there is a packet,
false if  time out
****************************************************************/

bool Datagram::waitAvailableTimeout(uint16_t timeout)
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
bool Datagram::available()
{   //  Serial.println("111  CHECK"); 
	if (_driver.CheckReceiveFlag()) {
		return true;
	}
	else {
		return false;
	}

}
void Datagram::waitAvailable()
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

byte  Datagram::recvData(uint8_t* buf)
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
		//  Serial.print("From: ");
		// Serial.println(_rxHeaderFrom,HEX);
		// Serial.print("SEQ: ");
		// Serial.println(_rxHeaderSeqNum,HEX);
		return (size - CC1120_HEADER_LEN);
	}
	return 0;
}

////////////////////////////////////////////////////////////////////
// Public methods
void Datagram::setTimeout(uint16_t timeout)
{
	_timeout = timeout;
}

////////////////////////////////////////////////////////////////////
void Datagram::setRetries(uint8_t retries)

{
	_retries = retries;
}

////////////////////////////////////////////////////////////////////
uint8_t Datagram::retries()
{
	return _retries;
}

void  Datagram::resetRetransmissions()
{
	_retransmissions = 0;
}

/*
////////////////////////////////////////////////////////////////////
/****************************************************************
*FUNCTION NAME:  sendtoGatewayWaitforACK (OLD sendtoWait() )
*FUNCTION     :The main FCU will use this function to send data to the gateway and wait
for ACK from the gateway
*INPUT        :none
*OUTPUT       : true if there is an ACK
false if the timer expires
****************************************************************/


void  Datagram::MainSendControlToSlave()
{
	setHeaderType(TYPE_COMMAND); // SHOULD FIX
								 //   sendto (); 

}

void  Datagram::RCSendControlToMain()
{

	setHeaderType(TYPE_COMMAND); // SHOULD FIX
								 // sendto( ); 

}

void  Datagram::acknowledge()
{
	setHeaderSeqNum(_rxHeaderSeqNum);
	setThisAddress(_thisAddress);
	setHeaderType(TYPE_ACK);
	uint16_t from = _rxHeaderFrom;
	setHeaderTo(_rxHeaderFrom);
	uint8_t ack = '!';
	sendto(&ack, sizeof(ack), from);
	//  waitPacketSent();
}


//////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
void Datagram::setMaxHops(uint8_t max_hops)
{
	_max_hops = max_hops;
}

////////////////////////////////////////////////////////////////////
void Datagram::addRouteTo(uint16_t  dest, uint16_t  next_hop, uint8_t state, uint8_t hop)
{
	uint8_t i;

	// First look for an existing entry we can update
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].dest == dest)
		{
			_routes[i].dest = dest;
			if (_routes[i].next_hop[0] != 0)
			{
				_routes[i].next_hop[1] = next_hop;
				_routes[i].hop[1] = hop;
			}
			else
			{
				_routes[i].next_hop[0] = next_hop;
				_routes[i].hop[0] = hop;
			}
			_routes[i].state = state;
			return;
		}
	}

	// Look for an invalid entry we can use
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].state == Invalid)
		{
			_routes[i].dest = dest;
			_routes[i].next_hop[0] = next_hop;
			_routes[i].hop[0] = hop;
			_routes[i].state = state;
			return;
		}
	}

	// Need to make room for a new one
	retireOldestRoute();
	// Should be an invalid slot now
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].state == Invalid)
		{
			_routes[i].dest = dest;
			_routes[i].next_hop[0] = next_hop;
			_routes[i].hop[0] = hop;
			_routes[i].state = state;
		}
	}
}

////////////////////////////////////////////////////////////////////
Datagram::RoutingTableEntry* Datagram::getRouteTo(uint16_t  dest)
{
	uint8_t i;
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
		if (_routes[i].dest == dest && _routes[i].state != Invalid)
			return &_routes[i];
	return NULL;
}

////////////////////////////////////////////////////////////////////
void Datagram::deleteRoute(uint8_t index)
{
	// Delete a route by copying following routes on top of it
	memcpy(&_routes[index], &_routes[index + 1],
		sizeof(RoutingTableEntry) * (ROUTING_TABLE_SIZE - index - 1));
	_routes[ROUTING_TABLE_SIZE - 1].state = Invalid;
}

////////////////////////////////////////////////////////////////////
void Datagram::printRoutingTable()
{
	//#ifdef HAVE_SERIAL
	uint8_t i;
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		Serial.print(i, DEC);
		Serial.print(" Dest: ");
		Serial.print(_routes[i].dest, DEC);
		Serial.print(" Next Hop[0]: ");
		Serial.print(_routes[i].next_hop[0], DEC);
		Serial.print(" Next Hop[1]: ");
		Serial.print(_routes[i].next_hop[1], DEC);
		Serial.print(" State: ");
		Serial.print(_routes[i].state, DEC);
		Serial.print(" Hop[0]: ");
		Serial.println(_routes[i].hop[0], DEC);
		Serial.print(" Hop[1]: ");
		Serial.println(_routes[i].hop[1], DEC);
	}
	//#endif
}

////////////////////////////////////////////////////////////////////
bool Datagram::deleteRouteTo(uint16_t dest)
{
	uint8_t i;
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		if (_routes[i].dest == dest)
		{
			deleteRoute(i);
			return true;
		}
	}
	return false;
}

////////////////////////////////////////////////////////////////////
void Datagram::retireOldestRoute()
{
	// We just obliterate the first in the table and clear the last
	deleteRoute(0);
}

////////////////////////////////////////////////////////////////////
void Datagram::clearRoutingTable()
{
	uint8_t i;
	for (i = 0; i < ROUTING_TABLE_SIZE; i++)
	{
		_routes[i].state = Invalid;
		_routes[i].next_hop[0] = 0;
		_routes[i].next_hop[1] = 0;
		_routes[i].hop[0] = 0;
		_routes[i].hop[1] = 0;
	}
}
//////////////////////////////////������� ����� �ϴ� �ڵ�//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//=====================================================================================================
// 2017-04-26 ver.1
/****************************************************************
*FUNCTION NAME:FromGatewatyToMaster
*FUNCTION     :read data received from RXfifo
*INPUT        :rxBuffer: buffer to store data
*OUTPUT       :size of data received
****************************************************************/
//����ÿ��� ack�� ���� ����(���� �ָ��ִ� ���ε� ack�� ���޾Ҵٰ� �ؼ� ��� ������ �ع����� ���� �ð�����)
//gateway address : 0x0000
//Master address : 0x0001����
void Datagram::FromGatewayToMaster() {
	uint16_t address_i;
	Serial.println("FromGatewayToMaster");
	Serial.println("[row1]");
	//row1 : ��� master�鿡�� �ѹ��� ������(1hop�� ��� �� �� üũ)
	while (receivedMasterNum[0] == 0)
	{
		if (G_find_1stRow_master() < 0)// ��� ������ 1hop �Ÿ� �ϰ��
			return;
	}

	if (G_find_2ndRow_master() < 0)
		return;

	unreceivedNum = G_find_multihop_node();
	Serial.println("[unreceived]");
	if (unreceivedNum != 0)//���� while�� �ؾߵ�, ������ ���(multi hop���� �ذ��� �ƾ�� �ϴµ�, ������ �� �ȵż� ������ �ֵ�)
	{
		for (i = 1; i <= NUM_OF_CONTRL; i++)
		{
			if (!checkReceive[i])
			{
				for (j = 0; j <= NUM_OF_CONTRL; j++)
				{
					if (_routes[j].state == Valid)
					{
						bufIdx = 0;
						address_i = convertToAddress(gatewayNumber, i, 0);
						temp_buf[bufIdx] = highByte(address_i);
						bufIdx++;
						temp_buf[bufIdx] = lowByte(address_i);
						bufIdx++;
						//from, to, src, dst, type, data, flags, seqnum
						send(_thisAddress, _routes[j].next_hop[0], _thisAddress, _routes[j].dest, UNRECEIVED_REQUEST, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
						startTime = millis();
						while ((millis() - startTime) < (_routes[j].hop[0] + 1) * 2 * TIME_TERM * 2)//Ÿ��Ʈ�ϰ�
						{
							SetReceive();
							if (available())
							{
								if (recvData(temp_buf))
								{
									if (checkReceive[_rxHeaderSource] == true && _rxHeaderSource != _thisAddress)
									{
										addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderFlags + 1);
									}
									if (_rxHeaderTo == _thisAddress)
									{
										printRecvPacketHeader();
										if (_rxHeaderType == UNRECEIVED_REQUEST_ACK && _rxHeaderSource == address_i && _rxHeaderDestination == _thisAddress && _rxHeaderFrom == _routes[j].next_hop[0])
										{
											Serial.println("unreceived request ack receive!!");
											addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _routes[j].hop[0] + 1);
											checkReceive[i] = true;
											unreceivedNum--;
											printRoutingTable();
											break;
										}
									}
								}
							}
						}
					}
					if (checkReceive[i])
						break;
				}
			}
			if (unreceivedNum == 0)
				break;
		}
	}
	printRoutingTable();
	for (int i = 1; i <= NUM_OF_CONTRL; i++)
	{
		address_i = convertToAddress(gatewayNumber, i, 0);
		send(_thisAddress, getRouteTo(address_i)->next_hop[0], _thisAddress, address_i, CHECK_ROUTING, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
	}
	//=====================================================================================================================================================
}
void Datagram::FromMasterToGateway() {

	Serial.println("FromMasterToGateway");
	while (1)
	{
		SetReceive();
		if (available())
		{
			if (recvData(temp_buf))
			{
				addRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1);
				M_findCandidateParents();
				if (getRouteTo(_rxHeaderSource)->next_hop[0] != 0 && getRouteTo(_rxHeaderSource)->state == Valid)//�����ϱ�
					addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderFlags + 1);
				if (_rxHeaderTo == _thisAddress)
				{
					//   Gateway sends requset packet to Master
					//   Normal case
					printRecvPacketHeader();
					Serial.print("data : ");
					Serial.println(word(temp_buf[0], temp_buf[1]));
					Serial.println(word(temp_buf[2], temp_buf[3]));
					if (_rxHeaderType == REQUEST_TYPE)
					{
						Serial.println("receive row1 request");
						_receivedRequestFlag = true;
						addRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1);
						printRoutingTable();
						//from, to, src, dst, type, data, flags, seqnum
						send(_thisAddress, GATEWAY_ADDR, _thisAddress, GATEWAY_ADDR, REQUEST_ACK_TYPE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
					}
					else if (_rxHeaderType == R2_REQUEST_TYPE)
					{
						Serial.println("receive R2 request and I succeed R1");
						//from, to, src, dst, type, data, flags, seqnum
						send(_thisAddress, GATEWAY_ADDR, _thisAddress, GATEWAY_ADDR, ACK, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
						M_find2ndRowMasters();
					}
					else if (_rxHeaderType == R2_REQUEST_REAL_TYPE)
					{
						Serial.println("receive R2 routing request and I didn't success in R1 request");
						M_masterSendRoutingReply();
					}/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					else if (_rxHeaderType == MULTI_HOP_REQUEST_TYPE)
					{
						if (_rxHeaderDestination == _thisAddress)
						{
							dest = word(temp_buf[0], temp_buf[1]);
							if (dest != _thisAddress)
							{
								//���� routing �ȵ� �� ���� ������
								Serial.println("received multihop request and send to unrouting Master");
								uint16_t dest = word(temp_buf[0], temp_buf[1]);
								//from, to, src, dst, type, data, flags, seqnum
								sendToWaitAck(_thisAddress, dest, _rxHeaderSource, dest, _rxHeaderType, NONE, _rxHeaderFlags + 1, NONE, temp_buf, sizeof(temp_buf), TIME_TERM);
							}
							else
							{
								//routing �� �Ǿ��ִ� �ְ� ����
								Serial.println("received multi hop request");
								M_masterSendRoutingReply();
							}
						}
						else//�߰� hop
						{
							Serial.println("received multihop request and send to routing Master. I'm middle Master");
							//from, to, src, dst, type, data, flags, seqnum
							sendToWaitAck(_thisAddress, getRouteTo(_rxHeaderDestination)->next_hop[0], _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, _rxHeaderFlags + 1, NONE, temp_buf, sizeof(temp_buf), TIME_TERM);
						}
					}
					else if (_rxHeaderType == MULTI_HOP_REQUEST_ACK_TYPE)
					{
						Serial.println("received multihop request  ack and send");
						addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderFlags + 1);
						printRoutingTable();
						//from, to, src, dst, type, data, flags, seqnum
						temp_buf[(_rxHeaderFlags + 1) * 2] = highByte(_thisAddress);
						temp_buf[(_rxHeaderFlags + 1) * 2 + 1] = lowByte(_thisAddress);
						send(_thisAddress, getRouteTo(_rxHeaderDestination)->next_hop[0], _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, _rxHeaderFlags + 1, NONE, temp_buf, sizeof(temp_buf));
					}
					else if (_rxHeaderType == UNRECEIVED_REQUEST)
					{
						if (_rxHeaderDestination == _thisAddress)
						{
							dest = word(temp_buf[0], temp_buf[1]);
							if (dest != _thisAddress)
							{
								//���� routing �ȵ� �� ���� ������
								Serial.println("receive unreceived_request and send to unrouting Master");
								//from, to, src, dst, type, data, flags, seqnum
								send(_thisAddress, dest, GATEWAY_ADDR, dest, UNRECEIVED_REQUEST, NONE, _rxHeaderFlags + 1, NONE, temp_buf, sizeof(temp_buf));
							}
							else
							{
								//routing �ȵǾ��ִ� �ְ� ����
								Serial.println("receive unreceived_request and send ack");
								_receivedRequestFlag = true;
								addRouteTo(GATEWAY_ADDR, _rxHeaderFrom, Valid, _rxHeaderFlags + 1);
								printRoutingTable();
								//from, to, src, dst, type, data, flags, seqnum
								send(_thisAddress, _rxHeaderFrom, _thisAddress, GATEWAY_ADDR, UNRECEIVED_REQUEST_ACK, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
							}
						}
						else//�߰� hop
						{
							Serial.println("receive unreceived_request and send to routing Master");
							//from, to, src, dst, type, data, flags, seqnum
							send(_thisAddress, getRouteTo(_rxHeaderDestination)->next_hop[0], _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, _rxHeaderFlags + 1, NONE, temp_buf, sizeof(temp_buf));
						}
					}
					else if (_rxHeaderType == UNRECEIVED_REQUEST_ACK)
					{
						Serial.println("receive unreceived_request  ack and send");
						addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderFlags + 1);
						printRoutingTable();
						//from, to, src, dst, type, data, flags, seqnum
						send(_thisAddress, getRouteTo(_rxHeaderDestination)->next_hop[0], _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, _rxHeaderFlags + 1, NONE, temp_buf, sizeof(temp_buf));
					}
					else if (_rxHeaderType == CHECK_ROUTING)
					{
						if (_rxHeaderDestination == _thisAddress)
							break;
						else
						{
							//from, to, src, dst, type, data, flags, seqnum
							send(_thisAddress, getRouteTo(_rxHeaderDestination)->next_hop[0], _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
						}
					}
				}
			}
		}
	}
}
void Datagram::send(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t headerData, uint8_t flags, uint8_t seqNum, byte* temp_buf, byte size)
{
	setHeaderFrom(from);
	setHeaderTo(to);
	setHeaderSource(src);
	setHeaderDestination(dst);
	setHeaderType(type);
	setHeaderData(headerData);
	setHeaderFlags(flags);
	setHeaderSeqNum(seqNum);
	_driver.SendData(temp_buf, size);
}
bool Datagram::sendToWait(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t headerData, uint8_t flags, uint8_t seqNum, byte* temp_buf, byte size, unsigned long time)
{
	unsigned long startTime = 0;
	int retry = 3;
	for (int i = 0; i < retry; i++)
	{
		setHeaderFrom(from);
		setHeaderTo(to);
		setHeaderSource(src);
		setHeaderDestination(dst);
		setHeaderType(type);
		setHeaderData(headerData);
		setHeaderSeqNum(seqNum);
		_driver.SendData(temp_buf, size);
		startTime = millis();
		while (millis() - startTime < TIME_TERM)
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

bool Datagram::sendToWaitAck(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t headerData, uint8_t flags, uint8_t seqNum, byte* temp_buf, byte size, unsigned long time)
{
	unsigned long startTime = 0;
	int retry = 3;
	setHeaderFrom(from);
	setHeaderTo(to);
	setHeaderSource(src);
	setHeaderDestination(dst);
	setHeaderType(type);
	setHeaderData(headerData);
	setHeaderSeqNum(seqNum);

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
					if (_rxHeaderType == ACK || _rxHeaderType == REQUEST_ACK_TYPE || _rxHeaderType == R2_REQUEST_ACK_TYPE || _rxHeaderType == MULTI_HOP_REQUEST_ACK_TYPE)
					{
						addRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1);
						return true;
					}
				}
			}
		}

	}
	return false;
}
void Datagram::printRecvPacketHeader()
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
}

/****************************************************************
*FUNCTION NAME:  convertToAddress(uint8_t gatewayNumber, uint8_t masterNumber,uint8_t slaveNumber)
*FUNCTION     :  2 ����Ʈ ������ �ּҷ� ��ȯ
*INPUT        : ����Ʈ ��ȣ, �Ž��� ��� ��ȣ, �����̺� ��ȣ
*OUTPUT       : 2����Ʈ  ��� �ּ�
****************************************************************/
uint16_t Datagram::convertToAddress(uint8_t gatewayNumber, uint8_t masterNumber, uint8_t slaveNumber)
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
*FUNCTION     :  �Ϲ����� �ּҷ� ������ ��ȣ�� ����
*INPUT        : 2����Ʈ �뵵 �ּ�
*OUTPUT       : ������ ��ȣ
****************************************************************/
uint8_t Datagram::convertToMasterNumber(uint16_t address)
{
	uint8_t masterNum;
	address = address & 0x03E0;
	masterNum = address >> 5;
	return masterNum;
}
/****************************************************************
*FUNCTION NAME:  G_find_1stRow_master( )
*FUNCTION     : Gateway�� direct�ϰ� �����Ҽ� �ִ� �Ž��� ���鿡�� ����� ��û �޽����� �����ϰ�,
������ ��ٸ���.
- ������ �����ϸ�, ����� ���̺� �����ϸ�,
*INPUT        : none
*OUTPUT       : 1 hop ���� �� ������ ����, ��� ������ 1hop ��� -1 return
****************************************************************/
int8_t Datagram::G_find_1stRow_master()
{
	uint16_t address_i;
	uint8_t number_of_unknown_node = 0;
	uint8_t i;
	for (i = 1; i <= NUM_OF_CONTRL; i++)
	{
		Serial.print("send");
		Serial.println(i);
		address_i = convertToAddress(gatewayNumber, i, 0);
		//from, to, src, dst, type, data, flags, seqnum, payload, size of payload
		send(_thisAddress, address_i, _thisAddress, address_i, REQUEST_TYPE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));


		startTime = millis();
		while ((millis() - startTime) < TIME_TERM * 2)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf) && (_rxHeaderTo == BROADCAST_ADDRESS || _rxHeaderTo == _thisAddress))
				{
					printRecvPacketHeader();
					if (_rxHeaderType == REQUEST_ACK_TYPE && _rxHeaderSource == address_i && _rxHeaderDestination == _thisAddress && _rxHeaderFrom == address_i)//���ǿ� �´´ٸ� �³״� 1hop�� master��
					{
						Serial.println("receive row1 request_ack");
						addRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1);//dst, next, state, hop count
						checkReceive[i] = true;
						path[i].address = address_i;
						path[i].next_node = NULL;
						printRoutingTable();
						break;
					}
				}
			}
		}
	}
	for (i = 1; i <= NUM_OF_CONTRL; i++)
	{
		if (!checkReceive[i])
		{
			number_of_unknown_node++;
			Serial.print("unreceive : ");
			Serial.println(i);
		}
		else if (i == NUM_OF_CONTRL && number_of_unknown_node == 0)
			return -1;
	}
	return number_of_unknown_node;

}
/****************************************************************
*FUNCTION NAME:  G_find_2ndRow_master( )
*FUNCTION     :  Gateway��  1 hop ���鿡��, ������� �ȵ� ������ �ּҸ� ���̷ε忡 �����Ͽ� ������.

*INPUT        : none
*OUTPUT       :
****************************************************************/
int8_t Datagram::G_find_2ndRow_master()
{
	//================================================================================================================================================
	//row2 : 1hop�� master�鿡�� ���� ������� �ȵ� ������ �ּҸ� ������ ��ó�� ������ �ִ��� ���
	uint16_t address;
	uint8_t highAddress;
	uint8_t lowAddress;
	Serial.println("[row2]");
	byte row_number = 1;
	uint16_t node_list[NUM_OF_CONTRL] = { 0 };
	byte number_of_node;
	byte i, j;
	uint8_t number_of_unknown_node = 0;
	bool receiving = false;
	uint8_t masterNum;
	number_of_node = G_get_i_row_node_list(row_number, node_list);

	for (i = 0; i < number_of_node; i++)
	{
		number_of_unknown_node = 0;
		bufIdx = 0;
		for (j = 1; j <= NUM_OF_CONTRL; j++)//routing�� �ȵ� ������ �ּҸ� table�� ������ �˾Ƴ�
		{
			if (!checkReceive[j])
			{
				number_of_unknown_node++;
				Serial.print("unreceive : ");
				Serial.println(j);
				address = convertToAddress(gatewayNumber, j, 0);
				//routing�� �ȵ� �ֵ��� �ּҸ� ���۵Ǵ� �迭�� �ִ´�.

				highAddress = (uint8_t)(address >> 8);
				lowAddress = (uint8_t)(0x00FF & address);

				temp_buf[bufIdx] = highAddress;
				bufIdx++;
				temp_buf[bufIdx] = lowAddress;
				bufIdx++;
			}
			else if (j == NUM_OF_CONTRL && bufIdx == 0)
				return -1;
		}
		Serial.print("send : ");
		Serial.println(node_list[i]);
		//from, to, src, dst, type, data, flags, seqnum, payload, size of payload, timeout
		receiving = sendToWaitAck(_thisAddress, node_list[i], _thisAddress, node_list[i], R2_REQUEST_TYPE, number_of_unknown_node, NONE, NONE, temp_buf, sizeof(temp_buf), TIME_TERM); //1hop�� node ������ ����
		if (receiving == false)
		{
			continue;
		}
		Serial.println(word(temp_buf[0], temp_buf[1]));
		startTime = millis();
		timeLimit = number_of_unknown_node * 2 * TIME_TERM * 2;
		while ((millis() - startTime) < timeLimit)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf))
				{
					if (checkReceive[convertToMasterNumber(_rxHeaderSource)] == true && _rxHeaderSource != _thisAddress)
					{
						addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderFlags + 1);
					}
					if (_rxHeaderTo == _thisAddress && _rxHeaderType == R2_REQUEST_ACK_TYPE && _rxHeaderFrom == node_list[i] && _rxHeaderSource > 0 && _rxHeaderSource <= NUM_OF_CONTRL)
					{
						addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, 2);
						masterNum = convertToMasterNumber(_rxHeaderSource);
						checkReceive[masterNum] = true;
						path[masterNum].address = _rxHeaderFrom;
						path[masterNum].next_node = (nodeForPath*)malloc(sizeof(nodeForPath));
						path[masterNum].next_node->address = _rxHeaderSource;
						path[masterNum].next_node->next_node = NULL;
						printRecvPacketHeader();
						if (_rxHeaderData > 1)
						{
							bufIdx = 0;
							for (i = 1; i < _rxHeaderData; i++)
							{
								addr = word(temp_buf[bufIdx], temp_buf[bufIdx + 1]);
								bufIdx += 2;
								addRouteTo(addr, _rxHeaderFrom, Valid, 2);
								masterNum = convertToMasterNumber(addr);
								checkReceive[masterNum] = true;
								path[masterNum].address = _rxHeaderFrom;
								path[masterNum].next_node = (nodeForPath*)malloc(sizeof(nodeForPath));
								path[masterNum].next_node->address = addr;
								path[masterNum].next_node->next_node = NULL;
							}
						}
						printRoutingTable();
						break;
					}
				}
			}
		}
	}
	for (i = 1; i <= NUM_OF_CONTRL; i++)
	{
		if (!checkReceive[i])
			number_of_unknown_node++;
		else if (i == NUM_OF_CONTRL && number_of_unknown_node == 0)
			return -1;
	}
}

/****************************************************************
*FUNCTION NAME:  M_findCandidateParents( )
*FUNCTION     :

*INPUT        :
*OUTPUT       :
****************************************************************/
void Datagram::M_findCandidateParents()
{
	if (_receivedRequestFlag == false && (_rxHeaderType == REQUEST_ACK_TYPE || _rxHeaderType == R2_REQUEST_ACK_TYPE || _rxHeaderType == MULTI_HOP_REQUEST_ACK_TYPE))
	{
		if (receivedType == 0)
		{
			Serial.println("receive propagation");
			receivedType = _rxHeaderType;
			candidateAddress = _rxHeaderFrom;
			candidateRSSI = _rxHeaderSeqNum;
			Serial.print("My choice : ");
			Serial.println(candidateAddress);
		}
		else if (receivedType == _rxHeaderType)
		{
			if (_driver.rssi >= candidateRSSI)
			{
				Serial.println("receive propagation");
				candidateRSSI2 = candidateRSSI;
				candidateAddress2 = candidateAddress;
				candidateAddress = _rxHeaderFrom;
				candidateRSSI = _driver.rssi;
				Serial.print("My choice : ");
				Serial.println(_rxHeaderFrom);
			}
			else if (_driver.rssi >= candidateRSSI2)
			{
				candidateAddress2 = _rxHeaderFrom;
				candidateRSSI2 = _driver.rssi;
			}
		}
	}

}
/****************************************************************
*FUNCTION NAME:  M_find2ndRowMasters( )
*FUNCTION     :

*INPUT        :
*OUTPUT       :
****************************************************************/
void Datagram::M_find2ndRowMasters()
{
	uint16_t childNodeList[32];
	uint16_t to_address_temp;
	byte temp[20];
	bool receivingR2Ack = false;
	Serial.println("receive R2 request and I succeed R1");
	uint8_t count = _rxHeaderData;  // Guin: number of unknown node
	uint8_t idx = 0;
	uint8_t i;
	uint8_t bufIdx = 0;
	bool receiving = false;

	for (i = 0; i < count; i++)
	{
		temp[bufIdx] = temp_buf[bufIdx];
		temp[bufIdx + 1] = temp_buf[bufIdx + 1];
		bufIdx += 2;
	}
	bufIdx = 0;
	Serial.print("count : ");
	Serial.println(count);
	for (i = 0; i < count; i++)
	{
		Serial.println("send R2 reequest to 2 hop");
		to_address_temp = word(temp[bufIdx], temp[bufIdx + 1]);
		receiving = sendToWaitAck(_thisAddress, to_address_temp, GATEWAY_ADDR, to_address_temp, R2_REQUEST_REAL_TYPE, NONE, 1, NONE, temp_buf, sizeof(temp_buf), TIME_TERM);
		bufIdx += 2;
		if (receiving) {
			Serial.println("receiving");
			receivingR2Ack = true;
			printRoutingTable();
			childNodeList[idx++] = _rxHeaderFrom;
		}
	}
	if (receivingR2Ack)
	{
		bufIdx = 0;
		for (i = 0; i < idx; i++)
		{
			temp_buf[bufIdx] = highByte(childNodeList[i]);
			bufIdx++;
			temp_buf[bufIdx] = lowByte(childNodeList[i]);
			bufIdx++;
		}
		//from, to, src, dst, type, data, flags, seqnum
		send(_thisAddress, GATEWAY_ADDR, childNodeList[0], GATEWAY_ADDR, R2_REQUEST_ACK_TYPE, idx, 1, NONE, temp_buf, sizeof(temp_buf));
	}
}
/****************************************************************
*FUNCTION NAME:M_masterSendRoutingReply( )
*FUNCTION     :

*INPUT        :
*OUTPUT       :
****************************************************************/
void Datagram::M_masterSendRoutingReply()
{
	if (candidateAddress == _rxHeaderFrom || candidateAddress2 == _rxHeaderFrom)
	{
		addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderFlags + 1);
		printRoutingTable();
		//from, to, src, dst, type, data, flags, seqnum
		if (_rxHeaderType == R2_REQUEST_REAL_TYPE)
			send(_thisAddress, _rxHeaderFrom, _thisAddress, GATEWAY_ADDR, R2_REQUEST_ACK_TYPE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
		else if (_rxHeaderType == MULTI_HOP_REQUEST_TYPE)
		{
			temp_buf[0] = highByte(_thisAddress);
			temp_buf[1] = lowByte(_thisAddress);
			send(_thisAddress, _rxHeaderFrom, _thisAddress, GATEWAY_ADDR, MULTI_HOP_REQUEST_ACK_TYPE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
		}
		_receivedRequestFlag = true;
	}
}
/****************************************************************
*FUNCTION NAME: G_get_i_row_node_list(uint8_t row_number, uint16_t *node_list )
*FUNCTION     : Gatway gets the list of node on the i_th row

*INPUT        : row number
*OUTPUT       : return value: number of node on the i_th row
****************************************************************/
uint8_t Datagram::G_get_i_row_node_list(uint8_t row_number, uint16_t *node_list)
{
	uint8_t i, j = 0;
	for (i = 1; j <= NUM_OF_CONTRL; i++)
	{
		if (_routes[i].hop[0] == row_number)
		{
			node_list[j] = _routes[i].dest;
			j++;
		}
	}
	return j;
}

/****************************************************************
*FUNCTION NAME: G_find_multihop_node ()
*FUNCTION     :  Gateway finished the routing discovery upto 2nd row.
Gateway tries to find the 3rd and more row nodes.
- ���� 2nd row node���� ������, ���� routing�� �ȵ� ����� �ּҸ�  payload�� �����Ͽ� �����Ѵ�.
- 3rd row ������ �����Ŀ���, routing�� �ȵ� ����� �ּҸ�  payload�� �����Ͽ�
�ֱٿ� ���� row�� ���鿡��  multihop routing ��û �޽����� �����Ѵ�.
*INPUT        :
*OUTPUT       :
****************************************************************/
int8_t Datagram::G_find_multihop_node()
{
	byte i, j, k;
	byte row_number;
	byte number_of_node;
	uint16_t node_list[NUM_OF_CONTRL] = { 0 };
	uint8_t number_of_unknown_node = 0;
	uint16_t address;
	uint8_t highAddress;
	uint8_t lowAddress;
	uint16_t address_temp;
	nodeForPath* node_temp;

	for (i = 1; i <= NUM_OF_CONTRL; i++)
	{
		if (!checkReceive[i])
			number_of_unknown_node++;
		else if (i == NUM_OF_CONTRL && number_of_unknown_node == 0)
			return 0;
	}

	number_of_node = G_get_i_row_node_list(2, node_list);  // the gateway needs the list of 2nd row masters which will send Multi_HOP_REQUEST
														   // to unknown nodes.
	row_number = 3;  // 
					 //=================================================================================================================================================
	Serial.println("[MULTI_HOP]");
	while (1)
	{
		for (i = 0; i < number_of_node; i++)
		{
			for (j = 1; j <= NUM_OF_CONTRL; j++)
			{
				if (!checkReceive[j])
				{
					bufIdx = 0;
					address = convertToAddress(gatewayNumber, j, 0);
					//routing�� �ȵ� ��� �ּҸ� ���۵Ǵ� ��Ŷ�� payload�� �ִ´�.

					highAddress = (uint8_t)(address >> 8);
					lowAddress = (uint8_t)(0x00FF & address);

					temp_buf[bufIdx] = highAddress;
					bufIdx++;
					temp_buf[bufIdx] = lowAddress;
					bufIdx++;

					send(_thisAddress, getRouteTo(node_list[i])->next_hop[0], _thisAddress, node_list[i], MULTI_HOP_REQUEST_TYPE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));

					startTime = millis();
					while ((millis() - startTime) < row_number * 2 * TIME_TERM * 2)//Ÿ��Ʈ�ϰ�
					{
						SetReceive();
						if (available())
						{
							if (recvData(temp_buf))
							{
								if (checkReceive[convertToMasterNumber(_rxHeaderSource)] == true && _rxHeaderSource != _thisAddress)
								{
									addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderFlags + 1);
								}
								if (_rxHeaderTo == _thisAddress)
								{
									printRecvPacketHeader();
									if (_rxHeaderType == MULTI_HOP_REQUEST_ACK_TYPE && _rxHeaderSource == address && _rxHeaderDestination == _thisAddress && _rxHeaderFrom == getRouteTo(node_list[i])->next_hop[0])
									{
										Serial.println("multihop request ack receive!!");
										addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, row_number);
										checkReceive[j] = true;
										node_temp = &path[j];
										for (k = _rxHeaderFlags; k >= 0; k--)
										{
											address_temp = word(temp_buf[k * 2], temp_buf[k * 2 + 1]);
											node_temp->address = address_temp;
											if (k != 0)
											{
												node_temp->next_node = (nodeForPath*)malloc(sizeof(nodeForPath));
												node_temp = node_temp->next_node;
											}
											else
												node_temp->next_node = NULL;
										}
										number_of_unknown_node--;
										printRoutingTable();
										break;
									}
								}
							}
						}
					}  // end of while
					if (number_of_unknown_node == 0)
						break;
				}  // end of if (!checkReceive[j])
			}  // end of for (j = 1; j <= NUM_OF_CONTRL; j++)
			if (number_of_unknown_node == 0)
				break;
		}
		if (number_of_unknown_node == 0)
			break;

		number_of_node = G_get_i_row_node_list(row_number, node_list);
		row_number++;
		if (number_of_node == 0)
			break;
	}
	return number_of_unknown_node;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////