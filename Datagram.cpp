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
				_routes[i].next_hop[1] = next_hop;
			else
			{
				_routes[i].next_hop[0] = next_hop;
				_routes[i].hop = hop;
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
			_routes[i].hop = hop;
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
			_routes[i].hop = hop;
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
		Serial.print(" Hop: ");
		Serial.println(_routes[i].hop, DEC);
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
		_routes[i].hop = 0;
	}
}
//////////////////////////////////여기부터 라우팅 하는 코드//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//=====================================================================================================
// 2017-04-26 ver.1
/****************************************************************
*FUNCTION NAME:FromGatewatyToMaster
*FUNCTION     :read data received from RXfifo
*INPUT        :rxBuffer: buffer to store data
*OUTPUT       :size of data received
****************************************************************/
//라우팅에는 ack을 받지 않음(원래 멀리있는 애인데 ack을 못받았다고 해서 계속 재전송 해버리면 괜히 시간낭비)
//gateway address : 0x0000
//Master address : 0x0001부터
void Datagram::FromGatewayToMaster() {

	Serial.println("FromGatewayToMaster");
	Serial.println("[row1]");
	//row1 : 모든 master들에게 한번씩 보내봄(1hop인 애들 이 때 체크)
	for (i = 1; i <= NUM_OF_CONTRL; i++)
	{
		Serial.print("send");
		Serial.println(i);
		send(_thisAddress, i, _thisAddress, i, REQUEST_TYPE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));//from, to, src, dst, type, data, flags, seqnum
		startTime = millis();
		while ((millis() - startTime) < TIME_TERM * 2)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf) && (_rxHeaderTo == BROADCAST_ADDRESS || _rxHeaderTo == _thisAddress))
				{
					printRecvPacketHeader();
					if (_rxHeaderType == REQUEST_ACK_TYPE && _rxHeaderSource == i && _rxHeaderDestination == _thisAddress && _rxHeaderFrom == i)//조건에 맞는다면 걔네는 1hop인 master들
					{
						Serial.println("receive row1 request_ack");
						//receivedMasterNum(크기가 2인 배열) : hop count가 n인 애들의 개수(n이 홀수면 idx가 0, n이 짝수면 idx가 1)
						//receivedMaster(2차원 배열) : hop count가 n인 애들의 주소를 넣음(n이 홀수면 receivedCotroller[0] 사용, n이 짝수면 receivedMaster[1] 사용)
						//checkReceive : routing이 된 주소를 true로 표시
						receivedMaster[0][receivedMasterNum[0]] = _rxHeaderFrom;
						addRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1);//dst, next, state, hop count
						checkReceive[_rxHeaderFrom] = true;
						printRoutingTable();
						receivedMasterNum[0]++;
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
			unreceivedNum++;
			Serial.print("unreceive : ");
			Serial.println(i);
		}
		else if (i == NUM_OF_CONTRL && unreceivedNum == 0)
			return;
	}
	//================================================================================================================================================
	//row2 : 1hop인 master들에게 아직 라우팅이 안된 애들의 주소를 보내서 근처에 걔네들이 있는지 물어봄
	Serial.println("[row2]");
	for (i = 0; i < receivedMasterNum[0]; i++)
	{
		unreceivedNum = 0;
		bufIdx = 0;
		for (j = 1; j <= NUM_OF_CONTRL; j++)//routing이 안된 애들의 주소를 table을 뒤져서 알아냄
		{
			if (!checkReceive[j])
			{
				unreceivedNum++;
				Serial.print("unreceive : ");
				Serial.println(j);
				//routing이 안된 애들의 주소를 전송되는 배열에 넣는다.
				temp_buf[bufIdx] = highByte(j);
				bufIdx++;
				temp_buf[bufIdx] = lowByte(j);
				bufIdx++;
			}
			else if (j == NUM_OF_CONTRL && bufIdx == 0)
				return;
		}
		Serial.print("send : ");
		Serial.println(receivedMaster[0][i]);
		send(_thisAddress, receivedMaster[0][i], _thisAddress, receivedMaster[0][i], R2_REQUEST_TYPE, unreceivedNum, NONE, NONE, temp_buf, sizeof(temp_buf));//from, to, src, dst, type, data, flags, seqnum
		//1hop인 애들한테 보냄
		Serial.println(word(temp_buf[0], temp_buf[1]));
		startTime = millis();
		timeLimit = unreceivedNum * 2 *TIME_TERM * 2;
		while ((millis() - startTime) < timeLimit)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf))
				{
					if (checkReceive[_rxHeaderSource] == true && _rxHeaderSource != _thisAddress)
					{
						addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid);
					}
					if (_rxHeaderTo == _thisAddress && _rxHeaderType == R2_REQUEST_ACK_TYPE && _rxHeaderFrom == receivedMaster[0][i] && _rxHeaderSource > 0 && _rxHeaderSource <= NUM_OF_CONTRL)
					{
						addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, 2);
						receivedMaster[1][receivedMasterNum[1]] = _rxHeaderSource;
						receivedMasterNum[1]++;
						checkReceive[_rxHeaderSource] = true;
						printRecvPacketHeader();
						if (_rxHeaderData > 1)
						{
							bufIdx = 0;
							for (i = 1; i < _rxHeaderData; i++)
							{
								addr = word(temp_buf[bufIdx], temp_buf[bufIdx + 1]);
								bufIdx += 2;
								addRouteTo(addr, _rxHeaderFrom, Valid, 2);
								receivedMaster[1][receivedMasterNum[1]] = addr;
								receivedMasterNum[1]++;
								checkReceive[addr] = true;
							}
						}
						printRoutingTable();
						break;
					}
				}
			}
		}
	}
	unreceivedNum = 0;
	for (i = 1; i <= NUM_OF_CONTRL; i++)
	{
		if (!checkReceive[i])
			unreceivedNum++;
		else if (i == NUM_OF_CONTRL && unreceivedNum == 0)
			return;
	}
	receivedMasterNum[0] = 0;
	//=================================================================================================================================================
	Serial.println("[MULTI_HOP]");
	while (1)
	{
		for (i = 0; i < receivedMasterNum[rowFlag % 2]; i++)
		{
			for (j = 1; j <= NUM_OF_CONTRL; j++)
			{
				if (!checkReceive[j])
				{
					bufIdx = 0;
					temp_buf[bufIdx] = highByte(j);
					bufIdx++;
					temp_buf[bufIdx] = lowByte(j);
					bufIdx++;
					send(_thisAddress, getRouteTo(receivedMaster[rowFlag % 2][i])->next_hop[0], _thisAddress, receivedMaster[rowFlag % 2][i], MULTI_HOP_REQUEST_TYPE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
					startTime = millis();
					while ((millis() - startTime) < rowFlag * 2 * TIME_TERM * 2)//타이트하게
					{
						SetReceive();
						if (available())
						{
							if (recvData(temp_buf))
							{
								if (checkReceive[_rxHeaderSource] == true && _rxHeaderSource != _thisAddress)
								{
									addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid);
								}
								if (_rxHeaderTo == _thisAddress)
								{
									printRecvPacketHeader();
									if (_rxHeaderType == MULTI_HOP_REQUEST_ACK_TYPE && j == _rxHeaderSource && _rxHeaderDestination == _thisAddress && _rxHeaderFrom == getRouteTo(receivedMaster[rowFlag % 2][i])->next_hop[0])
									{
										Serial.println("multihop request ack receive!!");
										addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, rowFlag);
										checkReceive[_rxHeaderSource] = true;
										unreceivedNum--;
										receivedMaster[(rowFlag + 1) % 2][receivedMasterNum[(rowFlag + 1) % 2]] = _rxHeaderSource;
										receivedMasterNum[(rowFlag + 1) % 2];
										printRoutingTable();
										break;
									}
								}
							}
						}
					}
					if (unreceivedNum == 0)
						break;
				}
			}
			if (unreceivedNum == 0)
				break;
		}
		if (unreceivedNum == 0)
			break;
		receivedMasterNum[rowFlag % 2] = 0;
		rowFlag++;
		if (receivedMasterNum[(rowFlag + 1) % 2] == 0)
			break;
	}
	Serial.println("[unreceived]");
	if (unreceivedNum != 0)//원래 while로 해야됨, 일일이 물어봄(multi hop에서 해결이 됐어야 하는데, 전송이 잘 안돼서 못받은 애들)
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
						temp_buf[bufIdx] = highByte(i);
						bufIdx++;
						temp_buf[bufIdx] = lowByte(i);
						bufIdx++;
						send(_thisAddress, _routes[j].next_hop[0], _thisAddress, _routes[j].dest, UNRECEIVED_REQUEST, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
						startTime = millis();
						while ((millis() - startTime) < (_routes[j].hop + 1) * 2 * TIME_TERM * 2)//타이트하게
						{
							SetReceive();
							if (available())
							{
								if (recvData(temp_buf))
								{
									if (checkReceive[_rxHeaderSource] == true && _rxHeaderSource != _thisAddress)
									{
										addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid);
									}
									if (_rxHeaderTo == _thisAddress)
									{
										printRecvPacketHeader();
										if (_rxHeaderType == UNRECEIVED_REQUEST_ACK && _rxHeaderSource == i && _rxHeaderDestination == _thisAddress && _rxHeaderFrom == _routes[j].next_hop[0])
										{
											Serial.println("unreceived request ack receive!!");
											addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _routes[j].hop + 1);
											checkReceive[_rxHeaderSource] = true;
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
		send(_thisAddress, getRouteTo(i)->next_hop[0], _thisAddress, i, CHECK_ROUTING, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
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
							candidateAddress = _rxHeaderFrom;
							candidateRSSI = _driver.rssi;
							Serial.print("My choice : ");
							Serial.println(_rxHeaderFrom);
						}
					}
				}
				if (getRouteTo(_rxHeaderSource)->next_hop[0] != 0)
					addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid);
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
						send(_thisAddress, GATEWAY_ADDR, _thisAddress, GATEWAY_ADDR, REQUEST_ACK_TYPE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));//from, to, src, dst, type, data, flags, seqnum
					}
					else if (_rxHeaderType == R2_REQUEST_TYPE)
					{

						Serial.println("receive R2 request and I succeed R1");
						count = _rxHeaderData;
						indirectAdrIdx = 0;
						bufIdx = 0;
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
							send(_thisAddress, word(temp[bufIdx], temp[bufIdx + 1]), GATEWAY_ADDR, word(temp[bufIdx], temp[bufIdx + 1]), R2_REQUEST_REAL_TYPE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
							bufIdx += 2;
							unsigned long startTime = millis();
							while (millis() - startTime < TIME_TERM)
							{
								SetReceive();
								if (available())
								{
									if (recvData(temp_buf) && _rxHeaderTo == _thisAddress && _rxHeaderType == R2_REQUEST_ACK_TYPE)
									{
										Serial.println("receiving");
										receivingR2Ack = true;
										addRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1);
										printRoutingTable();
										indirectAddress[indirectAdrIdx++] = _rxHeaderFrom;
										break;
									}
								}
							}
						}
						if (receivingR2Ack)
						{
							bufIdx = 0;
							for (i = 1; i < indirectAdrIdx; i++)
							{
								temp_buf[bufIdx] = highByte(indirectAddress[i]);
								bufIdx++;
								temp_buf[bufIdx] = lowByte(indirectAddress[i]);
								bufIdx++;
							}
							//from, to, src, dst, type, data, flags, seqnum
							send(_thisAddress, GATEWAY_ADDR, indirectAddress[0], GATEWAY_ADDR, R2_REQUEST_ACK_TYPE, indirectAdrIdx, NONE, NONE, temp_buf, sizeof(temp_buf));
						}
					}
					else if (_rxHeaderType == R2_REQUEST_REAL_TYPE)
					{
						if (candidateAddress == _rxHeaderFrom)
						{
							Serial.println("receive R2 request and I didn't success R1");
							addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, 2);
							printRoutingTable();
							//from, to, src, dst, type, data, flags, seqnum
							send(_thisAddress, _rxHeaderFrom, _thisAddress, GATEWAY_ADDR, R2_REQUEST_ACK_TYPE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
							_receivedRequestFlag = true;
						}
					}/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					else if (_rxHeaderType == MULTI_HOP_REQUEST_TYPE)
					{
						if (_rxHeaderDestination == _thisAddress)
						{
							dest = word(temp_buf[0], temp_buf[1]);
							if (dest != _thisAddress)
							{
								//아직 routing 안된 애 한테 보내줌
								Serial.println("received multihop request and send to unrouting Master");
								uint16_t dest = word(temp_buf[0], temp_buf[1]);
								//from, to, src, dst, type, data, flags, seqnum
								send(_thisAddress, dest, _thisAddress, dest, _rxHeaderType, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
							}
							else
							{
								//routing 안 되어있던 애가 받음
								if (candidateAddress == _rxHeaderFrom)
								{
									Serial.println("received multi hop request");
									_receivedRequestFlag = true;
									addRouteTo(GATEWAY_ADDR, _rxHeaderFrom, Valid);
									printRoutingTable();
									//from, to, src, dst, type, data, flags, seqnum
									send(_thisAddress, _rxHeaderFrom, _thisAddress, GATEWAY_ADDR, MULTI_HOP_REQUEST_ACK_TYPE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
								}
							}
						}
						else//중간 hop
						{
							Serial.println("received multihop request and send to routing Master. I'm middle Master");
							//from, to, src, dst, type, data, flags, seqnum
							send(_thisAddress, getRouteTo(_rxHeaderDestination)->next_hop[0], _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
						}
					}
					else if (_rxHeaderType == MULTI_HOP_REQUEST_ACK_TYPE)
					{
						Serial.println("received multihop request  ack and send");
						addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid);
						printRoutingTable();
						//from, to, src, dst, type, data, flags, seqnum
						send(_thisAddress, getRouteTo(_rxHeaderDestination)->next_hop[0], _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
					}
					else if (_rxHeaderType == UNRECEIVED_REQUEST)
					{
						if (_rxHeaderDestination == _thisAddress)
						{
							dest = word(temp_buf[0], temp_buf[1]);
							if (dest != _thisAddress)
							{
								//아직 routing 안된 애 한테 보내줌
								Serial.println("receive unreceived_request and send to unrouting Master");
								//from, to, src, dst, type, data, flags, seqnum
								send(_thisAddress, dest, _thisAddress, dest, UNRECEIVED_REQUEST, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
							}
							else
							{
								//routing 안되어있던 애가 받음
								Serial.println("receive unreceived_request and send ack");
								_receivedRequestFlag = true;
								addRouteTo(GATEWAY_ADDR, _rxHeaderFrom, Valid);
								printRoutingTable();
								//from, to, src, dst, type, data, flags, seqnum
								send(_thisAddress, _rxHeaderFrom, _thisAddress, GATEWAY_ADDR, UNRECEIVED_REQUEST_ACK, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
							}
						}
						else//중간 hop
						{
							Serial.println("receive unreceived_request and send to routing Master");
							//from, to, src, dst, type, data, flags, seqnum
							send(_thisAddress, getRouteTo(_rxHeaderDestination)->next_hop[0], _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
						}
					}
					else if (_rxHeaderType == UNRECEIVED_REQUEST_ACK)
					{
						Serial.println("receive unreceived_request  ack and send");
						addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid);
						printRoutingTable();
						//from, to, src, dst, type, data, flags, seqnum
						send(_thisAddress, getRouteTo(_rxHeaderDestination)->next_hop[0], _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
					}
					else if (_rxHeaderType == CHECK_ROUTING)
					{
						if(_rxHeaderDestination == _thisAddress)
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////