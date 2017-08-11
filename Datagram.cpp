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
void Datagram::setHeaderHop(uint8_t hop)
{
	_driver.setHeaderHop(hop);
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
uint8_t Datagram::headerHop()
{
	return(_driver.headerHop());
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
		_rxHeaderHop = headerHop();
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
			_routes[i].next_hop = next_hop;
			_routes[i].hop = hop;
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
			_routes[i].next_hop = next_hop;
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
			_routes[i].next_hop = next_hop;
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
		Serial.print(" Next Hop: ");
		Serial.print(_routes[i].next_hop, DEC);
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
		_routes[i].next_hop = 0;
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
//gateway address : 0x0000
//Master address : 0x0001부터
void Datagram::FromGatewayToMaster() {
	uint16_t address_i;
	Serial.println("FromGatewayToMaster");
	Serial.println("[row1]");
	//row1 : 모든 master들에게 한번씩 보내봄(1hop인 노드 이 때 체크)

	if (G_find_1stRow_master() < 0)// 모든 노드들이 1hop 거리 일경우
			return;

	if (G_find_2ndRow_master() < 0)
		return;

	printRoutingTable();
	while(G_find_multihop_node());

	for (int i = 1; i <= NUM_OF_CONTRL; i++)
	{
		address_i = convertToAddress(gatewayNumber, i, 0);
		send(_thisAddress, getRouteTo(address_i)->next_hop, _thisAddress, address_i, CHECK_ROUTING, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
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
						//from, to, src, dst, type, data, flags, seqnum, hop
						send(_thisAddress, GATEWAY_ADDR, _thisAddress, GATEWAY_ADDR, REQUEST_ACK_TYPE, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
					}
					else if (_rxHeaderType == R2_REQUEST_TYPE)
					{
						Serial.println("receive R2 request and I succeed R1");
						//from, to, src, dst, type, data, flags, seqnum, hop
						send(_thisAddress, GATEWAY_ADDR, _thisAddress, GATEWAY_ADDR, ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
						M_find2ndRowMasters();
					}
					else if (_rxHeaderType == R2_REQUEST_REAL_TYPE)
					{
						Serial.println("receive R2 routing request and I didn't success in R1 request");
						M_masterSendRoutingReply();
					}/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
					else if (_rxHeaderType == REQUEST_PATH_ONE_BY_ONE)
					{
						if (_rxHeaderDestination == _thisAddress)
						{
							dest = word(temp_buf[0], temp_buf[1]);
							if (dest != _thisAddress)
							{
								//아직 routing 안된 애 한테 보내줌
								Serial.println("received multihop request and send to unrouting Master");
								uint16_t dest = word(temp_buf[0], temp_buf[1]);
								//from, to, src, dst, type, data, flags, seqnum, hop
								sendToWaitAck(_thisAddress, dest, _rxHeaderSource, dest, _rxHeaderType, NONE, _rxHeaderFlags + 1, NONE, NONE, temp_buf, sizeof(temp_buf), TIME_TERM);
							}
							else
							{
								//routing 안 되어있던 애가 받음
								Serial.println("received multi hop request");
								M_masterSendRoutingReply();
							}
						}
						else//중간 hop
						{
							Serial.println("received multihop request and send to routing Master. I'm middle Master");
							//from, to, src, dst, type, data, flags, seqnum, hop
							sendToWaitAck(_thisAddress, getRouteTo(_rxHeaderDestination)->next_hop, _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, _rxHeaderFlags + 1, NONE, NONE, temp_buf, sizeof(temp_buf), TIME_TERM);
						}
					}
					else if (_rxHeaderType == REQUEST_PATH_ONE_BY_ONE_ACK)
					{
						Serial.println("received multihop request  ack and send");
						addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderFlags + 1);
						printRoutingTable();
						//from, to, src, dst, type, data, flags, seqnum, hop
						if (_rxHeaderFlags == 0)
						{
							temp_buf[0] = highByte(_thisAddress);
							temp_buf[1] = lowByte(_thisAddress);
						}
						send(_thisAddress, getRouteTo(_rxHeaderDestination)->next_hop, _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, _rxHeaderFlags + 1, NONE, NONE, temp_buf, sizeof(temp_buf));
					}
					else if (_rxHeaderType == CHECK_ROUTING)
					{
						if (_rxHeaderDestination == _thisAddress)
							break;
						else
						{
							//from, to, src, dst, type, data, flags, seqnum, hop
							send(_thisAddress, getRouteTo(_rxHeaderDestination)->next_hop, _rxHeaderSource, _rxHeaderDestination, _rxHeaderType, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
						}
					}
				}
			}
		}
	}
}
void Datagram::send(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, byte* temp_buf, byte size)
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
bool Datagram::sendToWait(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, byte* temp_buf, byte size, unsigned long time)
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

bool Datagram::sendToWaitAck(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, byte* temp_buf, byte size, unsigned long time)
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
	Serial.print("_rxHeaderHop : ");
	Serial.println(_rxHeaderHop);
}
void Datagram::printPath()
{
	uint16_t temp_address;
	Serial.println("-----PATH-----");
	byte index;
	for (uint8_t i = 1; i <= NUM_OF_CONTRL;i++)
	{
		Serial.print(i);
		Serial.print(" master : ");
		temp_address = convertToAddress(gatewayNumber, i, 0);
		while (temp_address != GATEWAY_ADDR)
		{
			Serial.print(temp_address);
			Serial.print(" -> ");
			index = convertToMasterNumber(temp_address);
			temp_address = parentMaster[index];
		}
		Serial.print(" -> ");
		Serial.println(GATEWAY_ADDR);
	}
	Serial.println("--------------");
}
/****************************************************************
*FUNCTION NAME:  convertToAddress(uint8_t gatewayNumber, uint8_t masterNumber,uint8_t slaveNumber)
*FUNCTION     :  2 바이트 형식의 주소로 변환
*INPUT        : 게이트 번호, 매스터 노드 번호, 슬레이브 번호
*OUTPUT       : 2바이트  노드 주소
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
*FUNCTION     :  일반적인 주소로 마스터 번호를 구함
*INPUT        : 2바이트 노도 주소
*OUTPUT       : 마스터 번호
****************************************************************/
uint8_t Datagram::convertToMasterNumber(uint16_t address)
{
	uint8_t masterNum;
	address = address & 0x03FF;
	masterNum = (uint8_t)address >> 5;
	return masterNum;
}
/****************************************************************
*FUNCTION NAME:  G_find_1stRow_master( )
*FUNCTION     : Gateway가 direct하게 연결할수 있는 매스터 노드들에게 라우팅 요청 메시지를 전송하고,
응답을 기다린다.
- 응답이 도착하면, 라우팅 테이블에 삽입하며,
*INPUT        : none
*OUTPUT       : 1 hop 보다 먼 노드들의 갯수, 모든 노드들이 1hop 경우 -1 return
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
		//from, to, src, dst, type, data, flags, seqnum, hop, payload, size of payload
		send(_thisAddress, address_i, _thisAddress, address_i, REQUEST_TYPE, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));


		startTime = millis();
		while ((millis() - startTime) < TIME_TERM * 2)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf) && (_rxHeaderTo == BROADCAST_ADDRESS || _rxHeaderTo == _thisAddress))
				{
					printRecvPacketHeader();
					if (_rxHeaderType == REQUEST_ACK_TYPE && _rxHeaderSource == address_i && _rxHeaderDestination == _thisAddress && _rxHeaderFrom == address_i)//조건에 맞는다면 걔네는 1hop인 master들
					{
						Serial.println("receive row1 request_ack");
						addRouteTo(_rxHeaderFrom, _rxHeaderFrom, Valid, 1);//dst, next, state, hop count
						checkReceive[i] = true;
						parentMaster[i] = _thisAddress;
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
*FUNCTION     :  Gateway가  1 hop 노드들에게, 라우팅이 안된 노드들의 주소를 페이로드에 포함하여 전송함.

*INPUT        : none
*OUTPUT       :
****************************************************************/
int8_t Datagram::G_find_2ndRow_master()
{
	//================================================================================================================================================
	//row2 : 1hop인 master들에게 아직 라우팅이 안된 노드들의 주소를 보내서 근처에 노드들이 있는지 물어봄
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
		for (j = 1; j <= NUM_OF_CONTRL; j++)//routing이 안된 노드들의 주소를 table을 뒤져서 알아냄
		{
			if (!checkReceive[j])
			{
				number_of_unknown_node++;
				Serial.print("unreceive : ");
				Serial.println(j);
				address = convertToAddress(gatewayNumber, j, 0);
				//routing이 안된 애들의 주소를 전송되는 배열에 넣는다.

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
		//from, to, src, dst, type, data, flags, seqnum, hop, payload, size of payload, timeout
		receiving = sendToWaitAck(_thisAddress, node_list[i], _thisAddress, node_list[i], R2_REQUEST_TYPE, number_of_unknown_node, NONE, NONE, NONE, temp_buf, sizeof(temp_buf), TIME_TERM); //1hop인 node 들한테 보냄
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
					if (_rxHeaderTo == _thisAddress && _rxHeaderType == R2_REQUEST_ACK_TYPE && _rxHeaderFrom == node_list[i] && _rxHeaderSource > 0 && _rxHeaderSource <= NUM_OF_CONTRL)
					{
						addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, 2);
						masterNum = convertToMasterNumber(_rxHeaderSource);
						checkReceive[masterNum] = true;
						printRecvPacketHeader();
						parentMaster[masterNum] = _rxHeaderFrom;
						if (_rxHeaderData > 1)
						{
							bufIdx = 0;
							for (i = 1; i < _rxHeaderData; i++)
							{
								addr = word(temp_buf[bufIdx], temp_buf[bufIdx + 1]);
								bufIdx += 2;
								addRouteTo(addr, _rxHeaderFrom, Valid, 2);
								masterNum = convertToMasterNumber(addr);
								parentMaster[masterNum] = _rxHeaderFrom;
								checkReceive[masterNum] = true;
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
	if (_receivedRequestFlag == false && (_rxHeaderType == REQUEST_ACK_TYPE || _rxHeaderType == R2_REQUEST_ACK_TYPE || _rxHeaderType == REQUEST_PATH_ONE_BY_ONE_ACK))
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
		receiving = sendToWaitAck(_thisAddress, to_address_temp, GATEWAY_ADDR, to_address_temp, R2_REQUEST_REAL_TYPE, NONE, 1, NONE, NONE, temp_buf, sizeof(temp_buf), TIME_TERM);
		bufIdx += 2;
		if (receiving) 
		{
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
		//from, to, src, dst, type, data, flags, seqnum, hop
		send(_thisAddress, GATEWAY_ADDR, childNodeList[0], GATEWAY_ADDR, R2_REQUEST_ACK_TYPE, idx, 1, NONE, NONE, temp_buf, sizeof(temp_buf));
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
	if (candidateAddress == _rxHeaderFrom)
	{
		addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderFlags + 1);
		printRoutingTable();
		//from, to, src, dst, type, data, flags, seqnum, hop
		if (_rxHeaderType == R2_REQUEST_REAL_TYPE)
			send(_thisAddress, _rxHeaderFrom, _thisAddress, GATEWAY_ADDR, R2_REQUEST_ACK_TYPE, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
		else if (_rxHeaderType == REQUEST_PATH_ONE_BY_ONE)
		{
			send(_thisAddress, _rxHeaderFrom, _thisAddress, GATEWAY_ADDR, REQUEST_PATH_ONE_BY_ONE_ACK, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf));
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
		if (_routes[i].hop == row_number)
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
- 먼저 2nd row node들을 구한후, 아직 routing이 안된 노드의 주소를  payload에 포함하여 전송한다.
- 3rd row 노드들을 구한후에는, routing이 안된 노드의 주소를  payload에 포함하여
최근에 구한 row의 노드들에게  multihop routing 요청 메시지를 전송한다.
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

	Serial.println("[MULTI_HOP]");
	while (number_of_node != 0)
	{
		for (i = 1; i <= NUM_OF_CONTRL; i++)
		{
			if (!checkReceive[i])
			{
				address = convertToAddress(gatewayNumber, i, 0);
				if (G_request_path_one_by_one(address, row_number, node_list, number_of_node))
					number_of_unknown_node--;
			}
			if (number_of_unknown_node == 0)
				break;
		}
		if (number_of_unknown_node == 0)
			break;

		number_of_node = G_get_i_row_node_list(row_number, node_list);
		row_number++;
	}
	return number_of_unknown_node;
}
/****************************************************************
*FUNCTION NAME:  G_request_path_one_by_one(uint16_t address, byte row_number, uint16_t* node_list, byte number_of_node)
*FUNCTION     :	 node_list에 있는 노드들에게 주소가 address인 노드가 인접해 있는지 물어봄

*INPUT        :  목적지 주소
*OUTPUT       :  성공 여부
****************************************************************/
bool Datagram::G_request_path_one_by_one(uint16_t address, byte row_number, uint16_t* node_list, byte number_of_node)
{
	bool result = false;
	uint16_t next_hop;
	uint8_t masterNum;
	masterNum = convertToMasterNumber(address);
	uint8_t highAddress, lowAddress;

	for (i = 0; i < number_of_node; i++)
	{
		highAddress = (uint8_t)(address >> 8);
		lowAddress = (uint8_t)(0x00FF & address);

		temp_buf[0] = highAddress;
		temp_buf[1] = lowAddress;

		next_hop = getRouteTo(node_list[i])->next_hop;
		//from, to, src, dst, type, data, flags, seqnum, hop

		if (!sendToWait(_thisAddress, next_hop, _thisAddress, node_list[i], REQUEST_PATH_ONE_BY_ONE, NONE, NONE, NONE, NONE, temp_buf, sizeof(temp_buf)))
			continue;
		startTime = millis();
		while (millis() - startTime < row_number * 2 * TIME_TERM * 2)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf) && _rxHeaderDestination == _thisAddress && _rxHeaderSource == address && _rxHeaderType == REQUEST_PATH_ONE_BY_ONE_ACK && _rxHeaderFrom == next_hop && _rxHeaderTo == _thisAddress)
				{
					printRecvPacketHeader();
					addRouteTo(_rxHeaderSource, _rxHeaderFrom, Valid, _rxHeaderFlags + 1);
					parentMaster[masterNum] = word(temp_buf[0], temp_buf[1]);
					checkReceive[masterNum] = true;
					printRoutingTable();
					return true;
				}
			}
		}
	}
	return false;
}
/****************************************************************
*FUNCTION NAME:  G_discoverNewPath(uint16_t address)
*FUNCTION     :

*INPUT        :
*OUTPUT       :
****************************************************************/
void Datagram::G_discoverNewPath(uint16_t address)
{
	byte row_number;
	byte number_of_node;
	uint16_t node_list[NUM_OF_CONTRL] = { 0 };
	uint8_t masterNum;

	row_number = getRouteTo(address)->hop;
	number_of_node = G_get_i_row_node_list(row_number, node_list);

	G_request_path_one_by_one(address, row_number + 1, node_list, number_of_node);//함수의 매개변수인 address에 대한 path 새로 지정

	for (int i = 1; i <= NUM_OF_CONTRL; i++)//함수의 매개변수인 address의 자식들의 path 새로 지정
	{
		if (parentMaster[i] == address)
		{
			G_request_path_one_by_one(convertToAddress(gatewayNumber, i, 0), row_number + 1, node_list, number_of_node);
		}
	}
}
/****************************************************************
*FUNCTION NAME: G_handel_CONTROL_message ()
*FUNCTION     :  Gateway handles of a transmittion of CONTROL message.
First, check if there is any control message from PC. If yes, send it to the destination node.
if there are more control message from PC, then process all.
*INPUT        :
*OUTPUT       :

****************************************************************/
void Datagram::G_handle_CONTROL_message(byte* maxOP, byte* curOP, byte list_of_message_From_PC[][10], byte fromFCU[][10])
{
	uint16_t  			To;
	uint16_t  			From;
	uint16_t  			Source;
	uint16_t			Destination;
	uint16_t			master_address;
	uint8_t				master_id;
	uint16_t			Temp;

	while (*maxOP != *curOP) // send all control message to the master, the control message has the highest priority to send.
	{

		master_id = (uint8_t)list_of_message_From_PC[*curOP][2];

		master_address = _thisAddress | (uint16_t)(master_id << 5);
		From = _thisAddress;
		Source = _thisAddress;
		To = getRouteTo(master_address)->next_hop;
		Destination = master_address;

		if (G_send_Control_wait_ACK_from_master(From, To, Source, Destination, CONTROL_MESSAGE, 0, 0, 0, 0,list_of_message_From_PC[*curOP], sizeof(list_of_message_From_PC[*curOP])))
		{ // if there is a response from the master, then update a report table based on the control message.
			for (int i = 0; i < 9; i++)
			{
				fromFCU[master_id][i] = list_of_message_From_PC[*curOP][i];
			}
			fromFCU[master_id][0] = 0xAD;
		}
		else  // if there is no response from the master, then update a report table as 0
		{
			for (int i = 0; i < 9; i++)
			{
				fromFCU[master_id][i] = 0;
			}

			fromFCU[master_id][0] = 0xEE;
			fromFCU[master_id][2] = master_id;
			for (int i = 0; i < 9; i++)
			{
				fromFCU[master_id][9] ^= fromFCU[master_id][i];
			}
		}
		*curOP++;
		if (*curOP >= 32)
			*curOP = 0;
	}
}
/****************************************************************
*FUNCTION NAME: G_send_Control_wait_ACK_from_master ()
*FUNCTION     :  Gateway sends a CONTROL message to the next node with the destination node address.
- First, the gateway should get the ACK from the next node. If not, retransmit
- Second, the gateway should get the CONTROL_ACK from the destination master node. If not, retransmit
*INPUT        :uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t headerData, uint8_t flags, uint8_t seqNum, byte* temp_buf, byte size,
*OUTPUT       : true if the destination receives the CONTROL message succefully.

****************************************************************/
bool Datagram::G_send_Control_wait_ACK_from_master(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t headerData, uint8_t flags, uint8_t seqNum, uint8_t hop, byte* temp_buf, byte size)
{
	unsigned long startTime = 0;
	unsigned long ete_startTime = 0; // end to end timer 
	int retry = 3;
	int ete_retry = 3;				// end to end retry
	setHeaderFrom(from);
	setHeaderTo(to);
	setHeaderSource(src);
	setHeaderDestination(dst);
	setHeaderType(type);
	setHeaderHop(hop);
	setHeaderData(headerData);
	setHeaderSeqNum(seqNum);
	bool receiveACK = false;
	for (int k = 0; k < ete_retry; k++)
	{
		ete_startTime = millis();
		for (int i = 0; i < retry; i++)  // the gateway sends a control message to a next node, and wait an ACK from the next node. If no ACK, then retransmit it.
		{
			_driver.SendData(temp_buf, size);
			startTime = millis();
			while (millis() - startTime < TIME_HOP) // TIME_HOP is about a time to get an ACK from the next node. It is around 350 ~ 400ms.
			{
				SetReceive();
				if (available())
				{
					if (recvData(temp_buf) && _rxHeaderTo == _thisAddress)
					{
						if (_rxHeaderType == ACK && _rxHeaderFrom == to)
						{
							receiveACK = true;
							break;
						}
						if (_rxHeaderType == ADD_ROUTE)
						{
							addRouteTo(headerSource(), _rxHeaderFrom, Valid, headerHop());
						}
						if (_rxHeaderType == CONTROL_ACK && _rxHeaderSource == dst)
						{
							return true;
						}
					}
				}
			}
			if (receiveACK) // if there is a ACK from the next node, then wait the CONTROL_ACK from the destination node.
				break;
		}
		while (millis() - ete_startTime < TIME_CONTROL)  // wait an ACK from the destination master node. TIME_CONTROL is a time for waiting a CONTROL_ACK from the destination node.
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf) && _rxHeaderTo == _thisAddress)
				{
					if (_rxHeaderType == ADD_ROUTE)
					{
						addRouteTo(headerSource(), _rxHeaderFrom, Valid, headerHop());
					}
					if (_rxHeaderType == CONTROL_ACK && _rxHeaderSource == dst)
					{
						return true;
					}
				}
			}
		}
	}
	return false;
}

bool Datagram::G_handle_SCAN_message(uint16_t master_address, byte fromPC[10], byte fromFCU[][10])
{
	unsigned long startTime = 0;
	unsigned long ete_startTime = 0; // end to end timer 
	int 		retry = 3;
	uint16_t  	to = getRouteTo(master_address)->next_hop;
	uint8_t 	master_id = convertToMasterNumber(master_address);

	setHeaderFrom(_thisAddress);
	setHeaderTo(to);
	setHeaderSource(_thisAddress);
	setHeaderDestination(master_address);
	setHeaderType(SCAN_MESSAGE);
	setHeaderHop(1);
	setHeaderData(NONE);
	setHeaderSeqNum(NONE);

	bool receiveACK = false;

	ete_startTime = millis();

	for (int i = 0; i < retry; i++)  // the gateway sends a scan message to a next node, and wait an ACK from the next node. If no ACK, then retransmit it.
	{
		_driver.SendData(fromPC, sizeof(fromPC));
		startTime = millis();
		while (millis() - startTime < TIME_HOP)
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf) && _rxHeaderTo == _thisAddress)
				{
					if (_rxHeaderType == ACK && _rxHeaderFrom == to)
					{
						receiveACK = true;
						break;
					}
					if (_rxHeaderType == SCAN_ACK && _rxHeaderFrom == to)
					{
						for (int j = 0; j < 10; j++)
						{
							fromFCU[master_id][j] = temp_buf[j];
						}
						return true;
					}
					if (_rxHeaderType == ADD_ROUTE)
					{
						addRouteTo(headerSource(), _rxHeaderFrom, Valid, headerHop());
					}
				}
			}
		}
		if (receiveACK) // if there is a ACK from the next node, then wait the SCAN_ACK from the destination node.
			break;
	}
	if (receiveACK)
	{
		while (millis() - ete_startTime < TIME_HOP * 6)  // wait an SCAN_ACK from the destination master node.
		{
			SetReceive();
			if (available())
			{
				if (recvData(temp_buf) && _rxHeaderTo == _thisAddress)
				{
					if (_rxHeaderType == ADD_ROUTE)
					{
						addRouteTo(headerSource(), _rxHeaderFrom, Valid, headerHop());
					}
					if (_rxHeaderType == SCAN_ACK && _rxHeaderSource == master_address)     //  if there is a SCAN_ACK from the desination node, update the report table
					{
						for (int j = 0; j < 10; j++)
						{
							fromFCU[master_id][j] = temp_buf[j];
						}
						return true;
					}
					if (_rxHeaderType == NACK)
						break;
				}
			}
		}
	}
	for (int i = 0; i < 9; i++)
	{
		fromFCU[master_id][i] = 0;
	}
	for (int i = 0; i < 9; i++)
	{
		fromFCU[master_id][9] ^= fromFCU[master_id][i];
	}
	fromFCU[master_id][0] = 0xEE;
	fromFCU[master_id][2] = master_id;

	return false;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////