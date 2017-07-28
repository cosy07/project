//2017-07-17
// Datagram.h
#ifndef Datagram_h
#define Datagram_h

#include <cc1120.h>



/// the default retry timeout in milliseconds
#define DEFAULT_TIMEOUT 100


/// The default number of retries
#define DEFAULT_RETRIES 3
// Default max number of hops we will route
#define DEFAULT_MAX_HOPS 5

// The default size of the routing table we keep
#define ROUTING_TABLE_SIZE 10
#define ROUTER_MAX_MESSAGE_LEN 48

#define ROUTER_ERROR_NONE              0
#define ROUTER_ERROR_INVALID_LENGTH    1
#define ROUTER_ERROR_NO_ROUTE          2
#define ROUTER_ERROR_TIMEOUT           3
#define ROUTER_ERROR_NO_REPLY          4
#define ROUTER_ERROR_UNABLE_TO_DELIVER 5

//=====================================================================================================
//	2017-04-26	ver.1
/****************************************************************
#define
****************************************************************/
#define REQUEST_TYPE					5
#define REQUEST_ACK_TYPE				6
#define R2_REQUEST_TYPE					7
#define R2_REQUEST_REAL_TYPE			8
#define R2_REQUEST_ACK_TYPE				9
#define MULTI_HOP_REQUEST_TYPE			10
#define MULTI_HOP_REQUEST_ACK_TYPE		11
#define UNRECEIVED_REQUEST				12
#define UNRECEIVED_REQUEST_ACK			13
#define CHECK_ROUTING					14
#define ACK								15
#define MAX_NUM_TO_MASTER				16

#define SCAN_REQUEST_TO_MASTER			17
#define SCAN_REQUEST_TO_RC_EXTERNAL		18
#define SCAN_REQUEST_TO_SLAVE			19
#define SCAN_RESPONSE_TO_RC				20
#define SCAN_RESPONSE_TO_MASTER			21
#define SCAN_RESPONSE_TO_GATEWAY		22

#define INSTRUCTION_FROM_GATEWAY		23
#define INSTRUCTION_FROM_RC				24

#define NONE						0
#define GATEWAY_ADDR				0x0000
#define TIME_TERM					2000
#define NUM_OF_CONTRL				3
class  Datagram
{
public:
	Datagram(ELECHOUSE_CC1120& driver, uint16_t thisAddress = 0);
	void	init();
	void 	init(byte ch);
	void 	SetReceive(void);
	void 	setThisAddress(uint16_t thisAddress);
	void 	setHeaderTo(uint16_t to);
	void 	setHeaderFrom(uint16_t from);
	void 	setHeaderSource(uint16_t address);
	void	setHeaderDestination(uint16_t address);
	void 	setHeaderType(uint8_t type);
	void 	setHeaderData(uint8_t data);
	void    setHeaderSeqNum(uint8_t seq);
	void    setHeaderFlags(uint8_t flags);

	uint16_t        headerTo();
	uint16_t        headerFrom();
	uint16_t        headerSource();
	uint16_t        headerDestination();
	uint8_t         headerType();
	uint8_t         headerData();
	uint8_t   		headerSeqNum();
	uint8_t   		headerFlags();

	bool  	       waitAvailableTimeout(uint16_t timeout);
	bool           available();
	void           waitAvailable();
	void 	       setTimeout(uint16_t timeout);
	void 	       setRetries(uint8_t retries);
	uint8_t	       retries();
	void   	       acknowledge();
	void           MainSendControlToSlave();
	void           RCSendControlToMain();
	void           resetRetransmissions();


	void           sendto(uint8_t* buf, uint8_t len, uint16_t address);
	byte           recvData(uint8_t* buf);
	/*
	bool            waitPacketSent();
	bool            waitPacketSent(uint16_t timeout);

	*/

	/// Defines the structure of the RHRouter message header, used to keep track of end-to-end delivery parameters
	typedef struct
	{
		uint16_t    dest;       ///< Destination node address
		uint16_t    source;     ///< Originator node address
		uint8_t    hops;       ///< Hops traversed so far
		uint8_t    id;         ///< Originator sequence number
		uint8_t    flags;      ///< Originator flags
							   // Data follows, Length is implicit in the overall message length
	} RoutedMessageHeader;

	/// Defines the structure of a Router message
	typedef struct
	{
		RoutedMessageHeader header;    ///< end-to-end delivery header
		uint8_t             data[ROUTER_MAX_MESSAGE_LEN]; ///< Application payload data
	} RoutedMessage;

	/// Values for the possible states for routes
	typedef enum
	{
		Invalid = 0,           ///< No valid route is known
		Discovering,           ///< Discovering a route (not currently used)
		Valid                  ///< Route is valid
	} RouteState;

	/// Defines an entry in the routing table
	typedef struct
	{
		uint16_t      dest;      ///< Destination node address
		uint16_t      next_hop[2];  ///< Send via this next hop address
		uint8_t      state;     ///< State of this route, one of RouteState
		uint8_t		 hop[2];
	} RoutingTableEntry;

	/// \param[in] thisAddress The address to assign to this node. Defaults to 0

	void setMaxHops(uint8_t max_hops);


	void addRouteTo(uint16_t  dest, uint16_t  next_hop, uint8_t state = Valid, uint8_t hop = 0);


	RoutingTableEntry* getRouteTo(uint16_t  dest);


	bool deleteRouteTo(uint16_t  dest);


	void retireOldestRoute();


	void clearRoutingTable();


	void printRoutingTable();


	void FromGatewayToMaster();

	int8_t G_find_1stRow_master();

	int8_t G_find_2ndRow_master();

	uint8_t G_get_i_row_node_list(uint8_t row_number, uint16_t *node_list);

	int8_t G_find_multihop_node();

	void FromMasterToGateway();

	void M_findCandidateParents();

	void M_find2ndRowMasters();

	void M_masterSendRoutingReply();

	uint16_t convertAddress(uint8_t gatewayNumber, uint8_t masterNumber, uint8_t slaveNumber);

	byte receiveInterrupt(byte*);

	byte receive(byte*);

	void send(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t headerData, uint8_t flags, uint8_t seqNum, byte* temp_buf, byte size);

	bool sendToWait(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t headerData, uint8_t flags, uint8_t seqNum, byte* temp_buf, byte size, unsigned long time);

	bool sendToWaitAck(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t headerData, uint8_t flags, uint8_t seqNum, byte* temp_buf, byte size, unsigned long time);

	void printRecvPacketHeader();

protected:

	//   static RoutedMessage _tmpMessage;

	RoutedMessage _tmpMessage;





	void 	deleteRoute(uint8_t index);


	uint8_t 	_lastE2ESequenceNumber;

	uint8_t     _max_hops;

	/// Temporary mesage buffer



	/// Local routing table
	RoutingTableEntry    _routes[ROUTING_TABLE_SIZE];


	ELECHOUSE_CC1120&   _driver;	   /// The Driver we are to use
	uint16_t         	_thisAddress;		/// The address of this node
	uint16_t  			_rxHeaderTo;
	uint16_t  			_rxHeaderFrom;
	uint16_t  			_rxHeaderSource;
	uint16_t			_rxHeaderDestination;
	uint8_t   			_rxHeaderType;
	uint8_t   			_rxHeaderData;
	uint8_t   			_rxHeaderFlags;
	uint8_t   			_rxHeaderSeqNum;

	/// Count of retransmissions we have had to send
	uint32_t _retransmissions;

	/// The last sequence number to be used
	/// Defaults to 0
	uint8_t _lastSequenceNumber;

	// Retransmit timeout (milliseconds)
	/// Defaults to 200
	uint16_t _timeout;

	// Retries (0 means one try only)
	/// Defaults to 3
	uint8_t _retries;

	//=================================================================================
	//	2017-04-27 ver.1.1
	bool _receivedRequestFlag = false;
	bool checkReceive[NUM_OF_CONTRL + 1] = { false };
	uint8_t receivedMasterNum[2] = { 0 };
	uint16_t receivedMaster[2][NUM_OF_CONTRL];


	uint16_t candidateAddress = 0;
	uint8_t candidateRSSI = 0;
	uint16_t candidateAddress2 = 0;
	uint8_t candidateRSSI2 = 0;



	byte temp_buf[20];
	uint8_t bufIdx = 0;
	uint8_t receivedType = 0;//To avoid receiving from Master that has same hop

	byte i;
	uint16_t j;
	uint16_t dest;
	uint8_t gatewayNumber = 0;

	unsigned long startTime;
	int8_t unreceivedNum = 0;
	uint16_t timeLimit;
	uint16_t addr;
	uint8_t rowFlag = 3;
};

#endif
