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

//=====================================================================================================
//	2017-04-26	ver.1
/****************************************************************
#define
****************************************************************/
#define REQUEST_TYPE					0
#define REQUEST_ACK_TYPE				1
#define R2_REQUEST_TYPE					2
#define R2_REQUEST_REAL_TYPE			3
#define R2_REQUEST_ACK_TYPE				4
#define REQUEST_PATH_ONE_BY_ONE			5
#define REQUEST_PATH_ONE_BY_ONE_ACK		6
#define CHECK_ROUTING					7

#define ACK								8
#define NACK							9

#define MAX_NUM_TO_MASTER				10

#define SCAN_REQUEST_TO_MASTER			11
#define SCAN_REQUEST_TO_RC_EXTERNAL		12
#define SCAN_REQUEST_TO_SLAVE			13
#define SCAN_RESPONSE_TO_RC				14
#define SCAN_RESPONSE_TO_MASTER			15
#define SCAN_RESPONSE_TO_GATEWAY		16

#define CONTROL_MESSAGE					17		//INSTRUCTION_FROM_GATEWAY
#define SCAN_MESSAGE					18		//SCAN_FROM_GATEWAY
#define SCAN_SLAVE						19
#define SCAN_ACK						20
#define CONTROL_ACK						21		//INSTRUCTION_ACK_FROM_MASTER
#define INSTRUCTION_FROM_RC				22
#define ADD_ROUTE						23
#define MAX_NUM_OF_SLAVE				24
#define SCAN_SLAVE_ACK					25
#define ERROR_MESSAGE					26
#define SCAN_UPDATE						27

#define NONE						0
#define GATEWAY_ADDR				0x0000
#define TIME_TERM					2000
#define TIME_HOP					400
#define TIME_CONTROL				5000
#define NUM_OF_CONTRL				1

void RS485_Write_Read(uint8_t *write_buf,uint8_t *read_buf);


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
	void	setHeaderHop(uint8_t hop);

	uint16_t        headerTo();
	uint16_t        headerFrom();
	uint16_t        headerSource();
	uint16_t        headerDestination();
	uint8_t         headerType();
	uint8_t         headerData();
	uint8_t   		headerSeqNum();
	uint8_t   		headerFlags();
	uint8_t			headerHop();

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
		uint16_t      next_hop;  ///< Send via this next hop address
		uint8_t      state;     ///< State of this route, one of RouteState
		uint8_t		 hop;
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

	uint16_t convertToAddress(uint8_t gatewayNumber, uint8_t masterNumber, uint8_t slaveNumber);

	uint8_t convertToMasterNumber(uint16_t address);

	byte receiveInterrupt(byte*);

	byte receive(byte*);

	void send(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size);

	bool sendToWaitAck(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size, unsigned long time = 2000);

	void printRecvPacketHeader();

	void printPath();
	
	bool G_request_path_one_by_one(uint16_t address, byte row_number, uint16_t* node_list, byte number_of_node);

	void G_discoverNewPath(uint16_t address);

	void G_handle_CONTROL_message(byte* maxOP, byte* curOP, byte list_of_message_from_PC[][10], byte fromFCU[][10]);

	bool G_send_Control_wait_ACK_from_master(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t headerData, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size);

	bool G_handle_SCAN_message(uint16_t master_address, byte fromPC[], byte fromFCU[][10]);

	bool sendToWaitBroadcast(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t headerData, uint8_t flags, uint8_t seqNum, uint8_t* temp_buf, uint8_t size, unsigned long time);

	bool M_handle_CONTROL_message( uint8_t *write_buf, Datagram manager2);
	
	bool M_handle_SCAN_SLAVE_message( int  slave_id,  uint8_t * read_buf  );
	
	bool  M_handle_SCAN_message( uint8_t *write_buf   );
	
	bool  M_handle_ERROR_message( uint16_t slave_address  );
	

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
	uint8_t				_rxHeaderHop;

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
	uint16_t parentMaster[NUM_OF_CONTRL + 1];

	uint16_t candidateAddress = 0;
	uint8_t candidateRSSI = 0;

	byte temp_buf[20];
	uint8_t bufIdx = 0;
	uint8_t receivedType = 0;//To avoid receiving from Master that has same hop

	byte i;
	uint16_t j;
	uint16_t dest;
 	uint8_t gatewayNumber = 0;

	unsigned long startTime;
	uint16_t timeLimit;
	uint16_t addr;
};

#endif
