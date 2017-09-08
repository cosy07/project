//2017-07-17
// Datagram.h
#ifndef Datagram_Slave_h
#define Datagram_Slave_h

#include <cc1120.h>
#include <Datagram.h>


//=====================================================================================================
//	2017-04-26	ver.1
/****************************************************************/

class  DatagramForSlave
{
public:
	DatagramForSlave(ELECHOUSE_CC1120& driver, uint16_t thisAddress = 0);
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
	void           resetRetransmissions();


	byte           recvData(uint8_t* buf);


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


	uint16_t convertToAddress(uint8_t gatewayNumber, uint8_t masterNumber, uint8_t slaveNumber);

	uint8_t convertToMasterNumber(uint16_t address);

	byte receiveInterrupt(byte*);

	byte receive(byte*);

	void send(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size);

	bool sendToWait(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size, unsigned long time = 0);

	bool sendToWaitAck(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t data, uint8_t flags, uint8_t seqNum, uint8_t hop, uint8_t* temp_buf, uint8_t size, unsigned long time = 0);

	void printRecvPacketHeader();

	bool sendToWaitBroadcast(uint16_t from, uint16_t to, uint16_t src, uint16_t dst, uint8_t type, uint8_t headerData, uint8_t flags, uint8_t seqNum, uint8_t* temp_buf, uint8_t size, unsigned long time);
	
	bool M_handle_SCAN_SLAVE_message( int  slave_id,  uint8_t * read_buf  );
	
	bool  M_handle_SCAN_message( uint8_t *write_buf   );
	
	bool  M_handle_ERROR_message( uint16_t slave_address  );
	

protected:

//	uint8_t 	_lastE2ESequenceNumber;

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
//	uint32_t _retransmissions;

	/// The last sequence number to be used
	/// Defaults to 0
//	uint8_t _lastSequenceNumber;

	// Retransmit timeout (milliseconds)
	/// Defaults to 200
//	uint16_t _timeout;

	// Retries (0 means one try only)
	/// Defaults to 3
	uint8_t _retries;

	//=================================================================================
	//	2017-04-27 ver.1.1


	byte temp_buf[20];
	uint8_t bufIdx = 0;
	uint8_t receivedType = 0;//To avoid receiving from Master that has same hop

	byte i;
	uint16_t j;
	uint16_t dest;
 	uint8_t gatewayNumber = 0;

	unsigned long startTime;
//	uint16_t timeLimit;
//	uint16_t addr;
};

#endif
