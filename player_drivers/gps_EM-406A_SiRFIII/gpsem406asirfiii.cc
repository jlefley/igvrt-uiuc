
/*
 * A custom driver for the EM-406A SiRF III GPS
 *
 * Interfaces:
 *   - Provides:
 *     - GPS
 *   - Requires:
 *     - <none>
 */

#include <libplayercore/playercore.h>
#include "gpsem406asirfiii_serial.h"

//c++ libs
#include <iostream>




////////////////////////////////////////////////////////////////////////////////
// The class for the driver
class GPSEM406ASIRFIIIDriver : public ThreadedDriver
{
public:
    
	// Constructor
	GPSEM406ASIRFIIIDriver(ConfigFile* cf, int section);

	// This method will be invoked on each incoming message
	virtual int ProcessMessage(QueuePointer &resp_queue, player_msghdr * hdr, void * data);

private:

	virtual void Main();
	virtual int MainSetup();
	virtual void MainQuit();

	void publishGPSData();

	//private data members
	player_devaddr_t gps_addr; // address of gps interface
	const char* serial_port; // name of serial port of gps
	GPSEM406ASIRFIIISerial g; // low-level serial object for gps
};

// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver* GPSEM406ASIRFIIIDriver_Init(ConfigFile* cf, int section)
{
	// Create and return a new instance of this driver
	return((Driver*)(new GPSEM406ASIRFIIIDriver(cf, section)));
}

// A driver registration function, again declared outside of the class so
// that it can be invoked without object context.  In this function, we add
// the driver into the given driver table, indicating which interface the
// driver can support and how to create a driver instance.
void GPSEM406ASIRFIIIDriver_Register(DriverTable* table)
{
	table->AddDriver("gpsem406asirfiii", GPSEM406ASIRFIIIDriver_Init);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
GPSEM406ASIRFIIIDriver::GPSEM406ASIRFIIIDriver(ConfigFile* cf, int section)
	: ThreadedDriver(cf,section,false,PLAYER_MSGQUEUE_DEFAULT_MAXLEN)
{
	memset(&this->gps_addr,0,sizeof(player_devaddr_t));

	// Do we create a gps interface?
	if(cf->ReadDeviceAddr(&(this->gps_addr), section, "provides", PLAYER_GPS_CODE, -1, NULL) == 0)
	{
		if(this->AddInterface(this->gps_addr) != 0)
		{
			this->SetError(-1);
			return;
		}
	}

	// Read options from the configuration file
	this->serial_port = cf->ReadString(section, "port", "/dev/ttyS0");

	return;
}


////////////////////////////////////////////////////////////////////////////////
// Set up the device.  Return 0 if things go well, and -1 otherwise.
// Here you do whatever is necessary to setup the device, like open and
// configure a serial port.
int GPSEM406ASIRFIIIDriver::MainSetup()
{
	PLAYER_MSG0(0, "GPS EM406A SiRF III driver starting up...");

	// connect to gps
	this->g.connect(this->serial_port, 4800);
	this->g.flush();
    
	PLAYER_MSG0(0, "GPS EM406A SiRF III driver ready.");

	return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
// Here you would shut the device down by, for example, closing a
// serial port.
void GPSEM406ASIRFIIIDriver::MainQuit()
{
	PLAYER_MSG0(0, "GPS EM406A SiRF III driver shutting down...");

	// close connection
	this->g.closeConnection();

	PLAYER_MSG0(0, "GPS EM406A SiRF III driver has been shutdown.");
}

// Process messages here.  Send a response if necessary, using Publish().
// If you handle the message successfully, return 0.  Otherwise,
// return -1, and a NACK will be sent for you, if a response is required.
int GPSEM406ASIRFIIIDriver::ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr, void * data)
{

	return 0;
}


void GPSEM406ASIRFIIIDriver::publishGPSData()
{
	player_gps_data_t gpsdata;
	memset(&gpsdata,0,sizeof(gpsdata));

	if(!this->g.validFix())
		return;

	gpsdata.time_sec = this->g.utc_sec();
	gpsdata.time_usec = this->g.utc_usec();
	gpsdata.latitude = (int)rint(this->g.lat()*10000000.0);
	gpsdata.longitude = (int)rint(this->g.lng()*10000000.0);

	//cout << "Lat,Lng: " << gpsdata.latitude << "," << gpsdata.longitude << endl;

	this->Publish(this->gps_addr, PLAYER_MSGTYPE_DATA, PLAYER_GPS_DATA_STATE, (void*)&gpsdata, sizeof(gpsdata), NULL);
}


////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void GPSEM406ASIRFIIIDriver::Main() 
{
	// The main loop; interact with the device here
	for(;;)
	{

		// test if we are supposed to cancel
		pthread_testcancel();

		// Process incoming messages.  ::ProcessMessage() is
		// called on each message.
		ProcessMessages();

		if(this->g.recvData())
		{
			publishGPSData();
		}
		else
		{
			PLAYER_ERROR("Bad motor gps read");
		}
		
	}
}

////////////////////////////////////////////////////////////////////////////////
// Extra stuff for building a shared object.

/* need the extern to avoid C++ name-mangling  */
extern "C"
{
	int player_driver_init(DriverTable* table)
	{
		PLAYER_MSG0(0, "GPS EM406A SiRF III driver initializing...");
		GPSEM406ASIRFIIIDriver_Register(table);
		PLAYER_MSG0(0, "GPS EM406A SiRF III driver initialization done.");
		return(0);
	}
}

