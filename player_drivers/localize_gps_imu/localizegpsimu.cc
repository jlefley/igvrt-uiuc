
/*
 * A custom driver for the Localize interface
 *
 * Interfaces:
 *   - Provides:
 *     - Localize
 *   - Requires:
 *     - GPS
 *     - IMU
 */

#include <libplayercore/playercore.h>

//c++ libs
#include <iostream>




////////////////////////////////////////////////////////////////////////////////
// The class for the driver
class LocalizeGPSIMUDriver : public ThreadedDriver
{
public:
    
	// Constructor
	LocalizeGPSIMUDriver(ConfigFile* cf, int section);

	// This method will be invoked on each incoming message
	virtual int ProcessMessage(QueuePointer &resp_queue, player_msghdr * hdr, void * data);

private:

	virtual void Main();
	virtual int MainSetup();
	virtual void MainQuit();

	void publishLocalizeData();

	//private data members
	player_devaddr_t localize_addr; // address of localize interface
};

// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver* LocalizeGPSIMUDriver_Init(ConfigFile* cf, int section)
{
	// Create and return a new instance of this driver
	return((Driver*)(new LocalizeGPSIMUDriver(cf, section)));
}

// A driver registration function, again declared outside of the class so
// that it can be invoked without object context.  In this function, we add
// the driver into the given driver table, indicating which interface the
// driver can support and how to create a driver instance.
void LocalizeGPSIMUDriver_Register(DriverTable* table)
{
	table->AddDriver("localizegpsimu", LocalizeGPSIMUDriver_Init);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
LocalizeGPSIMUDriver::LocalizeGPSIMUDriver(ConfigFile* cf, int section)
	: ThreadedDriver(cf,section,false,PLAYER_MSGQUEUE_DEFAULT_MAXLEN)
{
	memset(&this->localize_addr,0,sizeof(player_devaddr_t));

	// Do we create a localize interface?
	if(cf->ReadDeviceAddr(&(this->localize_addr), section, "provides", PLAYER_LOCALIZE_CODE, -1, NULL) == 0)
	{
		if(this->AddInterface(this->localize_addr) != 0)
		{
			this->SetError(-1);
			return;
		}
	}

	return;
}


////////////////////////////////////////////////////////////////////////////////
// Set up the device.  Return 0 if things go well, and -1 otherwise.
// Here you do whatever is necessary to setup the device, like open and
// configure a serial port.
int LocalizeGPSIMUDriver::MainSetup()
{
	PLAYER_MSG0(0, "Localize GPS IMU driver starting up...");


    
	PLAYER_MSG0(0, "Localize GPS IMU driver ready.");

	return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
// Here you would shut the device down by, for example, closing a
// serial port.
void LocalizeGPSIMUDriver::MainQuit()
{
	PLAYER_MSG0(0, "Localize GPS IMU driver shutting down...");

	

	PLAYER_MSG0(0, "Localize GPS IMU driver has been shutdown.");
}

// Process messages here.  Send a response if necessary, using Publish().
// If you handle the message successfully, return 0.  Otherwise,
// return -1, and a NACK will be sent for you, if a response is required.
int LocalizeGPSIMUDriver::ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr, void * data)
{

	return 0;
}


void LocalizeGPSIMUDriver::publishLocalizeData()
{
	player_localize_data_t localizedata;
	memset(&localizedata,0,sizeof(localizedata));

	player_localize_hypoth_t hypoths[1];

	hypoths[0].mean.px = 0;
	hypoths[0].mean.py = 0;
	hypoths[0].mean.pa = 0;
	hypoths[0].cov[0] = 0;
	hypoths[0].cov[1] = 0;
	hypoths[0].cov[2] = 0;
	hypoths[0].alpha = 0;

	localizedata.pending_count = 0;
	localizedata.pending_time = 0;
	localizedata.hypoths_count = 1;
	localizedata.hypoths = hypoths;

	this->Publish(this->localize_addr, PLAYER_MSGTYPE_DATA, PLAYER_LOCALIZE_DATA_HYPOTHS, (void*)&localizedata, sizeof(localizedata), NULL);
	
	usleep(100000);
}


////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void LocalizeGPSIMUDriver::Main() 
{
	// The main loop; interact with the device here
	for(;;)
	{

		// test if we are supposed to cancel
		pthread_testcancel();

		// Process incoming messages.  ::ProcessMessage() is
		// called on each message.
		ProcessMessages();

		publishLocalizeData();
		
	}
}

////////////////////////////////////////////////////////////////////////////////
// Extra stuff for building a shared object.

/* need the extern to avoid C++ name-mangling  */
extern "C"
{
	int player_driver_init(DriverTable* table)
	{
		PLAYER_MSG0(0, "Localize GPS IMU driver initializing...");
		LocalizeGPSIMUDriver_Register(table);
		PLAYER_MSG0(0, "Localize GPS IMU driver initialization done.");
		return(0);
	}
}

