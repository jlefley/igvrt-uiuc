
/*
 * A custom driver for the FieldForce TCM XB IMU
 *
 * Interfaces:
 *   - Provides:
 *     - IMU
 *   - Requires:
 *     - <none>
 */

#include <libplayercore/playercore.h>

//c++ libs
#include <iostream>




////////////////////////////////////////////////////////////////////////////////
// The class for the driver
class IMUFieldForceTCMXBDriver : public ThreadedDriver
{
public:
    
	// Constructor
	IMUFieldForceTCMXBDriver(ConfigFile* cf, int section);

	// This method will be invoked on each incoming message
	virtual int ProcessMessage(QueuePointer &resp_queue, player_msghdr * hdr, void * data);

private:

	virtual void Main();
	virtual int MainSetup();
	virtual void MainQuit();

	void publishIMUData();

	//private data members
	player_devaddr_t imu_addr; // address of imu interface
};

// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver* IMUFieldForceTCMXBDriver_Init(ConfigFile* cf, int section)
{
	// Create and return a new instance of this driver
	return((Driver*)(new IMUFieldForceTCMXBDriver(cf, section)));
}

// A driver registration function, again declared outside of the class so
// that it can be invoked without object context.  In this function, we add
// the driver into the given driver table, indicating which interface the
// driver can support and how to create a driver instance.
void IMUFieldForceTCMXBDriver_Register(DriverTable* table)
{
	table->AddDriver("imufieldforcetcmxb", IMUFieldForceTCMXBDriver_Init);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
IMUFieldForceTCMXBDriver::IMUFieldForceTCMXBDriver(ConfigFile* cf, int section)
	: ThreadedDriver(cf,section,false,PLAYER_MSGQUEUE_DEFAULT_MAXLEN)
{
	memset(&this->imu_addr,0,sizeof(player_devaddr_t));

	// Do we create a gps interface?
	if(cf->ReadDeviceAddr(&(this->imu_addr), section, "provides", PLAYER_IMU_CODE, -1, NULL) == 0)
	{
		if(this->AddInterface(this->imu_addr) != 0)
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
int IMUFieldForceTCMXBDriver::MainSetup()
{
	PLAYER_MSG0(0, "IMU FieldForce TCM XB driver starting up...");


    
	PLAYER_MSG0(0, "IMU FieldForce TCM XB driver ready.");

	return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
// Here you would shut the device down by, for example, closing a
// serial port.
void IMUFieldForceTCMXBDriver::MainQuit()
{
	PLAYER_MSG0(0, "IMU FieldForce TCM XB driver shutting down...");

	

	PLAYER_MSG0(0, "IMU FieldForce TCM XB driver has been shutdown.");
}

// Process messages here.  Send a response if necessary, using Publish().
// If you handle the message successfully, return 0.  Otherwise,
// return -1, and a NACK will be sent for you, if a response is required.
int IMUFieldForceTCMXBDriver::ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr, void * data)
{

	return 0;
}


void IMUFieldForceTCMXBDriver::publishIMUData()
{
	player_imu_data_state_t imudata;
	memset(&imudata,0,sizeof(imudata));

	imudata.pose.px = 0;
	imudata.pose.py = 0;
	imudata.pose.pz = 0;
	imudata.pose.proll = 0;
	imudata.pose.ppitch = 0;
	imudata.pose.pyaw = 0;

	this->Publish(this->imu_addr, PLAYER_MSGTYPE_DATA, PLAYER_IMU_DATA_STATE, (void*)&imudata, sizeof(imudata), NULL);

	usleep(100000);
}


////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void IMUFieldForceTCMXBDriver::Main() 
{
	// The main loop; interact with the device here
	for(;;)
	{

		// test if we are supposed to cancel
		pthread_testcancel();

		// Process incoming messages.  ::ProcessMessage() is
		// called on each message.
		ProcessMessages();

		publishIMUData();
		
	}
}

////////////////////////////////////////////////////////////////////////////////
// Extra stuff for building a shared object.

/* need the extern to avoid C++ name-mangling  */
extern "C"
{
	int player_driver_init(DriverTable* table)
	{
		PLAYER_MSG0(0, "IMU FieldForce TCM XB driver initializing...");
		IMUFieldForceTCMXBDriver_Register(table);
		PLAYER_MSG0(0, "IMU FieldForce TCM XB driver initialization done.");
		return(0);
	}
}

