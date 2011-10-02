
/*
 * A custom driver for the LSM303DLH and Sonar
 *
 * Interfaces:
 *   - Provides:
 *     - IMU
 *     - Sonar
 *   - Requires:
 *     - <none>
 */

#include <libplayercore/playercore.h>
#include "imulsm303dlhSerial.h"

//c++ libs
#include <iostream>




////////////////////////////////////////////////////////////////////////////////
// The class for the driver
class IMULSM303DLHDriver : public ThreadedDriver
{
public:
    
	// Constructor
	IMULSM303DLHDriver(ConfigFile* cf, int section);

	// This method will be invoked on each incoming message
	virtual int ProcessMessage(QueuePointer &resp_queue, player_msghdr * hdr, void * data);

private:

	virtual void Main();
	virtual int MainSetup();
	virtual void MainQuit();

	void publishIMUSonarData();

	const char* serial_port;

	//private data members
	player_devaddr_t imu_addr; // address of imu interface
	player_devaddr_t sonar_addr; //address of sonar interface
	imulsm303dlhSerial s; // low level serial interface to imu/sonar data
};

// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver* IMULSM303DLHDriver_Init(ConfigFile* cf, int section)
{
	// Create and return a new instance of this driver
	return((Driver*)(new IMULSM303DLHDriver(cf, section)));
}

// A driver registration function, again declared outside of the class so
// that it can be invoked without object context.  In this function, we add
// the driver into the given driver table, indicating which interface the
// driver can support and how to create a driver instance.
void IMULSM303DLHDriver_Register(DriverTable* table)
{
	table->AddDriver("imulsm303dlh", IMULSM303DLHDriver_Init);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
IMULSM303DLHDriver::IMULSM303DLHDriver(ConfigFile* cf, int section)
	: ThreadedDriver(cf,section,false,PLAYER_MSGQUEUE_DEFAULT_MAXLEN)
{
	memset(&this->imu_addr,0,sizeof(player_devaddr_t));

	// Do we create a imu interface?
	if(cf->ReadDeviceAddr(&(this->imu_addr), section, "provides", PLAYER_IMU_CODE, -1, NULL) == 0)
	{
		if(this->AddInterface(this->imu_addr) != 0)
		{
			this->SetError(-1);
			return;
		}
	}

	// Do we create a sonar interface?
	if(cf->ReadDeviceAddr(&(this->sonar_addr), section, "provides", PLAYER_SONAR_CODE, -1, NULL) == 0)
	{
		if(this->AddInterface(this->sonar_addr) != 0)
		{
			this->SetError(-1);
			return;
		}
	}

	this->serial_port = cf->ReadString(section, "port", "/dev/ttyS0");

	return;
}


////////////////////////////////////////////////////////////////////////////////
// Set up the device.  Return 0 if things go well, and -1 otherwise.
// Here you do whatever is necessary to setup the device, like open and
// configure a serial port.
int IMULSM303DLHDriver::MainSetup()
{
	PLAYER_MSG0(0, "IMU LSM303DLH/Sonar driver starting up...");

	if(!this->s.connect(this->serial_port, 9600))
		return -1;
	this->s.flush();
    
	PLAYER_MSG0(0, "IMU LSM303DLH/Sonar driver ready.");

	return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
// Here you would shut the device down by, for example, closing a
// serial port.
void IMULSM303DLHDriver::MainQuit()
{
	PLAYER_MSG0(0, "IMU LSM303DLH/Sonar driver shutting down...");

	this->s.closeConnection();	

	PLAYER_MSG0(0, "IMU LSM303DLH/Sonar driver has been shutdown.");
}

// Process messages here.  Send a response if necessary, using Publish().
// If you handle the message successfully, return 0.  Otherwise,
// return -1, and a NACK will be sent for you, if a response is required.
int IMULSM303DLHDriver::ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr, void * data)
{
	//////////////////////////////////////////////////
	// Sonar: PLAYER_SONAR_REQ_GET_GEO
	//////////////////////////////////////////////////
	if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ, PLAYER_SONAR_REQ_GET_GEOM, this->sonar_addr))
	{
		player_sonar_geom_t sonar_pose;
		player_pose3d_t sonar_pose_arr[1];

		memset(&sonar_pose, 0, sizeof sonar_pose);
		memset(&sonar_pose_arr[0], 0, sizeof sonar_pose_arr[0]);
		
		
		sonar_pose.poses_count = 1;
		sonar_pose.poses = sonar_pose_arr;

		this->Publish(this->sonar_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_SONAR_REQ_GET_GEOM, (void*)&sonar_pose, sizeof sonar_pose, NULL);
		return 0;
	}

	return 0;
}


void IMULSM303DLHDriver::publishIMUSonarData()
{
	float sonararr[1];

	////////////////////////////////
	// Publish IMU Data
	player_imu_data_state_t imudata;
	memset(&imudata,0,sizeof(imudata));

	imudata.pose.px = 0;
	imudata.pose.py = 0;
	imudata.pose.pz = 0;
	imudata.pose.proll = 0;
	imudata.pose.ppitch = 0;
	imudata.pose.pyaw = this->s.getHeading();

	cout << "Heading: " << this->s.getHeading() << " degrees" <<endl;

	this->Publish(this->imu_addr, PLAYER_MSGTYPE_DATA, PLAYER_IMU_DATA_STATE, (void*)&imudata, sizeof(imudata), NULL);

	///////////////////////////////
	// Publish Sonar Data
	player_sonar_data_t sonardata;
	memset(&sonardata,0,sizeof(sonardata));

	sonararr[0] = float(this->s.getDistance());

	sonardata.ranges_count = 1;
	sonardata.ranges = sonararr;

	cout << "Distance: " << this->s.getDistance() << " inches" << endl;

	this->Publish(this->sonar_addr, PLAYER_MSGTYPE_DATA, PLAYER_SONAR_DATA_RANGES, (void*)&sonardata, sizeof(sonardata), NULL);

	usleep(100000);
}


////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void IMULSM303DLHDriver::Main() 
{
	// The main loop; interact with the device here
	for(;;)
	{

		// test if we are supposed to cancel
		pthread_testcancel();

		// Process incoming messages.  ::ProcessMessage() is
		// called on each message.
		ProcessMessages();

		this->s.readMsg();

		publishIMUSonarData();
		
	}
}

////////////////////////////////////////////////////////////////////////////////
// Extra stuff for building a shared object.

/* need the extern to avoid C++ name-mangling  */
extern "C"
{
	int player_driver_init(DriverTable* table)
	{
		PLAYER_MSG0(0, "IMU LSM303DLH driver initializing...");
		IMULSM303DLHDriver_Register(table);
		PLAYER_MSG0(0, "IMU LSM303DLH driver initialization done.");
		return(0);
	}
}

