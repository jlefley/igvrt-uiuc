
/*
 * A custom driver for the Roomba Red with Firefly bluetooth adapter
 *
 * Interfaces:
 *   - Position2D
 *       - Sends: 
 *       - Recvs: PLAYER_POSITION2D_CMD_VEL
 *                PLAYER_POSITION2D_REQ_GET_GEOM
 */

#include <string.h>
#include <libplayercore/playercore.h>
#include <math.h>
#include "IRSonarProtoDev.h"

////////////////////////////////////////////////////////////////////////////////
// The class for the driver
class IRSonarProtoDriver : public ThreadedDriver
{
public:
    
	// Constructor
	IRSonarProtoDriver(ConfigFile* cf, int section);

	// This method will be invoked on each incoming message
	virtual int ProcessMessage(QueuePointer &resp_queue, player_msghdr * hdr, void * data);

private:

	virtual void Main();
	virtual int MainSetup();
	virtual void MainQuit();

	//private data members
	IRSonarProtoDev irs; //low-level object
	const char* serial_port; // name of serial port of roomba
	player_devaddr_t ir_addr; // address of ir interface
	player_devaddr_t sonar_addr; // address of ir interface
	int ir; // ir data
	int sonar; // sonar data
};

// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver* IRSonarProtoDriver_Init(ConfigFile* cf, int section)
{
	// Create and return a new instance of this driver
	return((Driver*)(new IRSonarProtoDriver(cf, section)));
}

// A driver registration function, again declared outside of the class so
// that it can be invoked without object context.  In this function, we add
// the driver into the given driver table, indicating which interface the
// driver can support and how to create a driver instance.
void IRSonarProtoDriver_Register(DriverTable* table)
{
	table->AddDriver("irsonarproto", IRSonarProtoDriver_Init);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
IRSonarProtoDriver::IRSonarProtoDriver(ConfigFile* cf, int section)
	: ThreadedDriver(cf,section,true,PLAYER_MSGQUEUE_DEFAULT_MAXLEN)
{
	memset(&this->ir_addr,0,sizeof(player_devaddr_t));
	memset(&this->sonar_addr,0,sizeof(player_devaddr_t));

	// Do we create a ir interface?
	if(cf->ReadDeviceAddr(&(this->ir_addr), section, "provides", PLAYER_IR_CODE, -1, NULL) == 0)
	{
		if(this->AddInterface(this->ir_addr) != 0)
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


	// Read options from the configuration file
	this->serial_port = cf->ReadString(section, "port", "/dev/ttyS0");

	return;
}


////////////////////////////////////////////////////////////////////////////////
// Set up the device.  Return 0 if things go well, and -1 otherwise.
// Here you do whatever is necessary to setup the device, like open and
// configure a serial port.
int IRSonarProtoDriver::MainSetup()
{
	PLAYER_MSG0(0, "IRSonarProto driver starting up...");

	// connect to sensors
	string s(this->serial_port);
	if(!irs.connect(s, 9600))
		return -1;
	
	// clear data
	ir = 0;
	sonar = 0;
    
	PLAYER_MSG0(0, "IRSonarProto driver ready.");

	return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
// Here you would shut the device down by, for example, closing a
// serial port.
void IRSonarProtoDriver::MainQuit()
{
	PLAYER_MSG0(0, "IRSonarProto driver shutting down...");

	irs.closeConnection();

	PLAYER_MSG0(0, "IRSonarProto driver has been shutdown.");
}

// Process messages here.  Send a response if necessary, using Publish().
// If you handle the message successfully, return 0.  Otherwise,
// return -1, and a NACK will be sent for you, if a response is required.
int IRSonarProtoDriver::ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr, void * data)
{

	//////////////////////////////////////////////////
	// IR: PLAYER_IR_POSE
	//////////////////////////////////////////////////
	if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ, PLAYER_IR_REQ_POSE, this->ir_addr))
	{
		player_ir_pose_t ir_pose;
		player_pose3d_t ir_pose_arr[1];

		memset(&ir_pose, 0, sizeof ir_pose);
		memset(&ir_pose_arr[0], 0, sizeof ir_pose_arr[0]);
		
		
		ir_pose.poses_count = 1;
		ir_pose.poses = ir_pose_arr;

		this->Publish(this->ir_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_IR_REQ_POSE, (void*)&ir_pose, sizeof ir_pose, NULL);
		return 0;
	}
	//////////////////////////////////////////////////
	// Sonar: PLAYER_SONAR_REQ_GET_GEO
	//////////////////////////////////////////////////
	else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ, PLAYER_SONAR_REQ_GET_GEOM, this->sonar_addr))
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


////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void IRSonarProtoDriver::Main() 
{
	float irarr[1];
	float sonararr[1];

	// The main loop; interact with the device here
	for(;;)
	{
		// test if we are supposed to cancel
		pthread_testcancel();

		// Process incoming messages.  ::ProcessMessage() is
		// called on each message.
		ProcessMessages();

		// Get sensor data
		if(irs.getData(this->ir, this->sonar))
		{
			//std::cout << this->ir << " " << this->sonar << std::endl;

			////////////////////////////
			// Publish ir data
			player_ir_data_t irdata;
			memset(&irdata,0,sizeof(irdata));

			irarr[0] = ((float)this->ir);

			irdata.voltages_count = 1;
			irdata.voltages = irarr;
			irdata.ranges_count = 1;
			irdata.ranges = irarr;

			this->Publish(this->ir_addr, PLAYER_MSGTYPE_DATA, PLAYER_IR_DATA_RANGES, (void*)&irdata, sizeof(irdata), NULL);

			////////////////////////////
			// Publish sonar data
			player_sonar_data_t sonardata;
			memset(&sonardata,0,sizeof(sonardata));

			sonararr[0] = ((float)this->sonar);

			sonardata.ranges_count = 1;
			sonardata.ranges = sonararr;

			this->Publish(this->sonar_addr, PLAYER_MSGTYPE_DATA, PLAYER_SONAR_DATA_RANGES, (void*)&sonardata, sizeof(sonardata), NULL);
		}
		else
		{
			irs.flush();
			PLAYER_ERROR("IRSonarProto bad sensor read.");
		}
		

		// Sleep (you might, for example, block on a read() instead)
		//usleep(10000);
	}
}

////////////////////////////////////////////////////////////////////////////////
// Extra stuff for building a shared object.

/* need the extern to avoid C++ name-mangling  */
extern "C"
{
	int player_driver_init(DriverTable* table)
	{
		PLAYER_MSG0(0, "IRSonarProto driver initializing...");
		IRSonarProtoDriver_Register(table);
		PLAYER_MSG0(0, "IRSonarProto driver initialization done.");
		return(0);
	}
}

