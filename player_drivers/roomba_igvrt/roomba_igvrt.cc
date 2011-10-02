
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
#define _USE_MATH_DEFINES
#include <math.h>
#include "Roomba.h"
#include <sys/time.h>
#include "../driverlib/Kalman.h"
#include "../driverlib/DiffSteering.h"

////////////////////////////////////////////////////////////////////////////////
// The class for the driver
class RoombaIGVRTDriver : public ThreadedDriver
{
public:
    
	// Constructor
	RoombaIGVRTDriver(ConfigFile* cf, int section);

	// This method will be invoked on each incoming message
	virtual int ProcessMessage(QueuePointer &resp_queue, player_msghdr * hdr, void * data);

private:

	virtual void Main();
	virtual int MainSetup();
	virtual void MainQuit();
	int SetSpeeds(double tv, double rv); // set translational and rotational velocity
	void UpdateOdometry();

	//private data members
	Roomba r; //low-level roomba object
	const char* serial_port; // name of serial port of roomba
	player_devaddr_t position_addr; // address of position2d interface
	player_devaddr_t bumper_addr;
	RespSensor0 sensorData; // data from roomba
	player_bumper_geom_t bump_geom;
	double vr_target, vl_target; // target right and left wheel velocities [m/s m/s]
	double vr_meas, vl_meas; // measured right and left wheel velocities [m/s m/s]
	timeval lastOdomTime; // last time of day odomotry was updated
	DiffSteering * ds; // object for computing differential steering odometry
};

// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver* RoombaIGVRTDriver_Init(ConfigFile* cf, int section)
{
	// Create and return a new instance of this driver
	return((Driver*)(new RoombaIGVRTDriver(cf, section)));
}

// A driver registration function, again declared outside of the class so
// that it can be invoked without object context.  In this function, we add
// the driver into the given driver table, indicating which interface the
// driver can support and how to create a driver instance.
void RoombaIGVRTDriver_Register(DriverTable* table)
{
	table->AddDriver("roomba_igvrt", RoombaIGVRTDriver_Init);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
RoombaIGVRTDriver::RoombaIGVRTDriver(ConfigFile* cf, int section)
	: ThreadedDriver(cf,section,true,PLAYER_MSGQUEUE_DEFAULT_MAXLEN)
{
	memset(&this->position_addr,0,sizeof(player_devaddr_t));
	memset(&this->bumper_addr,0,sizeof(player_devaddr_t));

	// Do we create a position interface?
	if(cf->ReadDeviceAddr(&(this->position_addr), section, "provides", PLAYER_POSITION2D_CODE, -1, NULL) == 0)
	{
		if(this->AddInterface(this->position_addr) != 0)
		{
			this->SetError(-1);
			return;
		}
	}

	// Do we create a bumper interface?
	if(cf->ReadDeviceAddr(&(this->bumper_addr), section, "provides", PLAYER_BUMPER_CODE, -1, NULL) == 0)
	{
		if(this->AddInterface(this->bumper_addr) != 0)
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
int RoombaIGVRTDriver::MainSetup()
{
	PLAYER_MSG0(0, "Roomba_IGVRT driver starting up...");

	// connect to roomba and turn it on
	string s(this->serial_port);
	if(!r.connect(s, 9600))
		return -1;

	if(!r.wakeUp())
		return -1;

	if(!r.changeMode(Roomba::FULL))
		return -1;
	
	// set up odometry
	vr_target = vl_target = vr_meas = vl_meas = 0;

	gettimeofday(&lastOdomTime, NULL);

	ds = new DiffSteering(
		0,				// init x
		0,				// init y
		0,				// init a
		0,				// init vr
		0,				// init vl
		0.0000005,			// proc var
		0.5,				// meas var
		Roomba::AXLE_LENGTH_MM*(1e-3)	// axel len
	);
	

	PLAYER_MSG0(0, "Roomba_IGVRT driver ready.");

	return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
// Here you would shut the device down by, for example, closing a
// serial port.
void RoombaIGVRTDriver::MainQuit()
{
	PLAYER_MSG0(0, "Roomba_IGVRT driver shutting down...");

	r.changeMode(Roomba::OFF);
	r.closeConnection();

	delete ds;

	PLAYER_MSG0(0, "Roomba_IGVRT driver has been shutdown.");
}

// Process messages here.  Send a response if necessary, using Publish().
// If you handle the message successfully, return 0.  Otherwise,
// return -1, and a NACK will be sent for you, if a response is required.
int RoombaIGVRTDriver::ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr, void * data)
{
	//////////////////////////////////////////////////
	// Position2D: CMD_VEL
	//////////////////////////////////////////////////
	if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_VEL, this->position_addr))
	{    
		if(data == NULL)
		{
			PLAYER_ERROR("Roomba_IGVRT: Position2D CMD_VEL msg has no data.");
			return -1;
		}

		// cast data to correct type
		player_position2d_cmd_vel_t* d = (player_position2d_cmd_vel_t*)data;

		if(SetSpeeds(d->vel.px, d->vel.pa) == -1)
		{
			PLAYER_ERROR("Roomba_IGVRT: SetSpeeds failed.");
			return -1;
		}
	}
	//////////////////////////////////////////////////
	// Position2D: REQ_GET_GEOM
	//////////////////////////////////////////////////
	else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_GET_GEOM, this->position_addr))
	{
		player_position2d_geom_t pos_geom;

		// Return the robot geometry.
		memset(&pos_geom, 0, sizeof pos_geom);
		// Assume that it turns about its geometric center, so zeros are fine
		pos_geom.size.sl = ((double)Roomba::DIAMETER_MM)/1000.0;
		pos_geom.size.sw = ((double)Roomba::DIAMETER_MM)/1000.0;

		this->Publish(this->position_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_GET_GEOM, (void*)&pos_geom, sizeof pos_geom, NULL);
		return 0;
	}
	//////////////////////////////////////////////////
	// Bumper: REQ_GET_GEOM
	//////////////////////////////////////////////////
	if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ, PLAYER_BUMPER_REQ_GET_GEOM, this->bumper_addr))
	{
		memset(&bump_geom, 0, sizeof bump_geom);

		bump_geom.bumper_def_count = 2;
		bump_geom.bumper_def = new player_bumper_define_t[bump_geom.bumper_def_count];
		if (!(bump_geom.bumper_def))
		{
			PLAYER_ERROR("Out of memory");
			return -1;
		}

		bump_geom.bumper_def[0].pose.px = 0.12;
		bump_geom.bumper_def[0].pose.py = 0.12;
		bump_geom.bumper_def[0].pose.pyaw = 45.0;
		bump_geom.bumper_def[0].length = 0.33;
		bump_geom.bumper_def[0].radius = 0.33 / 2.0;

		bump_geom.bumper_def[1].pose.px = 0.12;
		bump_geom.bumper_def[1].pose.py = -0.12;
		bump_geom.bumper_def[1].pose.pyaw = -45.0;
		bump_geom.bumper_def[1].length = 0.33;
		bump_geom.bumper_def[1].radius = 0.33 / 2.0;

		this->Publish(this->bumper_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_BUMPER_REQ_GET_GEOM, (void*)&bump_geom);
		delete []bump_geom.bumper_def;
		bump_geom.bumper_def = NULL;

		return 0;
  }

	return 0;
}


int RoombaIGVRTDriver::SetSpeeds(double tv, double rv)
{
	int16_t tv_mm;
	//int16_t rad_mm;

	tv_mm = (int16_t)rint(tv * 1e3);
	tv_mm = MAX(tv_mm, (-1)*Roomba::TVEL_MAX_MM_S);
	tv_mm = MIN(tv_mm, Roomba::TVEL_MAX_MM_S);

	if(rv == 0)
	{
		// Special cases: drive straight
		vr_target = vl_target = (tv_mm * 1e-3);

		if(r.issueNoRespCmd(new CmdDriveStraight(tv_mm)))
			return 0;
	}
	else if(tv == 0)
	{
		// Special cases: turn in place
		tv_mm = (int16_t)rint(Roomba::AXLE_LENGTH_MM * rv * 0.5);
		tv_mm = MAX(tv_mm, (-1)*Roomba::TVEL_MAX_MM_S);
		tv_mm = MIN(tv_mm, Roomba::TVEL_MAX_MM_S);
		
		vr_target = (tv_mm * 1e-3);
		vl_target = (-1)*vr_target;

		if(r.issueNoRespCmd(new CmdTurnInPlace(tv_mm)))
			return 0;
	}
	else
	{
		// do nothing
		return 0;
	}
	/*
	else
	{
		// General case: convert rv to turn radius
		rad_mm = (int16_t)rint(tv_mm / rv);

		rad_mm = MAX(rad_mm, (-1)*Roomba::RADIUS_MAX_MM);
		rad_mm = MIN(rad_mm, Roomba::RADIUS_MAX_MM);

		rv = ((double)tv_mm)/((double)rad_mm);

		vr_target = (tv_mm + (Roomba::AXLE_LENGTH_MM * rv / 2.0))*(1e-3);
		vl_target = (tv_mm - (Roomba::AXLE_LENGTH_MM * rv / 2.0))*(1e-3);

		if(r.issueNoRespCmd(new CmdDrive(tv_mm, rad_mm)))
			return 0;
	}
	*/

	return -1;
}

void RoombaIGVRTDriver::UpdateOdometry()
{
	// update time of day
	timeval curtime, timediff;

	gettimeofday(&curtime, NULL);
	timersub(&curtime, &lastOdomTime, &timediff);
	double dt = ((double)timediff.tv_sec) + (((double)timediff.tv_usec)/1000000.0);
	lastOdomTime = curtime;
	
	// update wheel velocities
	double Sr, Sl;
	
	Sr = sensorData.rightDistance() * 1e-3;
	Sl = sensorData.leftDistance() * 1e-3;

	vr_meas = Sr / dt;
	vl_meas = Sl / dt;

	// update diff steer object
	ds->update(dt, vr_target, vl_target, vr_meas, vl_meas);

	/*
	// update position
	double D, A, oa_new, b, r;

	D = sensorData.distance() * 1e-3;
	A = sensorData.angle() * 1e-3;
	b = Roomba::AXLE_LENGTH_MM * 1e-3;

	oa_new = Norm_Ang(((2*A)/b) + oa);

	if(A != 0)
	{
		r = ((b/2)*(D/A));

		ox += r*(sin(oa_new) - sin(oa));
		oy -= r*(cos(oa_new) - cos(oa));
	}
	else
	{
		ox += D*cos(oa_new);
		oy += D*sin(oa_new);
	}

	oa = oa_new;
	*/
}


////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void RoombaIGVRTDriver::Main() 
{
	// The main loop; interact with the device here
	for(;;)
	{
		// test if we are supposed to cancel
		pthread_testcancel();

		// Process incoming messages.  ::ProcessMessage() is
		// called on each message.
		ProcessMessages();

		// Get sensor data
		if(r.issueCmdWithResp(new CmdSensors(0), sensorData))
		{
			UpdateOdometry();

			cout << "Target (vr,vl): (" << vr_target << "," << vl_target << ")" << " | Meas (vr,vl): (" << vr_meas << "," << vl_meas << ")\n\n";

			////////////////////////////
			// Publish position2d data
			player_position2d_data_t posdata;
			memset(&posdata,0,sizeof(posdata));

			posdata.pos.px = ds->x_nofilt();
			posdata.pos.py = ds->y_nofilt();
			posdata.pos.pa = NORMALIZE(ds->a_nofilt());
			posdata.stall = 0;

			this->Publish(this->position_addr, PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE, (void*)&posdata, sizeof(posdata), NULL);

			////////////////////////////
			// Update bumper data
			player_bumper_data_t bumperdata;
			memset(&bumperdata,0,sizeof(bumperdata));

			bumperdata.bumpers_count = 2;
			if ((bumperdata.bumpers = new uint8_t[bumperdata.bumpers_count]) == NULL)
			{
				PLAYER_ERROR ("Failed to allocate memory for bumper data in roomba driver.");
			}
			else
			{
				bumperdata.bumpers[0] = sensorData.bumpLeft();
				bumperdata.bumpers[1] = sensorData.bumpRight();

				this->Publish(this->bumper_addr, PLAYER_MSGTYPE_DATA, PLAYER_BUMPER_DATA_STATE, (void*)&bumperdata);
				delete [] bumperdata.bumpers;
			}


		}
		else
		{
			PLAYER_ERROR("Roomba bad sensor read.");
		}
		

		// Sleep (you might, for example, block on a read() instead)
		usleep(10000);
	}
}

////////////////////////////////////////////////////////////////////////////////
// Extra stuff for building a shared object.

/* need the extern to avoid C++ name-mangling  */
extern "C"
{
	int player_driver_init(DriverTable* table)
	{
		PLAYER_MSG0(0, "Roomba_IGVRT driver initializing...");
		RoombaIGVRTDriver_Register(table);
		PLAYER_MSG0(0, "Roomba_IGVRT driver initialization done.");
		return(0);
	}
}

