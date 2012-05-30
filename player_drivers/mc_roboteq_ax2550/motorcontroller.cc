
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
#include <sys/time.h>
#include "MotorControllerSerial.h"

#define PI 3.14

////////////////////////////////////////////////////////////////////////////////
// The class for the driver
class MotorControllerDriver : public ThreadedDriver
{
public:
    
	// Constructor
	MotorControllerDriver(ConfigFile* cf, int section);

	// This method will be invoked on each incoming message
	virtual int ProcessMessage(QueuePointer &resp_queue, player_msghdr * hdr, void * data);

private:

	virtual void Main();
	virtual int MainSetup();
	virtual void MainQuit();
	int SetSpeeds(double tv, double rv); // Set tranlational velocity and rotational velocity
	void UpdateOdometry();
	double Norm_Ang(double ang);

	//private data members
	//motorcontroller interface
	MotorControllerSerial m; // low level serial object
	const char* serial_port; // name of serial port of Motor Controller
	player_devaddr_t position_addr; // address of position2d interface
	player_devaddr_t power_addr; // address of power interface
	player_devaddr_t dio_addr; // address of dio (digital I/O) interface
	double ox, oy, oa; // Integrated odometric position [m m rad]
	timeval start, stop, result; // Time during each odometric position updating
	bool init_time; // A flag of whether the timer is initialized
	bool flag1; //Flag for estop handling
};

// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver* MotorControllerDriver_Init(ConfigFile* cf, int section)
{
	// Create and return a new instance of this driver
	return((Driver*)(new MotorControllerDriver(cf, section)));
}

// A driver registration function, again declared outside of the class so
// that it can be invoked without object context.  In this function, we add
// the driver into the given driver table, indicating which interface the
// driver can support and how to create a driver instance.
void MotorControllerDriver_Register(DriverTable* table)
{
	table->AddDriver("motorcontroller", MotorControllerDriver_Init);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
MotorControllerDriver::MotorControllerDriver(ConfigFile* cf, int section)
	: ThreadedDriver(cf,section,true,PLAYER_MSGQUEUE_DEFAULT_MAXLEN)
{
	memset(&this->position_addr,0,sizeof(player_devaddr_t));

	// Do we create a position interface?
	if(cf->ReadDeviceAddr(&(this->position_addr), section, "provides", PLAYER_POSITION2D_CODE, -1, NULL) == 0)
	{
		if(this->AddInterface(this->position_addr) != 0)
		{
			PLAYER_ERROR("Error adding position2d interface\n");
			this->SetError(-1);
			return;
		}
	}
	
	
	memset(&this->power_addr,0,sizeof(player_devaddr_t));

	// Do we create a power interface?
	if(cf->ReadDeviceAddr(&(this->power_addr), section, "provides", PLAYER_POWER_CODE, -1, NULL) == 0)
	{
		if(this->AddInterface(this->power_addr) != 0)
		{
			PLAYER_ERROR("Error adding power interface\n");			
			this->SetError(-1);
			return;
		}
	}

	memset(&this->dio_addr,0,sizeof(player_devaddr_t));

	// Do we create a dio (digital I/O) interface?
	if(cf->ReadDeviceAddr(&(this->dio_addr), section, "provides", PLAYER_DIO_CODE, -1, NULL) == 0)
	{
		if(this->AddInterface(this->dio_addr) != 0)
		{
			PLAYER_ERROR("Error adding dio interface\n");
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
int MotorControllerDriver::MainSetup()
{
	PLAYER_MSG0(0, "Motor Controller driver starting up...");

	flag1 = 0;	
	
	// connect to motor controller
	this->m.connect(this->serial_port, 9600);
	this->m.flush();

	// clear data
	ox = oy = oa = 0;
    	
	// set flag of timer
	init_time = false; // Timer has not yet been initilized

	// tell the controller to enter serial mode
	/*	
	bool serialModeSuccess = false;
	for(int i=0; i < 10; i++)
	{
		if(!m.enterSerialMode())
		{
			PLAYER_ERROR("Did not enter serial mode.  Trying again.");
		}
		else
		{
			serialModeSuccess = true;
			break;
		}
	}

	if(!serialModeSuccess)
	{
		PLAYER_ERROR("CANNOT ENTER SERIAL MODE!!!!");
		return 1;
	}
	*/
	// setup ok
	PLAYER_MSG0(0, "Motor Controller driver ready.");

	return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
// Here you would shut the device down by, for example, closing a
// serial port.
void MotorControllerDriver::MainQuit()
{
	PLAYER_MSG0(0, "Motor Controller driver shutting down...");

	// close connection
	this->m.closeConnection();

	PLAYER_MSG0(0, "Motor Controller driver has been shutdown.");
}

// Process messages here.  Send a response if necessary, using Publish().
// If you handle the message successfully, return 0.  Otherwise,
// return -1, and a NACK will be sent for you, if a response is required.
int MotorControllerDriver::ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr, void * data)
{
	//////////////////////////////////////////////////
	// Position2D: CMD_VEL
	//////////////////////////////////////////////////
	if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD, PLAYER_POSITION2D_CMD_VEL, this->position_addr))
	{    
		if(data == NULL)
		{
			PLAYER_ERROR("Motor Controller: Position2D CMD_VEL msg has no data.");
			return -1;
		}

		// cast data to correct type
		player_position2d_cmd_vel_t* d = (player_position2d_cmd_vel_t*)data;

		// set speeds
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
		pos_geom.size.sl = 0.0;
		pos_geom.size.sw = 0.0;

		this->Publish(this->position_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_GET_GEOM, (void*)&pos_geom, sizeof pos_geom, NULL);
		return 0;
	}
	
	return 0;
}

int MotorControllerDriver::SetSpeeds(double tv, double rv)
{
	//int16_t tv_m, rad_m;
	int16_t v_right, v_left;
	

	v_right = (int16_t)rint(tv + (MotorControllerSerial::AXEL_LENGTH_M * rv / 2.0));
	v_left = (int16_t)rint(tv - (MotorControllerSerial::AXEL_LENGTH_M * rv / 2.0));

	cout << "tv: " << tv << endl;
	cout << "rv: " << rv << endl;
	cout << "v_right: " << v_right << endl;
	cout << "v_left: " << v_left << endl;
	
	//tv_mm = tv;
	//rad_mm = tv/rv;
	
	//TEMP FOR EOH POWER WHEELS DEMO
	/*
	v_right = (int16_t)rint(200*(tv + (MotorControllerSerial::AXEL_LENGTH_M * rv / 2.0)));
	v_left = (int16_t)rint(200*(tv - (MotorControllerSerial::AXEL_LENGTH_M * rv / 2.0)));

	cout << v_right << " " << v_left << endl;
	*/
	//END TEMP
	

	//TEMP FOR MC TEST CLIENT
	
	//v_right = (int16_t)rint(tv);
	//v_left = (int16_t)rint(rv);

	//END TEMP

	if(m.sendMotorCmd(v_left, v_right))
		return 0;

	return -1;
}

void MotorControllerDriver::UpdateOdometry()
{
	double vr, vl, R, oa_new, t;

  // Should we use average velocity: (v+v_old)/2
	vl = m.left(); // return value is int
	vr = m.right();

	// Get time
	gettimeofday(&stop, NULL); // get stop time
	timersub(&start, &stop, &result); // get result: time difference between start and stop time
	gettimeofday(&start, NULL); // get start time for next update
	t = result.tv_sec + result.tv_usec/1000000.0;
	
	oa_new = Norm_Ang(t * (vr-vl)/MotorControllerSerial::AXEL_LENGTH_M + oa);

	if(vr != vl)
	{
		R = 0.5 * MotorControllerSerial::AXEL_LENGTH_M * (vr+vl)/(vr-vl);
		ox += R*(sin(oa_new) - sin(oa));
		oy -= R*(cos(oa_new) - cos(oa));
	}
	else
	{
		// vr = vl, use L'Hospital's rule to approach ox and oy.
		R = vr*t;
		ox += R*cos(oa_new);
		oy += R*sin(oa_new);
	}

	oa = oa_new;
}

double MotorControllerDriver::Norm_Ang(double ang)
{
	int m = 0;
	
	// Normalize angle to [0 2*PI]
	if(ang >= (2*PI) || ang < 0)	
		m = floor(0.5*ang/PI);
	
	return ang - 2*PI*m;
}

////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void MotorControllerDriver::Main() 
{
	// The main loop; interact with the device here
	for(;;)
	{
		// test if we are supposed to cancel
		pthread_testcancel();

		// Process incoming messages.  ::ProcessMessage() is
		// called on each message.
		ProcessMessages();

		//Get sensor data
		if(m.recvSensorData())
		{
			//std::cout << "Left motor velocity: " << m.left() << endl;
			//std::cout << "Right motor velocity: " << m.right() << endl;
			//std::cout << "Main Batt Voltage: " << m.mainVoltage() << endl;
			//std::cout << "Internal Voltage: " << m.internalVoltage() << endl;
			//std::cout << "Estop: " << m.estopState() << endl;
			
			if(init_time == false)
      			{
      				gettimeofday(&start, NULL); // get start time
      				init_time = true;
      			}
      			else
				UpdateOdometry();

			////////////////////////////
			// Publish position2d data
			player_position2d_data_t posdata;
			memset(&posdata,0,sizeof(posdata));

			posdata.pos.px = ox;
			posdata.pos.py = oy;
			posdata.pos.pa = oa;
			posdata.stall = 0;

			this->Publish(this->position_addr, PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE, (void*)&posdata, sizeof(posdata), NULL);

			////////////////////////////
			// Publish power data
			player_power_data_t powerdata;
			memset(&powerdata,0,sizeof(powerdata));
			
			powerdata.valid = 1;
			powerdata.volts = m.mainVoltage();
			
			this->Publish(this->power_addr, PLAYER_MSGTYPE_DATA, PLAYER_POWER_DATA_STATE, (void*)&powerdata, sizeof(powerdata), NULL);

			////////////////////////////
			// Publish estop data
			player_dio_data_t estopdata;
			memset(&estopdata,0,sizeof(estopdata));
			
			estopdata.count = 1;
			estopdata.bits = m.estopState();
			
			this->Publish(this->dio_addr, PLAYER_MSGTYPE_DATA, PLAYER_DIO_DATA_VALUES, (void*)&estopdata, sizeof(estopdata), NULL);

		}
		else
		{
			//m.flush();
			//m.enterSerialMode();
			PLAYER_ERROR("Bad motor controller sensor read");
		}			

			//handel estop condition
			while (m.estopState() == 1)
			{
				m.recvSensorData();
				flag1=1;				
			}

			if (flag1 == 1)
			{
				flag1=0;
				m.restart();
				MainSetup();
				sleep(1);
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
		PLAYER_MSG0(0, "Motor Controller driver initializing...");
		MotorControllerDriver_Register(table);
		PLAYER_MSG0(0, "Motor Controller driver initialization done.");
		return(0);
	}
}

