
/*
 * A custom driver that takes an image from a camera and produces an occupancy map
 *
 * Interfaces:
 *   - Provides:
 *     - Map
 * Startup
 *   - Bind all cameras, set capture sizes
 *   - Allocate Mat buffers for images
 * Main loop
 *   - capture stereo pair
 *   - smooth image
 *   - calculate lines->3Dto2D, store in line buffer
 *   - calculate obstacles->stereo->3Dto2D, store in obstacle buffer
 *
 */

#define MAP_WIDTH 1000
#define MAP_HEIGHT 1000
#define MAP_SCALE 1.0 // 1 pixel = 1 cm

#define CAP_WIDTH 480
#define CAP_HEIGHT 320

#include <libplayercore/playercore.h>

//c++ libs
#include <iostream>
#include <deque>


//c libs
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>

// group libs
#include "../ai_lib/stereo.h"
#include "../ai_lib/lines.h"
#include "../ai_lib/obstacles.h"


using namespace std;


////////////////////////////////////////////////////////////////////////////////
// The class for the driver
class OpenCVMapDriver : public ThreadedDriver
{
public:
    
	// Constructor
	OpenCVMapDriver(ConfigFile* cf, int section);

	// This method will be invoked on each incoming message
	virtual int ProcessMessage(QueuePointer &resp_queue, player_msghdr * hdr, void * data);

private:

	virtual void Main();
	virtual int MainSetup();
	virtual void MainQuit();

	//private data members
	player_devaddr_t map_addr; // address of map interface
	VideoCapture left_cap;
	VideoCapture right_cap;
	Mat obstacles_world;
	Mat lines_world;

	class InstanceInfo {
	public:
		time_t capture_time;
		Mat left_eye, right_eye;
		// TODO: add localized position
	};
};

// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver* OpenCVMapDriver_Init(ConfigFile* cf, int section)
{
	// Create and return a new instance of this driver
	return((Driver*)(new OpenCVMapDriver(cf, section)));
}

// A driver registration function, again declared outside of the class so
// that it can be invoked without object context.  In this function, we add
// the driver into the given driver table, indicating which interface the
// driver can support and how to create a driver instance.
void OpenCVMapDriver_Register(DriverTable* table)
{
	table->AddDriver("opencvmap", OpenCVMapDriver_Init);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
OpenCVMapDriver::OpenCVMapDriver(ConfigFile* cf, int section)
	: ThreadedDriver(cf,section,false,PLAYER_MSGQUEUE_DEFAULT_MAXLEN)
{
	memset(&this->map_addr,0,sizeof(player_devaddr_t));

	// Do we create a map interface?
	if(cf->ReadDeviceAddr(&(this->map_addr), section, "provides", PLAYER_MAP_CODE, -1, NULL) == 0)
	{
		if(this->AddInterface(this->map_addr) != 0)
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
int OpenCVMapDriver::MainSetup()
{
	PLAYER_MSG0(0, "OpenCVMap driver starting up...");

    left_cap.open(0); // left index is 0
    right_cap.open(1); // right index is 1
    assert(left_cap.isOpened() && right_cap.isOpened());
    left_cap.set(CV_CAP_PROP_FRAME_WIDTH, CAP_WIDTH);
    left_cap.set(CV_CAP_PROP_FRAME_HEIGHT, CAP_HEIGHT);
    right_cap.set(CV_CAP_PROP_FRAME_WIDTH, CAP_WIDTH);
    right_cap.set(CV_CAP_PROP_FRAME_HEIGHT, CAP_HEIGHT);

    // allocate world matricies
    obstacles_world.create(MAP_WIDTH, MAP_HEIGHT);
    lines_world.create(MAP_WIDTH, MAP_HEIGHT);

	PLAYER_MSG0(0, "OpenCVMap driver ready.");

	return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
// Here you would shut the device down by, for example, closing a
// serial port.
void OpenCVMapDriver::MainQuit()
{
	PLAYER_MSG0(0, "OpenCVMap driver shutting down...");

	PLAYER_MSG0(0, "OpenCVMap driver has been shutdown.");
}

// Process messages here.  Send a response if necessary, using Publish().
// If you handle the message successfully, return 0.  Otherwise,
// return -1, and a NACK will be sent for you, if a response is required.
int OpenCVMapDriver::ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr, void * data)
{
	//////////////////////////////////////////////////
	// MAP: PLAYER_MAP_REQ_GET_INFO
	//////////////////////////////////////////////////
	if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ, PLAYER_MAP_REQ_GET_INFO, this->map_addr))
	{
		player_map_info_t map_info;

		memset(&map_info, 0, sizeof map_info);
		
		map_info.scale = MAP_SCALE;
		map_info.width = MAP_WIDTH;
		map_info.height = MAP_HEIGHT;
		map_info.origin.px = 0;
		map_info.origin.py = 0;
		map_info.origin.pa = 0;
		

		this->Publish(this->map_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_MAP_REQ_GET_INFO, (void*)&map_info, sizeof map_info, NULL);
		return 0;
	}
	//////////////////////////////////////////////////
	// MAP: PLAYER_MAP_REQ_GET_DATA
	//////////////////////////////////////////////////
	else if(Message::MatchMessage(hdr,PLAYER_MSGTYPE_REQ, PLAYER_MAP_REQ_GET_DATA, this->map_addr))
	{
		player_map_data_t map_data;

		memset(&map_data, 0, sizeof map_data);
		
		int8_t * occ_map = new int8_t[MAP_WIDTH*MAP_HEIGHT];


		for(int i = 0; i < (MAP_WIDTH*MAP_HEIGHT); i++)
		{
			if(i%2)
				occ_map[i] = 1;
			else
				occ_map[i] = 0;
		}

		map_data.col = 0;
		map_data.row = 0;
		map_data.width = MAP_WIDTH;
		map_data.height = MAP_HEIGHT;
		map_data.data_count = (MAP_WIDTH*MAP_HEIGHT);
		map_data.data = occ_map;
		

		this->Publish(this->map_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_MAP_REQ_GET_DATA, (void*)&map_data, sizeof map_data, NULL);

		delete[] occ_map;

		return 0;
	}


	return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void OpenCVMapDriver::Main() 
{
	InstanceInfo info;

	// The main loop; interact with the device here
	for(;;)
	{
		memset(&info, 0, sizeof(InstanceInfo));

		// test if we are supposed to cancel
		pthread_testcancel();

		// Process incoming messages.  ::ProcessMessage() is
		// called on each message.
		ProcessMessages();

		// do the actual processing work
		info.capture_time = time();
		if (!left_cap.grab() || !right_cap.grab()) {
			continue;
		}
		left_cap.retrieve(info.left_eye);
		right_cap.retrieve(info.right_eye);

		// lines

		// lines to 2D

		// error filter lines

		// obstacles
		
		// stereo

		// obstacles to 2D

		// error filter obstacles

		// clean images
		info.left_eye.release();
		info.right_eye.release();

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
		PLAYER_MSG0(0, "OpenCVMap driver initializing...");
		OpenCVMapDriver_Register(table);
		PLAYER_MSG0(0, "OpenCVMap driver initialization done.");
		return(0);
	}
}

