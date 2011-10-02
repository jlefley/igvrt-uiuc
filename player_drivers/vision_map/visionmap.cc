
/*
 * A custom driver that takes an image from a camera and produces an occupancy map
 *
 * Interfaces:
 *   - Provides:
 *     - Map
 *   - Requires:
 *     - Camera
 */

#define MAP_WIDTH 5
#define MAP_HEIGHT 5
#define MAP_SCALE 1

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



using namespace std;


////////////////////////////////////////////////////////////////////////////////
// The class for the driver
class VisionMapDriver : public ThreadedDriver
{
public:
    
	// Constructor
	VisionMapDriver(ConfigFile* cf, int section);

	// This method will be invoked on each incoming message
	virtual int ProcessMessage(QueuePointer &resp_queue, player_msghdr * hdr, void * data);

private:

	virtual void Main();
	virtual int MainSetup();
	virtual void MainQuit();

	void enqueueCameraData(player_camera_data_t * data, deque<player_camera_data_t*> & queue);
	void dequeueCameraData();
	void processCameraData(player_camera_data_t * left, player_camera_data_t * right);

	//private data members
	player_devaddr_t map_addr; // address of map interface

	Device *camera_left;
    	player_devaddr_t camera_left_id;
	deque<player_camera_data_t*> camera_left_queue;

	Device *camera_right;
    	player_devaddr_t camera_right_id;
	deque<player_camera_data_t*> camera_right_queue;

};

// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver* VisionMapDriver_Init(ConfigFile* cf, int section)
{
	// Create and return a new instance of this driver
	return((Driver*)(new VisionMapDriver(cf, section)));
}

// A driver registration function, again declared outside of the class so
// that it can be invoked without object context.  In this function, we add
// the driver into the given driver table, indicating which interface the
// driver can support and how to create a driver instance.
void VisionMapDriver_Register(DriverTable* table)
{
	table->AddDriver("visionmap", VisionMapDriver_Init);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
VisionMapDriver::VisionMapDriver(ConfigFile* cf, int section)
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

	// require left camera
	if (cf->ReadDeviceAddr(&this->camera_left_id, section, "requires", PLAYER_CAMERA_CODE, -1, "left") != 0)
	{
		this->SetError(-1);
		return;
	}

	// require right camera
	if (cf->ReadDeviceAddr(&this->camera_right_id, section, "requires", PLAYER_CAMERA_CODE, -1, "right") != 0)
	{
		this->SetError(-1);
		return;
	}

	return;
}


////////////////////////////////////////////////////////////////////////////////
// Set up the device.  Return 0 if things go well, and -1 otherwise.
// Here you do whatever is necessary to setup the device, like open and
// configure a serial port.
int VisionMapDriver::MainSetup()
{
	PLAYER_MSG0(0, "VisionMap driver starting up...");


	// Subscribe to the left camera.
	if(Device::MatchDeviceAddress(this->camera_left_id, this->device_addr))
	{
		PLAYER_ERROR("attempt to subscribe to self");
		return(-1);
	}
	if(!(this->camera_left = deviceTable->GetDevice(this->camera_left_id)))
	{
		PLAYER_ERROR("unable to locate left camera device");
		return(-1);
	}
	if(this->camera_left->Subscribe(this->InQueue) != 0)
	{
		PLAYER_ERROR("unable to subscribe to left camera device");
		return(-1);
	}

	// Subscribe to the right camera.
	if(Device::MatchDeviceAddress(this->camera_right_id, this->device_addr))
	{
		PLAYER_ERROR("attempt to subscribe to self");
		return(-1);
	}
	if(!(this->camera_right = deviceTable->GetDevice(this->camera_right_id)))
	{
		PLAYER_ERROR("unable to locate right camera device");
		return(-1);
	}
	if(this->camera_right->Subscribe(this->InQueue) != 0)
	{
		PLAYER_ERROR("unable to subscribe to right camera device");
		return(-1);
	}


    
	PLAYER_MSG0(0, "VisionMap driver ready.");

	return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
// Here you would shut the device down by, for example, closing a
// serial port.
void VisionMapDriver::MainQuit()
{
	PLAYER_MSG0(0, "VisionMap driver shutting down...");

	camera_left->Unsubscribe(InQueue);
	camera_right->Unsubscribe(InQueue);

	PLAYER_MSG0(0, "VisionMap driver has been shutdown.");
}

// Process messages here.  Send a response if necessary, using Publish().
// If you handle the message successfully, return 0.  Otherwise,
// return -1, and a NACK will be sent for you, if a response is required.
int VisionMapDriver::ProcessMessage(QueuePointer & resp_queue, player_msghdr * hdr, void * data)
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
	//////////////////////////////////////////////////
	// CAMERA(left): PLAYER_CAMERA_DATA_STATE
	//////////////////////////////////////////////////
	else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE, camera_left_id))
	{
		enqueueCameraData(reinterpret_cast<player_camera_data_t * > (data), camera_left_queue);

		return 0;
	}
	//////////////////////////////////////////////////
	// CAMERA(right): PLAYER_CAMERA_DATA_STATE
	//////////////////////////////////////////////////
	else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE, camera_right_id))
	{
		enqueueCameraData(reinterpret_cast<player_camera_data_t * > (data), camera_right_queue);

		return 0;
	}


	return 0;
}

void VisionMapDriver::enqueueCameraData(player_camera_data_t * data, deque<player_camera_data_t*> & queue)
{
	player_camera_data_t * data_copy = (player_camera_data_t*)malloc(sizeof(player_camera_data_t));

	memset(data_copy,0,sizeof(data_copy));

	data_copy->width = data->width;
	data_copy->height = data->height;
	data_copy->bpp = data->bpp;
	data_copy->format = data->format;
	data_copy->fdiv = data->fdiv;
	data_copy->compression = data->compression;
	data_copy->image_count = data->image_count;
	data_copy->image = (uint8_t*)malloc(data_copy->image_count);

	memcpy(data_copy->image, data->image, data_copy->image_count);

	queue.push_back(data_copy);
}

void VisionMapDriver::dequeueCameraData()
{
	if(camera_left_queue.size() >= 1 && camera_right_queue.size() >= 1)
	{
		player_camera_data_t * left = camera_left_queue.front();
		player_camera_data_t * right = camera_right_queue.front();

		camera_left_queue.pop_front();
		camera_right_queue.pop_front();

		processCameraData(left, right);

		free(left->image);
		free(left);

		free(right->image);
		free(right);
	}
}

void VisionMapDriver::processCameraData(player_camera_data_t * left, player_camera_data_t * right)
{
	// INSERT YOUR CODE HERE
}

////////////////////////////////////////////////////////////////////////////////
// Main function for device thread
void VisionMapDriver::Main() 
{
	// The main loop; interact with the device here
	for(;;)
	{
		dequeueCameraData();

		// test if we are supposed to cancel
		pthread_testcancel();

		// Process incoming messages.  ::ProcessMessage() is
		// called on each message.
		ProcessMessages();

		

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
		PLAYER_MSG0(0, "VisionMap driver initializing...");
		VisionMapDriver_Register(table);
		PLAYER_MSG0(0, "VisionMap driver initialization done.");
		return(0);
	}
}

