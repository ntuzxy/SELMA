#define __STDC_FORMAT_MACROS
#include <AER_display.h>
#include <SELMA_classifier.h>
#include <boost/asio.hpp>
#include <boost/cstdint.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include "protobuf/events.pb.h"
#include <inttypes.h>
#include "OK_interface.h"
#include <thread>
#include <chrono>

pthread_mutex_t display_mutex = PTHREAD_MUTEX_INITIALIZER; //might need one for each
pthread_mutex_t EM_display_mutex = PTHREAD_MUTEX_INITIALIZER;

static uint64_t x_pos, y_pos;
int MinMaxTrackerDisplay::g_track_id[4]={0,0,0,0};
//int MinMaxTrackerDisplay::g_track_id[8]={0,0,0,0,0,0,0,0};
int MinMaxTrackerDisplay::prev_track_id[4]={0,0,0,0};
int MinMaxTrackerDisplay::g_id=0;
#ifdef _useSDL
/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, 
              (4) Thread ID 
Output      : Returns information about the output event buffer 
Description : Displays the received TD data with Trackers on the image screen.
************************************************************************************************************/
int display::init_display_window(SDL_Window **window, SDL_Texture **texture, SDL_Renderer **renderer, const char* window_name)
{
		if(SDL_WasInit(SDL_INIT_VIDEO) == 0)
		{
			printf("Initializing SDL\n");	
			if(SDL_Init( SDL_INIT_VIDEO ) <0)
				printf("SDL_Init failed: %s\n", SDL_GetError());
		}else
		{
			printf("SDL already initialized\n");
		}

		if(x_pos == 0)
		{
			x_pos = DataSource::x_dim;
			y_pos = DataSource::y_dim;
		}
//		*window = SDL_CreateWindow(window_name, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, DataSource::x_dim, DataSource::y_dim, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
		*window = SDL_CreateWindow(window_name, x_pos, y_pos, DataSource::x_dim, DataSource::y_dim, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
		x_pos+= DataSource::x_dim*1.5;
		if (x_pos > DataSource::x_dim*4)
		{
			x_pos = DataSource::x_dim;
			y_pos+= DataSource::y_dim*1.5;	
		}
		
		if(*window == NULL)
			printf("ERROR: failed to create SDL window for %s\n", window_name);

    	*renderer = SDL_CreateRenderer
        (
	        *window,
	        -1,
	        SDL_RENDERER_ACCELERATED
        );
		if(*renderer == NULL)
			printf("ERROR: failed to create SDL renderer for %s\n", window_name);
		
		*texture = SDL_CreateTexture
        (
        	*renderer,
        	SDL_PIXELFORMAT_ARGB8888,
        	SDL_TEXTUREACCESS_STREAMING,
        	DataSource::x_dim, DataSource::y_dim
        );
		if(*texture == NULL)
			printf("ERROR: failed to create SDL texture for %s\n", window_name);

        printf("created window  %s\n", window_name);
		
		return 1;
}

void display::update_display(const void* pixels, SDL_Texture **texture, SDL_Renderer **renderer)
{
	//	printf("Update Display called\n");;
		SDL_UpdateTexture //use "pixels" to update the texture
        (
        	*texture,
          	NULL,
           	pixels,
           	DataSource::x_dim * 4
       	);
   //    	printf("SDL_UpdateTexture called\n");
       	
   		//copy the texture to the renderer
        SDL_RenderCopy(*renderer, *texture, NULL, NULL );
    //    printf("SDL render copy called\n");
        //push the updates to the screen
        SDL_RenderPresent( *renderer );
     //   printf("SDL render present called\n");
}

void display::destroy_SDL(SDL_Window **window, SDL_Texture **texture, SDL_Renderer **renderer)
{
	SDL_DestroyWindow(*window);
	SDL_DestroyRenderer(*renderer);
	SDL_DestroyTexture(*texture);
	if(SDL_WasInit(SDL_INIT_VIDEO) != 0)
		SDL_Quit();
}

void display::cross(uint32_t *TDpixels, int x, int y, int radius, uint32_t color)
{
	// Subcall to more general function 
	return cross(TDpixels, x, y, radius, radius, color);
}

/***
Generalization of cross function to support drawing rectangles instead of squares
*/
void display::cross(uint32_t *TDpixels, int x, int y, int radius_x, int radius_y, uint32_t color)
{
		int x_start, x_end, y_start, y_end;
		x_start = x-radius_x;					
		y_start = y-radius_y;
		x_end = x+radius_x;					
		y_end = y+radius_y;
		if(x_start<0)
			x_start=0;
		if(y_start<0)
			y_start=0;
		if(x_end>=DataSource::x_dim)
			x_end=(DataSource::x_dim-1);
		if(y_end>=DataSource::y_dim)
			y_end=(DataSource::y_dim-1);
		
		int x_loc = x_start;
		for(int y_loc = y_start; y_loc<y_end; y_loc++)
			TDpixels[(Uint32(y_loc) * DataSource::x_dim ) + Uint32(x_loc)] =  color;
		x_loc = x_end;
		for(int y_loc = y_start; y_loc<y_end; y_loc++)
			TDpixels[(Uint32(y_loc) * DataSource::x_dim ) + Uint32(x_loc)] =  color;

		int y_loc = y_start;		
		for(x_loc = x_start; x_loc<x_end; x_loc++)
			TDpixels[(Uint32(y_loc) * DataSource::x_dim ) + Uint32(x_loc)] = color;
		y_loc = y_end;		
		for(x_loc = x_start; x_loc<x_end; x_loc++)
			TDpixels[(Uint32(y_loc) * DataSource::x_dim ) + Uint32(x_loc)] = color;
}

TDdisplay::TDdisplay()
{
	printf("TD display object created\n");
	
	//init_display_window(&TDwindow, &TDtexture, &TDrenderer, "TD window");
	uint64_t num_pixels = uint64_t(DataSource::x_dim)*uint64_t(DataSource::y_dim);
	printf("TD display detect %i pixels\n",  num_pixels);
	TDpixels = new uint32_t[num_pixels];
   	for(uint64_t i=0; i<num_pixels; i++)
   		TDpixels[i] = background;
   	TDwindow = NULL;
   	
}

bool TDdisplay::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	
	//printf("TD display called\n");
	if(TDwindow == NULL)
		init_display_window(&TDwindow, &TDtexture, &TDrenderer, "TD window"); //Why can't this go in the class constructor?

	//update_display(&TDpixels[0], &TDtexture, &TDrenderer);

	//printf("TD display processing events \n");
	//printf("Display call number %i started\n", ++display_call_number);
	if(packet_info->packet_type == packet_type::TD_packet || packet_info->packet_type == packet_type::TD_EM_mixed_packet)
		if(packet_info->num_events>0)
		{
			//printf("TD display processing packet \n");
			for(int eventNum=0; eventNum<packet_info->num_events; eventNum++)
			{
				//printf("TD display processing event number %i of %i\n",  eventNum, packet_info->num_events);
				if (dataIN[eventNum].type == evt_type::TD)
				{
					//printf("displaying event %i of %i at x: %i y: %i  p: %i \n", eventNum, packet_info->num_events, dataIN[eventNum].x, dataIN[eventNum].y, dataIN[eventNum].subtype);
					//printf("linear address will be: %i \n", uint32_t(dataIN[eventNum].y) * uint32_t(DataSource::x_dim) + uint32_t(dataIN[eventNum].x));
					uint64_t linear_address = (uint32_t(dataIN[eventNum].y) * uint32_t(DataSource::x_dim) + uint32_t(dataIN[eventNum].x));
					if (dataIN[eventNum].subtype == TD_subtype::ON)
						TDpixels[linear_address] = ONcolor;
					else
						if(dataIN[eventNum].subtype == TD_subtype::OFF)
							TDpixels[linear_address] = OFFcolor;					
				}
			}
			//printf("finished displaying events \n");

			uint64_t end_time = dataIN[packet_info->num_events-1].t | (uint64_t(packet_info->time_start)<<16);
			//printf("calculated new frame end time \n");
			//printf("packet has %i events and time_start is %i and end time is  %i \n", packet_info->num_events, packet_info->time_start, end_time);
			if(end_time-prev_TD_display_time >= display_interval)
			{
			//	printf("Time to update the display \n");
				//printf("Display updated at time %i us\n", end_time);
				update_display(&TDpixels[0], &TDtexture, &TDrenderer);
			//	printf("Display Updated \n");
					//reset pixels to background color
	   			for(uint64_t pix_num=0; pix_num<uint64_t(DataSource::x_dim)*uint64_t(DataSource::y_dim); pix_num++)
	   				TDpixels[pix_num] = background;
	   		//	printf("Background image reset \n");
				prev_TD_display_time = end_time;
			}
			//printf("finished packet processing in TD \n");
		}

	if(exit_thread == 1)
	{
		printf("Destroying TD window\n");
		destroy_SDL(&TDwindow, &TDtexture, &TDrenderer);
		return 1;
	}
	//printf("Finished call to display, exit_threads is %i\n", exit_threads[mythread]);
	return 0;
	//printf("Display call number %i completed at time %i ms. Next frame is at %i\n", display_call_number, packet_info.time_start, prev_TD_display_time + display_interval);
	//return packet_info;
}




/************************************************************************************************************
Input      : (1) Information about the packet stream, (2) Buffer containing the event data, 
             (3) An output buffer containing the processed EM data. (4) The thread ID 
Output     : (5) Packet info on the event data
Description: Displays the event data based from the APS circuitry
*************************************************************************************************************/

//static Mat EMimage;

EMdisplay::EMdisplay(int scale, int offset)
{
	EMdisplay_scaling = scale;
	EMdisplay_offset = offset;
	//init_display_window(&EMwindow, &EMtexture, &EMrenderer, "EM window");

	EMpixels = new uint32_t[DataSource::x_dim*DataSource::y_dim];

	EMfirst = new uint32_t*[DataSource::y_dim];
	for(int y=0; y<DataSource::y_dim; y++)
		EMfirst[y] = new uint32_t[DataSource::x_dim];
	EMwindow = NULL;
}



bool EMdisplay::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	//printf("TD display called\n");
	if(EMwindow == NULL)	
		init_display_window(&EMwindow, &EMtexture, &EMrenderer, "TD window"); //Why can't this go in the class constructor?
	
	int pixel_value;
	int EMtime_difference;
	int EM_time;
	//	printf("Show APS called on thread %i\n", mythread);
	if(packet_info->packet_type == packet_type::EM_packet || packet_info->packet_type == packet_type::TD_EM_mixed_packet)
	{
		uint64_t time_MSBs = uint64_t(packet_info->time_start)<<16;

		for(int eventNum=0; eventNum<packet_info->num_events; eventNum++)
		{
			if (dataIN[eventNum].type == evt_type::EM)
			{
				EM_time = uint32_t(time_MSBs | dataIN[eventNum].t);
				if (dataIN[eventNum].subtype == EM_subtype::first_thresh)
					EMfirst[dataIN[eventNum].y][dataIN[eventNum].x] = EM_time;
				else
				{
					EMtime_difference = EM_time - EMfirst[dataIN[eventNum].y][dataIN[eventNum].x];
				//	printf("EMtime_difference is %i \n", EMtime_difference);
					if (EMtime_difference>0)
					{
						pixel_value = EMdisplay_offset + EMdisplay_scaling/EMtime_difference;
						pixel_value = std::min(pixel_value, 255);
						pixel_value = std::max(pixel_value, 0);
						//printf("pixel_value is %i \n", pixel_value);
	//					EMimage.at<unsigned char>(dataIN[eventNum].y,dataIN[eventNum].x) = pixel_value;
						EMpixels[ (Uint32(dataIN[eventNum].y) * DataSource::x_dim ) + dataIN[eventNum].x] =  (pixel_value << 16) | (pixel_value << 8) | (pixel_value);
					}
				}
			}
		}
		uint64_t end_time = (time_MSBs | dataIN[packet_info->num_events-1].t);
		if(end_time - prev_EM_diplay_time > display_interval)
		{
			prev_EM_diplay_time = end_time;
			update_display(&EMpixels[0], &EMtexture, &EMrenderer);
		}
	}
	if(exit_thread == 1)
	{
		destroy_SDL(&EMwindow, &EMtexture, &EMrenderer);
		return 1;
	}
	return 0;
}


MinMaxTrackerDisplay::MinMaxTrackerDisplay(bool run_SELMA)
{
	printf("Tracker Display object created\n");
	
	//init_display_window(&TDwindow, &TDtexture, &TDrenderer, "TD window");
	uint64_t num_pixels = uint64_t(DataSource::x_dim)*uint64_t(DataSource::y_dim);
	printf("Tracker display detect %i pixels\n",  num_pixels);
	TDpixels = new uint32_t[num_pixels];
   	for(uint64_t i=0; i<num_pixels; i++)
   		TDpixels[i] = background;
   	TDwindow = NULL;
//	if(TDwindow == NULL)
//		printf("TD Window Made NULL in the MMTracker Constructor\n");
//	else
//		printf("TD Window NOT Made NULL in the MMTracker Constructor\n");
   	tracks = (track_disp*)malloc(sizeof(track_disp)*MinMaxTrackerDisplay::num_tracks);

   	for(int i=0; i < MinMaxTrackerDisplay::num_tracks; i++)
   	{
   		tracks[i].min[0] = DataSource::x_dim;
   		tracks[i].min[1] = DataSource::y_dim;
   		tracks[i].max[0] = 0;
   		tracks[i].max[1] = 0;
   		tracks[i].active = false;
   		tracks[i].last_update = 0;
   		tracks[i].id = 0;
		tracks[i].colour = (i+1)*(0xFFFFFF/num_tracks);//*1048575;//1048575=0xFFFFF
//   		bool noConflict = false;
//   		while(!noConflict)
//   		{
//   			noConflict = true;
//			tracks[i].colour = rand()%0xFFFFFF;		   	
//			for (int j=0; j<i; j++)
//			{
//				if(tracks[j].colour == tracks[i].colour)
//					noConflict = false;
//			}
//		}
   	}
	if(run_SELMA)
	{
		configSELMA();
		SELMA_ON= true;
	}
}

/**
* Annotation writing constructor
**/
MinMaxTrackerDisplay::MinMaxTrackerDisplay(std::string output_filename_in)
{
	printf("Tracker Display object created\n");
	uint64_t num_pixels = uint64_t(DataSource::x_dim)*uint64_t(DataSource::y_dim);
	printf("Tracker display detect %i pixels\n",  num_pixels);
	TDpixels = new uint32_t[num_pixels];
   	for(uint64_t i=0; i<num_pixels; i++)
   		TDpixels[i] = background;
   	TDwindow = NULL;
   	tracks = (track_disp*)malloc(sizeof(track_disp)*MinMaxTrackerDisplay::num_tracks);

   	for(int i=0; i < MinMaxTrackerDisplay::num_tracks; i++)
   	{
   		tracks[i].min[0] = DataSource::x_dim;
   		tracks[i].min[1] = DataSource::y_dim;
   		tracks[i].max[0] = 0;
   		tracks[i].max[1] = 0;
   		tracks[i].active = false;
   		tracks[i].last_update = 0;
   		tracks[i].id = 0;
		tracks[i].colour = (i+1)*(0xFFFFFF/num_tracks);//*1048575;//1048575=0xFFFFF
   	}

	output_filename = output_filename_in;
	initAnnotationWriting();
}


/**
* TCP IP Constructors
**/
MinMaxTrackerDisplay::MinMaxTrackerDisplay(std::string ip_add_in, int port_in) 
{
	printf("Tracker Display object created\n");
	
	//init_display_window(&TDwindow, &TDtexture, &TDrenderer, "TD window");
	uint64_t num_pixels = uint64_t(DataSource::x_dim)*uint64_t(DataSource::y_dim);
	printf("Tracker display detect %i pixels\n",  num_pixels);
	TDpixels = new uint32_t[num_pixels];
   	for(uint64_t i=0; i<num_pixels; i++)
   		TDpixels[i] = background;
   	TDwindow = NULL;
//	if(TDwindow == NULL)
//		printf("TD Window Made NULL in the MMTracker Constructor\n");
//	else
//		printf("TD Window NOT Made NULL in the MMTracker Constructor\n");
   	tracks = (track_disp*)malloc(sizeof(track_disp)*MinMaxTrackerDisplay::num_tracks);

   	for(int i=0; i < MinMaxTrackerDisplay::num_tracks; i++)
   	{
   		tracks[i].min[0] = DataSource::x_dim;
   		tracks[i].min[1] = DataSource::y_dim;
   		tracks[i].max[0] = 0;
   		tracks[i].max[1] = 0;
   		tracks[i].active = false;
   		tracks[i].last_update = 0;
   		tracks[i].id = 0;
		tracks[i].colour = (i+1)*(0xFFFFFF/num_tracks);//*1048575;//1048575=0xFFFFF
   	}

	//TCP-IP Operations begin
	std::string logPrefix = MinMaxTrackerDisplay::logPrefix + "Constructor(IP,PORT): ";
	// std::cout << (logPrefix + "Setting Up SELMA CLassifier Networked Visualization\n");
	vis = true;
	// Setting Up TCP Connections
	try {

		std::cout << logPrefix + "Initalizing Ports and IP Address\n";
		socket = new boost::asio::ip::tcp::socket(io_service);
		messageBody = new std::string();
		boost::asio::ip::tcp::resolver resolver(io_service);
		boost::asio::ip::tcp::resolver::query query1(ip_add_in,
				boost::lexical_cast<std::string>(port_in));
		boost::asio::ip::tcp::resolver::iterator endpoint = resolver.resolve(query1);
		std::cout << logPrefix + "Connecting to IP Address\n";
		connect(*socket, endpoint);
		std::cout << logPrefix + "Connected to IP Address\n";
	} catch (std::system_error &e) 
	{
		std::cerr << "SELMAC TCP Connection Setup Exception: " << e.what() << std::endl;
	}
}

MinMaxTrackerDisplay::MinMaxTrackerDisplay(std::string ip_add_in, int track_port_in, int evt_port_in) 
{
	printf("Tracker Display object created\n");
	
	//init_display_window(&TDwindow, &TDtexture, &TDrenderer, "TD window");
	uint64_t num_pixels = uint64_t(DataSource::x_dim)*uint64_t(DataSource::y_dim);
	printf("Tracker display detect %i pixels\n",  num_pixels);
	TDpixels = new uint32_t[num_pixels];
   	for(uint64_t i=0; i<num_pixels; i++)
   		TDpixels[i] = background;
   	TDwindow = NULL;
//	if(TDwindow == NULL)
//		printf("TD Window Made NULL in the MMTracker Constructor\n");
//	else
//		printf("TD Window NOT Made NULL in the MMTracker Constructor\n");
   	tracks = (track_disp*)malloc(sizeof(track_disp)*MinMaxTrackerDisplay::num_tracks);

   	for(int i=0; i < MinMaxTrackerDisplay::num_tracks; i++)
   	{
   		tracks[i].min[0] = DataSource::x_dim;
   		tracks[i].min[1] = DataSource::y_dim;
   		tracks[i].max[0] = 0;
   		tracks[i].max[1] = 0;
   		tracks[i].active = false;
   		tracks[i].last_update = 0;
   		tracks[i].id = 0;
		tracks[i].colour = (i+1)*(0xFFFFFF/num_tracks);//*1048575;//1048575=0xFFFFF
   	}

	//TCP-IP Operations begin
	std::string logPrefix = MinMaxTrackerDisplay::logPrefix + "Constructor(IP,PORT): ";
	// std::cout << (logPrefix + "Setting Up SELMA CLassifier Networked Visualization\n");
	vis = true;
	vis_evt = true;
	// Setting Up TCP Connections
	try {

		std::cout << logPrefix + "Initalizing Ports and IP Address For Tracker Update\n";
		socket = new boost::asio::ip::tcp::socket(io_service);
		messageBody = new std::string();
		boost::asio::ip::tcp::resolver resolver(io_service);
		boost::asio::ip::tcp::resolver::query query1(ip_add_in,
				boost::lexical_cast<std::string>(track_port_in));
		boost::asio::ip::tcp::resolver::iterator endpoint = resolver.resolve(query1);
		std::cout << logPrefix + "Connecting to IP Address\n";
		connect(*socket, endpoint);
		std::cout << logPrefix + "Connected to IP Address\n";

		std::cout << logPrefix + "Initalizing Ports and IP Address For Event Update\n";
		evt_socket = new boost::asio::ip::tcp::socket(evt_io_service);
		boost::asio::ip::tcp::resolver::query query2(ip_add_in
,				boost::lexical_cast<std::string>(evt_port_in));
		boost::asio::ip::tcp::resolver resolver2(evt_io_service);

		boost::asio::ip::tcp::resolver::iterator evt_endpoint = resolver2.resolve(query2);
		std::cout << logPrefix + "Connecting to IP Address\n";
		connect(*evt_socket, evt_endpoint);
		std::cout << logPrefix + "Connected to IP Address\n";
	} catch (std::system_error &e) 
	{
		std::cerr << "SELMAC TCP Connection Setup Exception: " << e.what() << std::endl;
	}

	if(vis)
	{
		configSELMA();
		SELMA_ON= true;
	}
}

bool MinMaxTrackerDisplay::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	//printf("MMTracker::ProcessData Call\n");
	uint16_t delays[1024]={0};
	for(int i=0; i<1024;i++)
	{
		delays[i] = 15;
	}
	int num_tracks = MinMaxTrackerDisplay::num_tracks;
	if(TDwindow == NULL){
		//printf("TDWindow is NULL\n");
		init_display_window(&TDwindow, &TDtexture, &TDrenderer, "TD window"); //Why can't this go in the class constructor?
	}
	else{
		//printf("TDWindow is Not NULL\n");
	}
	//int filterTimeOverflow = packet_info->time_start; 
	int filterTimeOverflow = 0; 
	uint64_t track_time_MSBs = uint64_t(packet_info->time_start) << 16;
	uint64_t track_time;  
	//printf("MM Tracker Starts Processing...\n");	
	if(packet_info->packet_type == packet_type::TD_packet || packet_info->packet_type == packet_type::TD_EM_mixed_packet)
	{
		if(packet_info->num_events>0)
		{
			epf::Events events = epf::Events();
			// printf("Tracker TD display processing packet \n");
			for(int eventNum=0; eventNum<packet_info->num_events; eventNum++)
			{
				//epf::Events events = epf::Events();
				//track_time = dataIN[eventNum].t + filterTimeOverflow; //this is how to get the time of the current event 
				track_time = track_time_MSBs | dataIN[eventNum].t;				
				track_time = track_time + filterTimeOverflow;
				int event_type_for_print = dataIN[eventNum].type;
				//printf("displaying event type : %d\n",event_type_for_print);
				if (dataIN[eventNum].type == evt_type::timer_overflow)  
				{  
					filterTimeOverflow += 1<<16;  
					
				}else if (dataIN[eventNum].type == evt_type::TD)
				{
					//printf("displaying event %i of %i at x: %i y: %i  p: %i \n", eventNum, packet_info->num_events, dataIN[eventNum].x, dataIN[eventNum].y, dataIN[eventNum].subtype);
					//printf("linear address will be: %i \n", uint32_t(dataIN[eventNum].y) * uint32_t(DataSource::x_dim) + uint32_t(dataIN[eventNum].x));
					uint64_t linear_address = (uint32_t(dataIN[eventNum].y) * uint32_t(DataSource::x_dim) + uint32_t(dataIN[eventNum].x));
					if (dataIN[eventNum].subtype == TD_subtype::ON)
						TDpixels[linear_address] = ONcolor;
					else
						TDpixels[linear_address] = OFFcolor;
					if (vis_evt)
					{
						// Event Visualization Processing
						epf::Event * evt = events.add_evt();
						evt->set_x(dataIN[eventNum].x);
						evt->set_y(dataIN[eventNum].y);
						evt->set_p(dataIN[eventNum].subtype);	
					}
					
				}else if (dataIN[eventNum].type == evt_type::TD_tracker)
				{  					
					
//					//Tracker Info Extraction - RS					
					int evt_subtype_id = dataIN[eventNum].subtype;
					int tracker_id = evt_subtype_id & TD_tracker_subtype_mask::TRACKER_MASK;	
					tracker_id  = tracker_id >>TD_tracker_subtype_mask::BITSHIFT_AMT_TYPE; 
					int tracker = tracker_id % 8;
					//Single Tracker Solution. Other Trackers are generating events but are not integrated in the pipeline
					if(tracker < MinMaxTrackerDisplay::num_tracks){
						//printf ("Processing for Tracker : %i\n", tracker);
						int evt_sub_type = floor(tracker_id/8);
	//					//Track Info Extraction - RS
						int track_id = evt_subtype_id & TD_tracker_subtype_mask::TRACK_MASK;
						if((prev_track_id[tracker] != track_id))
						{
								g_id++;
								g_track_id[tracker]=g_id;
								prev_track_id[tracker] = track_id;
						}
						else
						{
							g_id = g_track_id[tracker];
						}
//						printf("Event Num %i : Tracker %i - (X=%i,Y=%i), subtype - %i, g_track_id = %i, track_id = %i, prev_track_id = %i\n", eventNum, \
								tracker, dataIN[eventNum].x, dataIN[eventNum].y, evt_sub_type, g_track_id, track_id, prev_track_id);
//						printf("Tr %i - track %i, g_id = %i || ", tracker, track_id, g_id);
//						printf("P: %i %i %i %i || ", prev_track_id[0],prev_track_id[1],prev_track_id[2],prev_track_id[3]);
//						printf("G: %i %i %i %i\n", g_track_id[0], g_track_id[1],g_track_id[2],g_track_id[3]);
					
						//printf("Tracker %i, track_id = %i, prev_track_id = %i, g_track_id = %i\n", tracker,track_id, prev_track_id,  g_track_id);
						int track_number = tracker;

						 switch(evt_sub_type)
					 	{
					 		case TD_tracker_subtype_types::MIN:
					 		{

					 			// MIN EVENTS						
					 			tracks[track_number].min[0] = dataIN[eventNum].x;
					 			tracks[track_number].min[1] = dataIN[eventNum].y;
					 			//printf("Tracker %i MIN X: %i MIN Y: %i for time %i \n", track_number, tracks[track_number].min[0], tracks[track_number].min[1], time);
					 			break;
					 		}
					 		case TD_tracker_subtype_types::MAX:
					 		{	// MAX EVENTS
					 			tracks[track_number].max[0] = dataIN[eventNum].x;
					 			tracks[track_number].max[1] = dataIN[eventNum].y;
					 			//printf("Tracker %i MAX X: %i MAX Y: %i for time %i \n", track_number, tracks[track_number].max[0], tracks[track_number].max[1], time);
// 								tracks[track_number].last_update = dataIN[eventNum].t;
					 			break;
					 		}
					 		default:
					 		{
					 			printf("Unexpected Event SubType = %d\n", evt_sub_type);
					 		}
						}
						
						if ((tracks[track_number].min[0] < tracks[track_number].max[0])
							&& (tracks[track_number].min[1] < tracks[track_number].max[1]))
						{
							tracks[track_number].active = true;
							tracks[track_number].id = g_track_id[track_number];
							tracks[track_number].last_update = track_time;
							//printf("track active = %i\n",track_number);
					 		//printf("Tracker ID %i, MIN X: %i MIN Y: %i\n", g_track_id, tracks[track_number].min[0], tracks[track_number].min[1]);

						}
						//printf("track_active - step \n");
					}
				}

			}
	
				if (events.evt_size() > 0) 
				{
//					printf("Creating Data Buffer\n");
					int msg_size = events.ByteSize();

					data_buffer.resize(msg_size + SELMA_PKT_HEADER_SIZE);

					//encode 4 bytes header to denote size of message body
					data_buffer[0] = static_cast<boost::uint8_t>((msg_size >> 24) & 0xFF);
					data_buffer[1] = static_cast<boost::uint8_t>((msg_size >> 16) & 0xFF);
					data_buffer[2] = static_cast<boost::uint8_t>((msg_size >> 8) & 0xFF);
					data_buffer[3] = static_cast<boost::uint8_t>(msg_size & 0xFF);

					//encode message body
					events.SerializeToArray(&data_buffer[SELMA_PKT_HEADER_SIZE], msg_size);
					evt_socket->send(boost::asio::buffer(data_buffer, msg_size + SELMA_PKT_HEADER_SIZE));
				}

			for(int i=0; i<num_tracks; i++)
			{		
				//printf("track-%i\n",i);			
				if(tracks[i].active)
				{
					//printf("Active Track:%i, Track Time %" PRIu64 ", Last Update %" PRIu64 "\n",tracks[i].id, track_time, tracks[i].last_update);
					if (((int64_t)track_time - (int64_t)tracks[i].last_update) > MinMaxTrackerDisplay::track_exp)
					{
						tracks[i].active = false;
						tracks[i].min[0] = DataSource::x_dim;
   						tracks[i].min[1] = DataSource::y_dim;
   						tracks[i].max[0] = 0;
   						tracks[i].max[1] = 0;
   						tracks[i].last_update = 2 << 48;   		
						//sendResultsToVisualizer(i,false);
					}else
					{
						//printf("Drawing for tracker:%i, track:%i, color=%lu\n", i, tracks[i].id, (unsigned long)tracks[i].colour);
						int track_rad_x = (tracks[i].max[0] - tracks[i].min[0]) / 2;
						int track_loc_x = tracks[i].min[0] + track_rad_x;
						int track_rad_y = (tracks[i].max[1] - tracks[i].min[1]) / 2;
						int track_loc_y = tracks[i].min[1] + track_rad_y;
						cross(TDpixels, track_loc_x, track_loc_y, track_rad_x, track_rad_y, tracks[i].colour);

						//NUS Visualizer Call
						uint16_t x_min = tracks[i].min[0];
						uint16_t x_max = tracks[i].max[0];
						uint16_t y_min = tracks[i].min[1];
						uint16_t y_max = tracks[i].max[1];
						uint16_t t_id  = i;//tracks[i].id;
						sendResultsToVisualizer(x_min, x_max, y_min, y_max, i); 
						tracks[i].last_update = track_time;
						if(writeAnnotations)
						{
							if(track_rad_x<500 && track_rad_x>-1)
							{
//								printf("Writing Annotations rad_x %d...\n", track_rad_x);
								writeAnnotationCSV(
										track_time, 
										floor(track_loc_x), 
										floor(track_loc_y), 
										floor(track_rad_x*2), 
										floor(track_rad_y*2), 
										tracks[i].id, 
										1);
							}
						}
						else
						{
							//printf("writeAnnotations is False...\n");
						}
						if(SELMA_ON)
						{
							//Getting Data From SELMA
							uint64_t end_time = dataIN[packet_info->num_events-1].t | (uint64_t(packet_info->time_start)<<16);
						  	//printf("Checking if Selma Status Bit is High...\n");
							if(isSelmaStatusBitHigh(SelmaStatusBits::SelmaIfROFifo2Empty, 0))
							//if(end_time-prev_TD_display_time >= display_interval)
							{
								printf("Stage 2 FIFO Not Empty...\n");
								std::pair <uint16_t*, int64_t*> output = sendInputToSELMA(delays);
								int64_t* stage2_data = output.second;								
								uint8_t confid[SELMA_NUM_CLASSES] = {0};
								//if(stage2_data[0] > 2<<25)
								//{
									convertStage2toConfidence(stage2_data, confid);
									printf("stage 2 conf car:%d, bus:%d, van:%d, hum:%d, bik:%d, tru:%d\n",confid[0],confid[1],confid[2],confid[3],confid[4],confid[5]);
									//printf("stage 2 data %" PRId64 " %" PRId64 " %" PRId64 " %" PRId64 " %" PRId64 " %" PRId64 "\n",stage2_data[0], \
									//		stage2_data[1], stage2_data[2], stage2_data[3], stage2_data[4], stage2_data[5]);	
									printf("stage 2 data car:%lld, bus:%lld, van%lld, hum:%lld, bik:%lld, tru:%lld \n\n",stage2_data[0], \
											stage2_data[1], stage2_data[2], stage2_data[3], stage2_data[4], stage2_data[5]);	
									//std::cout <<" | "<< std::hex << ((uint64_t)stage2_data[0])<< " | "<< std::hex << ((uint64_t)stage2_data[1])\
									//		 << " | "<< std::hex << ((uint64_t)stage2_data[2])<< " | "<< std::hex << ((uint64_t)stage2_data[3])\ 
									//		 << " | "<< std::hex << ((uint64_t)stage2_data[4])<< " | "<< std::hex << ((uint64_t)stage2_data[5]) << std::endl; //-RS 
									uint16_t x_min = tracks[0].min[0];
									uint16_t x_max = tracks[0].max[0];
									uint16_t y_min = tracks[0].min[1];
									uint16_t y_max = tracks[0].max[1];
									sendResultsToVisualizer(x_min, x_max, y_min, y_max, 0, confid); 
								//}
							}
						}
					}			
				}
			}

			uint64_t end_time = dataIN[packet_info->num_events-1].t | (uint64_t(packet_info->time_start)<<16);
			//printf("calculated new frame end time \n");
			//printf("packet has %i events and time_start is %i and end time is  %i \n", packet_info->num_events, packet_info->time_start, end_time);
			if(end_time-prev_TD_display_time >= display_interval)
			{
				//printf("Time to update the display \n");
				//printf("Display updated at time %i us\n", end_time);
				update_display(&TDpixels[0], &TDtexture, &TDrenderer);
				//printf("Display Updated \n");
				//reset pixels to background color
	   			for(uint64_t pix_num=0; pix_num<uint64_t(DataSource::x_dim)*uint64_t(DataSource::y_dim); pix_num++)
	  				TDpixels[pix_num] = background;
	   		//	printf("Background image reset \n");
				prev_TD_display_time = end_time;

				/*if(SELMA_ON)
				{
					//Getting Data From SELMA
					std::pair <uint16_t*, int64_t*> output = sendInputToSELMA(delays);
					int64_t* stage2_data = output.second;								
					uint8_t confid[SELMA_NUM_CLASSES] = {0};
					convertStage2toConfidence(stage2_data, confid);
					printf("stage 2 conf %d, %d, %d, %d, %d, %d\n",confid[0],confid[1],confid[2],confid[3],confid[4],confid[5]);
					printf("stage 2 conf %" PRId64 " %" PRId64 " %" PRId64 " %" PRId64 " %" PRId64 " %" PRId64 "\n",stage2_data[0], \
							stage2_data[1], stage2_data[2], stage2_data[3], stage2_data[4], stage2_data[5]);	
					uint16_t x_min = tracks[0].min[0];
					uint16_t x_max = tracks[0].max[0];
					uint16_t y_min = tracks[0].min[1];
					uint16_t y_max = tracks[0].max[1];
					sendResultsToVisualizer(x_min, x_max, y_min, y_max, 0, confid); 
				}*/
			}
				//printf("finished packet processing in TD \n");
			//printf("finished displaying events \n");

		}
	}
	if(exit_thread == 1)
	{
		printf("Destroying TD window\n");
		destroy_SDL(&TDwindow, &TDtexture, &TDrenderer);
		return 1;
	}

	//printf("Finished call to display, exit_threads is %i\n", exit_threads[mythread]);
	return 0;
	//printf("Display call number %i completed at time %i ms. Next frame is at %i\n", display_call_number, packet_info.time_start, prev_TD_display_time + display_interval);
	//return packet_info;
}

// Send ACTIVE status Code
void MinMaxTrackerDisplay::sendResultsToVisualizer(uint8_t id, bool status)
{
	if(vis)
	{
		std::string logPrefix = MinMaxTrackerDisplay::logPrefix + "sendResultsToVisualizer(): ";
		// std::cout << logPrefix + "Preparing Status Message to be sent to Visualizer" << std::endl;

		// Tracker Active Message
		auto statMessage = new tn::Message();
		auto statUpdate = new tn::Tracker_Status_Update();
		statUpdate->set_id(id);
		statUpdate->set_is_active(status);
		statMessage->set_msg_type(tn::Message_Type::TStatus);
		// std::cout << logPrefix + "Stat Message Type is : " + std::to_string(statMessage->msg_type()) << std::endl;		
		statMessage->set_allocated_t_status(statUpdate);
		
		// std::cout << logPrefix + "Sending Status Messages to Visualizer" << std::endl;
		send_tcp_message(statMessage);
		// std::cout << logPrefix + "Sent Status Messages to Visualizer" << std::endl;
	}
}

// Send Bounding Box Code + Active Track Status
void MinMaxTrackerDisplay::sendResultsToVisualizer(uint16_t x_min, uint16_t x_max, uint16_t y_min, uint16_t y_max, uint8_t id) 
{
	if(vis)
	{
		sendResultsToVisualizer(id, true);
		std::string logPrefix = MinMaxTrackerDisplay::logPrefix + "sendResultsToVisualizer(): ";
//		 std::cout << logPrefix + "Preparing Tracker Message to be sent to Visualizer" << std::endl;
		
		// Tracker Min Max Update
		auto minMessage = new tn::Message();
		auto minUpdate = new tn::Tracker_Min_Update();
		minUpdate->set_id(id);
		minUpdate->set_x(x_min);
		minUpdate->set_y(y_min);
		minMessage->set_msg_type(tn::Message_Type::TMin);
		minMessage->set_allocated_t_min(minUpdate);

		auto maxMessage = new tn::Message();
		auto maxUpdate = new tn::Tracker_Max_Update();
		maxUpdate->set_id(id);
		maxUpdate->set_x(x_max);
		maxUpdate->set_y(y_max);
		maxMessage->set_msg_type(tn::Message_Type::TMax);
		maxMessage->set_allocated_t_max(maxUpdate);

		// Sending of Messages
		//std::cout << logPrefix + "Sending Tracker Messages to Visualizer" << std::endl;
		send_tcp_message(minMessage);
		send_tcp_message(maxMessage);
		//std::cout << logPrefix + "Sent Tracker Messages to Visualizer" << std::endl;
	}
}

// Send Confidence Code Given uint_8 conf[6] array 
void MinMaxTrackerDisplay::sendResultsToVisualizer(uint16_t x_min, uint16_t x_max, uint16_t y_min, uint16_t y_max, uint8_t id, uint8_t* conf)
{
	if(vis)
	{
		sendResultsToVisualizer(x_min,x_max,y_min,y_max,id);
		std::string logPrefix = MinMaxTrackerDisplay::logPrefix + "sendResultsToVisualizer(): ";
		// std::cout << logPrefix + "Preparing Confidence Message to be sent to Visualizer" << std::endl;

		auto confMessage = new tn::Message();
		auto confUpdate = new tn::Confidence_Update();

		// Set ID
		confUpdate->set_id(id);

		// Confidence Message Preparation
		confUpdate->set_bike(conf[4]);
		confUpdate->set_bus(conf[1]);
		confUpdate->set_car(conf[0]);
		confUpdate->set_human(conf[3]);
		
		confUpdate->set_truck(conf[5]);
		confUpdate->set_van(conf[2]);
		confUpdate->set_tracked_veh(0);
		confUpdate->set_wheeled_veh(100-conf[3]);
		confUpdate->set_unknown(0);
		confMessage->set_msg_type(tn::Message_Type::Conf);
		confMessage->set_allocated_conf(confUpdate);

		// Sending of Messages
		// std::cout << logPrefix + "Sending Confidence Message to Visualizer" << std::endl;
		send_tcp_message(confMessage);
		// std::cout << logPrefix + "Sent Confidence Message to Visualizer" << std::endl;
	}
}

void MinMaxTrackerDisplay::convertStage2toConfidence(int64_t* stage2_data, uint8_t* confid)
{
	std::string logPrefix = MinMaxTrackerDisplay::logPrefix + "convertStage2toConfidence(): ";
	double min_value = 0;
	for (int i=0; i<SELMA_NUM_CLASSES; i++)
	{
		min_value = std::min((double)stage2_data[i],min_value); // Not Using Softmax directly as Sum overflows
	}

	double total_sum = 0;
	for (int i=0; i<SELMA_NUM_CLASSES; i++)
	{
		stage2_data[i] -= min_value;
		total_sum += std::max((double)stage2_data[i],(double)0); // Not Using Softmax directly as Sum overflows
	}
	// std::cout << logPrefix + "total_sum : " + std::to_string(total_sum) << std::endl;
	//std::cout << logPrefix;

	for (int i=0; i<SELMA_NUM_CLASSES; i++)
	{
		confid[i] = floor((std::max((double)stage2_data[i],(double)0) / total_sum) * 100);
	//	std::cout << std::to_string(stage2_data[i]) + "->"+ std::to_string(confid[i]) + " ";
	}
	//std::cout << std::endl;
}

void MinMaxTrackerDisplay::send_tcp_message(tn::Message* message) {
	int msg_size = message->ByteSize();
//	std::cout << "Message Msg Type is : " + std::to_string(message->msg_type()) << std::endl;
	data_buffer.resize(msg_size + SELMA_PKT_HEADER_SIZE);

	//encode 4 bytes header to denote size of message body
	data_buffer[0] = static_cast<boost::uint8_t>((msg_size >> 24) & 0xFF);
	data_buffer[1] = static_cast<boost::uint8_t>((msg_size >> 16) & 0xFF);
	data_buffer[2] = static_cast<boost::uint8_t>((msg_size >> 8) & 0xFF);
	data_buffer[3] = static_cast<boost::uint8_t>(msg_size & 0xFF);

	//encode message body
	message->SerializeToArray(&data_buffer[SELMA_PKT_HEADER_SIZE], msg_size);
	socket->send(boost::asio::buffer(data_buffer, msg_size + SELMA_PKT_HEADER_SIZE));
}


void MinMaxTrackerDisplay::initAnnotationWriting()
{
	writeAnnotationsHead();
	writeAnnotations = true;
}

std::string MinMaxTrackerDisplay::getAnnotationsHead(std::string recording_file, 
                              std::string picture_file, 
                              std::string annotation_file, 
                              std::string annotator, 
                              std::string sensor_dimensions_x, 
                              std::string sensor_dimensions_y)
{
	std::string header = "";
	std::string recording_header = "#This text file contains annotation data for recordings in: " + recording_file + "\n";
	std::string picture_header = "#The corresponding picture of the recording site is at: " + picture_file + "\n";
	std::string annotation_header = "#The annotation file is stored at: " + annotation_file + "\n";	
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	std::string yr = std::to_string(tm.tm_year + 1900);
	std::string mon = std::to_string(tm.tm_mon + 1);
	std::string day = std::to_string(tm.tm_mday);
	std::string h = std::to_string(tm.tm_hour);
	std::string m = std::to_string(tm.tm_min);
	std::string s = std::to_string(tm.tm_sec);
	std::string comments_header = "#Comments: #Event file for linux_aer created using SELMA_classifier at time " + 
								day + "-" + mon + "-" + yr + " " + h + ":" + m + ":" + s  + "\n";
	std::string annotator_header = "#The recordings are annotated by: " + annotator + "\n";
	std::string sensor_headers = "#Sensor Dimensions- Height = " + sensor_dimensions_y + " Pixels Width = " + sensor_dimensions_x  + " Pixels\n";
	std::string legend_header = "#LEGEND: 1-Car, 2-Bus, 3-Van, 4-Pedestrian, 5-Bike, 6-Truck, 7-Unknown\n";
	std::string csv_header = "#Time(us),x-Location,y-Location,x-size,y-size,track-num,class\n";
	header = recording_header + picture_header + annotation_header + comments_header + annotator_header + sensor_headers + legend_header + csv_header;
	return header;
}

void MinMaxTrackerDisplay::writeAnnotationsHead()
{
	std::string x_dim = std::to_string(DataSource::x_dim);
	std::string y_dim = std::to_string(DataSource::y_dim);
	std::string unspec = "Unspecified";
	std::string header = getAnnotationsHead(unspec, 
											unspec,
											unspec,
											"SELMAC_Track",
											x_dim,
											y_dim);
	// Opening File
	output_file.open(output_filename.c_str(), std::ios::out);

	output_file << header;
	// Closing
	output_file.close();	
}

void MinMaxTrackerDisplay::writeAnnotationCSV(uint64_t time, uint16_t x_loc, uint16_t y_loc, uint16_t xSize, uint16_t ySize, uint16_t track_num, uint16_t label)
{
	// Opening File
	output_file.open(output_filename.c_str(), std::ios::out | std::ofstream::app);

	output_file << std::to_string(time) << ", ";
	output_file << std::to_string(x_loc+1) << ", "; // Annotations coordinates are 1-indexed
	output_file << std::to_string(y_loc+1) << ", "; // Annotations coordinates are 1-indexed
	output_file << std::to_string(xSize) << ", ";
	output_file << std::to_string(ySize) << ", ";
	output_file << std::to_string(track_num) << ", ";
	output_file << std::to_string(label) << ", " << std::endl;

	// Closing
	output_file.close();
}


/**
 * Function Call To Configure SELMA Through MMTracker
 */
void MinMaxTrackerDisplay::configSELMA()
{
	pthread_mutex_lock(&OkInterface::ok_mutex);
	OkInterface::Instance()->ActivateTriggerIn(TriggerInAddress_SELMA::SelmaTriggers, SelmaTriggerWires::SelmaIfReset);
	pthread_mutex_unlock(&OkInterface::ok_mutex);

	waitSelmaStatusBit(SelmaStatusBits::SelmaIfConfigDone,1);
}


/**
 Function to Find if SELMA has written output to FIFO 
*/
bool MinMaxTrackerDisplay::isSelmaStatusBitHigh(uint32_t STATUS_BIT, bool STATUS_VALUE)
{
	bool isDone = false;
	pthread_mutex_lock(&OkInterface::ok_mutex);
	OkInterface::Instance()->UpdateWireOuts();
	// Read for config_done bit
	uint32_t selma_status = uint16_t(OkInterface::Instance()->GetWireOutValue(WireOutAddress_SELMA::selma_status));
	// std::cout << (logPrefix + "Releasing OkUSB Lock\n");
	pthread_mutex_unlock(&OkInterface::ok_mutex);
		 
	if (STATUS_VALUE) {
		isDone = ((selma_status & STATUS_BIT) >  0);
	} else {
		isDone = ((selma_status & STATUS_BIT) == 0);
		//std::cout << "Status Bits: "<<std::hex<<STATUS_BIT<<" | Selma Status Wire Out: "<< std::hex<<selma_status<<" | isDone: "<<std::to_string(isDone)<<std::endl;
	}
	return isDone;
}


/**
 Code to Send SELMA Control Signals
*/
void MinMaxTrackerDisplay::waitSelmaStatusBit(uint32_t STATUS_BIT, bool STATUS_VALUE)
{
	uint32_t sleep_time = SELMA_MIN_SLEEP_TIME; // in us
	bool isDone = false;
	uint64_t total_time = 0;
	// Start Activating Configuration 
	std::this_thread::sleep_for(std::chrono::microseconds(SELMA_MIN_SLEEP_TIME));
	total_time += SELMA_MIN_SLEEP_TIME;

	do{

		if(total_time > SELMA_MAX_TOTAL_TIME)
		{
			//std::cout << logPrefix + "Total Wait Time Exceeded " + std::to_string(SELMA_MAX_TOTAL_TIME) << std::endl; 
			//std::cout << logPrefix + "Exiting Program" << std::endl; 
			//throw std::runtime_error(logPrefix + "Likely Infinite Loop");
			throw std::runtime_error("Likely Infinite Loop");
		}

		// std::cout << (logPrefix + "Obtaining OkUSB Lock\n");
		pthread_mutex_lock(&OkInterface::ok_mutex);
		OkInterface::Instance()->UpdateWireOuts();
		// Read for config_done bit
		uint32_t selma_status = uint16_t(OkInterface::Instance()->GetWireOutValue(WireOutAddress_SELMA::selma_status));
		// std::cout << (logPrefix + "Releasing OkUSB Lock\n");
		pthread_mutex_unlock(&OkInterface::ok_mutex);
		 
		// isDone = (selma_status & STATUS_BIT) == STATUS_BIT;		
		// std::cout << logPrefix +  "Selma_Status " + std::to_string(selma_status) + " | isDone " + std::to_string(isDone) + " \n";
	if (STATUS_VALUE) {
		isDone = ((selma_status & STATUS_BIT) >  0);
	} else {
		isDone = ((selma_status & STATUS_BIT) == 0);
	}
		if(!isDone)		
		{
			sleep_time = sleep_time << 1; // Exponential backoff
			if(sleep_time > (1 << 16))
				sleep_time = SELMA_MIN_SLEEP_TIME; // min time = 1ms

//			usleep(sleep_time);
	std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
			total_time += sleep_time;
		}
	}while(!isDone);
}


std::pair<uint16_t*,int64_t*>  MinMaxTrackerDisplay::sendInputToSELMA(uint16_t delays[])
{
	std::string logPrefix = MinMaxTrackerDisplay::logPrefix + "sendInputToSELMA(): ";
	// INTPUT Buffers
	unsigned char raw_input_bytes[2*SELMA_NUM_INPUT_CHANNELS] = {0};
	// READOUT Buffers
 	unsigned char raw_stage1_data[2*SELMA_NUM_OUTPUT_CHANNELS] = {0};
 	uint16_t stage1_data[SELMA_NUM_OUTPUT_CHANNELS] = {0};
 	unsigned char raw_stage2_data[SELMA_NUM_BYTES_PER_CLASS*SELMA_NUM_CLASSES] = {0};	// Number of Bytes Per Channel * Number of Channels
 	int64_t stage2_data[SELMA_NUM_CLASSES] = {0};
 	// int64_t stage2_data_calculated[SELMA_NUM_CLASSES] = {0};
 	unsigned char raw_data_buffer_excess[2*SELMA_NUM_OUTPUT_CHANNELS] = {0};
  	//waitSelmaStatusBit(SelmaStatusBits::SelmaIfROFifo2Empty, 0);


	pthread_mutex_lock(&OkInterface::ok_mutex);
    long num_bytes_stage2 = OkInterface::Instance()->ReadFromBlockPipeOut(PipeOutAddress_SELMA::RO_F2DataIn, 2, SELMA_NUM_CLASSES*SELMA_NUM_BYTES_PER_CLASS, raw_stage2_data);
    long num_excess_bytes2 = OkInterface::Instance()->ReadFromBlockPipeOut(PipeOutAddress_SELMA::RO_F2DataIn, 2, 2*SELMA_NUM_OUTPUT_CHANNELS, raw_data_buffer_excess);
//	std::cout << logPrefix + "Raw Stage 2 Bytes read: " + std::to_string(num_bytes_stage2) + " \n";//-RS
	pthread_mutex_unlock(&OkInterface::ok_mutex);
	
	if (num_bytes_stage2 > 0) 
	{
	    // std::cout << logPrefix + "SELMA stage 2 Readout has " + std::to_string(num_bytes_stage2) + " bytes of data\n";
		//std::cout << logPrefix + "Raw Stage 2 Data:\n"; //-RS 
	    for(int i=0; i<SELMA_NUM_CLASSES; ++i)
	    {	    
	    	//for(int j=0; j<SELMA_NUM_BYTES_PER_CLASS; j++)
	    	//{
	    	//	 // std::cout << logPrefix + "Raw Stage 2 Bytes: " + std::to_string(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + j]) + " \n";
	    	//}   	    	
	    	// MSB First then LSB (for the 4 16-bit words) :  Internal to 16-bit words is LSB then MSB
	    	stage2_data[i] = 0;
		stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 0]) << (6 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 1]) << (7 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 2]) << (4 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 3]) << (5 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 4]) << (2 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 5]) << (3 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 6]));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 7]) << (1 * 8));

		//std::cout << logPrefix + "Raw Stage 2 Data: " + std::to_string(std::hex << ((uint64_t)stage2_data[i]))+ "\n"; //-RS 
		//std::cout << "Raw Stage 2 Data: " << std::hex << ((int64_t)stage2_data[i]) << std::endl; //-RS 
		std::cout << "Raw Stage 2 Data: " << ((int64_t)stage2_data[i]) << std::endl;
		printf("Hex Value: %x\n",stage2_data[i]);
		//std::cout <<" 	Hex: "<< std::hex << ((int64_t)stage2_data[i]) << std::endl; //-RS 
		//std::cout << std::to_string(stage2_data[i])+ ", "; //-RS 
	    }
      //std::cout << "\n";//-RS
      //std::cout << logPrefix + " \n************************************************\n";//-RS
    }	
    return std::pair<uint16_t*,int64_t*>(stage1_data, stage2_data);
}

/**
 * [SELMA_classifier::getDelayVal description]
 * @param  num_spk [description]
 * @return         [description]
 */
uint16_t MinMaxTrackerDisplay::getDelayVal(uint16_t num_spk)
{
	// delay_val = (31 / num_spk) - 1;
	// Old Formula : # spk = floor(31 / delay) ; delay = 31 / # spk
	// New Formula : # spk = floor(31 / (delay+1) ; delay = (31 / # spk) - 1 
	// Delay = # spk
	// 1 = 15    6 = 4    14-29=1 	 2 = 10    7 = 3    30-31=0
	// 3 = 7     8 = 3    4 = 6 	 9 = 3     5 = 5    10-13=2 
	uint16_t delay_val = 31;
	switch(num_spk)
	{
		case 15:  delay_val = 1; break;
		case 14:
		case 13:
		case 12:
		case 11:
		case 10: delay_val = 2; break;	
		case 9:
		case 8:
		case 7: delay_val = 3; break;
		case 6: delay_val = 4; break;
		case 5: delay_val = 5; break;
		case 4: delay_val = 6; break;
		case 3: delay_val = 7; break; 
		case 2: delay_val = 11; break;
		case 1: delay_val = 20; break;
		case 0: delay_val = 31; break;
		default: delay_val = 1; 
	}
	return delay_val;			
}


#else
	int init_showTD()
	{
		printf("ERROR: not compiled with SDL, so display functions will not be available \n");
		return 1;
	}

	bool showTD(packet_info_type* packet_info, event* dataIN, bool exit_thread)
	{
		printf("ERROR: not compiled with SDL, so display functions will not be available \n");
		return packet_info;
	}

	int init_showAPS(int scale, int offset)
	{
		printf("ERROR: not compiled with SDL, so display functions will not be available \n");
	}

	bool showAPS(packet_info_type* packet_info, event* dataIN, bool exit_thread)
	{
		printf("ERROR: not compiled with SDL, so display functions will not be available \n");
		return packet_info;
	}
#endif
