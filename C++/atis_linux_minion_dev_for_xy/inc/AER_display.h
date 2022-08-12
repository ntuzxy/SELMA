//FUNCTIONS FOR DISPLAYING AER DATA GO IN THIS FILE
#ifndef AER_DISPLAY_H
#define AER_DISPLAY_H

#include <stdint.h>
#include <stdio.h>
#ifdef _useOPENCV
	#include <opencv.hpp>
	#include <highgui.hpp>
	using namespace cv;
#endif
#include <iostream>
#include <fstream>
#include <dlfcn.h>
#include <event_framework.h>
#include <thread_launcher.h>
#include <mutex>
#include <cmath>

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include "../src/protobuf/tn_message.pb.h"

#define display_interval 100000        //minimum number of microseconds to occur between frames
#define video_interval 33333        //minimum number of microseconds to occur between frames

#ifdef _useSDL
	#include <SDL2/SDL.h>
	//static SDL_Event SDL_keyboard_event;


class display {
	protected:
		int init_display_window(SDL_Window **window, SDL_Texture **texture, SDL_Renderer **renderer, const char* window_name);
		void update_display(const void* pixels, SDL_Texture **texture, SDL_Renderer **renderer);
		void cross(uint32_t *TDpixels, int x, int y, int radius, uint32_t color);
		void cross(uint32_t *TDpixels, int x, int y, int radius_x, int radius_y, uint32_t color);
		void destroy_SDL(SDL_Window **window, SDL_Texture **texture, SDL_Renderer **renderer);

	public:
		const static uint32_t ONcolor = 0x00FFFFFF;
		const static uint32_t OFFcolor = 0x00000000;
		const static uint32_t background = 0x00808080;
};

class EMdisplay : public DataConsumer, public display {
	private:
		uint32_t** EMfirst; //left at ATIS dimensions since it is only used with ATIS for now
		uint32_t EMdisplay_scaling;
		uint32_t EMdisplay_offset;
		uint64_t prev_EM_diplay_time;
		SDL_Window *EMwindow;
		SDL_Renderer* EMrenderer;
		SDL_Texture* EMtexture;
		uint32_t *EMpixels;
	public:
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
		EMdisplay(int scale, int offset);
};


class TDdisplay : public DataConsumer, public display {
	private:
		uint64_t prev_TD_display_time;
		SDL_Window* TDwindow;
		SDL_Renderer* TDrenderer;
		SDL_Texture* TDtexture;
		uint32_t *TDpixels;
		enum intensity{black = 0, gray = 128, white = 255};
	public:
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
		TDdisplay();
};

//Class to Print Events To Console and Highlight any 
//unusual events

/*class EventFilter : public DataConsumer {
	private:
		int curr_x;
		int curr_y;
		uint64_t curr_t;
	public:
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);	
}*/



class MinMaxTrackerDisplay : public DataConsumer, public display {
	private: 
		const static uint num_tracks = 4;
		const static uint track_exp = 33000;
		const std::string logPrefix = "MMTracker: ";
		static int g_track_id[num_tracks];
		static int prev_track_id[num_tracks];
		static int g_id;
		// DISPLAY FOR TD EVENTS
		uint64_t prev_TD_display_time;
		SDL_Window* TDwindow;
		SDL_Renderer* TDrenderer;
		SDL_Texture* TDtexture;
		uint32_t *TDpixels;
		enum intensity{black = 0, gray = 128, white = 255};

		struct track_disp {
			uint16_t min[2];
			uint16_t max[2]; 
			bool active;
			uint64_t last_update;
			uint32_t colour;
			uint16_t id;
		};
		// INTERNAL TRACKER DISPLAY STATE
		track_disp* tracks;

		// SELMA DATA Functions
		bool SELMA_ON=false;
		void configSELMA(); //Starts SELMA Config Process
		void waitSelmaStatusBit(uint32_t STATUS_BIT, bool STATUS_VALUE);// Busy Waits For SELMA Status Bit to be Active
		bool isSelmaStatusBitHigh(uint32_t STATUS_BIT, bool STATUS_VALUE);// Busy Waits For SELMA Status Bit to be Active
		std::pair<uint16_t*, int64_t*> sendInputToSELMA(uint16_t delays[]);
	    void convertStage2toConfidence(int64_t* stage2_data, uint8_t* confid);
		uint16_t getDelayVal(uint16_t num_spk); //Spike Rate to Delay Conversion

    	// Comms Functionality
    	void sendResultsToComms(uint16_t width, uint16_t height,  uint16_t x_addr, uint16_t y_addr, int64_t* category_data, uint8_t id);

    	// Visualization Functionality
    	// For Communicating with Network
    	bool vis = false;
    	boost::asio::io_service io_service;
    	boost::asio::ip::tcp::socket * socket;
    	std::string * messageBody;
    	std::vector<uint8_t> data_buffer;
    	void sendResultsToVisualizer(uint16_t width, uint16_t height, uint16_t x_addr, uint16_t y_addr, uint8_t id, uint8_t* conf);
    	void sendResultsToVisualizer(uint16_t x_min, uint16_t x_max, uint16_t y_min, uint16_t y_max, uint8_t id);
    	void sendResultsToVisualizer(uint8_t id, bool status);
    	void send_tcp_message(tn::Message* message);
 
	   	// For Event Communication:
   		bool vis_evt = false;
   		boost::asio::io_service evt_io_service;
		boost::asio::ip::tcp::socket * evt_socket;

		// Annotations Writing Functions
		bool writeAnnotations = false;
		std::ofstream output_file;
		std::string output_filename;
		void initAnnotationWriting();
		std::string getAnnotationsHead(std::string recording_file, 
				              std::string picture_file, 
				              std::string annotation_file, 
				              std::string annotator, 
				              std::string sensor_dimensions_x, 
				              std::string sensor_dimensions_y);
		void writeAnnotationsHead();
		void writeAnnotationCSV(uint64_t time, uint16_t x_loc, uint16_t y_loc, uint16_t xSize, uint16_t ySize, uint16_t track_num, uint16_t label);

	public:
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
		MinMaxTrackerDisplay(bool run_SELMA);
    	MinMaxTrackerDisplay(std::string ip_add_in, int port_in);
    	MinMaxTrackerDisplay(std::string ip_add_in, int track_port_in, int evt_port_in); // Workaround for Synchronizing Event and Data Stream
		MinMaxTrackerDisplay(std::string output_filename_in); // For Generating Annotations for Tracker

};

#endif
const uint32_t invalid_marker_color = 0x00FF0000;
const uint32_t marker_color = 0x0000FF00;
const uint32_t inbetween_marker_color = 0x000000FF;
const uint32_t static_marker_color = 0x00FFFFFF;


//Write TD data to file
void init_write_video_TD(const char* filename);
bool write_video_TD(packet_info_type* packet_info, event* dataIN, bool exit_thread);

//Write APS data to file
void init_write_video_APS(const char* filename);
bool write_video_APS(packet_info_type* packet_info, event* dataIN, bool exit_thread);


#endif
