#ifndef TRACKING_H
#define TRACKING_H

#include <fstream>

#include <event_framework.h>
#include <AER_display.h>

#ifdef _useSDL
	static SDL_Window *track_window = NULL;
	static SDL_Renderer* track_renderer;
	static SDL_Texture* track_texture;
	static uint32_t *track_pixels;
#endif
//ACTIVE TRACKERS
#define mixing_factor 0.95 // the mixing factor for active trackers
#define time_mixing_factor ((double)0.95)
//#define tdiff_threshold 50000 //if active this determines whether the tracker remains active. Higher is more likely to remain active
#define tdiff_threshold 10000 //if active this determines whether the tracker remains active. Higher is more likely to remain active
//#define distance_threshold 400 //for active trackers this is the distance threshold for assigning an event
//#define distance_threshold 16

//INACTIVE TRACKERS
#define mixing_factor_2 0.95 // // the mixing factor for inactive trackers
//#define time_mixing_factor_2 ((double)0.9)
#define tdiff_threshold_2 1000 //if inactive this determines whether the tracker becomes active
#define distance_threshold_2 200000 //for active trackers this is the distance threshold for assigning an event
#define max_distance 2000000 //arbitrarily large number used as the initial value for minimum distance

#define tdiff_max 2000000.0 // a maximum cap on tdiff

#define active_time_threshold 2000000

#define tracker_update_period 50000

#define average_distance_threshold 0 //square value for the average distance a tracker should move between cleanups to be considered valid. 100 means 10 pixels per "tracker_update_period" microseconds

enum TD_tracker_subtype{loc, std_dev};

struct tracker_type
{
	double loc[2];
	double last_loc[2];
	double dist[2]; //how far does it move on average
	double tdiff_mean;
	int last_event;
	bool active;
	int active_time;
	uint32_t colour;
	uint16_t radius;
};

//these variables need to be accessed by both track and TDdisplay_track
static tracker_type *tracker;
static int num_trackers;

class track : public DataConsumer {
	private:
		int *distance;
		uint64_t last_tracker_cleanup;
		uint16_t distance_threshold;// 400
		//uint16_t radius;// 20
	public:
		track(uint16_t num_trackers_passed, uint16_t radius_passed);
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};
//A variation of the normal display function which includes trackers

#ifdef _useSDL
//int init_showTD_track();
class TDdisplay_track : public DataConsumer, public display {
	private:
		uint64_t last_TDtracker_display_time;
	public: 
		TDdisplay_track();
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};
#endif


//these functions are no longer implemented 
void init_write_video_TD_track(const char* filename);
bool write_video_TD_track(packet_info_type* packet_info, event* dataIN, bool exit_thread);


//static std::ofstream tracker_output_file;


#endif
