#ifndef EVENT_FRAMEWORK_H
#define EVENT_FRAMEWORK_H

#include <stdint.h>
#include <string>
#include <fstream>
#include <sstream>
#define max_read_length 8192

//struct declaration
//extern int x_dim, y_dim;

enum evt_type{
	TD = 0,
	EM = 1,
	FE = 240, //Frame Event for Davis the Paket Number is 2 but had to choose a unique number in evt_type
	timer_overflow = 2, 
	TD_filtered = 3,  
	EM_filtered = 4, 
	TD_tracker = 5,
	Orientation = 6, 
	TD_Feature = 7, 
	trigger = 8, 
	Orientation_filtered = 9, 
	Tracked = 10, 
	IMU6 = 11,
	TrackerID = 12,
	optical_flow = 20,
	stabilized = 100,
	ms_tick = 250,
	Reset = 253,
	invalid = 254,
	quit = 255,
};

enum TD_subtype{ON = 0, OFF = 1, out_of_range = 2};//TD_subtype;
enum EM_subtype{first_thresh = 0, second_thresh = 1};//EM_subtype;
enum trigger_subtype{start = 0, stop = 1};//TD_subtype;
enum optical_flow_subtype{location = 0, cartesian = 1, polar = 2};
enum TD_tracker_subtype_types{CENTER = 3, MIN = 0 , MAX = 1, ID = 2};
//enum TD_tracker_subtype_mask{ACTIVE =0x80, TYPE = 0x60, TRACK_NUM =0x1F, SUBTYPE=0xE0, BITSHIFT_AMT_TYPE = 5, BITSHIFT_AMT_ACTIVE = 7};
enum TD_tracker_subtype_mask{ TRACKER_MASK = 0xF0, TRACK_MASK =0x0F, BITSHIFT_AMT_TYPE = 4};

enum string_code { //enumerate the function names here
    eF_NN,
	eF_NN2,
	eF_NNhv,
	eF_NNp,
	eF_ISI,
	eF_undo,
	eF_RF,
	eF_RFp,
	eF_DP,
	eF_NNg,
	eF_ONN,
	eF_reform,
	eF_ROI,
	ef_ROImask,
	eF_NN_fpga,
	eF_RF_fpga,
	eF_offset,
	eF_TNF, // TN -> File
	eF_TNT, // TN -> TCP
	eF_TNC, // TN classifier
	eF_ETP, // Events -> TCP
	eF_SELMAC,
	eF_SELMAC_RUNDATA,
	eF_SELMAC_WRITEANNOT,
	e_testSELMA,
	eD_TD,
	eD_APS,
	eD_OR,
	writevideo_TD,
	writevideo_APS,
	writevideo_OR,
	eIO_write,
	eIO_read,
	eIO_realtime,
	eIO_biases,
	eIO_bitfile,
	eIO_limit_file_time,	
	eTime,
    bias18, 
    bias33,
	version4,
	eF_rangefix,
	eAPS,
	eIMU,
	eT_TR,
	eD_TD_track,
	eD_TD_SELMACtrack,
	eD_TD_MMtracker,
	eD_TD_MMtracker_SELMAC,
	eD_TD_MMtracker_WRITEANNOT,
	writevideo_TD_track,
	eO_GAB,
	#ifdef _useROS
	eR_TD,
	eR_track,
	eR_APS,
	eR_IMU,
	eR_Config,
	eROS_IN,
	#endif
	edavis240C,
	edavis640,
	eATIS,
	eIO_readparts,
	eNewThread,
	eForceSize,
	edefault,
	eSimulate,
	eTN,
	eEKFloc,
	eD_OF,
	eOF_fpga,
	eF_remove_fpga,
	eCorner,
	echange_type,
	e_ms_tick,
	e_Error
};

struct event
{
	unsigned char type;
	unsigned char subtype;
	//unsigned char y;	
	int16_t y;
	int16_t x;
	uint16_t t;
};

enum IMU_type{
	accel_x,
	accel_y,
	accel_z,
	gyro_x,
	gyro_y,
	gyro_z,
	temp,
	// The following are only present in IMU9 events (unavailable in DAVIS240C but here for future compatibility)
	comp_x,
	comp_y,
	comp_z
};

enum packet_type{
	unassigned_packet = 0,
	TD_EM_mixed_packet = 1,
	TD_packet = 2,
	EM_packet = 3,
	FRAME_packet = 4,
	timer_reset_packet = 5,
	IMU_packet = 6,
	DAVIS_SPECIAL_packet = 240
};


struct packet_info_type
{
	uint32_t num_events;
	uint32_t time_start;
	uint16_t packet_type;
	//uint8_t reserved; // reserved for possible future use
	uint16_t packet_data; // reserved for information specific to the packet type. For example, this holds the 16 lsbs of timestamp for a DAVIS frame packet.
};

// Types of device that can be used as a source
enum device_code{atis, davis240C, file, fileparts, ros_in, davis640};

// Parameters used for source initialization
// Take the default values away from header in future okay
struct OF_params
{
	unsigned char refrac_threshold;
	unsigned char old_pixel_threshold;
	unsigned char fit_distance_threshold;
	unsigned char goodness_threshold;
	unsigned char num_pixels_threshold;
	bool enable_OF;
	bool filter_by_goodness;
	bool cartesian_not_polar;
};

struct source_parameters_type
{
	OF_params* OF;
	device_code source_type;
	int x_dim;
	int y_dim; // X and Y dimensions of the source (resolution)
	float pix; //this pixel size (eg. 30e-6 for ATIS)
	std::string fpga_bitfile; // Location of the bitfile to program fpga
	std::string input_file; // Location of the input file when -fromfile is specified
	std::string ros_input_topic; // Name of the topic to subscribe to if Ros input is specified (/nm/epf/ros_input_topic/td_in)
	std::string biasfile; // File containing the programming biases
	std::string simulationfile; // Input file to be used for simulation
	std::string comments; // Comments used for documentation, file writing etc. Would be nice in future to change it to string to avoid them nasty segfaults
	bool autodetect_serial_number = true; // override if we are setting a custom serial number for the camera. true if it autodetected
	int camera_serial_number; // Serial number of the camera (DAVIS), needed if we want to connect to a specific one (i.e. for stereo applications)
	bool useGrayscale = false; // Self-explanatory
	bool realtime_playback = false; // If we want the playback to be in realtime or not
	bool useIMU = false; // For now only used for Davis (which has onboard IMU)
	bool davis_overwrite_default_biases = false; // Set to true if we want to overwrite the default biases
	bool simulate = false; //set to true to pipe pre-recorded data through the OK FPGA as if it were coming from ATIS
	int read_parts_starting_file = 1; // File number to start for read from file parts source

	float realtime_speedup_factor; // Defines the speedup of realtime playback (i.e. 2 means double speed)

	int atis_version = 6; // Version of the atis board

	//For now these are ATIS specific parameters for pushing filtering into the FPGA
	bool F_NN_enable = false;
	uint16_t F_NN_time = 0;
	bool F_RF_enable = false;
	uint16_t F_RF_time = 0;
	bool F_remove = 0; // to set the FPGA to remove filtered events instead of just tagging them as filtered
	bool ms_tick_enable = 0;
	bool enableTN = false; //determines whether the TN hardware interface is enabled for ATIS
};

string_code hashit (std::string const& inString); //here is where the strings get assigned

class DataSource {
	public:
		static int x_dim, y_dim;
		static float pix;
		virtual bool getData(packet_info_type* packet_info, event* dataOUT, bool exit_thread) = 0;
		DataSource(int x, int y, float p){x_dim = x; y_dim = y; pix = p;}
};

class DataConsumer {
	public:
		virtual bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread) = 0;
};
#endif
