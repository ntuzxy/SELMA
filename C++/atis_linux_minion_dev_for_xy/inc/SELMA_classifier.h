#ifndef SELMA_CLASSIFIER_H
#define SELMA_CLASSIFIER_H

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <string>

#include "event_framework.h"
#include "../src/protobuf/tn_message.pb.h"
#include "SELMA_IF.h"

// SAFEGUARDS FOR BAD LOOPS
// #define SELMA_MAX_TOTAL_TIME 5000000 // MAX Waiting for 5s Terminate Program as not responsive if > 5s of wait time

// // Directives for SELMA-16 Chip Operation
// #define SELMA_NUM_INPUT_CHANNELS 1024
// #define SELMA_NUM_OUTPUT_CHANNELS 2048
// #define SELMA_NUM_CLASSES 6
// #define SELMA_NUM_BYTES_PER_CLASS 8
// #define SELMA_MIN_SLEEP_TIME 10000

// Directives for Classifier
#define SELMA_XSIZE 32
#define SELMA_YSIZE 32
#define SELMA_CLASS_UPDATE_PERIOD 100000
#define SELMA_MIN_TIME_TO_CLASSIFY 256000

// Directives for Tracking:
#define SELMA_NUM_TRACKERS 8
//ACTIVE TRACKERS
#define SELMA_INIT_RADIUS 48
#define SELMA_MIXING_FACTOR 0.95 // the mixing factor for active trackers
#define SELMA_TIME_MIXING_FACTOR ((double)0.95)
#define SELMA_TDIFF_THRESHOLD 1000 //if active this determines whether the tracker remains active. Higher is more likely to remain active

//INACTIVE TRACKERS
#define SELMA_MIXING_FACTOR_2 0.95 // // the mixing factor for inactive trackers
#define SELMA_TDIFF_THRESHOLD_2 1000 //if inactive this determines whether the tracker becomes active
#define SELMA_DISTANCE_THRESHOLD_2 200000 //for active trackers this is the distance threshold for assigning an event
#define SELMA_MAX_DISTANCE 2000000 //arbitrarily large number used as the initial value for minimum distance
#define SELMA_TRACKER_UPDATE_PERIOD 50000

// TRACKER VISUALIZATION
#define active_time_threshold 2000000
#define tracker_update_period 30000
#define average_distance_threshold 0 //square value for the average distance a tracker should move between cleanups to be considered valid. 100 means 10 pixels per "tracker_update_period" microseconds
#define SELMA_PKT_HEADER_SIZE 4

// TRACKING STRUCTURES
struct selma_tracker_type {
    double loc[2];
    double last_loc[2];
    double dist[2]; //how far does it move on average
    double tdiff_mean;
    uint64_t last_event;
    bool active;
    uint64_t active_time;
    uint16_t radius;
    uint32_t colour;
    uint8_t id;
    uint8_t label;
    // uint64_t stage2_data[SELMA_NUM_CLASSES];
};
// #History of 10 classificaiton (Do Voting Over) (Accumulate stage2_data)
// Trial 

// enum PipeOutAddress_SELMA
// {
// 	RO_F1DataIn = 0xA1,
// 	RO_F2DataIn = 0xA2
// };

// enum PipeInAddress_SELMA
// {
//     SELMA_IF_PipeIn                  = 0x8A,     // FIFO PIPEIN - 1024 delay values 
//     COMMS_IF_OBJ_ID_FIFO_ADDR = 0x92,     // OBJ_ID - 1 8-bit word
//     COMMS_IF_X_ADDR           = 0x93, // X_POS  - 1 16-bit word
//     COMMS_IF_Y_ADDR           = 0x94, // X_ADDR - 1 16-bit word
//     COMMS_IF_WIDTH            = 0x95,  // WIDTH  - 1 16-bit word
//     COMMS_IF_HEIGHT           = 0x96, // HEIGHT - 1 16-bit word        
//     COMMS_IF_CLASSDATA        = 0x97, // CLASS_DATA(0-5) - 24 16-bit words
// };

// enum WireOutAddress_SELMA
// {
//     selma_status = 0x28
// };

// enum SelmaStatusBits 
// {
//     SelmaIfConfigActive = 2,  // Indicates the active status of the config sequence. (This signal goes back to '0' after the completion of config sequence.)
//     SelmaIfFeedinActive = 4,  // Indicates the active status of the feedin sequence. (This signal goes back to '0' after the completion of feedin sequence.)
//     SelmaIfConfigDone = 8,    // Indicates the completion of the config sequence. (This signal stays at '1', till cleared by the 'SelmaIfClearConfig' control bit.)
//     SelmaIfFeedinDone = 16,    // Indicates the completion of the feedin sequence. (This signal stays at '1', till cleared by the 'SelmaIfClearFeedin' control bit.)

//     SelmaIfROEnable = 32,   // Indicates Readout Stage is completed
//     SelmaIfRODone   = 64,    // Indicates the readout data from fifo1 and fifo2 has been read
//     SelmaIfROFifo1Empty = 128,  // readout fifo1 empty
//     SelmaIfROFifo2Empty = 256   // readout fifo2 empty
// };

// enum TriggerInAddress_SELMA
// {
// 	SelmaTriggers = 0x50
// };

// enum SelmaTriggerWires
// {
//     SelmaIfReset = 0,       // Resets the SELMA interface module
//     SelmaIfStartConfig = 1, // Starts the SELMA configuration sequence with the configuration from the built-in ROM
//     SelmaIfStartFeedin = 2, // Starts the SELMA spike feedin sequence with the preconfigured spike rate. (Currently this is 5 spikes per input. Once we test the basic interface, I will add spike input pipe from C++ to SELMA IF)
//     SelmaIfClearConfig = 3, // Clears the 'SelmaIfConfigDone' flag. This should be done before initiating next config sequence.
//     SelmaIfClearFeedin = 4, // Clears the 'SelmaIfFeedinDone' flag. This should be done before initiating next feedin sequence.
// };

static selma_tracker_type *trackers;

class SELMA_classifier: public DataConsumer 
{
public:
	
	SELMA_classifier();
    SELMA_classifier(std::string ip_add_in, int port_in);
    SELMA_classifier(std::string test_folder, std::string csv_filename); // For Passing Data Through SELMA
    SELMA_classifier(std::string ip_add_in, int track_port_in, int evt_port_in); // Workaround for Synchronizing Event and Data Stream
    SELMA_classifier(std::string output_filename_in); // For Generating Annotations for Tracker

	bool processData(packet_info_type* packet_info, event* dataIN,
			bool exit_thread);

private:

	const std::string logPrefix = "SELMA_CLASSIFER: ";

	// Main Flow Functions    
	void initSELMA(); // Initializes SELMA Configuration Process and Run Some Test Inputs to ensure boards are working
	void configSELMA(); // Starts SELMA Configuration Process
	bool testSELMA(); // Runs Some Test Inputs to ensure Boards are responsive to Inputs // Returns true if results are not as expected, false otherwise
	std::pair<bool,bool>  testSELMAFull(); // Same as testSELMA but also Verifies 2nd Stage Multiplier (Assumes Coefficients is 6,5,4,3,2,1) 
    void initClassificationState(); // Updates Classification State
    void initTrackerState(int num_tracks);

    void processEvent(event* evt, uint64_t time); // Updates Classification and Tracking State
    void updateClassificationState(event* evt, uint64_t time); // Updates Classification State


    std::pair<uint16_t*,int64_t*> sendInputToSELMA(uint16_t delays[]);    

    // SELMA Specfic Utility Functions
    void waitSelmaStatusBit(uint32_t STATUS_BIT, bool STATUS_VALUE); // Busy waits for SELMA Status Bit to be active 
    int clearDoneFlag(uint32_t DoneFlag, uint32_t ClearFlagTrigger); // Clears SELMA Status Flag 

    // General Event-based Utility Functions 
    uint16_t getAddrFromXY(uint16_t x_addr, uint16_t y_addr); // Conversion from x y to A
    std::pair<uint16_t, uint16_t> scaleToSize(uint16_t evt_x, uint16_t evt_y, uint16_t xSize, uint16_t ySize);
    uint16_t getDelayVal(uint16_t num_spk); // Spike Rate to Delay Value(SELMA_IF Input) Conversion 
    void convertEventRateToDelays(uint16_t* event_rate, uint16_t* delays, uint32_t array_length);
    std::string convertOpalKellyErrorCodeToString(int ErrorCode);
    void printOpalKellyReturnToString(std::string logPrefix, std::string topic, long returnCode);

    
    // Classification Functions:
    uint8_t next_id;
    uint8_t overflow_ctr;
    uint16_t** event_rates;// "Flattened Frames for Classification"
    // int** coefficients;
    uint64_t last_classification_time;
    void convertStage2toConfidence(int64_t* stage2_data, uint8_t* confid);

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

    // TRACKER : Copied From TN_Classifier
    int num_trackers;
    int *distance;
    uint64_t last_tracker_cleanup;
    uint16_t distance_threshold; // 400
    int track_to_classify;
    void calculate_distances(event event_in, int* distance);
    int assign_tracker(int* distance);
    void update_tracker(event event_in, uint64_t time, selma_tracker_type* tracker);
    void track(packet_info_type* packet_info, event* dataIN);

    // File Writing and Sending Data to SELMA
    bool isDonePassingData = false;
    void writeHiddenLayerOutToFile(uint16_t stage1_data[], std::string filename, int num_data);
    void writeHiddenLayerOutToFile(int64_t stage1_data[], std::string filename, int num_data);
    void passDataThroughSELMA(std::string test_folder, std::string csv_filename);

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
	
	//Copied From MinMax Tracker in AER_Display
	static int g_track_id;
	static int prev_track_id;
};
#endif 

#ifdef _useSDL
#include <AER_display.h>

static SDL_Window *selmac_track_window = NULL;
static SDL_Renderer* selmac_track_renderer;
static SDL_Texture* selmac_track_texture;
static uint32_t * selmac_track_pixels;

//int init_showTD_track();
class TDdisplay_track_SELMAC : public DataConsumer, public display {
    private:
        uint64_t last_TDtracker_display_time;
    public: 
        TDdisplay_track_SELMAC();
        bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};
#endif
