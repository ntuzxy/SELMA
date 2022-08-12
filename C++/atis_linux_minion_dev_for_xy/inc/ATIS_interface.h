#ifndef ATIS_INTERFACE_H
#define ATIS_INTERFACE_H

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include "OK_interface.h"
#include "event_framework.h"

//#include <opencv.hpp>

//using namespace cv;
//#define max_events_to_process 20000000 // if there are more than this number of events, the processing and display will only use every 2nd/3rd/4th event etc
                                      //#define display_interval 100000 //minimum number of microseconds to occur between frames

enum ControlSignalBits
        {
            couple =     1, //1<<0
        	sequential = 2, //1<<1
            LIFUdownB = 4, //1<<2
            APS_ROI_en = 8, //1<<3
            TD_ROI_en = 16, //1<<4
            remove_filtered = 32, //1<<5
            shutter = 64, //1<<6
            output_driver_default = 128, //1<<7
            BGenPower_Up  = 256, //1<<8
            ROI_TD_inv = 512, //1<<9
            TD_enable = 1024, //1<<10
            APS_enable = 2048, //1<<11
            Refraction_enable = 4096, //1<<12
            Filter_enable = 8192, //1<<13
            TN_enable = 16384, //1<<14
            enable_simulation = 32768 //1<<15
        };

enum ControlSignalBits2
{
	ms_tick_enable = 1
};

/*
enum ControlSignalBits
        {
            couple =     1<<0,
        	sequential = 1<<1,
            LIFUdownB = 1<<2,
            APS_ROI_en = 1<<3,
            TD_ROI_en = 1<<4,
            shutter = 1<<6,
            output_driver_default = 1<<7,
            BGenPower_Up  = 1<<8,
            ROI_TD_inv = 1<<9,
            TD_enable = 1<<10,
            APS_enable = 1<<11,
            Refraction_enable = 1 << 12,
            Filter_enable = 1 << 13,
            TN_enable = 1 << 14,
            enable_simulation = 1<<15
        };

*/
enum PipeOutAddress{
			EventsIn = 0xA0
		};

enum PipeInAddress
        {
            HMAX        = 0x80,
            //HMAX uses 0x80, 0x81, 0x82
            Motor       = 0x83,
            Biases      = 0x90,
            ROI         = 0x91,
            sim         = 0x88
        };

enum WireOutAddress{
	event_RAM_pages = 0x20, 
	sim_RAM_pages = 0x25
	};

enum WireInAddress
        {
            ControlSignals	 		= 0x00,
			ControlSignals2			= 0x04,
            WireInEvt           	= 0x09,
            Refractory_threshold   	= 0x10,
            Filter_threshold   	= 0x11,
            OF_control          = 0x12,
            OF_refrac_threshold = 0x13,
            OF_old_pixel_threshold = 0x14,
            OF_fit_distance_threshold = 0x15,
            OF_goodness_threshold = 0x16
        };

enum TriggerInAddress{ControlTriggers = 0x40};

enum TriggerWires{
			DAC_reset    		= 0,
            v4BiasValid_bit     = 1, 
            Reset_module_fifo   = 2,
            v4ROIdataValid      = 3,
            v4ROIprogram        = 4,
            reset_event_chain   = 7,
            WireInEventValid    = 10};


enum BiasName{ CtrlbiasLP,CtrlbiasLBBuff,CtrlbiasDelTD,CtrlbiasSeqDelAPS,CtrlbiasDelAPS,
               biasSendReqPdY,biasSendReqPdX,CtrlbiasGB,
               TDbiasReqPuY,TDbiasReqPuX,
               APSbiasReqPuY,APSbiasReqPuX,APSVrefL,APSVrefH,
               APSbiasOut,APSbiasHyst,APSbiasTail,
               TDbiasCas,TDbiasInv,TDbiasDiffOff,
               TDbiasDiffOn,TDbiasDiff,TDbiasFo,TDbiasRefr,TDbiasPR,TDbiasBulk,
               biasBuf,biasAPSreset};

struct bias{
    unsigned int voltage;
    unsigned int register_value;
    unsigned int polarity;
};





class AtisSource : public DataSource {

		std::vector<bias> BiasLookup18;  //VDAC 1.8V lookup table
		std::vector<bias> BiasLookup33;  //VDAC 3.3V lookup table

		//Array that contains information on which lookup table to reference 
		//to for all 28 bias registers. Refer to enum in ATIS_interface.h for 
		//the list of registers with respect to the specification.
		const int BiasType[28] = {1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
		int BiasData[28]; //Array to carry to voltage values after they are read from the bias control file 
		
        

		unsigned char* new_sim_event_bytes;
		unsigned char* old_sim_event_bytes;
		bool simulate;
		std::ifstream sim_input_file;
		uint64_t sim_input_file_size;
		event sim_overflow_event;
		std::string simulationfile;
		uint64_t sim_send_bytes_count, old_sim_byte_count, sim_time;
		packet_info_type sim_packet_info;
		event* sim_packet;

        uint32_t simulation_start_time_offset;
        bool first_packet;
		int initSimulation();
		int send_sim_data();
		int read_sim_packet();
		int encode_sim_packet();

		bool atis_quit; //flag indicating to shut down the program

		uint16_t ControlSignalWire;     //FPGA wire signal control
		uint16_t ControlSignalWire2 = 0;

		bool v4; // a flag specifying whether this is v4 of ATIS (by default it is v6 if v4=false)
		bool enable_APS;
		int initATIS_IO();	
		int writeBiases_v4();		
		int writeBiases_v6(unsigned char* bias_bytes);
		void createBiasData(int *biasData, char *biasStr);
		uint32_t LSB_overflow_counter;    //Counter to keep track of timestamp overflow in the event decoding module
		uint32_t MSB_overflow_counter;    //Counter to keep track of timestamp overflow in the event decoding module
		
		unsigned char event_bytes[max_read_length*1024]; //pre-allocated buffers to prevent stack overflow
		uint32_t decodeATISdata(uint32_t num_bytes, unsigned char* dataIN, event* dataOUT, packet_info_type* packet_info);
		int readBiasesFromFile(const char* biasfile_path, const char* bias_lookup_18, const char* bias_lookup_33, unsigned char* bias_bytes);
		int populateLookupTables(const char* lookup_table_path, bool is1800);
		int returnBiasfromVoltage(int voltage, BiasName bName, struct bias& b);
		int programBiasfromText_v6(const char* biasfile_path,const char* bias_lookup_18, const char* bias_lookup_33);
		uint32_t num_bytes_to_decode, num_bytes_decoded;
		bool get_new_data;
	public:
		AtisSource(source_parameters_type &parameters);
		bool getData(packet_info_type* packet_info, event* dataOUT, bool exit_thread);
};


#endif
