//ALL FILTERING FUNCTIONS GO IN THIS FILE
#ifndef FILTERS_H
#define FILTERS_H
#include <fstream>
#include "evt_tcp_publisher.h"

//#ifdef _useTN
//#include "TcpOstreamBuffer.h"
//#include "TnChannel.h"
//#endif

class rangecheck : public DataConsumer {
	public:
		rangecheck(){};
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};

// apply an offset to every TD event
class offset : public DataConsumer {
	private: 
		uint16_t x_offset,y_offset;
	public:
		offset(int x, int y);
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);

};

// This filter marks all filtered events as valid again
// Arguments: none
class unfilter : public DataConsumer {
	public:
		unfilter(){};
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};

// This filter removes all filtered events. It slows processing down because it needs to individually copy each event to an output buffer based on whether it has been filtered
// but it reduces file sizes which can be useful when reading an entire recording into Matlab
// Arguments: none
class discard_filtered : public DataConsumer {
	public:
		discard_filtered(){};
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};


// The standard nearest neighbour filter
// Arguments: 
// "input_NN_threshold" an integer specifying the time threshold in microseconds
class NN_filter : public DataConsumer {
	private:
		uint64_t** NN_last_spike;
		uint64_t NN_threshold;
	public:
		NN_filter(int input_NN_threshold);
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};

class NN_orientation_filter : public DataConsumer { 
	private:
		uint32_t** NN_orientation_last_spike;
		uint32_t NN_orientation_threshold;
	public:
		NN_orientation_filter(int input_NN_orientation_threshold);
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};

// The standard nearest neighbour filter with polarity (only considers events of the same polarity at neighbouring pixels)
// Arguments: 
// "input_NNp_threshold" an integer specifying the time threshold in microseconds
class NN_filter_polarity : public DataConsumer {
	private:
		int numFilteredEvents = 0;
		int numNotFilteredEvents = 0;
		int NNp_threshold;
		uint64_t*** NNp_last_spike;

	public:
		NN_filter_polarity(int input_NNp_threshold);
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};


// The standard nearest neighbour filter, but looking at a neighbourhood including pixels located 2 pixels away
// Arguments: 
// "input_NN2_threshold" an integer specifying the time threshold in microseconds
class NN2_filter : public DataConsumer{
	private:
		uint64_t NN2_threshold;
		uint64_t** NN2_last_spike;
	public:
		NN2_filter(int input_NN2_threshold);
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};


// The standard nearest neighbour filter, but only considering horizontal and vertical neighbours (not diagonal)
// Arguments: 
// "input_NNhv_threshold" an integer specifying the time threshold in microseconds
class NNhv_filter : public DataConsumer{
	private:
		uint32_t** NNhv_last_spike;
		uint32_t NNhv_threshold;

	public:
		NNhv_filter(int input_NNhv_threshold);
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};

// A refractory period filter, blocks all events within the refractory time of a previous event
// Arguments: 
// "input_RF_period" an uint32_teger specifying the refractory period in microseconds
class refractory_filter : public DataConsumer{
	private:
		uint32_t RF_period;
		uint32_t** RF_last_spike;
	public:
		refractory_filter(int input_RF_period);
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};

// A refractory period filter similar to above, but only blocks events of the same polarity within the refractory period
// Arguments: 
// "input_RFp_period" an integer specifying the refractory period in microseconds
class refractory_period_polarity : public DataConsumer{
	private:
		uint32_t RFp_period;
		uint32_t*** RFp_last_spike;
	public:
		refractory_period_polarity(int input_RFp_period);
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};


// A "dumb" filter which blocks an event if it has the same polarity as the previous valid event from the same pixel
// Arguments: none
class diff_polarity_filter : public DataConsumer{
	private:
		char** last_polarity;
	public:
		diff_polarity_filter();
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};

// A refractory period filter similar to above, but only blocks events of the same polarity within the refractory period
// Arguments: 
// "input_RFp_period" an integer specifying the refractory period in microseconds
class ISI_filter : public DataConsumer{
	private:
		uint32_t ISI_threshold;                   //ISI filter 
		float ISI_mix;
		float** average_ISI;
		uint32_t** ISI_last_spike;
	public:
		ISI_filter(int input_ISI_threshold, float input_ISI_mix);
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};

//not tested
class filter_NNg : public DataConsumer{
	private:
		int NN_side_length;
		int NN_t_length;
		uint32_t** NNg_last_spike;
		int min_num_spikes;
		int current_y;
		int current_x;
		int current_t;
		int offset_x = 0;
		int offset_y = 0;
		int num_pixels_within_thresh;
	public:
		filter_NNg(int input_NN_side_length, int input_NN_t_length, int input_min_num_spikes);
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};


class filter_ROI : public DataConsumer{
	private:
		int ROI_x, ROI_y, ROI_x_size, ROI_y_size;
	public:
		filter_ROI(int x, int y, int x_size, int y_size);
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};

class filter_ROImask : public DataConsumer{
	private:
		char *mask;
	public:
		filter_ROImask(std::string mask_file);
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};

class change_type : public DataConsumer{
	private:
	int from, to;
	public:
		change_type(int from_in, int to_in);
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);	
};

//#ifdef _useTN
//class tn_filter : public DataConsumer{
//private:
//	tnr::TnChannel * channel;
//	uint64_t last_tick_time_ms;
//	tnt::TcpOstreamBuffer * os_buffer;
//	std::ostream * os;
//	int width;
//	int height;
//	int num_channels;
//	void init(std::string model_path, std::string input_conn_path_in, std::string output_conn_path_in, int width_in, int height_in, int num_channels_in);
//	void send_event_to_tn(event evt);
//
//public:
//	tn_filter(std::string model_path, std::string input_conn_path_in, std::string output_conn_path_in, int width_in, int height_in, int num_channels_in, std::string output_sfcp_path_in);
//	tn_filter(std::string model_path, std::string input_conn_path_in, std::string output_conn_path_in, int width_in, int height_in, int num_channels_in, std::string output_ip_in, int output_port_in);
//	bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
//};
//
//#ifdef _useATIS
//class tn_usb : public DataConsumer{
//private:
//	tnr::TnChannel * channel;
//	uint64_t last_tick_time_ms;
//	int * class_map;
//	int * confidence_matrix; //accumulated spikes from TN
//	int width;
//	int height;
//	int num_channels;
//	void init(std::string model_path, std::string input_conn_path_in, std::string output_conn_path_in, int width_in, int height_in, int num_channels_in);
//	void send_event_to_tn(event evt);
//
//public:
//	tn_usb(std::string model_path, std::string input_conn_path_in, std::string output_conn_path_in, int width_in, int height_in, int num_channels_in, std::string class_map_path);
//	bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
//	~tn_usb();
//};
//#endif // _useATIS
//#endif // _useTN
#endif
