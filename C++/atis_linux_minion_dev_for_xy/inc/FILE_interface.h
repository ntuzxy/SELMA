#ifndef FILE_INTERFACE_H
#define FILE_INTERFACE_H
#include <fstream>

#include <event_framework.h>

class FileSource : public DataSource {
	private:	
		std::ifstream input_file;
		long input_file_size;

		//variables for keeping track of time
		struct timeval t_start, t_now;
		int64_t time_elapsed_microseconds;
		float set_real_time_multiplier;
		bool isRealTime;
		bool read_more_data;
		bool time_initialized;
		uint64_t offset_time; 
		uint16_t file_version;
		packet_info_type packet_info_INTERNAL;
		event* dataINTERNAL;
		int init_input_file_subfunction(const std::string path_with_filename, std::string& comments);
	public:	
		FileSource(source_parameters_type &parameters); //initializer
		bool getData(packet_info_type* packet_info, event* dataOUT, bool exit_thread); //getData	
};



class FilePartsSource : public DataSource {
	private:	
		std::ifstream input_file;
		long input_file_size;

		//variables for keeping track of time
		struct timeval t_start, t_now;
		uint64_t time_elapsed_microseconds;
		int set_real_time_multiplier;
		bool isRealTime;
		bool read_more_data;
		bool time_initialized;
		uint64_t offset_time; 
		uint16_t file_version;
		packet_info_type packet_info_INTERNAL;
		event* dataINTERNAL;
		std::string basename;
		int file_number;
		int init_input_file_subfunction(const std::string path_with_filename, std::string& comments, bool populate_comments);
	public:	
		FilePartsSource(source_parameters_type &parameters); //initializer
		bool getData(packet_info_type* packet_info, event* dataOUT, bool exit_thread); //getData	
};

#endif
