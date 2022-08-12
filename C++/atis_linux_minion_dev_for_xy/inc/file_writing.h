#ifndef FILE_WRITING_H
#define FILE_WRITING_H

#include <stdio.h>
#include <stdlib.h> 
#include <sys/time.h>
#include <unistd.h>
#include <limits.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <dlfcn.h>
#include <event_framework.h>

/************************************************************************************************************
Input      : Input filename and path
Output     : Returns 1 if successful, 0 if otherwise.
Description: A function to obtain the input file name and path for recordings
*************************************************************************************************************/
class file_write : public DataConsumer {
	private:
		std::ofstream output_file;
		std::string path_with_filename_base_internal;
		bool first_call_init_write_to_file = true;

		bool startedRecording = false;
		bool limit_record_time_per_file = false;
		bool file_end_time_overflowed = false;
		uint64_t file_end_time = 0;
		uint32_t time_per_file = 0;
		uint32_t file_number = 0;
		void limit_file_write_time(unsigned int write_time);
	public: 
		file_write(){};
		void set_parameters(const char* path_with_filename, const char* comments, unsigned int write_time);
		bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};

#endif
