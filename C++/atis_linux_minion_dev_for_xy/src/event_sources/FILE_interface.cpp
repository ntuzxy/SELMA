#include <sys/time.h>
#include <cstring>

#include <FILE_interface.h>

/*
class FileSource {
	//private:	
		static std::ifstream input_file;
		static long input_file_size;

		//variables for keeping track of time
		static struct timeval t_start, t_now;
		static int time_elapsed_microseconds;
		static bool isRealTime;
		
		static bool time_initialized;
		static unsigned int offset_time; 
		
		int init_input_file_subfunction(const char* path_with_filename_char, char* comments);
	public:	
		FileSource(const char* src_file, char* comments, bool setRealTime); //initializer
		bool getData(packet_info_type* packet_info, event* dataOUT); //getData	
};
*/

FileSource::FileSource(source_parameters_type &parameters)
: DataSource(304,240, 30e-6)
{
//	std::string filename = std::string(src_file, strlen(src_file));
	//packet_info_INTERNAL = new packet_info_type;
	dataINTERNAL = new event[max_read_length*(1024/4)];
	isRealTime = parameters.realtime_playback;
	set_real_time_multiplier = parameters.realtime_speedup_factor;
	offset_time = 0;
	read_more_data = true;
	time_initialized = false;
	gettimeofday(&t_start, NULL);
	gettimeofday(&t_now, NULL);
	time_elapsed_microseconds = 0;
	if(!init_input_file_subfunction(parameters.input_file.c_str(), parameters.comments))
	  	printf("file initialization failed\n");//return 1;
        else 
          	printf("file initialization successful\n");//return 0;
}

/************************************************************************************************************
Input      : Input filename and path
Output     : Returns 1 if successful, 0 if otherwise.
Description: A function to perform the auxillary function of reading the recorded bin file headers before the 
*************************************************************************************************************/
int FileSource::init_input_file_subfunction(const std::string path_with_filename, std::string& comments)
{

	printf("Will read from source file: %s\n", path_with_filename.c_str());

	input_file.open(path_with_filename.c_str(), std::ios::in | std::ios::binary);

	if (input_file.is_open())
		printf("Opened source file %s\nPrinting out file comments below:\n", path_with_filename.c_str());
	else 
	{
		printf("\n ERROR: input file not found    %s\n\n", path_with_filename.c_str());
                return 0;
	}
	int file_start_position = input_file.tellg();

	char file_version_chars[10]; //must be 3 to leave enough space for the end line character
	input_file.get((char*)file_version_chars, 10, '\n');
	if (file_version_chars[0] == '#')
		file_version = 0;
	else
		if (file_version_chars[0] == 'v')
			file_version = atoi(&file_version_chars[1]);
		else
			printf("ERROR: Cannot detect file version\n");

	printf("file is version %i\n", file_version);
	input_file.ignore(10000, '\n');
	while(input_file.peek() == '#')
		input_file.ignore(10000, '\n');
	int comments_end = input_file.tellg();
	
	//input_file.ignore(10000, '\n');
	uint16_t dimension[5]; //must be 3 to leave enough space for the end line character
	input_file.getline((char*)dimension, 9, '\n');
	//printf("%i characters read\n", input_file.gcount());
	if(input_file.gcount() ==1)
	{
		printf("No dimension specification found in file, defaulting to ATIS resolution of %i x %i\n", 304, 240);
		DataSource::x_dim = 304;
		DataSource::y_dim = 240;
		DataSource::pix = 30e-6;
	}else
	{
		DataSource::x_dim = dimension[0];
		DataSource::y_dim = dimension[1];
		//if (char*(&dimension[2]) == '\n')
		if(input_file.gcount() == 5)
		{
			DataSource::pix = 30e-6;
			printf("no pixel size specified in file. Resorting to default of 30e-6 for ATIS. Use the \"-forcesize\" command to override\n");
		}
		else
			DataSource::pix = *(float*)(&dimension[2]);
		printf("Dimension specification from file is %i x %i pixels and pixel size of %f\n", dimension[0], dimension[1], DataSource::pix);
//		input_file.ignore(10000, '\n');
	}
//	free(dimension);



	

	int file_binary_start_position = input_file.tellg(); //where does the binary portion of the file start?

	char *tmp_com = (char*)malloc((comments_end-file_start_position+2)*sizeof(char));
	comments += '#';
	input_file.seekg(0, std::ios::beg); //go back to the start
	
	while (input_file.tellg() <  comments_end)
	{
		input_file.getline(tmp_com, 10000, '\n'); //read in the comments
		comments += tmp_com;
		comments += '\n';
	}
	
	printf("Printing comments\n");
	printf("%s", comments.c_str());
	free(tmp_com);

	input_file.seekg(0, std::ios::end); //go to the end of the file
	input_file_size = (long)(input_file.tellg()) - file_binary_start_position; //determine how long the binary portion of the file is
	input_file.seekg(file_binary_start_position); //go back to the start of the binary portion
	printf("file size is  %ld\n", input_file_size);
	return 1;
}


bool FileSource::getData(packet_info_type* packet_info, event* dataOUT, bool exit_thread)
{
	if((input_file_size > 0) & (exit_thread == false))
	{
		if(read_more_data == true)
		{
			//printf("reading more data\n");
			input_file.read((char*)packet_info, sizeof(packet_info_type));
			input_file_size -= sizeof(packet_info_type);
			input_file.read((char*)dataOUT, packet_info->num_events*sizeof(event));
			input_file_size -= packet_info->num_events*sizeof(event);
			if(file_version == 0)
			{
				packet_info[0].time_start = packet_info[0].time_start>>16;	
				packet_info->packet_type = packet_type::TD_EM_mixed_packet;
			//	printf("version 0: modified time_start\n");
			}
		}else
		{

			memcpy(packet_info, &packet_info_INTERNAL, sizeof(packet_info_type));
			memcpy(dataOUT, dataINTERNAL, sizeof(event)*packet_info->num_events);
			//printf("version 0: no need for new data, instead copied %i bytes of old data", packet_info->num_events);
		}
		
		if(file_version == 0)
		{
			//printf("version 0: checking for overflow packets\n");
			read_more_data = true;
			for(uint32_t i=0; i<packet_info->num_events; i++)
				if(dataOUT[i].type == evt_type::timer_overflow)
				{
					memcpy(&packet_info_INTERNAL, packet_info, sizeof(packet_info_type)); //copy the packet header for re-use with the next packet
					packet_info_INTERNAL.num_events-=(i+1); //but the number of events will be fewer
					packet_info_INTERNAL.time_start++; //and the start time must be incremented
					packet_info->num_events = i;
					memcpy(dataINTERNAL, &dataOUT[i+1], sizeof(event)*packet_info_INTERNAL.num_events); // copy the remaining output data for use in the next iteration
					read_more_data = false;
			//		printf("version 0: overflow packet found\n");
				}
		}
		
		if (time_initialized == false)
		{	
			time_initialized = true;
			offset_time = (uint64_t(packet_info[0].time_start)<<16) | dataOUT[0].t;
			//printf("read time initialized\n");
		}

		// THIS PART FORCES REAL TIME BASED ON SYSTEM CLOCK
		if (isRealTime == true) 
		{
			

			int64_t packet_start_time = ((int64_t(packet_info[0].time_start)<<16) | dataOUT[0].t) - offset_time;
			
			packet_start_time = packet_start_time/set_real_time_multiplier;
			while (time_elapsed_microseconds <= packet_start_time) //loop until the current time matches or exceeds the packet start time
			{
				gettimeofday(&t_now, NULL);				
				time_elapsed_microseconds = int64_t((t_now.tv_sec - t_start.tv_sec)*1000000 + (t_now.tv_usec - t_start.tv_usec));
				//printf("must wait another %ld us before continuing\n", (packet_start_time-time_elapsed_microseconds));	
				//time_elapsed_microseconds = int(t_now.tv_usec - t_start.tv_usec);
			}
		}
		//printf("%d events, %d microseconds from source file\n", packet_info->num_events, (packet_info->time_end-packet_info->time_start+dataOUT[packet_info->num_events-1].t-dataOUT[0].t));
	}
	else
	{
		printf("END OF INPUT FILE REACHED \n");
		printf("input_file_size is %li and exit_thread is %i \n", input_file_size, exit_thread);
		packet_info[0].num_events = 0;
		printf("Exit threads initiated\n");
		gettimeofday(&t_now, NULL);				

		int64_t packet_start_time = ((int64_t(packet_info[0].time_start)<<16) | dataOUT[0].t) - offset_time;
		time_elapsed_microseconds = int64_t((t_now.tv_sec - t_start.tv_sec)*1000000 + (t_now.tv_usec - t_start.tv_usec));
		printf("%lli microseconds elapsed while processing %lli microseconds of data\n", time_elapsed_microseconds, packet_start_time);
		input_file.close();
		return 1; //indicates that an exit command has occurred	
	}
	return 0;
}


FilePartsSource::FilePartsSource(source_parameters_type &parameters)
: DataSource(304,240,30e-6)
{
//	std::string filename = std::string(src_file, strlen(src_file));
	//packet_info_INTERNAL = new packet_info_type;
	dataINTERNAL = new event[max_read_length*(1024/4)];
	isRealTime = parameters.realtime_playback;
	set_real_time_multiplier = parameters.realtime_speedup_factor;
	offset_time = 0;
	read_more_data = true;
	time_initialized = false;
	gettimeofday(&t_start, NULL);
	gettimeofday(&t_now, NULL);
	time_elapsed_microseconds = 0;
	file_number = parameters.read_parts_starting_file;
	basename = std::string(parameters.input_file);
	if(!init_input_file_subfunction(basename, parameters.comments, true))
	  	printf("file initialization failed\n");//return 1;
	else 
       	printf("file initialization successful\n");//return 0;
}

int FilePartsSource::init_input_file_subfunction(const std::string path_with_filename, std::string &comments, bool populate_comments)
{
	std::string file_name(path_with_filename);
	char file_num_str[13];
	sprintf(file_num_str, "/%u.bin", file_number++);
	file_name.append(file_num_str);
	input_file.open(file_name.c_str(), std::ios::in | std::ios::binary);

	if (!input_file.is_open())
	{
		if(populate_comments == true)
			printf("\n ERROR: input file not found    %s\n\n", path_with_filename.c_str());
		else
			printf("\n No more files %s\n\n", path_with_filename.c_str());
		
		return 0;
	}
	
	int file_start_position = input_file.tellg();

	char file_version_chars[10]; //must be 3 to leave enough space for the end line character
	input_file.get((char*)file_version_chars, 10, '\n');
	if (file_version_chars[0] == '#')
		file_version = 0;
	else
		if (file_version_chars[0] == 'v')
			file_version = atoi(&file_version_chars[1]);
		else
			printf("ERROR: Cannot detect file version\n");

	printf("file is version %i\n", file_version);
	input_file.ignore(10000, '\n');
	while(input_file.peek() == '#')
		input_file.ignore(10000, '\n');
	int comments_end = input_file.tellg();
	//input_file.ignore(10000, '\n');
	uint16_t dimension[3]; //must be 3 to leave enough space for the end line character
	input_file.getline((char*)dimension, 5, '\n');
	//printf("%i characters read\n", input_file.gcount());
	if(input_file.gcount() ==1)
	{
		//printf("No dimension specification found in file, defaulting to ATIS resolution of %i x %i\n", 304, 240);
		DataSource::x_dim = 304;
		DataSource::y_dim = 240;
	}else
	{
		//printf("Dimension specification from file is %i x %i pixels\n", dimension[0], dimension[1]);
		DataSource::x_dim = dimension[0];
		DataSource::y_dim = dimension[1];
//		input_file.ignore(10000, '\n');
	}
//	free(dimension);

	int file_binary_start_position = input_file.tellg(); //where does the binary portion of the file start?

	if (populate_comments)
	{
		char *tmp_com;
		printf("Allocating %i bytes for comments\n", comments_end-file_start_position);
		tmp_com = (char*)malloc((comments_end-file_start_position)*sizeof(char)); //allocate enough memory for the comments

		input_file.seekg(0, std::ios::beg); //go back to the start
		input_file.read(tmp_com, comments_end-file_start_position); //read in the comments
		comments += tmp_com;
		free(tmp_com);
		//printf("%s", *comments);
	}
	input_file.seekg(0, std::ios::end); //go to the end of the file
	input_file_size = (long)(input_file.tellg()) - file_binary_start_position; //determine how long the binary portion of the file is
	input_file.seekg(file_binary_start_position); //go back to the start of the binary portion
	
	/*// read in the first packetinfo to get the start time.
	packet_info_type packet_info;
	input_file.read((char*)&packet_info, sizeof(packet_info_type));
	packet_info.time_start;
*/
	input_file.seekg(file_binary_start_position); //go back to the start of the binary portion

	printf("file size is  %ld\n", input_file_size);
	return 1;
}


bool FilePartsSource::getData(packet_info_type* packet_info, event* dataOUT, bool exit_thread)
{
	if((input_file_size > 0) & (exit_thread == false))
	{
		if(read_more_data == true)
		{
			//printf("reading more data\n");
			input_file.read((char*)packet_info, sizeof(packet_info_type));
			input_file_size -= sizeof(packet_info_type);
			input_file.read((char*)dataOUT, packet_info->num_events*sizeof(event));
			input_file_size -= packet_info->num_events*sizeof(event);
			if(file_version == 0)
			{
				packet_info[0].time_start = packet_info[0].time_start>>16;	
				packet_info->packet_type = packet_type::TD_EM_mixed_packet;
			//	printf("version 0: modified time_start\n");
			}
		}else
		{

			memcpy(packet_info, &packet_info_INTERNAL, sizeof(packet_info_type));
			memcpy(dataOUT, dataINTERNAL, sizeof(event)*packet_info->num_events);
			//printf("version 0: no need for new data, instead copied %i bytes of old data", packet_info->num_events);
		}
		
		if(file_version == 0)
		{
			//printf("version 0: checking for overflow packets\n");
			read_more_data = true;
			for(uint32_t i=0; i<packet_info->num_events; i++)
				if(dataOUT[i].type == evt_type::timer_overflow)
				{
					memcpy(&packet_info_INTERNAL, packet_info, sizeof(packet_info_type)); //copy the packet header for re-use with the next packet
					packet_info_INTERNAL.num_events-=(i+1); //but the number of events will be fewer
					packet_info_INTERNAL.time_start++; //and the start time must be incremented
					packet_info->num_events = i;
					memcpy(dataINTERNAL, &dataOUT[i+1], sizeof(event)*packet_info_INTERNAL.num_events); // copy the remaining output data for use in the next iteration
					read_more_data = false;
			//		printf("version 0: overflow packet found\n");
				}
		}
		

		if (time_initialized == false)
		{	
			time_initialized = true;
			offset_time = (uint64_t(packet_info[0].time_start)<<16) | dataOUT[0].t;
			//printf("read time initialized to %ld \n", offset_time);
		}
		// THIS PART FORCES REAL TIME BASED ON SYSTEM CLOCK
		if (isRealTime == true) 
		{
			int64_t packet_start_time = ((int64_t(packet_info[0].time_start)<<16) | dataOUT[0].t) - offset_time;
			
			packet_start_time = packet_start_time/set_real_time_multiplier;
			while (time_elapsed_microseconds <= packet_start_time) //loop until the current time matches or exceeds the packet start time
			{
				gettimeofday(&t_now, NULL);				
				time_elapsed_microseconds = int64_t((t_now.tv_sec - t_start.tv_sec)*1000000 + (t_now.tv_usec - t_start.tv_usec));
				//printf("Packet start time is %ld, time elapsed is %ld us, must wait another %ld us before continuing\n", packet_start_time, time_elapsed_microseconds, (packet_start_time-time_elapsed_microseconds));
				//time_elapsed_microseconds = int(t_now.tv_usec - t_start.tv_usec);
			}
		}
		//printf("%d events, %d microseconds from source file\n", packet_info->num_events, (packet_info->time_end-packet_info->time_start+dataOUT[packet_info->num_events-1].t-dataOUT[0].t));
	}
	else
	{
		input_file.close();
		packet_info[0].num_events = 0;
		std::string tmp; // Only for the sake of calling the function below, unused
		if(exit_thread == false)
		{
			printf("End of file %i reached, moving on to next file \n", file_number);
			if(!init_input_file_subfunction(basename, tmp, false))
			{
				printf("No more files, Exit threads initiated\n");
				return 1; //indicates that an exit command has occurred			
			}
		}else
		{
			printf("Exit threads initiated\n");
			return 1; //indicates that an exit command has occurred	
		}
	}
	return 0;
}