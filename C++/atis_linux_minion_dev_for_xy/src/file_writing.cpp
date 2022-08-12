#include <limits.h>
#include <fstream>
#include <cstring>

#include <file_writing.h>

/************************************************************************************************************
Input      : Input filename and path
Output     : Returns 1 if successful, 0 if otherwise.
Description: A function to obtain the input file name and path for recordings
*************************************************************************************************************/
/*static std::ofstream output_file;
static char* path_with_filename_base_internal;
static bool first_call_init_write_to_file = true;
*/

void file_write::set_parameters(const char* path_with_filename, const char* comments,  unsigned int write_time)
{
	if(write_time>0)
 		limit_file_write_time(write_time);
	std::string filename (path_with_filename);
	if(first_call_init_write_to_file == true)
	{
		first_call_init_write_to_file = false;
		path_with_filename_base_internal = std::string(path_with_filename);
		//make copies of the input paramters to be used when writing to multiple files over time (if the time for each file is limited)
	}
	
	char extension[20];	
	if (limit_record_time_per_file == true)
	{
		std::string folder_name("mkdir ");
		folder_name.append(filename);
		system(folder_name.c_str());
		sprintf(extension, "%s%u%s", "/", file_number++, ".bin");
		filename.append(extension);
	}
	else
	{
		//sprintf(extension, "%s", ".bin");
		//filename.append(extension);
	}
	//printf(filename.c_str());

	char timeStr[150];
	output_file.open(filename.c_str(), std::ios::out | std::ios::binary);

	if (output_file.is_open())
		printf("Data will be logged to %s\n", filename.c_str());
	else
		printf("\nERROR: problem opening file:%s\n\n", filename.c_str());

    output_file << "v2" << "\n";

    time_t t = time(NULL);
	struct tm tm = *localtime(&t);
    sprintf(timeStr,"#Recorded DateTime: %d-%d-%d %d:%d:%d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    printf("%s\n",timeStr);
	output_file << timeStr << "\n";
        
 	output_file << comments;
 	output_file.write((char*)(&DataSource::x_dim), sizeof(uint16_t));
 	output_file.write((char*)(&DataSource::y_dim), sizeof(uint16_t));
 	output_file.write((char*)(&DataSource::pix), sizeof(float));
 	output_file << '\n';

 	
/*	output_file << "#This is the file header which will specify the format\n";
	output_file << "#It can consist of multiple lines, each beginning with '#'\n";
	output_file << "#After the comments, there must be a one line break before the binary portion starts\n\n";
*/
}

/************************************************************************************************************
Input      : (1) Buffer to voltage values. (2) Pointer to the bias data 
Output     : Returns 1 if successful, 0 if otherwise.
Description: A function which writes a collection of events to persistent storage. It performs the addition task 
             of filtering the first few frames, when the ATIS sensor has just started up and is in the process of 
             stablizing the hardware.   
*************************************************************************************************************/
void file_write::limit_file_write_time(unsigned int write_time)
{
	time_per_file = write_time*1000000;
	file_end_time = write_time*1000000;
	limit_record_time_per_file = true;
}

//unsigned int file_number = 0;
bool file_write::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
		if (~startedRecording) {
			if (packet_info->time_start > 1000000>>16) {
				startedRecording = true;
			}
		}

    	if(startedRecording && packet_info->num_events != 0) 
    	{
           	output_file.write((char*)packet_info, sizeof(packet_info_type));
		   	output_file.write((char*)dataIN, packet_info->num_events*sizeof(event));
			if (limit_record_time_per_file == true)
			{
				if (uint64_t(packet_info->time_start)<<16 > file_end_time)
				{	
					if (file_end_time_overflowed == false)
					{
						printf("File end-time reached. Closing file and opening a new one.\n");
						output_file.close();
						//char path_with_filename_internal[std::strlen(path_with_filename_base_internal)+20];
						//sprintf(path_with_filename_internal, "%s%u", path_with_filename_base_internal, file_number++);
						char temp_comments[20] = {"#fileparts\n"};					
						
						set_parameters(path_with_filename_base_internal.c_str(), temp_comments, 0);
						if (UINT_MAX-time_per_file < file_end_time) //check if overflow condition will occur
							file_end_time_overflowed = true;				
						file_end_time+=time_per_file;
					}
				//	file_end_time = file_end_time & 0xFFFFFFFF; //maximum of 32 bits
				}else
				{
					file_end_time_overflowed = false; // if the packet info time is less than the file end time, then the packet info time has also overflowed.
				}
		   	}//gettimeofday(&t_end, NULL);
    	}

	    if(exit_thread == 1)
    	{
			output_file.close();
        	return 1;	
    	}

    	return 0;
}
