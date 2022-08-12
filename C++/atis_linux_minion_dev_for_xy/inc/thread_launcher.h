#ifndef THREAD_LAUNCHER_H
#define THREAD_LAUNCHER_H

#ifdef _useSDL
	#include <SDL.h>
	static SDL_Event SDL_keyboard_event;
#endif

#include <mutex>
#include <stdio.h>
#include <iostream>
#include <pthread.h>

#include <event_framework.h>

#define max_read_length 8192   //6144 is the max if dataIN (4 bytes per event) goes on the stack, but if we declare it outside the  function, it goes in main mem. 
#define num_threads 10
#define num_functions_per_thread 20
#define num_stages 10

extern bool exit_threads[num_stages];


class thread_launcher {
	private:

		static void* function_launcher(void* data);
		static void* last_function_launcher(void* data);
		static void* temp_last_function_launcher(void* data);
		
	public:
	//THESE ARE ACCESSED BY OTHER FUNCTIONS... could be cleaned up and made private too
	//extern bool data_available[num_stages][2];	
	//static void init_source(device_code event_source, char** comments, const std::string bitfile, const std::string biasfile, bool useGrayScale, bool useIMU, int version, uint64_t filenumber);
	static void init_source(source_parameters_type &parameters);
	static void run_source(void);
	static int launch_processing_chain();
	static bool safe_to_exit();	
	thread_launcher();
};

//these are defined in thread_launcher.cpp
extern DataSource *source_pointer;
extern DataConsumer *DataConsumer_pointer[num_threads][num_functions_per_thread];
extern bool initialized_functions[num_threads][num_functions_per_thread];

#endif
