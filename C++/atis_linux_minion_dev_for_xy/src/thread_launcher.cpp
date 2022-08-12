#include <ATIS_interface.h>
#ifdef _useDAVIS
	#include <DAVIS_interface.h>
#endif
#include <FILE_interface.h>

#include <thread_launcher.h>
#include <string.h>

#ifdef _useROS
#include <ros_interface.h>
#endif

/************************************************************************************************************
Input      : 
Output     : 
Description:
*************************************************************************************************************/
//CAN THESE BE MADE STATIC MEMBERS OF THE THREADLAUNCHER CLASS???
bool exit_threads[num_stages]; 

DataSource *source_pointer;
DataConsumer *DataConsumer_pointer[num_threads][num_functions_per_thread];

int thr_num;
pthread_mutex_t processing_mutex[num_stages][2]; 					 //mutex array used to control buffer access between input and decoding stage

//"event_struct" is an array of event buffers for passing data between functions in the processing chain
event event_struct[num_stages][2][max_read_length*(1024/4)];
packet_info_type packet_info[num_stages][2];  //array used to remember how much data is in each buffer

int data_available_number[num_stages][2];
bool data_available [num_stages][2];

pthread_t threadID[num_threads]; //a vector of threadIDs
int thread_number[num_threads]; //a vector holding the thread number/identifier

int thread_launcher::launch_processing_chain()
{
	thr_num = 0;
	thr_num++;

	for(int stage = 0; stage<num_stages; stage++)
	{
		//initializing the mutexes
		processing_mutex[stage][0] = PTHREAD_MUTEX_INITIALIZER;
		processing_mutex[stage][1] = PTHREAD_MUTEX_INITIALIZER;
	}

	//launch the processing stages
	//while(function_pointer[thr_num+1][0] != NULL)
	while(DataConsumer_pointer[thr_num+1][0] != NULL)
	{
		printf("launching intermediate processing thread on thread %i\n", thr_num);
		thread_number[thr_num] = thr_num;
		pthread_create(&threadID[thread_number[thr_num]], NULL, function_launcher, (void*)&thread_number[thr_num]);
		thr_num++;
	}

	//launch the last processing stage (doesn't need to check whether it's output buffers are ready)
	thread_number[thr_num] = thr_num;
	pthread_create(&threadID[thread_number[thr_num]], NULL, last_function_launcher, (void*)&thread_number[thr_num]);

	return 0;
}

/************************************************************************************************************
Input      : 
Output     : 
Description:
*************************************************************************************************************/

void thread_launcher::init_source(source_parameters_type &parameters)
{
	printf("Source thread initialized on main thread \n");
	#ifdef _useSDL
	if(SDL_WasInit(SDL_INIT_VIDEO) == 0)
	{
		printf("Initializing SDL\n");	
		if(SDL_Init( SDL_INIT_VIDEO ) <0)
			printf("SDL_Init failed: %s\n", SDL_GetError());
	}
	#endif
	switch (parameters.source_type) 
	{
//------------------------------------------- DEVICES ------------------------------------------------------//
		#ifdef _useDAVIS
		case davis240C:
			{
				printf("DAVIS 240C selected, configuring with default settings\n");

				source_pointer = new Davis240CSource(parameters);
				break;
			}
		case davis640:
			{
				printf("DAVIS 640 selected, configuring with default settings\n");

				source_pointer = new Davis640Source(parameters);
				break;
			}
		#endif

		#ifdef _useATIS
		case atis:
			{			
				printf("Configuring ATIS with bitfile... %s ...  and biases file ... %s ...\n", parameters.fpga_bitfile.c_str(), parameters.biasfile.c_str());
				source_pointer = new AtisSource(parameters);
				break;
			}
		#endif
		case file:
			{
				printf("Will launch from file \n");
				source_pointer = new FileSource(parameters);
				break;		
			}
		case fileparts:
			{
				printf("Will launch from file in parts\n");
				source_pointer = new FilePartsSource(parameters);
				break;		
			}
		#ifdef _useROS
		case ros_in:
			{
				printf("Initializing ROS source with name %s...\n", parameters.ros_input_topic.c_str());
				source_pointer = new RosSource(parameters);
				break;
			}
		#endif
	}

}

void thread_launcher::run_source(void)
{
	printf("Source thread acquiring data on main thread \n");
	int packet_counter = 1;
	exit_threads[0] = false;
	// Part where the data is actually retrieved
	while(exit_threads[1] == 0)
	{	
		pthread_mutex_lock (&processing_mutex[0][0]);
		if(data_available[0][0] == 0)
		{
			packet_info[0][0].num_events = 0;
			while((exit_threads[1] == 0) & (packet_info[0][0].num_events == 0))
				exit_threads[1] = source_pointer->getData(&packet_info[0][0], event_struct[0][0], exit_threads[0]);

			data_available[0][0] = 1;
			data_available_number[0][0] = packet_counter++;
		}
		pthread_mutex_unlock (&processing_mutex[0][0]);
		
		if(exit_threads[1] == 0)
		{
			pthread_mutex_lock (&processing_mutex[0][1]);
			if(data_available[0][1] == 0)
			{
				packet_info[0][1].num_events = 0;
				while((exit_threads[1] == 0) & (packet_info[0][1].num_events == 0))
					exit_threads[1] = source_pointer->getData(&packet_info[0][1], event_struct[0][1], exit_threads[0]);
				data_available[0][1] = 1;
				data_available_number[0][1] = packet_counter++;
			}
			pthread_mutex_unlock (&processing_mutex[0][1]);
		}

		
		#ifdef _useSDL
		while(SDL_PollEvent( &SDL_keyboard_event ))
			{
				if(SDL_keyboard_event.type == SDL_QUIT)
				{	
					exit_threads[0] = 1;
					printf("exit_threads triggered \n");
				}
				else
					if(SDL_keyboard_event.type == SDL_KEYDOWN)
						if(SDL_keyboard_event.key.keysym.sym == SDLK_ESCAPE)
						{
							exit_threads[0] = 1;
							printf("exit_threads triggered \n");
						}
			}				
		#endif
	}
	// Deallocate the memory before exiting
	delete source_pointer;
	printf("Thread event source exiting now... \n");
}





/************************************************************************************************************
Input      : 
Output     : 
Description:
*************************************************************************************************************/
//function for launching other functions in ping-pong buffer mode
void* thread_launcher::function_launcher(void* data)
{
	int mythread = *((int*)data); //my (i.e. this function's) threadID 
	printf("New processing thread running on thread number %d\n", mythread);
	int me = mythread;
	int packet_counter = 1;
	int function_number = 0;
	bool exit_this_thread;
	while(exit_threads[mythread+1] == 0)
	{	
		pthread_mutex_lock (&processing_mutex[me-1][0]);
		if((data_available[me-1][0] == 1) & (data_available_number[me-1][0] == packet_counter))
		{
			pthread_mutex_lock (&processing_mutex[me][0]);
			if(data_available[me][0] == 0)
			{
				function_number = 0;
				exit_this_thread = exit_threads[mythread];
				while(DataConsumer_pointer[me][function_number]!=NULL)
					if(packet_info[me-1][0].num_events>0)
						exit_this_thread = DataConsumer_pointer[me][function_number++]->processData(&packet_info[me-1][0], event_struct[me-1][0], exit_this_thread);
					else
						function_number++;
				exit_threads[mythread+1] = exit_this_thread;
				memcpy(event_struct[me][0], event_struct[me-1][0], packet_info[me][0].num_events*sizeof(event));
				packet_info[me][0] = packet_info[me-1][0];
				data_available[me-1][0] = 0;
				data_available[me][0] = 1;
				data_available_number[me][0] = packet_counter++;
			}
			pthread_mutex_unlock (&processing_mutex[me][0]);
		}
		pthread_mutex_unlock (&processing_mutex[me-1][0]);
		
		pthread_mutex_lock (&processing_mutex[me-1][1]);
		if(exit_threads[mythread+1] == 0)
		if((data_available[me-1][1] == 1) & (data_available_number[me-1][1] == packet_counter))
		{
			pthread_mutex_lock (&processing_mutex[me][1]);
			if(data_available[me][1] == 0)
			{
				function_number = 0;
				exit_this_thread = exit_threads[mythread];
				while(DataConsumer_pointer[me][function_number]!=NULL)
					if(packet_info[me-1][1].num_events>0)
						exit_this_thread = DataConsumer_pointer[me][function_number++]->processData(&packet_info[me-1][1], event_struct[me-1][1], exit_this_thread);
					else
						function_number++;	
//					exit_this_thread = function_pointer[me][function_number++](&packet_info[me-1][1], event_struct[me-1][1], exit_this_thread);
				exit_threads[mythread+1] = exit_this_thread;
				memcpy(event_struct[me][1], event_struct[me-1][1], packet_info[me][1].num_events*sizeof(event));
				packet_info[me][1] = packet_info[me-1][1];
				data_available[me-1][1] = 0;
				data_available[me][1] = 1;
				data_available_number[me][1] = packet_counter++;			
			}
			pthread_mutex_unlock (&processing_mutex[me][1]);
		}
		pthread_mutex_unlock (&processing_mutex[me-1][1]);
	}
	printf("Thread %d exiting now... \n", mythread);
}


void* thread_launcher::last_function_launcher(void* data)
{
	int mythread = *((int*)data); //my (i.e. this function's) threadID 
	printf("Last processing thread running on thread number %d\n", mythread);
	int me = mythread;
	int packet_counter = 1;
	int function_number = 0;
	bool exit_this_thread;

	while(exit_threads[mythread+1] == 0)
	{	
		pthread_mutex_lock (&processing_mutex[me-1][0]);
		if((data_available[me-1][0] == 1) & (data_available_number[me-1][0] == packet_counter))
		{
			pthread_mutex_lock (&processing_mutex[me][0]);
			function_number = 0;
			exit_this_thread = exit_threads[mythread];
			while(DataConsumer_pointer[me][function_number]!=NULL)
				if(packet_info[me-1][0].num_events>0)
					exit_this_thread = DataConsumer_pointer[me][function_number++]->processData(&packet_info[me-1][0], event_struct[me-1][0], exit_this_thread);
				else
					function_number++;
			exit_threads[mythread+1] = exit_this_thread;
			//last function call doesn't need to have the copy from event_struct[me-1][0], event_struct[me][0],
			data_available[me-1][0] = 0;
			packet_counter++;
			pthread_mutex_unlock (&processing_mutex[me][0]);
		}
		pthread_mutex_unlock (&processing_mutex[me-1][0]);
		
		
		pthread_mutex_lock (&processing_mutex[me-1][1]);
		if(exit_threads[mythread+1] == 0)
		if((data_available[me-1][1] == 1) & (data_available_number[me-1][1] == packet_counter))
		{
			pthread_mutex_lock (&processing_mutex[me][1]);
			function_number = 0;
			exit_this_thread = exit_threads[mythread];
			while(DataConsumer_pointer[me][function_number]!=NULL)
				if(packet_info[me-1][1].num_events>0)
					exit_this_thread = DataConsumer_pointer[me][function_number++]->processData(&packet_info[me-1][1], event_struct[me-1][1], exit_this_thread);
				else
					function_number++;
			exit_threads[mythread+1] = exit_this_thread;
			data_available[me-1][1] = 0;
			packet_counter++;
			pthread_mutex_unlock (&processing_mutex[me][1]);
		}
		pthread_mutex_unlock (&processing_mutex[me-1][1]);
	}
	printf("Thread %d exiting now (last function)... \n", mythread);
}


//returns true only if all threads have safely exited
bool thread_launcher::safe_to_exit()
{
	//printf("Waiting for %i threads to finish... currenty %i %i %i %i %i %i \n", thr_num+1, exit_threads[0], exit_threads[1], exit_threads[2], exit_threads[3], exit_threads[4], exit_threads[5]);
	int finish = thr_num+1;
	for(int i=1; i<thr_num+2; i++)
	{
		finish -= exit_threads[i];		
	}
	if(finish>0)
		return false;
	else
		return true;
}
