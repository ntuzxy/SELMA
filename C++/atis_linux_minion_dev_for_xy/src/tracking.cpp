#ifdef _useOPENCV
	#include <opencv.hpp>
	#include <highgui.hpp>
	using namespace cv;
#endif
#include <thread_launcher.h>

#include <tracking.h>

pthread_mutex_t tracker_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t tracker_display_mutex = PTHREAD_MUTEX_INITIALIZER;


track::track(uint16_t num_trackers_passed, uint16_t radius_passed)
{
	distance_threshold = radius_passed*radius_passed;
	num_trackers = num_trackers_passed;
	distance = (int*)malloc(sizeof(int)*num_trackers);
	tracker = (tracker_type*)malloc(sizeof(tracker_type)*num_trackers);
	for(int i =0; i<num_trackers; i++)
	{
		tracker[i].loc[0] = rand()%DataSource::x_dim;
		tracker[i].loc[1] = rand()%DataSource::y_dim;
		tracker[i].colour = rand()%0xFFFFFF;

		tracker[i].last_loc[0] = tracker[i].loc[0];
		tracker[i].last_loc[1] = tracker[i].loc[1];
		tracker[i].radius = radius_passed;
		printf("Tracker randomly initialized to location [%.1f, %.1f] and color R=%i, G=%i, B=%i\n", tracker[i].loc[0], tracker[i].loc[1], (tracker[i].colour>>16)%0xFF, (tracker[i].colour>>8)%0xFF, tracker[i].colour%0xFF);
	}
}


/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, 
              (4) Thread ID 
Output      : Returns information about the output event buffer 
Description : Performs tracking of quadcopter(s) Generally, the algorithm iterates through all events to find 
	      the active trackers based on the received events 
************************************************************************************************************/
bool track::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
//	printf("tracking %G\n", tracker[0].loc[0]);
	uint64_t track_time_MSBs = uint64_t(packet_info->time_start)<<16;
	uint64_t time;
	int min_inactive_distance, min_active_distance;
	int min_inactive_distance_index, min_active_distance_index;
	
	
	
	//printf("%i events in packet\n", packet_info->num_events);
	if(packet_info->packet_type == packet_type::TD_packet || packet_info->packet_type == packet_type::TD_EM_mixed_packet)
	{
		pthread_mutex_lock(&tracker_mutex);
		for(uint32_t eventNum=0; eventNum<packet_info->num_events; eventNum++)
		{
			if (dataIN[eventNum].type == evt_type::TD)
			{
				time = track_time_MSBs | dataIN[eventNum].t;
				min_inactive_distance = max_distance;
				min_active_distance = max_distance;
				min_inactive_distance_index = 0;
				min_active_distance_index = 0;
	//			printf("Event received at [%i, %i] at time %i\n", dataIN[eventNum].x, dataIN[eventNum].y, time);

				for(uint32_t i=0; i<num_trackers; i++)
				{
					distance[i] = pow((dataIN[eventNum].x-(int)tracker[i].loc[0]),2) 
	                                            + pow((dataIN[eventNum].y-(int)tracker[i].loc[1]),2);

					if(tracker[i].active == 1)
					{
						if((distance[i]<min_active_distance))
						{
							min_active_distance = distance[i];
							min_active_distance_index = i;
						}
					}else
					if(distance[i]<min_inactive_distance)
					{
						min_inactive_distance = distance[i];
						min_inactive_distance_index = i;
					}
				}

				if(min_active_distance < distance_threshold)
				{
					tracker[min_active_distance_index].loc[0] = tracker[min_active_distance_index].loc[0]*(mixing_factor) + dataIN[eventNum].x*(1-mixing_factor);
					tracker[min_active_distance_index].loc[1] = tracker[min_active_distance_index].loc[1]*(mixing_factor) + dataIN[eventNum].y*(1-mixing_factor);

					//printf("Time of active event is %i, and time difference for tracker %i is %i. The current tdiff is %f\n", time, min_active_distance_index, (time - tracker[min_active_distance_index].last_event), tracker[min_active_distance_index].tdiff_mean);
					//printf("The equation is %f * %f + %f * %f = ", tracker[min_active_distance_index].tdiff_mean, (time_mixing_factor), ((double)(time - tracker[min_active_distance_index].last_event)), (1-time_mixing_factor));
					tracker[min_active_distance_index].tdiff_mean = tracker[min_active_distance_index].tdiff_mean*(time_mixing_factor) + ((double)(time - tracker[min_active_distance_index].last_event))*(1-time_mixing_factor);
					tracker[min_active_distance_index].last_event = time;
					//printf("%f\n", tracker[min_active_distance_index].tdiff_mean);
					

					dataIN[eventNum].type = evt_type::Tracked;
					dataIN[eventNum].subtype = min_active_distance_index;

					if(tracker[min_active_distance_index].tdiff_mean < tdiff_threshold)
						tracker[min_active_distance_index].active = 1;
					else
						tracker[min_active_distance_index].active = 0;
					
					
				}else 
				if(min_inactive_distance < distance_threshold_2)
				{
					
					//tracker[min_distance_index].loc[0] = tracker[min_distance_index].loc[0]*(mixing_factor) + dataIN[eventNum].x*(1-mixing_factor);
					//tracker[min_distance_index].loc[1] = tracker[min_distance_index].loc[1]*(mixing_factor) + dataIN[eventNum].y*(1-mixing_factor);
					tracker[min_inactive_distance_index].loc[0] = tracker[min_inactive_distance_index].loc[0]*(mixing_factor_2) + dataIN[eventNum].x*(1-mixing_factor_2);
					tracker[min_inactive_distance_index].loc[1] = tracker[min_inactive_distance_index].loc[1]*(mixing_factor_2) + dataIN[eventNum].y*(1-mixing_factor_2);

					if(min_inactive_distance < distance_threshold)
					{
					//	printf("Time of inactive event is %i, and time difference for tracker %i is %i. The current tdiff is %f\n", time, min_inactive_distance_index, (time - tracker[min_inactive_distance_index].last_event), tracker[min_inactive_distance_index].tdiff_mean);
					//	printf("The equation is %f * %f + %f * %f = ", tracker[min_inactive_distance_index].tdiff_mean, (time_mixing_factor), ((double)(time - tracker[min_inactive_distance_index].last_event)), (1.0-time_mixing_factor));
						tracker[min_inactive_distance_index].tdiff_mean = tracker[min_inactive_distance_index].tdiff_mean*(time_mixing_factor) + ((double)(time - tracker[min_inactive_distance_index].last_event))*(1-time_mixing_factor);
					//	printf("%f\n", tracker[min_inactive_distance_index].tdiff_mean);				
					}
					//tracker[min_distance_index].tdiff_mean = tracker[min_distance_index].tdiff_mean*(mixing_factor_2) + (time - tracker[min_distance_index].last_event)*(1-mixing_factor_2);
					tracker[min_inactive_distance_index].last_event = time;
					//Here - adjust
					//if(tracker[min_distance_index].tdiff_mean < tdiff_threshold)
					if(tracker[min_inactive_distance_index].tdiff_mean < tdiff_threshold_2)
					{
						tracker[min_inactive_distance_index].active = 1;
						tracker[min_inactive_distance_index].active_time = time;
					}
					else
						tracker[min_inactive_distance_index].active = 0;
				}

			}/*else
			if (dataIN[eventNum].type == evt_type::timer_overflow)
			{
				track_time_MSBs += 1<<16;
			}*/
		}
	
		// periodic cleanup of trackers
		time = track_time_MSBs| dataIN[packet_info->num_events-1].t;
		if(time - last_tracker_cleanup > tracker_update_period)
		{
			for(uint32_t i=0; i<num_trackers; i++)
			{
	//			printf("Cleaning up trackers at time %i. The last event for tracker %i was at %i. The current tdiff is %f\n", time, i, tracker[i].last_event, tracker[i].tdiff_mean);
	//			printf("%f * %f  +  %f  * %f ", tracker[i].tdiff_mean, mixing_factor, (double)(time - tracker[i].last_event), (1.0-mixing_factor));
				tracker[i].tdiff_mean = tracker[i].tdiff_mean*mixing_factor + ((double)(time - tracker[i].last_event))*(1.0-mixing_factor);
	//			printf("= %f\n", tracker[i].tdiff_mean);
	//			printf("%i - %i = %f\n", packet_info->time_end, tracker[i].last_event, (double)(time | - tracker[i].last_event));


				tracker[i].dist[0] = tracker[i].dist[0]*mixing_factor + abs(tracker[i].loc[0] - tracker[i].last_loc[0])*(1.0-mixing_factor);
				tracker[i].dist[1] = tracker[i].dist[1]*mixing_factor + abs(tracker[i].loc[1] - tracker[i].last_loc[1])*(1.0-mixing_factor);
				tracker[i].last_loc[0] = tracker[i].loc[0];
				tracker[i].last_loc[1] = tracker[i].loc[1];
			//	printf("tracker %i is active %i and has an average distance of %f %f and tdiff of %f\n", i, tracker[i].active, tracker[i].dist[0], tracker[i].dist[1], tracker[i].tdiff_mean);

				if (tracker[i].tdiff_mean>tdiff_max)
					tracker[i].tdiff_mean = tdiff_max;
	//				printf("tdiff limited to max\n");

	/*			if(packet_info->time_end - tracker[i].last_event > tracker_idle_threshold)
					tracker[i].active = 0;
	*/
				if(tracker[i].tdiff_mean > tdiff_threshold)
					tracker[i].active = 0;
			}
		//	printf("Tracker cleanup performed at time %i microseconds\n", packet_info->time_start);
			last_tracker_cleanup = time;
		}
	
/*
	if(!tracker_output_file.is_open())
	{
		sleep(1);
		std::string filename= "";
		if(exit_thread == 0)
		{
			std::string filepath = "../recordings/";
			std::string filename = "trackers.bin";
			std::string path_with_filename;
			path_with_filename =  filepath + filename; 
			printf("The output filename to be used for trackers is %s\n", path_with_filename.c_str());
			tracker_output_file.open(path_with_filename.c_str(), std::ios::out | std::ios::binary);
			if (tracker_output_file.is_open())
			{
				printf("Trackers will be logged to %s\n", path_with_filename.c_str());
			}
			else
			{
				printf("\nERROR: problem opening tracker file:%s\n\n", path_with_filename.c_str());
			}
			tracker_output_file << "#This is the file header which will specify the format\n";
			tracker_output_file << "#It can consist of multiple lines, each beginning with '#'\n";
			tracker_output_file << "#After the comments, there must be a one line break before the binary portion starts\n\n";

		}else
		{
			printf("Something is wrong (perhaps the input file doesn't exist?), so the program will exit without asking for an output file name\n");
		}
	}

	if(packet_info->num_events != 0)
	{
//		printf("Writing %i trackers to output file...\n", num_trackers);
		tracker_output_file.write((char*)&tracker, num_trackers*sizeof(tracker_type));
	}
*/
	// Add tracking events in the center of active trackers
		uint16_t last_t_tracker = dataIN[packet_info->num_events-1].t;
		for (int i=0; i<num_trackers; ++i)
		{
			if (tracker[i].active == true)
			{
				//printf("Pushing a new event for the tracker");
				dataIN[packet_info->num_events].type = evt_type::TD_tracker;
				dataIN[packet_info->num_events].subtype = i;
				dataIN[packet_info->num_events].y = tracker[i].loc[1];
				dataIN[packet_info->num_events].x = tracker[i].loc[0];
				dataIN[packet_info->num_events].t = last_t_tracker;
				packet_info->num_events++;
			}
		}
		pthread_mutex_unlock(&tracker_mutex);
	}
	

	if(exit_thread == 1)
	{
//		tracker_output_file.close();
		printf("tracker quitting\n");
		return 1;
	}
//	memcpy(dataOUT, dataIN, packet_info->num_events*sizeof(event));
	return 0;
}



#ifdef _useOPENCV


	static int last_TDtracker_display_time = -1e9;
	Mat TDimage;
	/************************************************************************************************************
	Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, 
		      (4) Thread ID 
	Output      : Returns information about the output event buffer 
	Description : Displays the received TD data with Trackers on the image screen.
	************************************************************************************************************/
	int init_showTD_track()
	{
		TDimage = Mat(240, 304, CV_8U, 128);
		//-------------------------------- this causes trouble
		//namedWindow("Display TD Image with tracking ",WINDOW_NORMAL);
		//--------------------------------	
		return 1;
	}

	bool showTD_track(packet_info_type* packet_info, event* dataIN, bool exit_thread)
	{
		int TD_track_display_MSBs = packet_info->time_start;
		int end_time = dataIN[packet_info->num_events-1].t | packet_info->time_start;
	
		for(int eventNum=0; eventNum<packet_info->num_events; eventNum++)
		{
			if (dataIN[eventNum].type == evt_type::TD)
			{
				if (dataIN[eventNum].subtype == TD_subtype::ON)
					TDimage.at<unsigned char>(dataIN[eventNum].y,dataIN[eventNum].x) = 255;
				else
					TDimage.at<unsigned char>(dataIN[eventNum].y,dataIN[eventNum].x) = 0;
			}else
			if (dataIN[eventNum].type == evt_type::timer_overflow)
				TD_track_display_MSBs += 1<<16;
		}
		
		if(end_time-last_TDtracker_display_time >= display_interval)
		{
			//printf("Displaying TD Image... \n");
			Scalar black = Scalar(0);
			Scalar white = Scalar(255);
			pthread_mutex_lock(&tracker_mutex);
			for(int i=0; i<num_trackers; i++)
			{

				if(tracker[i].active == 1)
					circle(TDimage, Point(tracker[i].loc[0], tracker[i].loc[1]), 10, white, 2, 8, 0);
				else
					circle(TDimage, Point(tracker[i].loc[0], tracker[i].loc[1]), 10, black, 2, 8, 0);

	/*			//Add a delay here to improve visualisation
				for(int i=0; i<1000;i++)
					for(int j=0; j<10000;j++);
	*/
			}
			pthread_mutex_unlock(&tracker_mutex);

			pthread_mutex_lock(&tracker_display_mutex);




			imshow("Display TD Image with tracking", TDimage);
		
			TDimage = Mat(240, 304, CV_8U, 128);
			if ((waitKey(1) & 0xFF) == 27)
			{
	//			exit_threads[mythread] = 1;
			// exit_threads[1] = 0;
				return 1;
			}

			pthread_mutex_unlock(&tracker_display_mutex);
			last_TDtracker_display_time = end_time;
		}

		if(exit_thread == 1)
		{
			destroyWindow("Display TD Image with tracking ");
			return 1;
		}

		//memcpy(dataOUT, dataIN, packet_info->num_events*sizeof(event));
		return 0;
	}



	//---------------------------------------------------- write video with tracking ------------------------------------------------------
	
	static int last_video_display_time_TD;
	static Mat video_image_TD;
	static VideoWriter TDvideo;
	void init_write_video_TD_track(const char* filename)
	{
		last_video_display_time_TD = 0;
		video_image_TD = Mat(240, 304, CV_8U, 128);
		TDvideo = VideoWriter(std::string(filename, strlen(filename)) , CV_FOURCC('P','I','M','1'), 30, Size(304,240), false);
		return;
	}

	bool write_video_TD_track(packet_info_type* packet_info, event* dataIN, bool exit_thread)
	{

		int video_display_TimeOverflow = packet_info->time_start;
		int end_time = dataIN[packet_info->num_events-1].t | packet_info->time_start;
	
			for(int eventNum=0; eventNum<packet_info->num_events; eventNum++)
			{
				if (dataIN[eventNum].type == evt_type::TD)
				{
					if (dataIN[eventNum].subtype == TD_subtype::ON)
						video_image_TD.at<unsigned char>(dataIN[eventNum].y,dataIN[eventNum].x) = 255;
					else
						video_image_TD.at<unsigned char>(dataIN[eventNum].y,dataIN[eventNum].x) = 0;
				}else
				if (dataIN[eventNum].type == evt_type::timer_overflow)
					video_display_TimeOverflow += 1<<16;
			}
		
		if(end_time-last_video_display_time_TD >= video_interval)
		{
		        Scalar black = Scalar(0);
			Scalar white = Scalar(255);
			pthread_mutex_lock(&tracker_mutex);
			for(int i=0; i<num_trackers; i++)
			{
				if(tracker[i].active == 1)
					circle(video_image_TD, Point(tracker[i].loc[0], tracker[i].loc[1]), 10, white, 2, 8, 0);
				else
					circle(video_image_TD, Point(tracker[i].loc[0], tracker[i].loc[1]), 10, black, 2, 8, 0);

			}
			pthread_mutex_unlock(&tracker_mutex);

			TDvideo.write(video_image_TD);

			video_image_TD = Mat(240, 304, CV_8U, 128);
			if ((waitKey(1) & 0xFF) == 27)
			{
				//exit_threads[1] = 0;
				return 1;
			}

			last_video_display_time_TD = end_time;
		}

		if(exit_thread == 1)
			return 1;
//		memcpy(dataOUT, dataIN, packet_info->num_events*sizeof(event));
		return 0;
	}
#else
#ifdef _useSDL
	TDdisplay_track::TDdisplay_track()
	{
		track_window = NULL;
		track_pixels = new uint32_t[DataSource::x_dim*DataSource::y_dim];
   		for(uint32_t i=0; i<DataSource::x_dim*DataSource::y_dim; i++)
			track_pixels[i] = background;		
		last_TDtracker_display_time = 0;
	}

	bool TDdisplay_track::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
	{
		if(track_window == NULL)
			init_display_window(&track_window, &track_texture, &track_renderer, "Tracking Window");
		if(packet_info->packet_type == packet_type::TD_packet || packet_info->packet_type == packet_type::TD_EM_mixed_packet)
		{

			uint64_t TD_track_display_MSBs = uint64_t(packet_info->time_start)<<16;
			uint64_t end_time = TD_track_display_MSBs | dataIN[packet_info->num_events-1].t;
		
			for(uint32_t eventNum=0; eventNum<packet_info->num_events; eventNum++)
			{
				if (dataIN[eventNum].type == evt_type::TD)
				{
					if (dataIN[eventNum].subtype == TD_subtype::ON)
						track_pixels[ (uint32_t(dataIN[eventNum].y) * DataSource::x_dim ) + dataIN[eventNum].x] = ONcolor;
					else
						track_pixels[ (uint32_t(dataIN[eventNum].y) * DataSource::x_dim ) + dataIN[eventNum].x] = OFFcolor;
				}else
				if (dataIN[eventNum].type == evt_type::Tracked)
				{
					track_pixels[ (uint32_t(dataIN[eventNum].y) * DataSource::x_dim ) + dataIN[eventNum].x] = tracker[dataIN[eventNum].subtype].colour;
				}
			/*	else
				if (dataIN[eventNum].type == evt_type::timer_overflow)
					TD_track_display_MSBs += 1<<16;*/
			}
			if(end_time-last_TDtracker_display_time >= display_interval)
			{
				for(uint32_t i=0; i<num_trackers; i++)
				{
					//printf("Tracker %i is at location %f, %f, and has a tdiff_mean of %f, and is active (true/false) %i\n",i,tracker[i].loc[0],tracker[i].loc[1], tracker[i].tdiff_mean, tracker[i].active);
					if(tracker[i].active == 1)
						if (end_time-tracker[i].active_time > active_time_threshold)
							if (tracker[i].dist[0]*tracker[i].dist[0] + tracker[i].dist[1]*tracker[i].dist[1] > average_distance_threshold)
								cross(track_pixels, tracker[i].loc[0], tracker[i].loc[1], tracker[i].radius/2-1, marker_color);
							else
								cross(track_pixels, tracker[i].loc[0], tracker[i].loc[1], tracker[i].radius/2-1, static_marker_color);
						else
							cross(track_pixels, tracker[i].loc[0], tracker[i].loc[1], tracker[i].radius/2-1, inbetween_marker_color);
					else
						cross(track_pixels, tracker[i].loc[0], tracker[i].loc[1], tracker[i].radius/2-1, invalid_marker_color);
					cross(track_pixels, tracker[i].loc[0], tracker[i].loc[1], tracker[i].radius/2, tracker[i].colour);

				}
				update_display(&track_pixels[0], &track_texture, &track_renderer);
				for(uint32_t i=0; i<DataSource::x_dim*DataSource::y_dim; i++)
	   				track_pixels[i] = background;
/*
				while(SDL_PollEvent( &SDL_keyboard_event ))
				{
					if(SDL_keyboard_event.type == SDL_QUIT)
					{	
						exit_threads[0] = 1;
						printf("TD track display triggered exit_threads \n");
					}
					else
						if(SDL_keyboard_event.type == SDL_KEYDOWN)
							if(SDL_keyboard_event.key.keysym.sym == SDLK_ESCAPE)
							{
								exit_threads[0] = 1;
								printf("TD track display triggered exit_threads \n");
							}
				}
				*/
				last_TDtracker_display_time = end_time;
			}
		}
		if(exit_thread == 1)
		{
			printf("Show Tracker quitting \n");
			SDL_DestroyWindow(track_window );
			SDL_DestroyRenderer(track_renderer);
			//Quit SDL subsystems
			SDL_Quit();
			return 1;
		}
		return 0;
	}
/*
	bool showTD_track(packet_info_type* packet_info, event* dataIN, bool exit_thread)
	{

		if(TDwindow == NULL)
		{
			printf("Initializing SDL for TD \n");
			if(SDL_Init( SDL_INIT_VIDEO ) <0)
				printf("SDL_Init failed: %s\n", SDL_GetError());	
			TDwindow = SDL_CreateWindow( "TD Window", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, DataSource::x_dim, DataSource::y_dim, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
			if(TDwindow == NULL)
				printf("ERROR: failed to create SDL window for TD\n");

			TDscreenSurface = SDL_GetWindowSurface( TDwindow );
			SDL_FillRect( TDscreenSurface, NULL, SDL_MapRGB( TDscreenSurface->format, 0x80, 0x80, 0x80) );

			TDpixels = (uint32_t *)TDscreenSurface->pixels; 
		}

		int TD_track_display_MSBs = packet_info->time_start;
		int end_time = dataIN[packet_info->num_events-1].t | packet_info->time_end;
	
		for(int eventNum=0; eventNum<packet_info->num_events; eventNum++)
		{
			if (dataIN[eventNum].type == evt_type::TD)
			{
				if (dataIN[eventNum].subtype == TD_subtype::ON)
					TDpixels[ (Uint32(dataIN[eventNum].y) * TDscreenSurface->w ) + dataIN[eventNum].x] = ONcolor;
				else
					TDpixels[ (Uint32(dataIN[eventNum].y) * TDscreenSurface->w ) + dataIN[eventNum].x] = OFFcolor;
			}else
			if (dataIN[eventNum].type == evt_type::Tracked)
			{
				TDpixels[ (Uint32(dataIN[eventNum].y) * TDscreenSurface->w ) + dataIN[eventNum].x] = tracker[dataIN[eventNum].subtype].colour;
			}
			else
			if (dataIN[eventNum].type == evt_type::timer_overflow)
				TD_track_display_MSBs += 1<<16;
		}
		
		if(end_time-last_TDtracker_display_time >= display_interval)
		{
			for(int i=0; i<num_trackers; i++)
			{
//				printf("Tracker %i is at location %f, %f, and has a tdiff_mean of %f, and is active (true/false) %i\n",i,tracker[i].loc[0],tracker[i].loc[1], tracker[i].tdiff_mean, tracker[i].active);
				if(tracker[i].active == 1)
					if (end_time-tracker[i].active_time > active_time_threshold)
						if (tracker[i].dist[0]*tracker[i].dist[0] + tracker[i].dist[1]*tracker[i].dist[1] > average_distance_threshold)
							cross(TDpixels, tracker[i].loc[0], tracker[i].loc[1], 9, marker_color);
						else
							cross(TDpixels, tracker[i].loc[0], tracker[i].loc[1], 9, static_marker_color);
					else
						cross(TDpixels, tracker[i].loc[0], tracker[i].loc[1], 9, inbetween_marker_color);
				else
					cross(TDpixels, tracker[i].loc[0], tracker[i].loc[1], 9, invalid_marker_color);
				cross(TDpixels, tracker[i].loc[0], tracker[i].loc[1], 10, tracker[i].colour);

			}
			SDL_UpdateWindowSurface( TDwindow );
			SDL_FillRect(TDscreenSurface, NULL, SDL_MapRGB(TDscreenSurface->format, 0x80, 0x80, 0x80));	
			while(SDL_PollEvent( &SDL_keyboard_event ))
			{
				if(SDL_keyboard_event.type == SDL_QUIT)
				{	
					exit_threads[0] = 1;
					printf("TD track display triggered exit_threads \n");
				}
				else
					if(SDL_keyboard_event.type == SDL_KEYDOWN)
						if(SDL_keyboard_event.key.keysym.sym == SDLK_ESCAPE)
						{
							exit_threads[0] = 1;
							printf("TD track display triggered exit_threads \n");
						}
			}
			last_TDtracker_display_time = end_time;
		}

		if(exit_thread == 1)
		{
			printf("Show Tracker quitting \n");
			SDL_DestroyWindow(TDwindow);
			//Quit SDL subsystems
			SDL_Quit();
			return 1;
		}
		return 0;
	}*/

/*	void cross(uint32_t *TDpixels, int x, int y, int radius, uint32_t color)
	{
//		printf("Drawing tracker \n");
		int x_start, x_end, y_start, y_end;
		x_start = x-radius;					
		y_start = y-radius;
		x_end = x+radius;					
		y_end = y+radius;
		if(x_start<0)
			x_start=0;
		if(y_start<0)
			y_start=0;
		if(x_end>=DataSource::x_dim)
			x_end=(DataSource::x_dim-1);
		if(y_end>=DataSource::y_dim)
			y_end=(DataSource::y_dim-1);
		
		int x_loc = x_start;
		for(int y_loc = y_start; y_loc<y_end; y_loc++)
			TDpixels[(Uint32(y_loc) * DataSource::x_dim ) + Uint32(x_loc)] =  color;
		x_loc = x_end;
		for(int y_loc = y_start; y_loc<y_end; y_loc++)
			TDpixels[(Uint32(y_loc) * DataSource::x_dim ) + Uint32(x_loc)] =  color;

		int y_loc = y_start;		
		for(x_loc = x_start; x_loc<x_end; x_loc++)
			TDpixels[(Uint32(y_loc) * DataSource::x_dim ) + Uint32(x_loc)] = color;
		y_loc = y_end;		
		for(x_loc = x_start; x_loc<x_end; x_loc++)
			TDpixels[(Uint32(y_loc) * DataSource::x_dim ) + Uint32(x_loc)] = color;


	}*/
#else

	int init_showTD_track()
	{
		printf("ERROR: program was compiled without OPENCV, video writing not possible \n");
		return 1;
	}

	bool showTD_track(packet_info_type* packet_info, event* dataIN, bool exit_thread)
	{
		printf("ERROR: program was compiled without OPENCV, video writing not possible \n");
		return 0;
	}

	void init_write_video_TD_track(const char* filename)
	{
		printf("ERROR: program was compiled without OPENCV, video writing not possible \n");
		return;
	}

	bool write_video_TD_track(packet_info_type* packet_info, event* dataIN, bool exit_thread)
	{
		printf("ERROR: program was compiled without OPENCV, video writing not possible \n");
		return 0;
	}
#endif
#endif
