//ALL FILTERING FUNCTIONS GO IN THIS FILE
#include <thread_launcher.h>
#include <algorithm>
#include <cstring>

#include <filters.h>

//#ifdef _useTN
//#include "Logging.h"
//#include "ParameterManager.h"
//#endif
#ifdef _useATIS
#include "../inc/OK_interface.h"
#endif

using namespace std;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool rangecheck::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	for(int i=0; i<packet_info->num_events; i++)
	{
		if (dataIN[i].type == evt_type::TD) 
		{
			if ((dataIN[i].x <0) || (dataIN[i].y <0) || (dataIN[i].x >= DataSource::x_dim) || (dataIN[i].y >= DataSource::y_dim))
			{
				dataIN[i].y = 0;
				dataIN[i].x = 0;
			}
		}
	}
	return exit_thread;
}



offset::offset(int x, int y)
{
	x_offset = x;
	y_offset = y;
}

bool offset::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	for(int i=0; i<packet_info->num_events; i++)
	{
		if (dataIN[i].type == evt_type::TD) 
		{
			dataIN[i].x += x_offset;
			dataIN[i].y += y_offset;
		}
	}
	return exit_thread;
}


change_type::change_type(int from_in, int to_in)
{
	from = from_in;
	to = to_in;
}

bool change_type::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	for(int i=0; i<packet_info->num_events; i++)
		if (dataIN[i].type == from) 
			dataIN[i].type = to;

	return exit_thread;
}

/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, (4) Thread ID 
Output      : Returns the Data buffer information packet.
Description : Removes and filtering done by previous functions. Useful if you have accidentally written to file post-filtering, but still want to see the raw data. 
*************************************************************************************************************/
bool unfilter::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	for(int i=0; i<packet_info->num_events; i++)
	{
		if (dataIN[i].type == evt_type::TD_filtered) 
		{
			dataIN[i].type = evt_type::TD;
		}
	}
	return exit_thread;
}


bool discard_filtered::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	int eventNumOut = 0;
	for(int eventNum=0; eventNum<packet_info->num_events; eventNum++)
		if (dataIN[eventNum].type == evt_type::TD)
			if(eventNumOut != eventNum)
				memcpy(&dataIN[eventNumOut++], &dataIN[eventNum], sizeof(event));
		
	packet_info->num_events = eventNumOut;
	
	return exit_thread;
}

// -------------------------------------------------------------------------------------------------------------------

/************************************************************************************************************
Input       : (1) The Nearest Neighbor filter
Output      : None
Description : Sets the time threshold for the Nearest Neighbor filter
************************************************************************************************************/
NN_filter::NN_filter(int input_NN_threshold)
{
	NN_threshold = input_NN_threshold;
	NN_last_spike = new uint64_t*[DataSource::y_dim];
	for(uint64_t i=0; i<DataSource::y_dim; i++)
		NN_last_spike[i] = new uint64_t[DataSource::x_dim];
}


/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, 
              (4) Thread ID 
Output      : Returns the Data buffer information packet.
Description : This function implements the Nearest Neighbors filter. (total of eight pixels) 
************************************************************************************************************/
bool NN_filter::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	uint64_t filterTime_MSBs = uint64_t(packet_info->time_start)<<16;
	uint64_t time;
	uint64_t max_time, time_diff;
	for(int i=0; i<packet_info->num_events; i++)
	{
		if (dataIN[i].type == evt_type::TD) 
		{
			time = filterTime_MSBs | dataIN[i].t;
			NN_last_spike[dataIN[i].y][dataIN[i].x] = time;
			if ((dataIN[i].y<DataSource::y_dim-1) & (dataIN[i].y>0) & (dataIN[i].x>0) & (dataIN[i].x<DataSource::x_dim-1))
			{
				max_time = 0;

				max_time = max(NN_last_spike[dataIN[i].y-1][dataIN[i].x-1], max_time);
				max_time = max(NN_last_spike[dataIN[i].y][dataIN[i].x-1], max_time);
				max_time = max(NN_last_spike[dataIN[i].y+1][dataIN[i].x-1], max_time);

				max_time = max(NN_last_spike[dataIN[i].y-1][dataIN[i].x], max_time);
	//			max_time = max(NN_last_spike[dataIN[i].y][dataIN[i].x], max_time);
				max_time = max(NN_last_spike[dataIN[i].y+1][dataIN[i].x], max_time);

				max_time = max(NN_last_spike[dataIN[i].y-1][dataIN[i].x+1], max_time);
				max_time = max(NN_last_spike[dataIN[i].y][dataIN[i].x+1], max_time);
				max_time = max(NN_last_spike[dataIN[i].y+1][dataIN[i].x+1], max_time);
				time_diff = (time - max_time);
				if (time_diff >= NN_threshold)
				{
					dataIN[i].type = evt_type::TD_filtered;
					//numFilteredEvents++;

				}
			}
		}/*else 
		if (dataIN[i].type == evt_type::timer_overflow)
		{
			filterTime_MSBs += 1<<16;
		}*/
	}
	return exit_thread;
}


/************************************************************************************************************
Input       : (1) The Nearest Neighbor Orientation filter
Output      : None
Description : Sets the time threshold for the Nearest Neighbor Orientation filter with at least
************************************************************************************************************/

NN_orientation_filter::NN_orientation_filter(int input_NN_orientation_threshold)
{
	NN_orientation_threshold = input_NN_orientation_threshold;
	NN_orientation_last_spike = new uint32_t*[DataSource::y_dim];
	for(int y=0; y<DataSource::y_dim; y++)
		NN_orientation_last_spike[y] = new uint32_t[DataSource::x_dim];
}


/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, 
              (4) Thread ID 
Output      : Returns the Data buffer information packet.
Description : This function implements the Orientation Nearest Neighbors filter. (total of eight pixels) 
************************************************************************************************************/
bool NN_orientation_filter::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	uint64_t filterTime_MSBs = uint64_t(packet_info->time_start)<<16;
	uint64_t time;
	uint32_t max_time, time_diff;
	
	for(int i=0; i<packet_info->num_events; i++)
	{
		if (dataIN[i].type == evt_type::Orientation) 
		{
			time = filterTime_MSBs | dataIN[i].t;
			NN_orientation_last_spike[dataIN[i].y][dataIN[i].x] = uint32_t(time);
			if ((dataIN[i].y<DataSource::y_dim-1) & (dataIN[i].y>0) & (dataIN[i].x>0) & (dataIN[i].x<DataSource::x_dim-1))
			{
				max_time = 0;

				max_time = max(NN_orientation_last_spike[dataIN[i].y-1][dataIN[i].x-1], max_time);
				max_time = max(NN_orientation_last_spike[dataIN[i].y][dataIN[i].x-1], max_time);
				max_time = max(NN_orientation_last_spike[dataIN[i].y+1][dataIN[i].x-1], max_time);

				max_time = max(NN_orientation_last_spike[dataIN[i].y-1][dataIN[i].x], max_time);
	//			max_time = max(NN_orientation_last_spike[dataIN[i].y][dataIN[i].x], max_time);
				max_time = max(NN_orientation_last_spike[dataIN[i].y+1][dataIN[i].x], max_time);

				max_time = max(NN_orientation_last_spike[dataIN[i].y-1][dataIN[i].x+1], max_time);
				max_time = max(NN_orientation_last_spike[dataIN[i].y][dataIN[i].x+1], max_time);
				max_time = max(NN_orientation_last_spike[dataIN[i].y+1][dataIN[i].x+1], max_time);
				time_diff = (time - max_time);
				if (time_diff >= NN_orientation_threshold)
				{
					//std::cout << "Filtering orientation event" << std::endl;
					dataIN[i].type = evt_type::Orientation_filtered;
					//numFilteredEvents++;

				}
//				else
//					numNotFilteredEvents++;
			}
		}/*else 
		if (dataIN[i].type == evt_type::timer_overflow)
		{
			filterTime_MSBs += 1<<16;
		}else
		{
			non_TD_events++;
		}*/
	}
	//std::cout << "Events in x < 240 = " << cnt_less240 << ", x > 240 = " << cnt_more240 << std::endl;

	return exit_thread;
}



// ---------------------------------------------------------------------------------------------------------------------
/*int numFilteredEvents = 0;
int numNotFilteredEvents = 0;
int NNp_threshold;
int NNp_last_spike[240][304][2];
*/
/************************************************************************************************************
Input       : (1) The refractory period threshold with polarity
Output      : None
Description : Function to initialize the Refractory filter with Polarity
*************************************************************************************************************/
NN_filter_polarity::NN_filter_polarity(int input_NNp_threshold)
{
	NNp_threshold = input_NNp_threshold;
	NNp_last_spike = new uint64_t**[DataSource::y_dim];
	for(int y=0; y<DataSource::y_dim; y++)
	{	
		NNp_last_spike[y] = new uint64_t*[DataSource::x_dim];
		for(int x=0; x<DataSource::x_dim; x++)
			NNp_last_spike[y][x] = new uint64_t[2];
	}
}

/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, 
              (4) Thread ID 
Output      : Returns the Data buffer information packet.
Description : This function implements the refractory filter with polarity considerations taken into account. 
************************************************************************************************************/
bool NN_filter_polarity::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	uint64_t filterTime_MSBs = uint64_t(packet_info->time_start)<<16;
	uint64_t time;
	uint64_t max_time, time_diff;
	//memcpy(dataIN, dataIN, packet_info->num_events*sizeof(event));
	//int increment = 1+packet_info->num_events/max_events_to_process;
	for(int i=0; i<packet_info->num_events; i++)
	{
		if (dataIN[i].type == evt_type::TD) 
		{
			if ((dataIN[i].y<DataSource::y_dim-1) & (dataIN[i].y>0) & (dataIN[i].x>0) & (dataIN[i].x<DataSource::x_dim-1))
			{
				time = dataIN[i].t | filterTime_MSBs;
				NNp_last_spike[dataIN[i].y][dataIN[i].x][dataIN[i].subtype] = time;
				max_time = 0;

				max_time = max(NNp_last_spike[dataIN[i].y-1][dataIN[i].x-1][dataIN[i].subtype], max_time);
				max_time = max(NNp_last_spike[dataIN[i].y][dataIN[i].x-1][dataIN[i].subtype], max_time);
				max_time = max(NNp_last_spike[dataIN[i].y+1][dataIN[i].x-1][dataIN[i].subtype], max_time);

				max_time = max(NNp_last_spike[dataIN[i].y-1][dataIN[i].x][dataIN[i].subtype], max_time);
				//max_time = max(NNp_last_spike[dataIN[i].y][dataIN[i].x][dataIN[i].subtype], max_time);
				max_time = max(NNp_last_spike[dataIN[i].y+1][dataIN[i].x][dataIN[i].subtype], max_time);

				max_time = max(NNp_last_spike[dataIN[i].y-1][dataIN[i].x+1][dataIN[i].subtype], max_time);
				max_time = max(NNp_last_spike[dataIN[i].y][dataIN[i].x+1][dataIN[i].subtype], max_time);
				max_time = max(NNp_last_spike[dataIN[i].y+1][dataIN[i].x+1][dataIN[i].subtype], max_time);
				time_diff = (time - max_time);
				if (time_diff >= NNp_threshold)
				{
					dataIN[i].type = evt_type::TD_filtered;
					numFilteredEvents++;
				}
				else
					numNotFilteredEvents++;
			}else
			{
				dataIN[i].type = evt_type::TD_filtered;
				numFilteredEvents++;
			}


		}/*else 
		if (dataIN[i].type == evt_type::timer_overflow)
		{
			filterTime_MSBs += 1<<16;
		}else
		{
			non_TD_events++;
		}*/
	}

	return exit_thread;
}


//--------------------------------------------------------------------------------------------------------------------------------

/************************************************************************************************************
Input       : (1) The Nearest Neighbor with 2 event
Output      : None
Description : Sets the time threshold for the Nearest Neighbor filter with at least 2 neighbor events
************************************************************************************************************/
NN2_filter::NN2_filter(int input_NN2_threshold)
{
	NN2_threshold = input_NN2_threshold;
	NN2_last_spike = new uint64_t*[DataSource::y_dim];
	for(uint16_t y=0; y<DataSource::y_dim;y++)
		NN2_last_spike[y] = new uint64_t[DataSource::x_dim];
}


/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, 
              (4) Thread ID 
Output      : Returns information about the output event buffer 
Description : Similar to standard Nearest Neighbor filter, but requires support from at least two neighbouring pixels
************************************************************************************************************/
bool NN2_filter::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	uint64_t filterTime_MSBs = uint64_t(packet_info->time_start)<<16;
	uint64_t time, time_diff;
	uint64_t max_time [8];
	int num_TDevents = 0;
	//memcpy(dataOUT, dataIN, packet_info.num_events*sizeof(event));
	//int increment = 1+packet_info->num_events/max_events_to_process;
	for(int i=0; i<packet_info->num_events; i++)
	{
		if (dataIN[i].type == evt_type::TD) 
		{
			if ((dataIN[i].y<DataSource::y_dim-1) & (dataIN[i].y>0) & (dataIN[i].x>0) & (dataIN[i].x<DataSource::x_dim-1))
			{
				NN2_last_spike[dataIN[i].y][dataIN[i].x] = time;
				time = dataIN[i].t | filterTime_MSBs;
				max_time[0] = NN2_last_spike[dataIN[i].y-1][dataIN[i].x-1];
				max_time[1] = NN2_last_spike[dataIN[i].y][dataIN[i].x-1];
				max_time[2] = NN2_last_spike[dataIN[i].y+1][dataIN[i].x-1];
	
				max_time[3] = NN2_last_spike[dataIN[i].y-1][dataIN[i].x];
	//			max_time[] = NN2_last_spike[dataIN[i].y][dataIN[i].x];
				max_time[4] = NN2_last_spike[dataIN[i].y+1][dataIN[i].x];

				max_time[5] = NN2_last_spike[dataIN[i].y-1][dataIN[i].x+1];
				max_time[6] = NN2_last_spike[dataIN[i].y][dataIN[i].x+1];
				max_time[7] = NN2_last_spike[dataIN[i].y+1][dataIN[i].x+1];
				std::sort(max_time, max_time + 8);
				time_diff = (time - max_time[6]);
				if (time_diff >=  NN2_threshold)
				{
					dataIN[i].type = evt_type::TD_filtered;
	//				numFilteredEvents++;
				}
					//else
		//				numNotFilteredEvents++;
			}else
			{
				dataIN[i].type = evt_type::TD_filtered;	
			}
		}/*else 
		if (dataIN[i].type == evt_type::timer_overflow)
		{
			filterTime_MSBs += 1<<16;
		}else
		{
			non_TD_events++;
		}*/
	}
        
	return exit_thread;
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



/************************************************************************************************************
Input       :(1) The Nearest Neighbor horizontal and vertical threshold (usually 1)
Output      : None
Description : Function to initialize the Nearest Neighbor (horizontal & Vertical) filter 
************************************************************************************************************/
NNhv_filter::NNhv_filter(int input_NNhv_threshold)
{
	NNhv_threshold = input_NNhv_threshold;
	NNhv_last_spike = new uint32_t*[DataSource::y_dim];
	for(int y=0; y<DataSource::y_dim; y++)
		NNhv_last_spike[y] = new uint32_t[DataSource::x_dim];
	
}

/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, 
              (4) Thread ID 
Output      : Returns the Data buffer information packet.
Description : This function implements the Nearest Neighbors filter. Similar to standard filter but does not look 
              at diagonal pixels
************************************************************************************************************/

bool NNhv_filter::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	uint64_t filterTime_MSBs = uint64_t(packet_info->time_start)<<16;
	uint64_t time;
	uint32_t max_time, time_diff;
	//int num_TDevents = 0;
//	memcpy(dataOUT, dataIN, packet_info.num_events*sizeof(event));
	//int increment = 1+packet_info->num_events/max_events_to_process;
	for(int i=0; i<packet_info->num_events; i++)
	{
		if (dataIN[i].type == evt_type::TD) 
		{
			time = dataIN[i].t | filterTime_MSBs;
			

				NNhv_last_spike[dataIN[i].y][dataIN[i].x] = time;
				if ((dataIN[i].y<DataSource::y_dim-1) & (dataIN[i].y>0) & (dataIN[i].x>0) & (dataIN[i].x<DataSource::x_dim-1))
				{
					max_time = 0;
	
					//max_time = max(NNhv_last_spike[dataIN[i].y-1][dataIN[i].x-1], max_time);
					max_time = max(NNhv_last_spike[dataIN[i].y][dataIN[i].x-1], max_time);
					//max_time = max(NNhv_last_spike[dataIN[i].y+1][dataIN[i].x-1], max_time);
	
					max_time = max(NNhv_last_spike[dataIN[i].y-1][dataIN[i].x], max_time);
					//max_time = max(NNhv_last_spike[dataIN[i].y][dataIN[i].x], max_time);
					max_time = max(NNhv_last_spike[dataIN[i].y+1][dataIN[i].x], max_time);
	
					//max_time = max(NNhv_last_spike[dataIN[i].y-1][dataIN[i].x+1], max_time);
					max_time = max(NNhv_last_spike[dataIN[i].y][dataIN[i].x+1], max_time);
					//max_time = max(NNhv_last_spike[dataIN[i].y+1][dataIN[i].x+1], max_time);
					time_diff = (time - max_time);
					if (time_diff >=  NNhv_threshold)
					{
						dataIN[i].type = evt_type::TD_filtered;
					//	numFilteredEvents++;
					}
					//else
					//	numNotFilteredEvents++;
			}
		}/*else 
		if (dataIN[i].type == evt_type::timer_overflow)
		{
			filterTime_MSBs += 1<<16;
		}else
		{
			non_TD_events++;
		}*/
	}

	return exit_thread;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/************************************************************************************************************
Input       : (1) The refractory period
Output      : None
Description : Function to initialize the Refractoy filter.
*************************************************************************************************************/
refractory_filter::refractory_filter(int input_RF_period)
{
	RF_period = input_RF_period;
		RF_last_spike = new uint32_t*[DataSource::y_dim];
	for(int y=0; y<DataSource::y_dim; y++)
		RF_last_spike[y] = new uint32_t[DataSource::x_dim];
}

/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, (4) Thread ID 
Output      : Returns the Data buffer information packet.
Description : Function to initialize the Refractoy filter.
*************************************************************************************************************/
bool refractory_filter::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	uint64_t filterTime_MSBs = uint64_t(packet_info->time_start)<<16;
	uint64_t time;
//	memcpy(dataOUT, dataIN, packet_info.num_events*sizeof(event));
	for(int i=0; i<packet_info->num_events; i++)
	{
		if (dataIN[i].type == evt_type::TD) 
		{
			time = dataIN[i].t | filterTime_MSBs;
			if (time - RF_last_spike[dataIN[i].y][dataIN[i].x] < RF_period)
			{
				dataIN[i].type = evt_type::TD_filtered;
			}else
			{
				RF_last_spike[dataIN[i].y][dataIN[i].x] = time;
			}
		}
/*		else
		if (dataIN[i].type == evt_type::timer_overflow)
		{
			filterTime_MSBs += 1<<16;
		}
*/	}
	return exit_thread;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/************************************************************************************************************
Input       : (1) The refractory period
Output      : None
Description : Function to initialize the Refractoy filter.
*************************************************************************************************************/
refractory_period_polarity::refractory_period_polarity(int input_RFp_period)
{
	RFp_period = input_RFp_period;
	RFp_last_spike = new uint32_t**[DataSource::y_dim];
	for(int y=0; y<DataSource::y_dim; y++)
	{	
		RFp_last_spike[y] = new uint32_t*[DataSource::x_dim];
		for(int x=0; x<DataSource::x_dim; x++)
			RFp_last_spike[y][x] = new uint32_t[2];
	}
}

/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, (4) Thread ID 
Output      : Returns the Data buffer information packet.
Description : Function to implement the refractory filter. Filters off events when their intervals are smaller 
              than the refractory period.
*************************************************************************************************************/
//treats positive and negative events with separate polarities
bool refractory_period_polarity::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	uint64_t filterTime_MSBs = uint64_t(packet_info->time_start)<<16;
	uint64_t time;
//	memcpy(dataOUT, dataIN, packet_info.num_events*sizeof(event));
	//int increment = 1+packet_info->num_events/max_events_to_process;
	for(int i=0; i<packet_info->num_events; i++)
	{
		if (dataIN[i].type == evt_type::TD) 
		{
			time = dataIN[i].t | filterTime_MSBs;
			if (time - RFp_last_spike[dataIN[i].y][dataIN[i].x][dataIN[i].subtype] < RFp_period)
			{
				dataIN[i].type = evt_type::TD_filtered;
			}else
			{
				RFp_last_spike[dataIN[i].y][dataIN[i].x][dataIN[i].subtype] = time;
			}
		}/*
		else
		if (dataIN[i].type == evt_type::timer_overflow)
		{
			filterTime_MSBs += 1<<16;
		}*/
	}
	return exit_thread;
}



//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//char last_polarity[240][304];
//static char polarity_before_last[240][304];

/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, (4) Thread ID 
Output      : Returns the Data buffer information packet.
Description : Function to implement the Differing Polarity filter. Returns an event only if the polarity of the event 
              is different from its previous in the same pixel. 
*************************************************************************************************************/
diff_polarity_filter::diff_polarity_filter()
{
	last_polarity = new char*[DataSource::y_dim];
	for(int y=0; y<DataSource::y_dim; y++)
		last_polarity[y] = new char[DataSource::x_dim];

}

bool diff_polarity_filter::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	//memcpy(dataOUT, dataIN, packet_info.num_events*sizeof(event));
	for(int i=0; i<packet_info->num_events; i++)
	{
		if (dataIN[i].type == evt_type::TD) 
		{
			if (last_polarity[dataIN[i].y][dataIN[i].x] == dataIN[i].subtype) //&& polarity_before_last[dataIN[i].y][dataIN[i].x] == dataIN[i].subtype)
			{
				dataIN[i].type = evt_type::TD_filtered;
			}else
			{
				last_polarity[dataIN[i].y][dataIN[i].x] = dataIN[i].subtype;
                                //last_polarity[dataIN[i-1].y][dataIN[i-1].x] = dataIN[i-1].subtype;
			}
		}
	}
	return exit_thread;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
/*
static int ISI_threshold;                   //ISI filter 
static float ISI_mix;
static float average_ISI[240][304];
static int ISI_last_spike[240][304];
*/
/******************************************************************************************************************
Input       : (1) ISI avarage duration threshold
Output      : None
Description : Initialization function for the ISI filter, removes the need for extern or global variables which can 
              get messy
******************************************************************************************************************/
ISI_filter::ISI_filter(int input_ISI_threshold, float input_ISI_mix)
{
	ISI_mix = input_ISI_mix;
	ISI_threshold = input_ISI_threshold;
	average_ISI = new float*[DataSource::y_dim];
	ISI_last_spike = new uint32_t*[DataSource::y_dim];
	for(int y=0; y<DataSource::y_dim; y++)
	{
		average_ISI[y] = new float[DataSource::x_dim];
		ISI_last_spike[y] = new uint32_t[DataSource::x_dim];
	}
}



/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, (4) Thread ID 
Output      : Returns the Data buffer information packet.
Description : Function to implement the Inter Spike Interval filter. Treats all events as spikes and calculates 
              the average inter spike interval using the exponential moving average. 
*************************************************************************************************************/
bool ISI_filter::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	uint64_t filterTime_MSBs = uint64_t(packet_info->time_start)<<16;
	uint64_t time;
//	memcpy(dataIN, dataIN, packet_info.num_events*sizeof(event));
	for(int i=0; i<packet_info->num_events; i++)
	{
		if (dataIN[i].type == evt_type::TD) 
		{
			time = dataIN[i].t | filterTime_MSBs;
			if (average_ISI[dataIN[i].y][dataIN[i].x] < ISI_threshold)
			{
				dataIN[i].type = evt_type::TD_filtered;
			}
			average_ISI[dataIN[i].y][dataIN[i].x] = average_ISI[dataIN[i].y][dataIN[i].x]*ISI_mix + (time - ISI_last_spike[dataIN[i].y][dataIN[i].x])*(1-ISI_mix);
			ISI_last_spike[dataIN[i].y][dataIN[i].x] = time;
		}
	}
	return exit_thread;
}



//---------------------------------------------------------------------------------------------------------------------------------

/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, 
              (4) Thread ID 
Output      : Returns information about the output event buffer 
Description : This filter keeps an event if min_num_spikes events occurred within a defined square region of pixels 
	      (surrounding the pixel which registered the current event) within the last NN_t_length microseconds. 
              The region is a square of side NN_side_length*2 - 1. Regions which would overlap the edges of the frame 
	      are kept the same shape, however the pixel responsible for the current event is off-centre.
************************************************************************************************************/
filter_NNg::filter_NNg(int input_NN_side_length, int input_NN_t_length, int input_min_num_spikes)
{
	
	NN_side_length = input_NN_side_length;
	NN_t_length = input_NN_t_length;
	min_num_spikes = input_min_num_spikes;

	// Initialise last spike times to -1000 seconds
	NNg_last_spike = new uint32_t*[DataSource::y_dim];
	for(int y = 0; y < DataSource::y_dim; y++)
	{
		NNg_last_spike[y] = new uint32_t[DataSource::x_dim];
		for(int x = 0; x <DataSource::x_dim; x++)
			NNg_last_spike[y][x] = -1000000000;	
	}
}


/************************************************************************************************************
Input       : (1) Data buffer information. (2) The input data buffer, (2) The output data buffer, 
              (4) Thread ID 
Output      : Returns information about the output event buffer 
Description : The noise filter that computes the number of activations within the NN boundaries across the specified 
              time threshold  
************************************************************************************************************/
bool filter_NNg::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{

	uint64_t filterTime_MSBs = uint64_t(packet_info->time_start)<<16;
	
//	memcpy(dataOUT, dataIN, packet_info.num_events*sizeof(event));
	//int increment = 1+packet_info->num_events/max_events_to_process;
	for(int i=0; i<packet_info->num_events; i++)
	{
		if (dataIN[i].type == evt_type::TD) 
		{	
				// Look at region in space: Take pixels in each direction from centre

				current_y = dataIN[i].y;
				current_x = dataIN[i].x;			
				current_t = dataIN[i].t | filterTime_MSBs;				

				num_pixels_within_thresh = 0;
				offset_x = 0;
				offset_y = 0;

				// Deal with edge cases (do all comparisons here). Shift square asymmetrically around
				// current pixel.

				if(current_x < NN_side_length)
				{
					offset_x = NN_side_length - current_x;	
				}
				else if(current_x > DataSource::x_dim - NN_side_length)
				{
					offset_x = -(NN_side_length - current_x);	
				}	
							
				if(current_y < NN_side_length)
				{
					offset_y = NN_side_length - current_y;	
				}
				else if(current_y > DataSource::y_dim - NN_side_length)
				{
					offset_y = -(NN_side_length - current_y);	
				}

				for(int j = 1; j < NN_side_length*2; j = j + 1)
				{
			
					for(int k = 1; k < NN_side_length*2; k = k + 1)
					{

				 	// Look for last value here
						if(current_t - NNg_last_spike[current_y - NN_side_length + j + offset_y][current_x - NN_side_length + k + offset_x] < NN_t_length)
						{		
							num_pixels_within_thresh = num_pixels_within_thresh + 1;

				 		}
					}
				}
							
				NNg_last_spike[current_y][current_x] = dataIN[i].t | filterTime_MSBs;

				// Filter out pixels that don't fit within the time threshold
				if (num_pixels_within_thresh < min_num_spikes)
				{
				dataIN[i].type = evt_type::TD_filtered;
				}
				
		}/*else 
		if (dataIN[i].type == evt_type::timer_overflow)
		{
			filterTime_MSBs += 1<<16;
		}*/
	}

	return exit_thread;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


filter_ROI::filter_ROI(int x, int y, int x_size, int y_size)
{
	printf("Initialized ROI with x=%i y=%i x_size=%i y_size=%i\n", x, y, x_size, y_size);
	ROI_x = x;
	ROI_y = y;
	ROI_x_size = x_size;
	ROI_y_size = y_size;
}

bool filter_ROI::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	
	for(int event_num=0; event_num<packet_info->num_events; event_num++)
	{
		if (dataIN[event_num].type == evt_type::TD)
		{
			if ((dataIN[event_num].x > ROI_x)   &   (dataIN[event_num].x < ROI_x + ROI_x_size)   &   (dataIN[event_num].y > ROI_y)   &   (dataIN[event_num].y < ROI_y + ROI_y_size))
			{
				//printf("Event at %i %i of type %i \n", dataIN[event_num].x, dataIN[event_num].y,dataIN[event_num].type);
				dataIN[event_num].type = evt_type::TD_filtered;
				//printf("Changed to type %i \n",dataIN[event_num].type);
			}
		}
	}

	return exit_thread;
}

filter_ROImask::filter_ROImask(std::string mask_file)
{
	std::ifstream input_file;
	input_file.open(mask_file.c_str(), std::ios::in | std::ios::binary);
	if (!input_file.is_open())
		printf("Error. ROI mask file not found\n");
	// Open the file and read the mask file
	// TODO implement checks?
	// Initialize the array
	mask = new char[DataSource::y_dim * DataSource::x_dim];
	input_file.read(mask, DataSource::y_dim * DataSource::x_dim * sizeof(char));
}

bool filter_ROImask::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	for(int event_num=0; event_num<packet_info->num_events; event_num++)
	{
		if (dataIN[event_num].type == evt_type::TD)
		{
			if (mask[dataIN[event_num].y * DataSource::x_dim + dataIN[event_num].x] == 0)
			{
				//printf("Event at %i %i of type %i \n", dataIN[event_num].x, dataIN[event_num].y,dataIN[event_num].type);
				dataIN[event_num].type = evt_type::TD_filtered;
				//printf("Changed to type %i \n",dataIN[event_num].type);
			}
		}
	}
	return exit_thread;
}

//#ifdef _useTN
//tn_filter::tn_filter(std::string model_path_in, std::string input_conn_path_in, std::string output_conn_path_in, int width_in, int height_in, int num_channels_in, std::string output_sfcp_path_in)
//{
//	init(model_path_in, input_conn_path_in, output_conn_path_in, width_in, height_in, num_channels_in);
//
//	// save the output as an sfcp
//	os = new std::ofstream(output_sfcp_path_in, std::ofstream::binary);
//	channel->register_output_stream(*os);
//}
//
//tn_filter::tn_filter(std::string model_path_in, std::string input_conn_path_in, std::string output_conn_path_in, int width_in, int height_in, int num_channels_in, std::string output_ip_in, int output_port_in)
//{
//	init(model_path_in, input_conn_path_in, output_conn_path_in, width_in, height_in, num_channels_in);
//
//	// send the output as sfcp over tcp
//	os_buffer = new tnt::TcpOstreamBuffer(output_ip_in, output_port_in);
//	os = new std::ostream(os_buffer);
//	channel->register_output_stream(*os);
//}
//
//void tn_filter::init(std::string model_path_in, std::string input_conn_path_in, std::string output_conn_path_in, int width_in, int height_in, int num_channels_in)
//{
//	width = width_in;
//	height = height_in;
//	num_channels = num_channels_in;
//	last_tick_time_ms = 0;
//
//	// Initialize TN parameters
//	tnt::ParameterManager_ptr manager = tnt::make_manager_ptr("tn-filter","TN filter in EKF to send spikes to TN.");
//	tnt::Parameter<std::string> model("model", "The .tnbm model file to load.", model_path_in);
//	tnt::Parameter<std::string> in_connector("in-connector", "The input connector binary file.", input_conn_path_in);
//	tnt::Parameter<std::string> out_connector("out-connector", "The output connector binary file.", output_conn_path_in);
//	manager->register_parameter(model, in_connector, out_connector);
//
//	// Check that channel data is good
//	std::ifstream model_file(model());
//	if (!model_file.good()) {
//		throw std::runtime_error("Error opening TN model file.");
//	}
//
//	std::ifstream in_conn_file(in_connector());
//	if (!in_conn_file.good()) {
//		throw std::runtime_error("Error opening TN input connector file.");
//	}
//
//	std::ifstream out_conn_file(out_connector());
//	if (!out_conn_file.good()) {
//		throw std::runtime_error("Error opening TN output connector file.");
//	}
//
//	// Instantiate a channel.
//	channel = new tnr::TnChannel(model(), in_connector(), out_connector());
//}
//
//bool tn_filter::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread) {
//	try
//	{
//		const uint32_t num_events = packet_info->num_events;
//		const uint64_t filterTime_MSBs = uint64_t(packet_info->time_start)<<16;
//		uint64_t time_us;
//		uint64_t num_ticks;
//
//		for (int i = 0; i < num_events; i++) {
//			if (dataIN[i].type == evt_type::TD) {
//				time_us = dataIN[i].t | filterTime_MSBs;
//				if (last_tick_time_ms == 0) {
//					last_tick_time_ms = time_us / 1000;
//				} else if ((time_us / 1000) > last_tick_time_ms) {
//					num_ticks = (time_us / 1000) - last_tick_time_ms;
//					last_tick_time_ms = time_us / 1000;
//
//					for (int j = 0; j < num_ticks; j++) {
//						channel->advance_tick();
//					}
//				}
//
//				send_event_to_tn(dataIN[i]);
//			}
//		}
//
//		if(exit_thread)
//		{
//			channel->close();
//		}
//
//	} catch (std::exception const & e) {
//		LOG(Error) << (e.what());
//		channel->close();
//		return 1;
//	}
//
//	return exit_thread;
//}
//
//void tn_filter::send_event_to_tn(event evt) {
//	// printf("%i, %i, type: %i, subtype: %i\n", evt.x, evt.y, evt.type, evt.subtype);
//	// subtype holds the polarity (0, 1, etc)
//	// [x, y, p] maps to pin ((p*(width*height)) + (x*height) + y)
//	try {
//		uint32_t target_pin = (evt.subtype * width * height) + (evt.x * height) + evt.y;
//		//printf("\tpin: %d, id: %d\n", target_pin, channel->input_connectors().front().id);
//		channel->push(channel->input_connectors().front().id, target_pin);
//	} catch (std::exception const & e) {
//		LOG(Error) << (e.what());
//	}
//}
//
//#ifdef _useATIS
//tn_usb::tn_usb(std::string model_path_in, std::string input_conn_path_in, std::string output_conn_path_in, int width_in, int height_in, int num_channels_in, std::string class_map_path)
//{
//	// TODO properly initialize the class map from some file
//	class_map = new int [4080];
//	for (int i = 0; i < 4080; i++) {
//		class_map[i] = i % 7;
//	}
//	confidence_matrix = new int[7];
//	init(model_path_in, input_conn_path_in, output_conn_path_in, width_in, height_in, num_channels_in);
//
//
//	//Register callback to process output spikes
//	channel->register_spike_callback([this](std::vector<tnr::spike_t> const & data)
//			{
//				for (tnr::spike_t const & spike : data) {
//					std::cout << "\tspike " << spike.pin << std::endl;
//					++confidence_matrix[class_map[spike.pin]];
//				}
//			});
//
//	//Register callback to process the end of a tick
//	channel->register_tick_callback([this](std::uint_fast32_t const tick)
//				{
//					std::cout << "Tick " << tick << " complete." << std::endl;
//
//					// get a lock on the usb interface
//					pthread_mutex_lock(&OkInterface::ok_mutex);
//					// TODO send confidence matrix to opal kelly usb
//					// OkInterface::Instance->doSomethings();
//					// unlock
//					pthread_mutex_unlock(&OkInterface::ok_mutex);
//
//					// clear the confidence_matrix table
//					for (int i=0; i<7; i++) {
//						std::cout << "\t" << confidence_matrix[i];
//						confidence_matrix[i]=0;
//					}
//				});
//}
//
//void tn_usb::init(std::string model_path_in, std::string input_conn_path_in, std::string output_conn_path_in, int width_in, int height_in, int num_channels_in)
//{
//	width = width_in;
//	height = height_in;
//	num_channels = num_channels_in;
//	last_tick_time_ms = 0;
//
//	// Initialize TN parameters
//	tnt::ParameterManager_ptr manager = tnt::make_manager_ptr("tn-filter","TN filter in EKF to send spikes to TN.");
//	tnt::Parameter<std::string> model("model", "The .tnbm model file to load.", model_path_in);
//	tnt::Parameter<std::string> in_connector("in-connector", "The input connector binary file.", input_conn_path_in);
//	tnt::Parameter<std::string> out_connector("out-connector", "The output connector binary file.", output_conn_path_in);
//	manager->register_parameter(model, in_connector, out_connector);
//
//	// Check that channel data is good
//	std::ifstream model_file(model());
//	if (!model_file.good()) {
//		throw std::runtime_error("Error opening TN model file.");
//	}
//
//	std::ifstream in_conn_file(in_connector());
//	if (!in_conn_file.good()) {
//		throw std::runtime_error("Error opening TN input connector file.");
//	}
//
//	std::ifstream out_conn_file(out_connector());
//	if (!out_conn_file.good()) {
//		throw std::runtime_error("Error opening TN output connector file.");
//	}
//
//	// Instantiate a channel.
//	channel = new tnr::TnChannel(model(), in_connector(), out_connector());
//}
//
//bool tn_usb::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread) {
//	try
//	{
//		const uint32_t num_events = packet_info->num_events;
//		const uint64_t filterTime_MSBs = uint64_t(packet_info->time_start)<<16;
//		uint64_t time_us;
//		uint64_t num_ticks;
//
//		for (int i = 0; i < num_events; i++) {
//			if (dataIN[i].type == evt_type::TD) {
//				time_us = dataIN[i].t | filterTime_MSBs;
//				if (last_tick_time_ms == 0) {
//					last_tick_time_ms = time_us / 1000;
//				} else if ((time_us / 1000) > last_tick_time_ms) {
//					num_ticks = (time_us / 1000) - last_tick_time_ms;
//					last_tick_time_ms = time_us / 1000;
//
//					for (int j = 0; j < num_ticks; j++) {
//						channel->advance_tick();
//					}
//				}
//
//				send_event_to_tn(dataIN[i]);
//			}
//		}
//
//		if(exit_thread)
//		{
//			channel->close();
//		}
//
//	} catch (std::exception const & e) {
//		LOG(Error) << (e.what());
//		channel->close();
//		return 1;
//	}
//
//	return exit_thread;
//}
//
//void tn_usb::send_event_to_tn(event evt) {
//	// printf("%i, %i, type: %i, subtype: %i\n", evt.x, evt.y, evt.type, evt.subtype);
//	// subtype holds the polarity (0, 1, etc)
//	// [x, y, p] maps to pin ((p*(width*height)) + (x*height) + y)
//	try {
//		uint32_t target_pin = (evt.subtype * width * height) + (evt.x * height) + evt.y;
//		//printf("\tpin: %d, id: %d\n", target_pin, channel->input_connectors().front().id);
//		channel->push(channel->input_connectors().front().id, target_pin);
//	} catch (std::exception const & e) {
//		LOG(Error) << (e.what());
//	}
//}
//
//tn_usb::~tn_usb() {
//	delete [] class_map;
//}
//#endif // _useATIS
//#endif // _useTN
