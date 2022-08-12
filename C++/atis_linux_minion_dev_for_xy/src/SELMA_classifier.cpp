#include <algorithm>
#include <cstdint>
#include <exception>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <string>
#include <system_error>
#include <thread>
#include <chrono>

#include <boost/asio.hpp>
#include <boost/cstdint.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include "SELMA_classifier.h"
#ifdef _useATIS
#include "OK_interface.h"
#endif
#include "protobuf/tn_message.pb.h"
#include "protobuf/events.pb.h"

int SELMA_classifier::g_track_id=0;
int SELMA_classifier::prev_track_id=0;

SELMA_classifier::SELMA_classifier()
{
	std::string logPrefix = SELMA_classifier::logPrefix + "Constructor(): ";
	std::cout << (logPrefix + "Constructed SELMA CLassifier\n");

	configSELMA();
//	 std::pair<bool,bool>  isSELMANotWorking = testSELMAFull();
//	 bool isSELMANotIncreasing = isSELMANotWorking.first;
//	 bool isSELMA2ndStageNotCorrect = isSELMANotWorking.second;
//	 if(isSELMA2ndStageNotCorrect)
//	 {
//	 	std::cout << logPrefix + "Selma Output 2nd stage is inaccurate\n";
//	 }else
//	 {
//	 	std::cout << logPrefix + "Selma Output 2nd stage is matching\n";
//	 }
//
//
//	//bool isSELMANotIncreasing = testSELMA();
//	// bool isSELMANotIncreasing = false;
//	if(!isSELMANotIncreasing)
//	{
//		std::cout << logPrefix + "Selma Output 1st stage is Working as Expected\n";
//	}else
//	{
//		std::cout << logPrefix + "Selma Output 1st stage is not Increasing\n";
//	}

//	initClassificationState();
//	initTrackerState(SELMA_NUM_TRACKERS);
}

SELMA_classifier::SELMA_classifier(std::string ip_add_in, int port_in) : SELMA_classifier()
{
	std::string logPrefix = SELMA_classifier::logPrefix + "Constructor(IP,PORT): ";
	// std::cout << (logPrefix + "Setting Up SELMA CLassifier Networked Visualization\n");
	vis = true;
	// Setting Up TCP Connections
	try {

		std::cout << logPrefix + "Initalizing Ports and IP Address\n";
		socket = new boost::asio::ip::tcp::socket(io_service);
		messageBody = new std::string();
		boost::asio::ip::tcp::resolver resolver(io_service);
		boost::asio::ip::tcp::resolver::query query1(ip_add_in,
				boost::lexical_cast<std::string>(port_in));
		boost::asio::ip::tcp::resolver::iterator endpoint = resolver.resolve(query1);
		std::cout << logPrefix + "Connecting to IP Address\n";
		connect(*socket, endpoint);
		std::cout << logPrefix + "Connected to IP Address\n";
	} catch (std::system_error &e) 
	{
		std::cerr << "SELMAC TCP Connection Setup Exception: " << e.what() << std::endl;
	}
}

SELMA_classifier::SELMA_classifier(std::string ip_add_in, int track_port_in, int evt_port_in) : SELMA_classifier()
{
	std::string logPrefix = SELMA_classifier::logPrefix + "Constructor(IP,PORT): ";
	// std::cout << (logPrefix + "Setting Up SELMA CLassifier Networked Visualization\n");
	vis = true;
	vis_evt = true;
	// Setting Up TCP Connections
	try {

		std::cout << logPrefix + "Initalizing Ports and IP Address For Tracker Update\n";
		socket = new boost::asio::ip::tcp::socket(io_service);
		messageBody = new std::string();
		boost::asio::ip::tcp::resolver resolver(io_service);
		boost::asio::ip::tcp::resolver::query query1(ip_add_in,
				boost::lexical_cast<std::string>(track_port_in));
		boost::asio::ip::tcp::resolver::iterator endpoint = resolver.resolve(query1);
		std::cout << logPrefix + "Connecting to IP Address\n";
		connect(*socket, endpoint);
		std::cout << logPrefix + "Connected to IP Address\n";

		std::cout << logPrefix + "Initalizing Ports and IP Address For Event Update\n";
		evt_socket = new boost::asio::ip::tcp::socket(evt_io_service);
		boost::asio::ip::tcp::resolver::query query2(ip_add_in
,				boost::lexical_cast<std::string>(evt_port_in));
		boost::asio::ip::tcp::resolver resolver2(evt_io_service);

		boost::asio::ip::tcp::resolver::iterator evt_endpoint = resolver2.resolve(query2);
		std::cout << logPrefix + "Connecting to IP Address\n";
		connect(*evt_socket, evt_endpoint);
		std::cout << logPrefix + "Connected to IP Address\n";
	} catch (std::system_error &e) 
	{
		std::cerr << "SELMAC TCP Connection Setup Exception: " << e.what() << std::endl;
	}
}

SELMA_classifier::SELMA_classifier(std::string test_folder, std::string csv_filename) : SELMA_classifier()
{
	passDataThroughSELMA(test_folder , csv_filename);	

	// std::string test_folder = "/home/yanxian/Desktop/MATLAB.lnk/NeuProwler_Clustering/Data/Raw/NUS_Traffic_Scenes/time_binned_data/onlyNov/";
		// for(int i=9; i>=0; i--)
		// {
		// 	uint32_t bin_window = 2 << i;
		// 	std::string test_file = "_32x32y_"+std::to_string(bin_window)+"ms_tlc_digitalScale.csv";
		// 	//printf("%d : = %s\n",bin_window, test_file.c_str());
		// }
}

SELMA_classifier::SELMA_classifier(std::string output_filename_in) : SELMA_classifier()
{
	output_filename = output_filename_in;
	initAnnotationWriting();
}

void SELMA_classifier::initAnnotationWriting()
{
	writeAnnotationsHead();
	writeAnnotations = true;
}

// SELMA_classifier::~SELMA_classifier()
// {
// 	// NO IDEA how to CLOSE ASYNC SOCKET
// 	// TODO: Find out and Do cleanup properly
// 	// if(socket != NULL)
// 	// {
// 	// 	int retCode = close(socket);
// 	// }
// }


void SELMA_classifier::initClassificationState()
{
	event_rates = new uint16_t*[SELMA_NUM_TRACKERS];
	for(int i=0; i< SELMA_NUM_TRACKERS; i++)
	{
		event_rates[i] = new uint16_t[SELMA_NUM_INPUT_CHANNELS];
	}
	track_to_classify = 0 ;
	last_classification_time = 0;   

	// // Coefficient Initialization
	// coefficients = new int*[SELMA_NUM_CLASSES];
	// for(int i=0; i< SELMA_NUM_CLASSES; i++)
	// {
	// 	coefficients[i] = new int[SELMA_NUM_OUTPUT_CHANNELS];
	// }
}

void SELMA_classifier::initTrackerState(int num_tracks)
{
	std::string logPrefix = SELMA_classifier::logPrefix + "initTrackerState(): ";

	num_trackers = num_tracks;
	next_id = 1;
	overflow_ctr = 0;

	// distance_threshold = SELMA_INIT_RADIUS * SELMA_INIT_RADIUS;	
	distance_threshold = 2 * SELMA_INIT_RADIUS;	
	distance = (int*) malloc(sizeof(int) * num_trackers);
	for(int i=0; i<num_trackers; i++)
	{
		distance[i] = 0;
	}

	// std::cout << logPrefix + "Initialized Distance Between Tracks" << std::endl;
	trackers = (selma_tracker_type*) malloc(
			sizeof(selma_tracker_type) * num_trackers);
	for (int i = 0; i < num_trackers; i++) {
		// std::cout << logPrefix + "Initializing Tracker " + std::to_string(i) << std::endl;
		// std::cout << logPrefix + "Dimensions : "+ std::to_string(DataSource::x_dim) + "," + std::to_string(DataSource::y_dim) + ")\n";
		trackers[i].loc[0] = std::rand() % DataSource::x_dim;
		trackers[i].loc[1] = std::rand() % DataSource::y_dim;

		trackers[i].last_loc[0] = trackers[i].loc[0];
		trackers[i].last_loc[1] = trackers[i].loc[1];
		trackers[i].radius = SELMA_INIT_RADIUS;
		trackers[i].colour = rand()%0xFFFFFF;
		trackers[i].id = next_id;
		trackers[i].label = 7;
		trackers[i].active = 0;
		// for (int j =0 ; j< SELMA_NUM_CLASSES; j++)
		// {
		// 	trackers[i].stage2_data[j] = 0;
		// }

		printf("Tracker randomly initialized to location [%.1f, %.1f]\n",
				trackers[i].loc[0], trackers[i].loc[1]);
	}
}

bool SELMA_classifier::processData(
	packet_info_type* packet_info, 
	event* dataIN,
	bool exit_thread) 
{
	uint16_t delays[1024] = {0};
	for(uint32_t j=1; j<1024; j++)
	{
		delays[j] = getDelayVal(0);
	}
	std::pair<uint16_t*,int64_t*>  output = sendInputToSELMA(delays);
	// return 1;
	if(isDonePassingData)
	{
		return 1; // Passing Data Should be done in constructor
	}

	std::string logPrefix = SELMA_classifier::logPrefix + "processData(): ";

	uint64_t track_time_MSBs = uint64_t(packet_info->time_start) << 16;
	uint64_t time;

	// Event Visualization Code::
	

	if (packet_info->packet_type == packet_type::TD_packet
			|| packet_info->packet_type == packet_type::TD_EM_mixed_packet) 
	{
		// std::cout << logPrefix + "Processing Incoming TD PACKET";
		for(uint32_t eventNum=0; eventNum<packet_info->num_events; eventNum++) //loop through the output buffer events  
		{  
			epf::Events events = epf::Events();
			if (dataIN[eventNum].type == evt_type::TD ) 
				// dataIN[eventNum].type == evt_type::TD_filtered) 
			{
				time = track_time_MSBs | dataIN[eventNum].t;
				if (dataIN[eventNum].type == evt_type::TD)
				{
					processEvent(&dataIN[eventNum], time);
					// Do All the Update Here?  
				}
				if (vis_evt)
				{
					// Event Visualization Processing
					epf::Event * evt = events.add_evt();
					evt->set_x(dataIN[eventNum].x);
					evt->set_y(dataIN[eventNum].y);
					evt->set_p(dataIN[eventNum].subtype);	
				}
			}
//			else if(dataIN[eventNum].type == evt_type::TD_tracker)
//			{
//				printf("Detected TD Tracker\n");
//				//Tracker Info Extraction - RS					
//				int evt_subtype_id = dataIN[eventNum].subtype;
//				int tracker_id = evt_subtype_id & TD_tracker_subtype_mask::TRACKER_MASK;	
//				tracker_id  = tracker_id >>TD_tracker_subtype_mask::BITSHIFT_AMT_TYPE; 
//				int tracker = tracker_id % 8;
//				//Single Tracker Solution. Other Trackers are generating events but are not integrated in the pipeline
//				if(tracker ==0){
//					//printf ("Processing for Tracker : %i\n", tracker);
//					int evt_sub_type = floor(tracker_id/8);
//						
//					//Track Info Extraction - RS
//					int track_id = evt_subtype_id & TD_tracker_subtype_mask::TRACK_MASK;
//					if(tracker==0 && (prev_track_id != track_id))
//					{
//							g_track_id++;
//							prev_track_id = track_id;
//					}
//					//printf("Event Num %i : Tracker %i - (X=%i,Y=%i), subtype - %i, g_track_id = %i, track_id = %i, prev_track_id = %i\n", eventNum, \
//							tracker, dataIN[eventNum].x, dataIN[eventNum].y, evt_sub_type, g_track_id, track_id, prev_track_id);
//				
//					printf("Tracker %i, track_id = %i, prev_track_id = %i, g_track_id = %i\n", tracker,track_id, prev_track_id,  g_track_id);
//					//Ambiguous Naming previosuly. [TODO: clarify]
//					int track_number = tracker;
//
//					 switch(evt_sub_type)
//				 	{
//				 		case TD_tracker_subtype_types::MIN:
//				 		{
//
//				 			// MIN EVENTS						
//				 			tracks[track_number].min[0] = dataIN[eventNum].x;
//				 			tracks[track_number].min[1] = dataIN[eventNum].y;
//				 			printf("Tracker %i MIN X: %i MIN Y: %i for time %i \n", track_number, tracks[track_number].min[0], tracks[track_number].min[1], time);
//				 			break;
//				 		}
//				 		case TD_tracker_subtype_types::MAX:
//				 		{	// MAX EVENTS
//				 			tracks[track_number].max[0] = dataIN[eventNum].x;
//				 			tracks[track_number].max[1] = dataIN[eventNum].y;
//				 			printf("Tracker %i MAX X: %i MAX Y: %i for time %i \n", track_number, tracks[track_number].max[0], tracks[track_number].max[1], time);
//				 			break;
//				 		}
//				 		default:
//				 		{
//				 			printf("Unexpected Event SubType = %d\n", evt_sub_type);
//				 		}
//					}
//					if ((tracks[track_number].min[0] < tracks[track_number].max[0])
//						&& (tracks[track_number].min[1] < tracks[track_number].max[1])){
//						tracks[track_number].active = true;
//						tracks[track_number].id = g_track_id;
//						//printf("track active = %i\n",track_number);
//					}
//					//printf("track_active - step \n");
//				}
//			}
			if (events.evt_size() > 0) 
			{
				int msg_size = events.ByteSize();

				data_buffer.resize(msg_size + SELMA_PKT_HEADER_SIZE);

				//encode 4 bytes header to denote size of message body
				data_buffer[0] = static_cast<boost::uint8_t>((msg_size >> 24) & 0xFF);
				data_buffer[1] = static_cast<boost::uint8_t>((msg_size >> 16) & 0xFF);
				data_buffer[2] = static_cast<boost::uint8_t>((msg_size >> 8) & 0xFF);
				data_buffer[3] = static_cast<boost::uint8_t>(msg_size & 0xFF);

				//encode message body
				events.SerializeToArray(&data_buffer[SELMA_PKT_HEADER_SIZE], msg_size);
				evt_socket->send(boost::asio::buffer(data_buffer, msg_size + SELMA_PKT_HEADER_SIZE));
			}
		}
	}

	// the below lines are required to make sure the program terminates cleanly. Might be moved to a separate function in future  	
	if (exit_thread == 1) {
		std::cout << (logPrefix + "SELMA classifier quitting\n");
		return 1;
	}

	return exit_thread;
}


void SELMA_classifier::configSELMA()
{
	std::string logPrefix = SELMA_classifier::logPrefix + "configSELMA(): ";
	// std::cout << (logPrefix + "Obtaining OkUSB Lock\n");
	pthread_mutex_lock(&OkInterface::ok_mutex);
	std::cout << (logPrefix + "Starting SELMA Configuration\n");

	// Reset SELMA 
	OkInterface::Instance()->ActivateTriggerIn(TriggerInAddress_SELMA::SelmaTriggers, SelmaTriggerWires::SelmaIfReset); 	//reset SELMA_IF_Input
//	usleep(1000*30);
	// Start Activating Configuration 
//	OkInterface::Instance()->ActivateTriggerIn(TriggerInAddress_SELMA::SelmaTriggers, SelmaTriggerWires::SelmaIfStartConfig); 	//SELMA IF Start Config
	// Checking for Configuration Completion
	// std::cout << (logPrefix + "Releasing OkUSB Lock\n");
	pthread_mutex_unlock(&OkInterface::ok_mutex);

	waitSelmaStatusBit(SelmaStatusBits::SelmaIfConfigDone, 1);
	// Clearing Config Done Flag
//	clearDoneFlag(SelmaStatusBits::SelmaIfConfigDone,SelmaTriggerWires::SelmaIfClearConfig);	
	std::cout << (logPrefix + "SELMA Configuration Complete\n");			
}

bool SELMA_classifier::testSELMA()
{
	std::string logPrefix = SELMA_classifier::logPrefix + "testSELMA(): ";

	// Configuration Parameters
	uint32_t num_addr = 1024;
	uint32_t max_num_spk = 15;

	bool isNotIncreasingForAll = false;
	// bool isIncreasingForSome = false;
	// Generate Test Vectors
	for(uint32_t addr=0; addr<num_addr; addr+=128) //128 for Test
	{
		uint16_t delays[1024] = {0};
		std::cout << logPrefix  + "Starting Test for Board # : " + std::to_string(addr >> 8) << std::endl;
		// bool isNotIncreasingForBoard = false;

		for(uint32_t j=1; j<num_addr; j++){ delays[j] = getDelayVal(0); }

		std::pair<uint16_t*,int64_t*>  output = sendInputToSELMA(delays);

		for(uint16_t num_spk=1; num_spk <= max_num_spk; num_spk+=1)
		{	
			std::pair<uint16_t*, int64_t*> last_output = output;

			uint16_t delay_val = getDelayVal(num_spk);					
			delays[addr] = delay_val;
			std::pair<uint16_t*,int64_t*>  output = sendInputToSELMA(delays);
			// uint16_t* stage1_data = output.first;
			// int64_t* stage2_data = output.second;

			for(int k=0; k < 2048;k++)
			{
				if (output.first[k] < last_output.first[k] )
				{
					std::cout << "Not Increasing for Board # : " + std::to_string(addr >> 8) + " for " + std::to_string(num_spk) +"spikes " << std::endl;
					// isNotIncreasingForBoard = true;
					isNotIncreasingForAll = true;
					break;
				}
			}
		}
		// if (!isNotIncreasingForBoard)
		// {
		// 	isIncreasingForSome = true;
		// }
	}
	return isNotIncreasingForAll;
}

std::pair<bool,bool> SELMA_classifier::testSELMAFull()
{
	std::string logPrefix = SELMA_classifier::logPrefix + "testSELMAFull(): ";

	// Configuration Parameters
	uint32_t num_addr = 1024;
	uint32_t max_num_spk = 15;
	uint32_t num_iter = 2;// 10

	bool isNotIncreasingForAll = false;
	// bool isIncreasingForSome = false;
	bool is2ndStageWrong = false;
	// Generate Test Vectors
//	for(uint32_t addr=0; addr<num_addr; addr+=128) //128 for Test
//	{
		uint16_t delays[1024] = {0};
//		std::cout << logPrefix  + "Starting Test for Board # : " + std::to_string(addr >> 8) << std::endl;
//		// bool isNotIncreasingForBoard = false;
//
		for(uint32_t j=1; j<num_addr; j++){ delays[j] = getDelayVal(0); }
//
//		std::pair<uint16_t*,int64_t*>  output = sendInputToSELMA(delays);
//
//		for(uint16_t num_spk=1; num_spk <= max_num_spk; num_spk+=1)
//		{	
//			std::cout << logPrefix  + "Running Test for NUM_SPIKES =  : " + std::to_string(num_spk) << std::endl;
//			std::pair<uint16_t*, int64_t*> last_output = output;
//			for(uint32_t iter=0; iter<num_iter; iter++)
//			{
//				// std::cout << logPrefix  + "Starting Trial " + std::to_string(iter) + " for Board # : " + std::to_string(addr >> 8) << std::endl;
//
//				uint16_t delay_val = getDelayVal(num_spk);					
//				//delays[addr] = delay_val;
//				for(int k=0;k<15;k++)
//				{delays[addr+k] = delay_val;
//				}				
				std::pair<uint16_t*,int64_t*>  output = sendInputToSELMA(delays);
//				uint16_t* stage1_data = output.first;
//				int64_t* stage2_data = output.second;
//				int64_t stage2_estimate[SELMA_NUM_CLASSES] = {0};
//
//				for(int k=0; k < SELMA_NUM_OUTPUT_CHANNELS;k++)
//				{
//					if (output.first[k] < last_output.first[k] )
//					{
//						std::cout << "Not Increasing for Board # : " + std::to_string(addr >> 8) + " for " + std::to_string(num_spk) +"spikes " << std::endl;
//						// isNotIncreasingForBoard = true;
//						isNotIncreasingForAll = true;
//					}
//					
//					for (int m=0; m <SELMA_NUM_CLASSES; m++)
//					{
//						//stage2_estimate[m] += output.first[k] * (6-m);
//						//stage2_estimate[m] += output.first[k] * (m+1);
//						//stage2_estimate[m] += output.first[k] * ((k+1) & 127);	
//						stage2_estimate[m] += output.first[k] * (((k+1)*(m+1) & 126) + 1 );
//					}
//				}
//
////				for(int k=0; k<SELMA_NUM_CLASSES; k++)
////				{
////					int64_t stage2_act_est_diff = output.second[k] - stage2_estimate[k];
////					if (stage2_act_est_diff != 0)
////					{
////						// std::cout << "Stage2 Output is Not Correct for Board # : " + std::to_string(addr >> 8) + " for " + std::to_string(num_spk) +"spikes for class " + std::to_string(k) << std::endl;
////						std::cout << "Stage2 Mismatch: (Class@NumSpk@Address,FPGA_MAC,C++_MAC,(FPGA-C++)MAC) = (" +std::to_string(k+1) +"," + std::to_string(num_spk) + "@" + std::to_string(addr) + ',' + std::to_string(output.second[k]) + ',' + std::to_string(stage2_estimate[k]) + "," + std::to_string(stage2_act_est_diff) + ")" << std::endl;
////						is2ndStageWrong = true;
////					}
////				}
//				std::string file_prefix = "addr_" + std::to_string(addr);
//				file_prefix += "_numSpk_" + std::to_string(num_spk);
//
//				std::string filename = file_prefix + ".csv";
//				writeHiddenLayerOutToFile(stage1_data, "stage1_" + filename, 2048);
//				writeHiddenLayerOutToFile(stage2_data, "stage2_" + filename, 6);
//			}
//		}
//		// if (!isNotIncreasingForBoard)
//		// {
//		// 	isIncreasingForSome = true;
//		// }
//	}
	return std::pair<bool,bool>(isNotIncreasingForAll,is2ndStageWrong);
}

std::pair<uint16_t*,int64_t*>  SELMA_classifier::sendInputToSELMA(uint16_t delays[])
{
	std::string logPrefix = SELMA_classifier::logPrefix + "sendInputToSELMA(): ";
	// INTPUT Buffers
	unsigned char raw_input_bytes[2*SELMA_NUM_INPUT_CHANNELS] = {0};
	// READOUT Buffers
 	unsigned char raw_stage1_data[2*SELMA_NUM_OUTPUT_CHANNELS] = {0};
 	uint16_t stage1_data[SELMA_NUM_OUTPUT_CHANNELS] = {0};
 	unsigned char raw_stage2_data[SELMA_NUM_BYTES_PER_CLASS*SELMA_NUM_CLASSES] = {0};	// Number of Bytes Per Channel * Number of Channels
 	int64_t stage2_data[SELMA_NUM_CLASSES] = {0};
 	// int64_t stage2_data_calculated[SELMA_NUM_CLASSES] = {0};
 	unsigned char raw_data_buffer_excess[2*SELMA_NUM_OUTPUT_CHANNELS] = {0};
    // return std::pair<uint16_t*,int64_t*>(stage1_data, stage2_data);

  		// // Send Input into Fifo
	// std::cout << (logPrefix + "Preparing SELMA Feedin Data\n");
	//for(int i=0; i<SELMA_NUM_INPUT_CHANNELS; ++i)
	//{
	// 	raw_input_bytes[i*2] = delays[i]; // Only Write LSB Values [Pipe in follows LSB->MSB]
	//}

	// RESET Fixes Some 2nd Stage Issues
	// std::cout << (logPrefix + "Obtaining OkUSB Lock\n");
//	pthread_mutex_lock(&OkInterface::ok_mutex);
	//
	// Trigger Input Reset
//	OkInterface::Instance()->ActivateTriggerIn(TriggerInAddress_SELMA::SelmaTriggers, SelmaTriggerWires::SelmaIfReset); 	//reset SELMA_IF_Input
	// std::cout << (logPrefix + "Releasing OkUSB Lock\n");
//	pthread_mutex_unlock(&OkInterface::ok_mutex);
//	usleep(1000*30);

	// std::cout << (logPrefix + "Obtaining OkUSB Lock\n");
//	pthread_mutex_lock(&OkInterface::ok_mutex);
//	long num_bytes_written = OkInterface::Instance()->WriteToBlockPipeIn(PipeInAddress_SELMA::SELMA_IF_PipeIn, 2, sizeof(raw_input_bytes), raw_input_bytes);
//	printOpalKellyReturnToString(logPrefix, "Input", num_bytes_written);
	
//	// Trigger Feedin Start
//	// std::cout << logPrefix + "Starting SELMA Feedin Processing\n";
//	OkInterface::Instance()->ActivateTriggerIn(TriggerInAddress_SELMA::SelmaTriggers, SelmaTriggerWires::SelmaIfStartFeedin); 	//SELMA IF Start Config
//	// std::cout << (logPrefix + "Releasing OkUSB Lock\n");
//	pthread_mutex_unlock(&OkInterface::ok_mutex);
//	//waitSelmaStatusBit(SelmaStatusBits::SelmaIfFeedinDone);
  	// std::cout << logPrefix + "SELMA Feedin Processing Complete\n";
//	clearDoneFlag(SelmaStatusBits::SelmaIfFeedinDone, SelmaTriggerWires::SelmaIfClearFeedin);

	//usleep(1000*950);	
//	std::this_thread::sleep_for(std::chrono::seconds(1));
	// Readout Part Data from Pipes
//  	waitSelmaStatusBit(SelmaStatusBits::SelmaIfROEnable);
  	// std::cout << logPrefix + "Starting SELMA Readout From USB\n";
  	waitSelmaStatusBit(SelmaStatusBits::SelmaIfROFifo2Empty, 0);
//	usleep(1000*5);
  	// std::cout << (logPrefix + "Obtaining OkUSB Lock\n");
//	pthread_mutex_lock(&OkInterface::ok_mutex);
//	OkInterface::Instance()->UpdateWireOuts();
//	long num_bytes_stage1 = OkInterface::Instance()->ReadFromBlockPipeOut(PipeOutAddress_SELMA::RO_F1DataIn, 2, 2*SELMA_NUM_OUTPUT_CHANNELS, raw_stage1_data);
   long num_bytes_stage2 = OkInterface::Instance()->ReadFromBlockPipeOut(PipeOutAddress_SELMA::RO_F2DataIn,48, SELMA_NUM_CLASSES*SELMA_NUM_BYTES_PER_CLASS, raw_stage2_data);
//    long num_excess_bytes = OkInterface::Instance()->ReadFromBlockPipeOut(PipeOutAddress_SELMA::RO_F1DataIn, 2, 2*SELMA_NUM_OUTPUT_CHANNELS, raw_data_buffer_excess);
 ////   long num_excess_bytes2 = OkInterface::Instance()->ReadFromBlockPipeOut(PipeOutAddress_SELMA::RO_F2DataIn, 2, 2*SELMA_NUM_OUTPUT_CHANNELS, raw_data_buffer_excess);
	// std::cout << (logPrefix + "Releasing OkUSB Lock\n");
//	pthread_mutex_unlock(&OkInterface::ok_mutex);

//	if (num_bytes_stage1 > 0) {
//	    // std::cout << logPrefix + "SELMA stage 1 Readout has " + std::to_string(num_bytes_stage1) + " bytes of data\n";
//	    for(int i=0; i<SELMA_NUM_OUTPUT_CHANNELS; ++i)
//	    {
//	    	// LSB then MSB
//	    	stage1_data[i] = uint16_t(raw_stage1_data[i*2]) | (uint16_t(raw_stage1_data[i*2+1]) << 8);
//			if (stage1_data[i] == 0)
//			{
//				std::cout << logPrefix + "Stage1 Output: (Chn,Val) : (" + std::to_string(i) +"," + std::to_string(stage1_data[i])+ ")" << std::endl;
//			}
//	    }
//    }else {
//    	std::cout << logPrefix + "SELMA Stage 1 Error Code is : " + std::to_string(num_bytes_stage1) + " \n";
//    }   

//	std::cout << logPrefix + "Raw Stage 2 Bytes read: " + std::to_string(num_bytes_stage2) + " \n";//-RS
	
	if (num_bytes_stage2 > 0) 
	{
	    // std::cout << logPrefix + "SELMA stage 2 Readout has " + std::to_string(num_bytes_stage2) + " bytes of data\n";
		//std::cout << logPrefix + "Raw Stage 2 Data:\n"; //-RS 
	    for(int i=0; i<SELMA_NUM_CLASSES; ++i)
	    {	    
	    	//for(int j=0; j<SELMA_NUM_BYTES_PER_CLASS; j++)
	    	//{
	    	//	 // std::cout << logPrefix + "Raw Stage 2 Bytes: " + std::to_string(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + j]) + " \n";
	    	//}   	    	
	    	// MSB First then LSB (for the 4 16-bit words) :  Internal to 16-bit words is LSB then MSB
	    	stage2_data[i] = 0;
		stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 0]) << (6 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 1]) << (7 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 2]) << (4 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 3]) << (5 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 4]) << (2 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 5]) << (3 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 6]));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 7]) << (1 * 8));

		//std::cout << logPrefix + "Raw Stage 2 Data: " + std::to_string(stage2_data[i])+ ","; //-RS 
		//std::cout << std::to_string(stage2_data[i])+ ", "; //-RS 
	    }
      //std::cout << "\n";//-RS
      //std::cout << logPrefix + " \n************************************************\n";//-RS
    }else {
//    	std::cout << logPrefix + "SELMA Stage 2 Error Code is : " + std::to_string(num_bytes_stage2) + " \n";
    }
	//std::cout << logPrefix + "Raw Stage 2 Excess Bytes read: " + std::to_string(num_excess_bytes2) + " \n";
    // SOFTWARE STAGE 2
    // Difference of LogNormal
    // c:16:2048
    // 
    // Coeffiicent Multiplication
    // for (int i=0; i<SELMA_NUM_OUTPUT_CHANNELS; i++)
    // {
    // 	for (int j=0; j<SELMA_NUM_CLASSES;j++)
    // 	{
    // 		// W/o DOLN
    // 		stage2_data_calculated[j] = 	stage1_data[i] * coefficients[j][i];

    // 		// W/ DOLN
    // 		// stage2_data_calculated[j] =  (stage1_data[i] - stage1_data[(i+16 % SELMA_NUM_OUTPUT_CHANNELS)]) * coefficients[j][i];
    // 	}
    	
    // }   
		
    return std::pair<uint16_t*,int64_t*>(stage1_data, stage2_data);
}


/**
 Code to Send SELMA Control Signals
*/
void SELMA_classifier::waitSelmaStatusBit(uint32_t STATUS_BIT, bool STATUS_VALUE)
{
	std::string logPrefix = SELMA_classifier::logPrefix + "waitSelmaStatusBit(" + std::to_string(STATUS_BIT) + " Expecting: " + std::to_string(STATUS_VALUE) + "): "; 
	uint32_t sleep_time = SELMA_MIN_SLEEP_TIME; // in us
	bool isDone = false;
	uint64_t total_time = 0;
	// Start Activating Configuration 
//	usleep(SELMA_MIN_SLEEP_TIME);
	std::this_thread::sleep_for(std::chrono::microseconds(SELMA_MIN_SLEEP_TIME));
	total_time += SELMA_MIN_SLEEP_TIME;

	do{

		if(total_time > SELMA_MAX_TOTAL_TIME)
		{
			std::cout << logPrefix + "Total Wait Time Exceeded " + std::to_string(SELMA_MAX_TOTAL_TIME) << std::endl; 
			std::cout << logPrefix + "Exiting Program" << std::endl; 
			throw std::runtime_error(logPrefix + "Likely Infinite Loop");
		}

		// std::cout << (logPrefix + "Obtaining OkUSB Lock\n");
		pthread_mutex_lock(&OkInterface::ok_mutex);
		OkInterface::Instance()->UpdateWireOuts();
		// Read for config_done bit
		uint32_t selma_status = uint16_t(OkInterface::Instance()->GetWireOutValue(WireOutAddress_SELMA::selma_status));
		// std::cout << (logPrefix + "Releasing OkUSB Lock\n");
		pthread_mutex_unlock(&OkInterface::ok_mutex);
		 
		// isDone = (selma_status & STATUS_BIT) == STATUS_BIT;		
		// std::cout << logPrefix +  "Selma_Status " + std::to_string(selma_status) + " | isDone " + std::to_string(isDone) + " \n";
	if (STATUS_VALUE) {
		isDone = ((selma_status & STATUS_BIT) >  0);
	} else {
		isDone = ((selma_status & STATUS_BIT) == 0);
	}
		if(!isDone)		
		{
			sleep_time = sleep_time << 1; // Exponential backoff
			if(sleep_time > (1 << 16))
				sleep_time = SELMA_MIN_SLEEP_TIME; // min time = 1ms

//			usleep(sleep_time);
	std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
			total_time += sleep_time;
		}
	}while(!isDone);
}

int SELMA_classifier::clearDoneFlag(uint32_t DoneFlag, uint32_t ClearFlagTrigger)
{
	std::string logPrefix = SELMA_classifier::logPrefix + "clearDoneFlag(" + std::to_string(DoneFlag) + "): ";

	uint32_t sleep_time = SELMA_MIN_SLEEP_TIME; // in us
	uint64_t total_time = 0;
	// std::cout << (logPrefix + "Obtaining OkUSB Lock\n");
	pthread_mutex_lock(&OkInterface::ok_mutex);
	OkInterface::Instance()->UpdateWireOuts();	
	uint32_t selma_status = uint32_t(OkInterface::Instance()->GetWireOutValue(WireOutAddress_SELMA::selma_status));
	// std::cout << (logPrefix + "Releasing OkUSB Lock\n");
	pthread_mutex_unlock(&OkInterface::ok_mutex);

	bool isDoneCleared = false;
	bool isDone = ((selma_status & DoneFlag) == DoneFlag);
	// std::cout << logPrefix +  "Selma_Status " + std::to_string(selma_status) + " | isDone " + std::to_string(isDone) + " \n";

	if(isDone)
	{		
		do
		{
			if(total_time > SELMA_MAX_TOTAL_TIME)
			{
				std::cout << logPrefix + "Total Wait Time Exceeded " + std::to_string(SELMA_MAX_TOTAL_TIME) << std::endl; 
				std::cout << logPrefix + "Exiting Program" << std::endl; 
				throw std::runtime_error(logPrefix + "Likely Infinite Loop");
			}
			// std::cout << (logPrefix + "Obtaining OkUSB Lock\n");
			pthread_mutex_lock(&OkInterface::ok_mutex);
	
			// Start Activating Configuration 
			OkInterface::Instance()->ActivateTriggerIn(TriggerInAddress_SELMA::SelmaTriggers, ClearFlagTrigger); 
			// wait for 1ms before polling for change in wireout
			usleep(SELMA_MIN_SLEEP_TIME);
			total_time += SELMA_MIN_SLEEP_TIME;
			// Readout Wire Status Signal to confirm clearing of config_done_signal
			OkInterface::Instance()->UpdateWireOuts();
			selma_status = uint32_t(OkInterface::Instance()->GetWireOutValue(WireOutAddress_SELMA::selma_status));
			// std::cout << (logPrefix + "Releasing OkUSB Lock\n");
			pthread_mutex_unlock(&OkInterface::ok_mutex);
			isDone = ((selma_status & DoneFlag) == DoneFlag);
			// std::cout << logPrefix +  "Selma_Status " + std::to_string(selma_status) + " | isDone " + std::to_string(isDone) + " \n";
			 
			
		if(~isDone)
		{
			isDoneCleared = true;
			break;
		}else
		{
			sleep_time = sleep_time << 1; // Exponential backoff
			if(sleep_time > (1 << 16))
				sleep_time = SELMA_MIN_SLEEP_TIME; // min time = 1ms

			usleep(sleep_time);
			total_time += sleep_time;
		}
		}while(!isDoneCleared);
	}
	else
	{	
		std::cout << logPrefix + ("Signal Done not Asserted. Unable to Clear");
		return -1;
	}

	return 1;
}

/**
 * [SELMA_classifier::getDelayVal description]
 * @param  num_spk [description]
 * @return         [description]
 */
uint16_t SELMA_classifier::getDelayVal(uint16_t num_spk)
{
	// delay_val = (31 / num_spk) - 1;
	// Old Formula : # spk = floor(31 / delay) ; delay = 31 / # spk
	// New Formula : # spk = floor(31 / (delay+1) ; delay = (31 / # spk) - 1 
	// Delay = # spk
	// 1 = 15    6 = 4    14-29=1 	 2 = 10    7 = 3    30-31=0
	// 3 = 7     8 = 3    4 = 6 	 9 = 3     5 = 5    10-13=2 
	uint16_t delay_val = 31;
	switch(num_spk)
	{
		case 15:  delay_val = 1; break;
		case 14:
		case 13:
		case 12:
		case 11:
		case 10: delay_val = 2; break;	
		case 9:
		case 8:
		case 7: delay_val = 3; break;
		case 6: delay_val = 4; break;
		case 5: delay_val = 5; break;
		case 4: delay_val = 6; break;
		case 3: delay_val = 7; break; 
		case 2: delay_val = 11; break;
		case 1: delay_val = 20; break;
		case 0: delay_val = 31; break;
		default: delay_val = 1; 
	}
	return delay_val;			
}

uint16_t SELMA_classifier::getAddrFromXY(uint16_t x_addr, uint16_t y_addr)
{
	uint16_t addr = 0;

	addr = addr | ((x_addr & 0x001F) >> 3) << 8;
	addr = addr | (x_addr & 0x0007) << 4;
	addr = addr | (y_addr & 0x001F) >> 4 << 7;
	addr = addr | (y_addr & 0x000F);

	return addr;
}

std::pair<uint16_t, uint16_t> SELMA_classifier::scaleToSize(uint16_t evt_x, uint16_t evt_y, uint16_t xSize, uint16_t ySize)
{
	std::string logPrefix = SELMA_classifier::logPrefix + "scaleToSize(): ";
	uint16_t newEvtX = 0;
	uint16_t newEvtY = 0;

	// BIT SHIFT SCALING
	uint16_t numBitsToShiftX = std::max(ceil(log2(xSize)) - floor(log2(SELMA_XSIZE)),(double)0); //At most Don't need to shift
	uint16_t numBitsToShiftY = std::max(ceil(log2(ySize)) - floor(log2(SELMA_YSIZE)),(double)0);
	uint16_t numBitsToShift = std::max(numBitsToShiftY, numBitsToShiftX);
	// std::cout << logPrefix + "NumBitsToShift : " + std::to_string(numBitsToShift) + "\n";
	newEvtX = evt_x >> numBitsToShift;
	newEvtY = evt_y >> numBitsToShift;
	// std::cout << logPrefix + "(" + std::to_string(evt_x) + "->" + std::to_string(newEvtX) + "),(" + std::to_string(evt_y) + "->" + std::to_string(newEvtY) + ")\n";
	return std::pair<uint16_t, uint16_t>(newEvtX, newEvtY);
}

//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------
//-------------------------------TRACKING CODE : SHOULD BE SEPARATED--------------------------------------
//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------

void SELMA_classifier::processEvent(event* evt, uint64_t time)
{
	// Tracking
	calculate_distances(*evt, distance);
	int tracker_number = assign_tracker(distance);
	if (tracker_number != -1)
	{
		// std::cout << "Event Assigned to Track " + std::to_string(tracker_number) + "\n"; 
		 //Track Update for Event
		update_tracker(*evt, time,
				 &trackers[tracker_number]);

		// Shifting to Topleft (Spherical Tracker)
		// Events not inside the box might still be assigned to the box !!! Need to Differentiate
		// std::cout << "Shifting Event to : " + std::to_string((double)evt->x - (double)(trackers[tracker_number].loc[0] - trackers[tracker_number].radius)) + "," + std::to_string((double)evt->y - (double)(trackers[tracker_number].loc[1] - trackers[tracker_number].radius)) + "\n";
		std::pair<uint16_t, uint16_t> translatedEvtsCoord = std::pair<uint16_t,uint16_t>((double)evt->x - std::min(trackers[tracker_number].loc[0] - trackers[tracker_number].radius,(double)evt->x),
																	  					 (double)evt->y - std::min(trackers[tracker_number].loc[1] - trackers[tracker_number].radius,(double)evt->y));
		// Scale Event to Correct Size
		std::pair<uint16_t, uint16_t> scaledEvtsCoord = scaleToSize(std::min((double)translatedEvtsCoord.first,(double)SELMA_XSIZE-1), std::min((double)translatedEvtsCoord.second,(double)SELMA_YSIZE-1), trackers[tracker_number].radius*2, trackers[tracker_number].radius*2);

		// Bin Event
		uint16_t addr = getAddrFromXY(scaledEvtsCoord.first, scaledEvtsCoord.second);
		uint16_t track_num = tracker_number;
		event_rates[track_num][addr] += 1;
	}

	// Track Update by Time
	if (time - last_tracker_cleanup > SELMA_TRACKER_UPDATE_PERIOD) 
	{
		event dummy_event;
		for (int i = 0; i < num_trackers; i++) {
			auto tracker = trackers[i];
			dummy_event.x = round(tracker.loc[0]);
			dummy_event.y = round(tracker.loc[1]);
			update_tracker(dummy_event, time, &trackers[i]);

			if (trackers[i].active)
			{
			// 	uint16_t x_min = std::max((double)tracker.loc[0] - (double)tracker.radius,(double)0);
			// 	uint16_t x_max = std::min((double)tracker.loc[0] + (double)tracker.radius,(double)DataSource::x_dim);
			// 	uint16_t y_min = std::max((double)tracker.loc[1] - (double)tracker.radius,(double)0);
			// 	uint16_t y_max = std::min((double)tracker.loc[1] + (double)tracker.radius,(double)DataSource::y_dim);

			// 	sendResultsToVisualizer(x_min,x_max,y_min,y_max,tracker.id);
				if(writeAnnotations)
				{
					writeAnnotationCSV(
						time, 
						floor(tracker.loc[0]), 
						floor(tracker.loc[1]), 
						floor(tracker.radius*2), 
						floor(tracker.radius*2), 
						tracker.id + (overflow_ctr << 8), 
						tracker.label);
				}
			 }
		}
	}

	last_tracker_cleanup = time;

	// Classify Active Tracks based on Time
	// std::cout << "Checking Classification Time " + std::to_string(time) + ">" + std::to_string(last_classification_time + SELMA_CLASS_UPDATE_PERIOD )+"\n";
	// // Classification Rate of NUM_TRACKS / 1s 100 ms closest power is 2^17 = 131072 or 2^16 = 65536
	while (time > (last_classification_time + SELMA_CLASS_UPDATE_PERIOD))
	{
		int last_idx = track_to_classify-1;
		if (last_idx < 0)
		{
			last_idx = SELMA_NUM_TRACKERS-1;
		}
		// std::cout << std::to_string(last_idx) << std::endl;
		// Search for Active Tracks to Classify
		for(int i=track_to_classify; i != last_idx; i=(i+1) % SELMA_NUM_TRACKERS)
		{
			if(trackers[i].active == 1)
			{
				track_to_classify = i;
				// std::cout << std::to_string(i) + "!=" + std::to_string(last_idx) + ",";
				// std::cout << std::endl << std::endl;
				break;
			}
		}

		// std::cout << "Attempting to Classify " + std::to_string(track_to_classify) + " with last_idx " + std::to_string(last_idx)  + "\n";
		
		if((trackers[track_to_classify].active == 1) && (time - trackers[track_to_classify].active_time > SELMA_MIN_TIME_TO_CLASSIFY))
		{
			// std::cout << "Classifying Track " + std::to_string(track_to_classify) + "\n";
			// Sending frame to classify
			uint16_t delays[SELMA_NUM_INPUT_CHANNELS] = {0};
			convertEventRateToDelays(event_rates[track_to_classify], delays, SELMA_NUM_INPUT_CHANNELS);

			// Clearing Event Rates for next Window
			for(int i=0; i<SELMA_NUM_INPUT_CHANNELS; i++)
			{
				event_rates[track_to_classify][i] = 0;
			}
			 
			std::pair<uint16_t*,int64_t*>  output = sendInputToSELMA(delays);
			int64_t* stage2_data = output.second;	

			// Sending Packet to Comms Module
			uint8_t track_id = trackers[track_to_classify].id;
			uint16_t width = trackers[track_to_classify].radius*2;
			uint16_t height = trackers[track_to_classify].radius*2;
			uint16_t x = trackers[track_to_classify].loc[0];
			uint16_t y = trackers[track_to_classify].loc[1];		
			uint8_t confid[SELMA_NUM_CLASSES] = {0};
			convertStage2toConfidence(stage2_data, confid);

			sendResultsToComms(width,height,x,y,stage2_data,track_id);
			
			uint16_t x_min = std::max((double)x - (double)width/2,(double)0);
			uint16_t x_max = std::min((double)x + (double)width/2,(double)DataSource::x_dim);
			uint16_t y_min = std::max((double)y - (double)width/2,(double)0);
			uint16_t y_max = std::min((double)y + (double)width/2,(double)DataSource::y_dim);

			sendResultsToVisualizer(x_min,x_max,y_min,y_max,track_id,confid);	

			uint8_t max_label = 1;
			uint8_t max_value = 0;
			// Finding Class Label
			for (int i=0;i<SELMA_NUM_CLASSES; i++)
			{
				if (stage2_data[i] > max_value)
				{
					max_label = i+1;
					max_value = stage2_data[i];					
				}
			}


			trackers[track_to_classify].label = max_label;
			// bool stage2_overflowed = false;
			// // Do Accumulation and Voting Here
			// for (int i=0;i<SELMA_NUM_CLASSES; i++)
			// {
			// 	// Only Accumulate Positive Values
			// 	uint64_t new_value = std::max((double)stage2_data[i],0.0) + trackers[track_to_classify].stage2_data[i];
			// 	if (new_value < trackers[track_to_classify].stage2_data[i])
			// 	{
			// 		// Overflow occured 
			// 		// Set to UINT64_MAX 
			// 		new_value = UINTMAX_MAX;					
			// 		stage2_overflowed = true;
			// 	}

			// 	trackers[track_to_classify].stage2_data[i] = new_value;				
			// }

			// // Attempt to Normalize if overflow Occured (Subtract Min Value from All Classes)
			// if (stage2_overflowed)
			// {	
			// 	uint64_t min_value = 0;
			// 	for (int i=0;i<SELMA_NUM_CLASSES; i++)
			// 	{
			// 		min_value = std::min(trackers[track_to_classify].stage2_data[i],min_value);	
			// 	}				

			// 	for (int i=0;i<SELMA_NUM_CLASSES; i++)
			// 	{
			// 		// By Defintion should be >= 0 
			// 		trackers[track_to_classify].stage2_data[i] = trackers[track_to_classify].stage2_data[i] - min_value;				
			// 	}				
			// }

			// uint8_t max_label_voting = 1;
			// uint64_t max_value_voting = 0;
			// // Finding Class Label
			// for (int i=0;i<SELMA_NUM_CLASSES; i++)
			// {
			// 	if (trackers[track_to_classify].stage2_data[i] > max_value_voting)
			// 	{
			// 		max_label_voting = i+1;
			// 		max_value_voting = trackers[track_to_classify].stage2_data[i];					
			// 	}
			// }

			if(writeAnnotations)
			{
				writeAnnotationCSV(
					time, 
					floor(trackers[track_to_classify].loc[0]), 
					floor(trackers[track_to_classify].loc[1]), 
					floor(trackers[track_to_classify].radius*2), 
					floor(trackers[track_to_classify].radius*2), 
					trackers[track_to_classify].id + (overflow_ctr << 8), 
					trackers[track_to_classify].label);
			}
		}

		track_to_classify = (track_to_classify + 1) % SELMA_NUM_TRACKERS;

		last_classification_time += SELMA_CLASS_UPDATE_PERIOD;
	}
}

void SELMA_classifier::sendResultsToComms(uint16_t width, uint16_t height, uint16_t x_addr, uint16_t y_addr, int64_t* category_data, uint8_t id)
{
	std::string logPrefix = SELMA_classifier::logPrefix + "sendResultsToComms(): ";

	// Data Preparation
	std::cout << (logPrefix + "Preparing Comms Input Data\n");
	// Buffer Init
	unsigned char raw_id_bytes[2] = {0};
	unsigned char raw_width_bytes[2] = {0};
	unsigned char raw_height_bytes[2] = {0};
	unsigned char raw_x_addr_bytes[2] = {0};
	unsigned char raw_y_addr_bytes[2] = {0};
	unsigned char raw_category_data_bytes[SELMA_NUM_CLASSES*SELMA_NUM_BYTES_PER_CLASS] = {0};
	
	// Filling of Buffers
	// [Pipe in follows LSB->MSB]
	raw_id_bytes[0] = id & 0x00FF;
	raw_width_bytes[0] = width & 0x00FF;
	raw_width_bytes[1] = (width >> 8) & 0x00FF;
	raw_height_bytes[0] = height & 0x00FF;
	raw_height_bytes[1] = (height >> 8) & 0x00FF;
	raw_x_addr_bytes[0] = x_addr & 0x00FF;
	raw_x_addr_bytes[1] = (x_addr >> 8) & 0x00FF;
	raw_y_addr_bytes[0] = y_addr & 0x00FF;
	raw_y_addr_bytes[1] = (y_addr >> 8) & 0x00FF;

	// Filling up Raw Stage 2 Data Bytes
	for(int i=0; i < SELMA_NUM_CLASSES; i++)
	{

		int64_t class_data = category_data[i];
		// int64_t class_data = 0x0102030405060708;
		std::cout << logPrefix + "Class Data is " + std::to_string(class_data) + "\n";
		// MSB to LSB for word level breakdwon
		uint16_t word_1st = (class_data & 0xFFFF000000000000) >> (6*8);
		uint16_t word_2nd = (class_data & 0x0000FFFF00000000) >> (4*8);
		uint16_t word_3rd = (class_data & 0x00000000FFFF0000) >> (2*8);
		uint16_t word_4th = (class_data & 0x000000000000FFFF) >> (0*8);
		// std::cout << logPrefix + "Word Data is " + std::to_string(word_1st) + "," + std::to_string(word_2nd) + "," + std::to_string(word_3rd) + "," + std::to_string(word_4th) +"\n";


		// Filling up of Buffers LSB then MSB
		raw_category_data_bytes[i*SELMA_NUM_BYTES_PER_CLASS + 0] = word_1st & 0x00FF;
		raw_category_data_bytes[i*SELMA_NUM_BYTES_PER_CLASS + 1] = (word_1st & 0xFF00) >> 8;
		// std::cout << logPrefix + "Byte Data is " + std::to_string(raw_category_data_bytes[i*SELMA_NUM_BYTES_PER_CLASS + 0]) + "," + std::to_string(raw_category_data_bytes[i*SELMA_NUM_BYTES_PER_CLASS + 1]) + ",";
		raw_category_data_bytes[i*SELMA_NUM_BYTES_PER_CLASS + 2] = word_2nd & 0x00FF;
		raw_category_data_bytes[i*SELMA_NUM_BYTES_PER_CLASS + 3] = (word_2nd & 0xFF00) >> 8;
		// std::cout << std::to_string(raw_category_data_bytes[i*SELMA_NUM_BYTES_PER_CLASS + 2]) + "," + std::to_string(raw_category_data_bytes[i*SELMA_NUM_BYTES_PER_CLASS + 3]) + ",";
		raw_category_data_bytes[i*SELMA_NUM_BYTES_PER_CLASS + 4] = word_3rd & 0x00FF;
		raw_category_data_bytes[i*SELMA_NUM_BYTES_PER_CLASS + 5] = (word_3rd & 0xFF00) >> 8;
		// std::cout << std::to_string(raw_category_data_bytes[i*SELMA_NUM_BYTES_PER_CLASS + 4]) + "," + std::to_string(raw_category_data_bytes[i*SELMA_NUM_BYTES_PER_CLASS + 5]) + ",";
		raw_category_data_bytes[i*SELMA_NUM_BYTES_PER_CLASS + 6] = word_4th & 0x00FF;
		raw_category_data_bytes[i*SELMA_NUM_BYTES_PER_CLASS + 7] = (word_4th & 0xFF00) >> 8;
		// std::cout << std::to_string(raw_category_data_bytes[i*SELMA_NUM_BYTES_PER_CLASS + 6]) + "," + std::to_string(raw_category_data_bytes[i*SELMA_NUM_BYTES_PER_CLASS + 7]) + ",";
			// std::cout << std::endl;
	}
	

	// Block Sending To Pipe
	std::cout << (logPrefix + "Sending Comms Input Data\n");
	std::cout << logPrefix + "Sending To Comms W,H,X,Y,ID: " + std::to_string(width) + "," + std::to_string(height) + "," + std::to_string(x_addr) + "," + std::to_string(y_addr) + "," + std::to_string(id) << std::endl;
	// std::cout << (logPrefix + "Obtaining OkUSB Lock\n");
	pthread_mutex_lock(&OkInterface::ok_mutex);

	long num_id_bytes_written = OkInterface::Instance()->WriteToBlockPipeIn(PipeInAddress_SELMA::COMMS_IF_OBJ_ID_FIFO_ADDR, 2, 2, raw_id_bytes);
	long num_width_bytes_written = OkInterface::Instance()->WriteToBlockPipeIn(PipeInAddress_SELMA::COMMS_IF_WIDTH, 2, 2, raw_width_bytes);
	long num_height_bytes_written = OkInterface::Instance()->WriteToBlockPipeIn(PipeInAddress_SELMA::COMMS_IF_HEIGHT, 2, 2, raw_height_bytes);
	long num_x_addr_bytes_written = OkInterface::Instance()->WriteToBlockPipeIn(PipeInAddress_SELMA::COMMS_IF_X_ADDR, 2, 2, raw_x_addr_bytes);
	long num_y_addr_bytes_written = OkInterface::Instance()->WriteToBlockPipeIn(PipeInAddress_SELMA::COMMS_IF_Y_ADDR, 2, 2, raw_y_addr_bytes);
	long num_category_bytes_written = OkInterface::Instance()->WriteToBlockPipeIn(PipeInAddress_SELMA::COMMS_IF_CLASSDATA, 2, SELMA_NUM_CLASSES*SELMA_NUM_BYTES_PER_CLASS, raw_category_data_bytes);
	// std::cout << (logPrefix + "Releasing OkUSB Lock\n");
	pthread_mutex_unlock(&OkInterface::ok_mutex);

	// Confirmation For Correctness
	printOpalKellyReturnToString(logPrefix, "ID", num_id_bytes_written);
	printOpalKellyReturnToString(logPrefix, "Width", num_width_bytes_written);
	printOpalKellyReturnToString(logPrefix, "Height", num_height_bytes_written);
	printOpalKellyReturnToString(logPrefix, "X_addr", num_x_addr_bytes_written);
	printOpalKellyReturnToString(logPrefix, "Y_addr", num_y_addr_bytes_written);
	printOpalKellyReturnToString(logPrefix, "CatData", num_category_bytes_written);
}

// Send ACTIVE status Code
void SELMA_classifier::sendResultsToVisualizer(uint8_t id, bool status)
{
	if(vis)
	{
		std::string logPrefix = SELMA_classifier::logPrefix + "sendResultsToVisualizer(): ";
		// std::cout << logPrefix + "Preparing Status Message to be sent to Visualizer" << std::endl;

		// Tracker Active Message
		auto statMessage = new tn::Message();
		auto statUpdate = new tn::Tracker_Status_Update();
		statUpdate->set_id(id);
		statUpdate->set_is_active(status);
		statMessage->set_msg_type(tn::Message_Type::TStatus);
		// std::cout << logPrefix + "Stat Message Type is : " + std::to_string(statMessage->msg_type()) << std::endl;		
		statMessage->set_allocated_t_status(statUpdate);
		
		// std::cout << logPrefix + "Sending Status Messages to Visualizer" << std::endl;
		send_tcp_message(statMessage);
		// std::cout << logPrefix + "Sent Status Messages to Visualizer" << std::endl;
	}
}

// Send Bounding Box Code + Active Track Status
void SELMA_classifier::sendResultsToVisualizer(uint16_t x_min, uint16_t x_max, uint16_t y_min, uint16_t y_max, uint8_t id) 
{
	if(vis)
	{
		sendResultsToVisualizer(id, true);
		std::string logPrefix = SELMA_classifier::logPrefix + "sendResultsToVisualizer(): ";
		// std::cout << logPrefix + "Preparing Tracker Message to be sent to Visualizer" << std::endl;
		
		// Tracker Min Max Update
		auto minMessage = new tn::Message();
		auto minUpdate = new tn::Tracker_Min_Update();
		minUpdate->set_id(id);
		minUpdate->set_x(x_min);
		minUpdate->set_y(y_min);
		minMessage->set_msg_type(tn::Message_Type::TMin);
		minMessage->set_allocated_t_min(minUpdate);

		auto maxMessage = new tn::Message();
		auto maxUpdate = new tn::Tracker_Max_Update();
		maxUpdate->set_id(id);
		maxUpdate->set_x(x_max);
		maxUpdate->set_y(y_max);
		maxMessage->set_msg_type(tn::Message_Type::TMax);
		maxMessage->set_allocated_t_max(maxUpdate);

		// Sending of Messages
		// std::cout << logPrefix + "Sending Tracker Messages to Visualizer" << std::endl;
		send_tcp_message(minMessage);
		send_tcp_message(maxMessage);
		// std::cout << logPrefix + "Sent Tracker Messages to Visualizer" << std::endl;
	}
}

// Send Confidence Code Given uint_8 conf[6] array 
void SELMA_classifier::sendResultsToVisualizer(uint16_t x_min, uint16_t x_max, uint16_t y_min, uint16_t y_max, uint8_t id, uint8_t* conf)
{
	if(vis)
	{
		sendResultsToVisualizer(x_min,x_max,y_min,y_max,id);
		std::string logPrefix = SELMA_classifier::logPrefix + "sendResultsToVisualizer(): ";
		// std::cout << logPrefix + "Preparing Confidence Message to be sent to Visualizer" << std::endl;

		auto confMessage = new tn::Message();
		auto confUpdate = new tn::Confidence_Update();

		// Set ID
		confUpdate->set_id(id);

		// Confidence Message Preparation
		confUpdate->set_bike(conf[4]);
		confUpdate->set_bus(conf[1]);
		confUpdate->set_car(conf[0]);
		confUpdate->set_human(conf[3]);
		
		confUpdate->set_truck(conf[5]);
		confUpdate->set_van(conf[2]);
		confUpdate->set_tracked_veh(0);
		confUpdate->set_wheeled_veh(100-conf[3]);
		confUpdate->set_unknown(0);
		confMessage->set_msg_type(tn::Message_Type::Conf);
		confMessage->set_allocated_conf(confUpdate);

		// Sending of Messages
		// std::cout << logPrefix + "Sending Confidence Message to Visualizer" << std::endl;
		send_tcp_message(confMessage);
		// std::cout << logPrefix + "Sent Confidence Message to Visualizer" << std::endl;
	}
}

void SELMA_classifier::send_tcp_message(tn::Message* message) {
	int msg_size = message->ByteSize();
	// std::cout << "Message Msg Type is : " + std::to_string(message->msg_type()) << std::endl;
	data_buffer.resize(msg_size + SELMA_PKT_HEADER_SIZE);

	//encode 4 bytes header to denote size of message body
	data_buffer[0] = static_cast<boost::uint8_t>((msg_size >> 24) & 0xFF);
	data_buffer[1] = static_cast<boost::uint8_t>((msg_size >> 16) & 0xFF);
	data_buffer[2] = static_cast<boost::uint8_t>((msg_size >> 8) & 0xFF);
	data_buffer[3] = static_cast<boost::uint8_t>(msg_size & 0xFF);

	//encode message body
	message->SerializeToArray(&data_buffer[SELMA_PKT_HEADER_SIZE], msg_size);
	socket->send(boost::asio::buffer(data_buffer, msg_size + SELMA_PKT_HEADER_SIZE));
}

void SELMA_classifier::convertStage2toConfidence(int64_t* stage2_data, uint8_t* confid)
{
	std::string logPrefix = SELMA_classifier::logPrefix + "convertStage2toConfidence(): ";
	double min_value = 0;
	for (int i=0; i<SELMA_NUM_CLASSES; i++)
	{
		min_value = std::min((double)stage2_data[i],min_value); // Not Using Softmax directly as Sum overflows
	}

	double total_sum = 0;
	for (int i=0; i<SELMA_NUM_CLASSES; i++)
	{
		stage2_data[i] -= min_value;
		total_sum += std::max((double)stage2_data[i],(double)0); // Not Using Softmax directly as Sum overflows
	}
	// std::cout << logPrefix + "total_sum : " + std::to_string(total_sum) << std::endl;
	std::cout << logPrefix;

	for (int i=0; i<SELMA_NUM_CLASSES; i++)
	{
		confid[i] = floor((std::max((double)stage2_data[i],(double)0) / total_sum) * 100);
		std::cout << std::to_string(stage2_data[i]) + "->"+ std::to_string(confid[i]) + " ";
	}
	std::cout << std::endl;
}


void SELMA_classifier::calculate_distances(event event_in, int* distance) {
	for (int ii = 0; ii < num_trackers; ii++)
		distance[ii] = pow((event_in.x - (int) trackers[ii].loc[0]), 2)
				+ pow((event_in.y - (int) trackers[ii].loc[1]), 2);
	// printf("Distances are %i %i %i\n", distance[0], distance[1], distance[2]);
}

int SELMA_classifier::assign_tracker(int* distance) {
	int assigned_active_tracker = -1; //-1 means no tracker assigned
	int assigned_inactive_tracker = -1; //-1 means no tracker assigned

	int min_inactive_distance = SELMA_MAX_DISTANCE;
	int min_active_distance = SELMA_MAX_DISTANCE;
	for (int ii = 0; ii < num_trackers; ii++) {
		if (trackers[ii].active) {
			if (distance[ii] < min_active_distance) {
				min_active_distance = distance[ii];
				assigned_active_tracker = ii;
			}
		} else if (distance[ii] < min_inactive_distance) {
			min_inactive_distance = distance[ii];
			assigned_inactive_tracker = ii;
		}
	}

	if (min_active_distance < distance_threshold)
		return assigned_active_tracker;
	else if (min_inactive_distance < SELMA_DISTANCE_THRESHOLD_2)
		return assigned_inactive_tracker;
	else
		return -1;
}

void SELMA_classifier::update_tracker(event event_in, uint64_t time,
		selma_tracker_type* tracker) {
	//update location
	if (tracker->active) {
		tracker->loc[0] = tracker->loc[0] * (SELMA_MIXING_FACTOR)
				+ event_in.x * (1 - SELMA_MIXING_FACTOR);
		tracker->loc[1] = tracker->loc[1] * (SELMA_MIXING_FACTOR)
				+ event_in.y * (1 - SELMA_MIXING_FACTOR);
		uint16_t x_min = std::max((double)tracker->loc[0] - (double)tracker->radius,(double)0);
		uint16_t x_max = std::min((double)tracker->loc[0] + (double)tracker->radius,(double)DataSource::x_dim);
		uint16_t y_min = std::max((double)tracker->loc[1] - (double)tracker->radius,(double)0);
		uint16_t y_max = std::min((double)tracker->loc[1] + (double)tracker->radius,(double)DataSource::y_dim);

		sendResultsToVisualizer(x_min,x_max,y_min,y_max,tracker->id);
		
	} else {
		tracker->loc[0] = tracker->loc[0] * (SELMA_MIXING_FACTOR_2)
				+ event_in.x * (1 - SELMA_MIXING_FACTOR_2);
		tracker->loc[1] = tracker->loc[1] * (SELMA_MIXING_FACTOR_2)
				+ event_in.y * (1 - SELMA_MIXING_FACTOR_2);
	}

	//update times
	tracker->tdiff_mean = tracker->tdiff_mean * (SELMA_TIME_MIXING_FACTOR)
			+ ((double) (time - tracker->last_event))
					* (1 - SELMA_TIME_MIXING_FACTOR);
	tracker->last_event = time;

	//update active status
	if (tracker->active == 1) //if active
	{
		if (tracker->tdiff_mean < SELMA_TDIFF_THRESHOLD) {
			tracker->active = 1;
		} else {
			tracker->active = 0;
			if(vis)
			{
				sendResultsToVisualizer(tracker->id,tracker->active);	
			}
			tracker->label = 7; // Reset Label to 7
		}

		if(writeAnnotations)
		{
			writeAnnotationCSV(
				time, 
				floor(tracker->loc[0]), 
				floor(tracker->loc[1]), 
				floor(tracker->radius*2), 
				floor(tracker->radius*2), 
				tracker->id + (overflow_ctr << 8), 
				tracker->label);
		}
		
	} else { //if inactive
		if (tracker->tdiff_mean < SELMA_TDIFF_THRESHOLD_2) {
			tracker->active = 1;
			tracker->active_time = time; //the time at which it became active
			tracker->id = next_id;
			tracker->label = 7; // Reset Label to Unknown
			next_id ++;
			
			if (tracker->id > next_id)
			{
				overflow_ctr ++;
				std::cout << "ID Overflowed to :" + std::to_string(next_id) << std::endl;
			}else
			{
				std::cout << "Incrementing ID to :" + std::to_string(next_id) << std::endl;	
			}
			
			sendResultsToVisualizer(tracker->id,tracker->active);	
			if(writeAnnotations)
			{
				writeAnnotationCSV(
					time, 
					floor(tracker->loc[0]), 
					floor(tracker->loc[1]), 
					floor(tracker->radius*2), 
					floor(tracker->radius*2), 
					tracker->id + (overflow_ctr << 8), 
					tracker->label);
			}
		} else {
			tracker->active = 0;
		}
	}

	// std::cout << "Tracker State is: (" + std::to_string(tracker->loc[0]) + "," + std::to_string(tracker->loc[1]) + ")\n";
}


/**
	Converts event rate to delays (SELMA_IF's input)
	INPUT:
		Accepts Event_rate vector 
		Buffer for Output Delays as Input
	Event Rate and Delays should be of the same size
*/
void SELMA_classifier::convertEventRateToDelays(uint16_t* event_rate, uint16_t* delays, uint32_t array_length)
{	
	uint32_t threshold = 1;
	uint16_t default_event_rate = 10;
	//uint32_t array_length = sizeof(event_rate) / sizeof(event_rate[0]);
	for(uint32_t i=0; i< array_length; i++)
	{
		// Applying Thresholding:
		if (event_rate[i] >= threshold)
		{
			event_rate[i] = default_event_rate;
		}

		// Converting to Delays
		delays[i] = getDelayVal(event_rate[i]);
	}
}

void SELMA_classifier::printOpalKellyReturnToString(std::string logPrefix, std::string topic, long returnCode)
{
	if (returnCode > 0)
	{
		// std::cout << logPrefix + "Number of " + topic + " Bytes written : " + std::to_string(returnCode) +" \n";
	}else
	{
		std::cout << logPrefix + topic + " Write Error Code is : " + std::to_string(returnCode) + " " + convertOpalKellyErrorCodeToString(returnCode)+ " \n";
	}
}

std::string SELMA_classifier::convertOpalKellyErrorCodeToString(int ErrorCode)
{
	std::string errorStr = "Unrecognised Error String";
	switch(ErrorCode)
	{
		case 0: errorStr = "NoError"; break;
		case -1: errorStr = "Failed"; break;
		case -2: errorStr = "Timeout"; break;
		case -3: errorStr = "DoneNotHigh"; break;
		case -4: errorStr = "TxfrErr"; break;
		case -5: errorStr = "CommErr"; break;
		case -6: errorStr = "InvalidBitStream"; break;
		case -7: errorStr = "FileError"; break;
		case -8: errorStr = "DeviceNotOpen"; break;
		case -9: errorStr = "InvalidEndPt"; break;
		case -10: errorStr = "InvalidBlockSize"; break;
		case -11: errorStr = "I2CRestrictedAddr"; break;
		case -12: errorStr = "I2CBitError"; break;
		case -13: errorStr = "I2CNack"; break;
		case -14: errorStr = "I2CUnknownStatus"; break;
		case -15: errorStr = "UnsupportedFeature"; break;
		case -16: errorStr = "FifoUnderflow"; break;
		case -17: errorStr = "FifoOverflow"; break;
		case -18: errorStr = "DataAlighnmentErr"; break;
		case -19: errorStr = "InvalidResetProfile"; break;
		case -20: errorStr = "InvalidParam"; break;
		default: errorStr = "Unrecognized ErrorCode : " + std::to_string(ErrorCode);
	}

	return errorStr;
}


//
//
//		Sending Data Through SELMA
//
//
void SELMA_classifier::passDataThroughSELMA(std::string test_folder, std::string csv_filename)
{	
	std::string logPrefix = SELMA_classifier::logPrefix + "passDataThroughSELMA() : ";

	// Read File
	std::ifstream input_file;
	input_file.open((test_folder + csv_filename).c_str(), std::ios::in);
	if(input_file.is_open())
	{
		std::string cur_line;
		
		// If starting from Previous Job
		// Check Output File To see if Exist and How many Entries
		// Assumes files have similar names
		std::string label_out_filename = "label" + csv_filename;
		int num_lines = 0;
		std::ifstream out_file;
		out_file.open(label_out_filename.c_str(), std::ios::in);
		if (out_file.is_open())
		{
			while(getline(out_file,cur_line))
			{
				// Count Number of Lines
				num_lines ++;
			}
			out_file.close();
		}
		
		int line_num = num_lines + 1;
		while(getline(input_file,cur_line))
		{		
			// Skip as many lines as num_lines
			// printf("Num Lines to Left Skip: %d\n",num_lines);
			if(num_lines > 0)
			{
				num_lines --;
				continue;
			}
			std::cout << logPrefix + "Processing Line # : " + std::to_string(line_num)  << std::endl;
			// For each Input Example
			// Setting up Buffers for Values
			uint16_t cur_label[1] = {0};
			uint16_t event_rates[SELMA_NUM_INPUT_CHANNELS] = {0};
			uint16_t delays[SELMA_NUM_INPUT_CHANNELS] = {0};

			// Parsing the CSV string into array 
			// Assumes CSV String has the format : [Class_label,A0,A1,...,A1023];		
			uint32_t current_idx = 0;
			size_t lastPos = 0;
			size_t nextPos = cur_line.find_first_of(",\n",lastPos);
			
			while(nextPos != std::string::npos)
			{			
				//printf("Location Found is : %u , ", nextPos);
				//printf("Value found is : %d\n", atoi(cur_line.substr(lastPos,nextPos).c_str()));

				if(current_idx == 0)
				{
					cur_label[0] = (uint16_t)atoi(cur_line.substr(lastPos,nextPos).c_str());
					//printf("Current Label is : %d \n", cur_label[0]);
					//printf("With Values: ");
				}else if(current_idx <= SELMA_NUM_INPUT_CHANNELS+1)
				{
					event_rates[current_idx - 1] = (uint16_t)atoi(cur_line.substr(lastPos,nextPos).c_str());
					//printf("%d ,", event_rates[current_idx-1]);
				}else
				{
					// Exceeds Expected Count
					printf("Unexpected Error Occured while reading CSV FILE\n");
					break;
				}
				current_idx ++;
				lastPos = nextPos + 1;
				nextPos = cur_line.find_first_of(",\n",lastPos);
			}

			// Send Input into SELMA
			convertEventRateToDelays(event_rates, delays, SELMA_NUM_INPUT_CHANNELS);
			std::pair<uint16_t*,int64_t*>  output = sendInputToSELMA(delays);
			uint16_t* stage1_data = output.first;
			int64_t* stage2_data = output.second;
			
			int num_data = 2048;
			
			writeHiddenLayerOutToFile(stage1_data, "stage1_output" + csv_filename, num_data);
			writeHiddenLayerOutToFile(cur_label, label_out_filename,1);
			writeHiddenLayerOutToFile(stage2_data, "stage2_output" + csv_filename, 6);
			line_num ++; 
		}
		
		input_file.close();
		isDonePassingData = true;
	}else
	{
		printf("Unable to find File : %s\n",csv_filename.c_str());
	}
		
}


void SELMA_classifier::writeHiddenLayerOutToFile(uint16_t stage1_data[], std::string filename, int num_data)
{
	// printf("Starting HiddenLayer File Write to %s\n", filename.c_str());
	std::ofstream output_file;
	output_file.open(filename.c_str(), std::ios::out | std::ofstream::app);

	//printf("Opening File %s \n", filename);
	//int num_data = sizeof(stage1_data) / sizeof(stage1_data[0]);
	for(int i=0; i<num_data; ++i)
	{
		//printf("Writing %d-th Data %d to file\n", i, stage1_data[i]);
	 	output_file << std::to_string(stage1_data[i]);
		output_file << ", ";
	}
	output_file << "\n";
	output_file.close();
}


void SELMA_classifier::writeHiddenLayerOutToFile(int64_t stage2_data[], std::string filename, int num_data)
{
	// printf("Starting HiddenLayer File Write to %s\n", filename.c_str());
	std::ofstream output_file;
	output_file.open(filename.c_str(), std::ios::out | std::ofstream::app);

	//printf("Opening File %s \n", filename);
	//int num_data = sizeof(stage2_data) / sizeof(stage2_data[0]);
	for(int i=0; i<num_data; ++i)
	{
		//printf("Writing %d-th Data %d to file\n", i, stage2_data[i]);
	 	output_file << std::to_string(stage2_data[i]);
		output_file << ", ";
	}
	output_file << "\n";
	output_file.close();
}

std::string SELMA_classifier::getAnnotationsHead(std::string recording_file, 
                              std::string picture_file, 
                              std::string annotation_file, 
                              std::string annotator, 
                              std::string sensor_dimensions_x, 
                              std::string sensor_dimensions_y)
{
	std::string header = "";
	std::string recording_header = "#This text file contains annotation data for recordings in: " + recording_file + "\n";
	std::string picture_header = "#The corresponding picture of the recording site is at: " + picture_file + "\n";
	std::string annotation_header = "#The annotation file is stored at: " + annotation_file + "\n";	
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	std::string yr = std::to_string(tm.tm_year + 1900);
	std::string mon = std::to_string(tm.tm_mon + 1);
	std::string day = std::to_string(tm.tm_mday);
	std::string h = std::to_string(tm.tm_hour);
	std::string m = std::to_string(tm.tm_min);
	std::string s = std::to_string(tm.tm_sec);
	std::string comments_header = "#Comments: #Event file for linux_aer created using SELMA_classifier at time " + 
								day + "-" + mon + "-" + yr + " " + h + ":" + m + ":" + s  + "\n";
	std::string annotator_header = "#The recordings are annotated by: " + annotator + "\n";
	std::string sensor_headers = "#Sensor Dimensions- Height = " + sensor_dimensions_y + " Pixels Width = " + sensor_dimensions_x  + " Pixels\n";
	std::string legend_header = "#LEGEND: 1-Car, 2-Bus, 3-Van, 4-Pedestrian, 5-Bike, 6-Truck, 7-Unknown\n";
	std::string csv_header = "#Time(us),x-Location,y-Location,x-size,y-size,track-num,class\n";
	header = recording_header + picture_header + annotation_header + comments_header + annotator_header + sensor_headers + legend_header + csv_header;
	return header;
}

void SELMA_classifier::writeAnnotationsHead()
{
	std::string x_dim = std::to_string(DataSource::x_dim);
	std::string y_dim = std::to_string(DataSource::y_dim);
	std::string unspec = "Unspecified";
	std::string header = getAnnotationsHead(unspec, 
											unspec,
											unspec,
											"SELMAC_Track",
											x_dim,
											y_dim);
	// Opening File
	output_file.open(output_filename.c_str(), std::ios::out);

	output_file << header;
	// Closing
	output_file.close();	
}

void SELMA_classifier::writeAnnotationCSV(uint64_t time, uint16_t x_loc, uint16_t y_loc, uint16_t xSize, uint16_t ySize, uint16_t track_num, uint16_t label)
{
	// Opening File
	output_file.open(output_filename.c_str(), std::ios::out | std::ofstream::app);

	output_file << std::to_string(time) << ", ";
	output_file << std::to_string(x_loc+1) << ", "; // Annotations coordinates are 1-indexed
	output_file << std::to_string(y_loc+1) << ", "; // Annotations coordinates are 1-indexed
	output_file << std::to_string(xSize) << ", ";
	output_file << std::to_string(ySize) << ", ";
	output_file << std::to_string(track_num) << ", ";
	output_file << std::to_string(label) << ", " << std::endl;

	// Closing
	output_file.close();
}


#ifdef _useSDL
	TDdisplay_track_SELMAC::TDdisplay_track_SELMAC()
	{
		selmac_track_window = NULL;
		selmac_track_pixels = new uint32_t[DataSource::x_dim*DataSource::y_dim];
			for(uint32_t i=0; i<DataSource::x_dim*DataSource::y_dim; i++)
			selmac_track_pixels[i] = background;		
		last_TDtracker_display_time = 0;
	}

	bool TDdisplay_track_SELMAC::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
	{
		if(selmac_track_window == NULL)
			init_display_window(&selmac_track_window, &selmac_track_texture, &selmac_track_renderer, "Tracking Window");
		if(packet_info->packet_type == packet_type::TD_packet || packet_info->packet_type == packet_type::TD_EM_mixed_packet)
		{

			uint64_t TD_track_display_MSBs = uint64_t(packet_info->time_start)<<16;
			uint64_t end_time = TD_track_display_MSBs | dataIN[packet_info->num_events-1].t;
		
			for(uint32_t eventNum=0; eventNum<packet_info->num_events; eventNum++)
			{
				if (dataIN[eventNum].type == evt_type::TD)
				{
					if (dataIN[eventNum].subtype == TD_subtype::ON)
						selmac_track_pixels[ (uint32_t(dataIN[eventNum].y) * DataSource::x_dim ) + dataIN[eventNum].x] = ONcolor;
					else
						selmac_track_pixels[ (uint32_t(dataIN[eventNum].y) * DataSource::x_dim ) + dataIN[eventNum].x] = OFFcolor;
				}else
				if (dataIN[eventNum].type == evt_type::Tracked)
				{
					selmac_track_pixels[ (uint32_t(dataIN[eventNum].y) * DataSource::x_dim ) + dataIN[eventNum].x] = trackers[dataIN[eventNum].subtype].colour;
				}
			}
			if(end_time-last_TDtracker_display_time >= display_interval)
			{
				for(uint32_t i=0; i<SELMA_NUM_TRACKERS; i++)
				{
					//printf("Tracker %i is at location %f, %f, and has a tdiff_mean of %f, and is active (true/false) %i\n",i,tracker[i].loc[0],tracker[i].loc[1], tracker[i].tdiff_mean, tracker[i].active);
					if(trackers[i].active == 1)
						if (end_time-trackers[i].active_time > active_time_threshold)
							if (trackers[i].dist[0]*trackers[i].dist[0] + trackers[i].dist[1]*trackers[i].dist[1] > average_distance_threshold)
								cross(selmac_track_pixels, trackers[i].loc[0], trackers[i].loc[1], trackers[i].radius/2-1, marker_color);
							else
								cross(selmac_track_pixels, trackers[i].loc[0], trackers[i].loc[1], trackers[i].radius/2-1, static_marker_color);
						else
							cross(selmac_track_pixels, trackers[i].loc[0], trackers[i].loc[1], trackers[i].radius/2-1, inbetween_marker_color);
					else
					{
						cross(selmac_track_pixels, trackers[i].loc[0], trackers[i].loc[1], trackers[i].radius/2-1, invalid_marker_color);
					}
						
					cross(selmac_track_pixels, trackers[i].loc[0], trackers[i].loc[1], trackers[i].radius/2, trackers[i].colour);

				}
				update_display(&selmac_track_pixels[0], &selmac_track_texture, &selmac_track_renderer);
				for(uint32_t i=0; i<DataSource::x_dim*DataSource::y_dim; i++)
	   				selmac_track_pixels[i] = background;

				last_TDtracker_display_time = end_time;
			}
		}
		if(exit_thread == 1)
		{
			printf("Show Tracker quitting \n");
			SDL_DestroyWindow(selmac_track_window );
			SDL_DestroyRenderer(selmac_track_renderer);
			SDL_Quit();
			return 1;
		}
		return 0;
	}
#endif
