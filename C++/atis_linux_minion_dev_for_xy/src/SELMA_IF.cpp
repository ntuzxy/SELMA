#include <cstdint>
#include <exception>
#include <iostream>
#include <unistd.h>
#include <string>

#include "SELMA_IF.h"
#include "OK_interface.h"


SELMA_IF::SELMA_IF()
{
	std::string logPrefix = SELMA_IF::logPrefix + "Constructor(): ";
	std::cout << (logPrefix + "Constructed SELMA Interface\n");
	// Running Must Run Setup Stuff
	configSELMA();
}

SELMA_IF::SELMA_IF(bool testFull) : SELMA_IF()
{
	if(testFull)
	{
		testSELMAFull();	
	}
}

bool SELMA_IF::processData(
	packet_info_type* packet_info, 
	event* dataIN,
	bool exit_thread) 
{	
	std::string logPrefix = SELMA_IF::logPrefix + "processData(): ";	
	// the below lines are required to make sure the program terminates cleanly. Might be moved to a separate function in future  	
	if (exit_thread == 1) {
		std::cout << (logPrefix + "SELMA_IF quitting\n");
		return 1;
	}

	// return exit_thread;
	return 1;
}


void SELMA_IF::configSELMA()
{
	std::string logPrefix = SELMA_IF::logPrefix + "configSELMA(): ";
	// std::cout << (logPrefix + "Obtaining OkUSB Lock\n");
	pthread_mutex_lock(&OkInterface::ok_mutex);
	std::cout << (logPrefix + "Starting SELMA Configuration\n");

	// Reset SELMA 
	OkInterface::Instance()->ActivateTriggerIn(TriggerInAddress_SELMA::SelmaTriggers, SelmaTriggerWires::SelmaIfReset); 	//reset SELMA_IF_Input
	usleep(1000*30);
	// Start Activating Configuration 
	OkInterface::Instance()->ActivateTriggerIn(TriggerInAddress_SELMA::SelmaTriggers, SelmaTriggerWires::SelmaIfStartConfig); 	//SELMA IF Start Config
	// Checking for Configuration Completion
	// std::cout << (logPrefix + "Releasing OkUSB Lock\n");
	pthread_mutex_unlock(&OkInterface::ok_mutex);

	waitSelmaStatusBit(SelmaStatusBits::SelmaIfConfigDone);
	// Clearing Config Done Flag
	clearDoneFlag(SelmaStatusBits::SelmaIfConfigDone,SelmaTriggerWires::SelmaIfClearConfig);	
	std::cout << (logPrefix + "SELMA Configuration Complete\n");			
}

std::pair<uint16_t*,int64_t*>  SELMA_IF::sendInputToSELMA(uint16_t* delays)
{
	std::string logPrefix = SELMA_IF::logPrefix + "sendInputToSELMA(): ";
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
	for(int i=0; i<SELMA_NUM_INPUT_CHANNELS; ++i)
	{
	 	raw_input_bytes[i*2] = delays[i]; // Only Write LSB Values [Pipe in follows LSB->MSB]
	}

	// RESET Fixes Some 2nd Stage Issues
	// std::cout << (logPrefix + "Obtaining OkUSB Lock\n");
	pthread_mutex_lock(&OkInterface::ok_mutex);
	//
	// Trigger Input Reset
	OkInterface::Instance()->ActivateTriggerIn(TriggerInAddress_SELMA::SelmaTriggers, SelmaTriggerWires::SelmaIfReset); 	//reset SELMA_IF_Input
	// std::cout << (logPrefix + "Releasing OkUSB Lock\n");
	pthread_mutex_unlock(&OkInterface::ok_mutex);
	usleep(1000*30);

	// std::cout << (logPrefix + "Obtaining OkUSB Lock\n");
	pthread_mutex_lock(&OkInterface::ok_mutex);
	long num_bytes_written = OkInterface::Instance()->WriteToBlockPipeIn(PipeInAddress_SELMA::SELMA_IF_PipeIn, 2, sizeof(raw_input_bytes), raw_input_bytes);
	printOpalKellyReturnToString(logPrefix, "Input", num_bytes_written);
	
	// Trigger Feedin Start
	// std::cout << logPrefix + "Starting SELMA Feedin Processing\n";
	OkInterface::Instance()->ActivateTriggerIn(TriggerInAddress_SELMA::SelmaTriggers, SelmaTriggerWires::SelmaIfStartFeedin); 	//SELMA IF Start Config
	// std::cout << (logPrefix + "Releasing OkUSB Lock\n");
	pthread_mutex_unlock(&OkInterface::ok_mutex);
	waitSelmaStatusBit(SelmaStatusBits::SelmaIfFeedinDone);
  	// std::cout << logPrefix + "SELMA Feedin Processing Complete\n";
  	clearDoneFlag(SelmaStatusBits::SelmaIfFeedinDone, SelmaTriggerWires::SelmaIfClearFeedin);
	
	// Readout Part Data from Pipes
  	waitSelmaStatusBit(SelmaStatusBits::SelmaIfROEnable);
  	// std::cout << logPrefix + "Starting SELMA Readout From USB\n";
	usleep(1000*5);
  	// std::cout << (logPrefix + "Obtaining OkUSB Lock\n");
	pthread_mutex_lock(&OkInterface::ok_mutex);
	OkInterface::Instance()->UpdateWireOuts();
	long num_bytes_stage1 = OkInterface::Instance()->ReadFromBlockPipeOut(PipeOutAddress_SELMA::RO_F1DataIn, 2, 2*SELMA_NUM_OUTPUT_CHANNELS, raw_stage1_data);
    long num_bytes_stage2 = OkInterface::Instance()->ReadFromBlockPipeOut(PipeOutAddress_SELMA::RO_F2DataIn, 2, SELMA_NUM_CLASSES*SELMA_NUM_BYTES_PER_CLASS, raw_stage2_data);
    long num_excess_bytes = OkInterface::Instance()->ReadFromBlockPipeOut(PipeOutAddress_SELMA::RO_F1DataIn, 2, 2*SELMA_NUM_OUTPUT_CHANNELS, raw_data_buffer_excess);
    long num_excess_bytes2 = OkInterface::Instance()->ReadFromBlockPipeOut(PipeOutAddress_SELMA::RO_F2DataIn, 2, 2*SELMA_NUM_OUTPUT_CHANNELS, raw_data_buffer_excess);
	// std::cout << (logPrefix + "Releasing OkUSB Lock\n");
	pthread_mutex_unlock(&OkInterface::ok_mutex);

	if (num_bytes_stage1 > 0) {
	    // std::cout << logPrefix + "SELMA stage 1 Readout has " + std::to_string(num_bytes_stage1) + " bytes of data\n";
	    for(int i=0; i<SELMA_NUM_OUTPUT_CHANNELS; ++i)
	    {
	    	// LSB then MSB
	    	stage1_data[i] = uint16_t(raw_stage1_data[i*2]) | (uint16_t(raw_stage1_data[i*2+1]) << 8);
			if (stage1_data[i] == 0)
			{
				std::cout << logPrefix + "Stage1 Output: (Chn,Val) : (" + std::to_string(i) +"," + std::to_string(stage1_data[i])+ ")" << std::endl;
			}
	    }
    }else {
    	std::cout << logPrefix + "SELMA Stage 1 Error Code is : " + std::to_string(num_bytes_stage1) + " \n";
    }

	if (num_bytes_stage2 > 0) 
	{
	    // std::cout << logPrefix + "SELMA stage 2 Readout has " + std::to_string(num_bytes_stage2) + " bytes of data\n";
	    for(int i=0; i<SELMA_NUM_CLASSES; ++i)
	    {	    
	    	for(int j=0; j<SELMA_NUM_BYTES_PER_CLASS; j++)
	    	{
	    		 // std::cout << logPrefix + "Raw Stage 2 Bytes: " + std::to_string(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + j]) + " \n";
	    	}   	    	
	    	// MSB First then LSB (for the 4 16-bit words) :  Internal to 16-bit words is LSB then MSB
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 0]) << (6 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 1]) << (7 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 2]) << (4 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 3]) << (5 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 4]) << (2 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 5]) << (3 * 8));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 6]));
	    	stage2_data[i] = stage2_data[i] | (int64_t(raw_stage2_data[i*SELMA_NUM_BYTES_PER_CLASS + 7]) << (1 * 8));
	    }
    }else {
    	std::cout << logPrefix + "SELMA Stage 2 Error Code is : " + std::to_string(num_bytes_stage2) + " \n";
    }
		
    return std::pair<uint16_t*,int64_t*>(stage1_data, stage2_data);
}


/**
 Code to Send SELMA Control Signals
*/
void SELMA_IF::waitSelmaStatusBit(uint32_t STATUS_BIT)
{
	std::string logPrefix = SELMA_IF::logPrefix + "waitSelmaStatusBit(" + std::to_string(STATUS_BIT) + "): "; 
	bool isHigh = false;
	uint64_t total_time = 0;
	uint32_t selma_status = 0;
	// Start Activating Configuration 
	do
	{
		usleep(SELMA_MIN_SLEEP_TIME);
		total_time += SELMA_MIN_SLEEP_TIME;

		if(total_time > SELMA_MAX_TOTAL_TIME)
		{
			std::cout << logPrefix + "Total Wait Time Exceeded " + std::to_string(SELMA_MAX_TOTAL_TIME) << std::endl; 
			std::cout << logPrefix + "SELMA_STATUS is " + std::to_string(selma_status) << std::endl; 
			std::cout << logPrefix + "Exiting Program" << std::endl; 
			throw std::runtime_error(logPrefix + "Likely Infinite Loop");
		}

		// std::cout << (logPrefix + "Obtaining OkUSB Lock\n");
		pthread_mutex_lock(&OkInterface::ok_mutex);
		OkInterface::Instance()->UpdateWireOuts();
		// Read for config_done bit
		selma_status = uint16_t(OkInterface::Instance()->GetWireOutValue(WireOutAddress_SELMA::selma_status));
		// std::cout << (logPrefix + "Releasing OkUSB Lock\n");
		pthread_mutex_unlock(&OkInterface::ok_mutex);
		 
		isHigh = (selma_status & STATUS_BIT) == STATUS_BIT;		
		// std::cout << logPrefix +  "Selma_Status " + std::to_string(selma_status) + " | isHigh " + std::to_string(isHigh) + " \n";		
	}while(!isHigh);
}

void SELMA_IF::clearDoneFlag(uint32_t DoneFlag, uint32_t ClearFlagTrigger)
{
	std::string logPrefix = SELMA_IF::logPrefix + "clearDoneFlag(" + std::to_string(DoneFlag) + "): ";
	
	uint64_t total_time = 0;
	// std::cout << (logPrefix + "Obtaining OkUSB Lock\n");
	pthread_mutex_lock(&OkInterface::ok_mutex);
	OkInterface::Instance()->UpdateWireOuts();	
	uint32_t selma_status = uint32_t(OkInterface::Instance()->GetWireOutValue(WireOutAddress_SELMA::selma_status));
	// std::cout << (logPrefix + "Releasing OkUSB Lock\n");
	pthread_mutex_unlock(&OkInterface::ok_mutex);

	bool isHighCleared = false;
	bool isHigh = ((selma_status & DoneFlag) == DoneFlag);
	// std::cout << logPrefix +  "Selma_Status " + std::to_string(selma_status) + " | isHigh " + std::to_string(isHigh) + " \n";

	if(isHigh)
	{		
		do
		{
			usleep(SELMA_MIN_SLEEP_TIME);
			total_time += SELMA_MIN_SLEEP_TIME;

			if(total_time > SELMA_MAX_TOTAL_TIME)
			{
				std::cout << logPrefix + "Total Wait Time Exceeded " + std::to_string(SELMA_MAX_TOTAL_TIME) << std::endl; 
				std::cout << logPrefix + "SELMA_STATUS is " + std::to_string(selma_status) << std::endl; 
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
			isHigh = ((selma_status & DoneFlag) == DoneFlag);
			// std::cout << logPrefix +  "Selma_Status " + std::to_string(selma_status) + " | isHigh " + std::to_string(isHigh) + " \n";			
		}while(isHigh);
	}
	else
	{	
		throw std::runtime_error(logPrefix + "Signal Done not Asserted. Unable to Clear");
	}
}

/**
 * [SELMA_classifier::getDelayVal description]
 * @param  num_spk [description]
 * @return         [description]
 */
uint16_t SELMA_IF::getDelayVal(uint16_t num_spk)
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

void SELMA_IF::testSELMAFull()
{
	std::string logPrefix = SELMA_IF::logPrefix + "testSELMAFull(): ";

	// Configuration Parameters
	bool isNotIncreasingForAll = false;
	bool hasCrossTalkForSome = false;
	bool isNotConsistentForSome = false;
	bool is2ndStageWrong = false;

	// Data Buffers Setup
	std::pair<uint16_t*,int64_t*> output;
	uint16_t stage1_data_baseline[SELMA_NUM_OUTPUT_CHANNELS] = {0};

	uint16_t** stage1_data_all = new uint16_t*[SELMA_MAX_NUM_SPIKES]; 
	int64_t** stage2_data_all = new int64_t*[SELMA_MAX_NUM_SPIKES];
	for(int i=0; i< SELMA_MAX_NUM_SPIKES; i++)
	{
		stage1_data_all[i] = new uint16_t[SELMA_NUM_OUTPUT_CHANNELS];
		stage2_data_all[i] = new int64_t[SELMA_NUM_CLASSES];
	}

	uint16_t** stage1_data_consistency_check = new uint16_t*[SELMA_NUM_REPEAT];
	for(int i=0; i<SELMA_NUM_REPEAT; i++)
	{
		stage1_data_consistency_check[i] = new uint16_t[SELMA_NUM_OUTPUT_CHANNELS];
	}

	uint16_t delays[SELMA_NUM_INPUT_CHANNELS] = {0};
	for(uint32_t j=0; j<SELMA_NUM_INPUT_CHANNELS; j++){ delays[j] = getDelayVal(0); }

	output = sendInputToSELMA(delays); // Baseline;				
	for(int i=0;i<SELMA_NUM_OUTPUT_CHANNELS;i++){ stage1_data_baseline[i] = output.first[i];} 

	// Generate Test Vectors
	for(uint32_t addr=0; addr<SELMA_NUM_INPUT_CHANNELS; addr+=128) //128 for Test
	{
		// Input Setup
		uint8_t chip_grp = addr >> 7;
		uint8_t p_value = chip_grp >> 1; // %% Readout Number Order = P(3), P(2), P(1), P(0) // board_sel = 3 - board_num;

		std::cout << logPrefix  + "Starting Test for Chip Group #" + std::to_string(chip_grp) + " P" +std::to_string(p_value) << std::endl;
		// Initializing Inputs for all to send 0 spikes
		for(uint32_t j=0; j<SELMA_NUM_INPUT_CHANNELS; j++){ delays[j] = getDelayVal(0); }

		for(uint16_t num_spk=1; num_spk <= SELMA_MAX_NUM_SPIKES; num_spk+=1)
		{	
			// Input for Num Spk Setup
			uint16_t delay_val = getDelayVal(num_spk);
			delays[addr] = delay_val;
						
			for(uint32_t iter=0; iter<SELMA_NUM_REPEAT; iter++)
			{							
				output = sendInputToSELMA(delays);

				for(int j=0; j<SELMA_NUM_OUTPUT_CHANNELS; j++){ stage1_data_consistency_check[iter][j] = output.first[j];}

				std::string file_prefix = "addr_" + std::to_string(addr);
				file_prefix += "_numSpk_" + std::to_string(num_spk);

				std::string filename = file_prefix + ".csv";
				writeHiddenLayerOutToFile(output.first, "stage1_" + filename, 2048);
				writeHiddenLayerOutToFile(output.second, "stage2_" + filename, 6);
				
			}
			// Only use the last iteration Output for Checking Monotonicity and Cross Talk
			for(int j=0; j<SELMA_NUM_OUTPUT_CHANNELS; j++){ stage1_data_all[num_spk-1][j] = output.first[j];}
			for(int j=0; j<SELMA_NUM_CLASSES;j++) {stage2_data_all[num_spk-1][j] = output.second[j];}

			// Do Consistency Check Here
		   std::cout << logPrefix + "Doing Consistency Check for Chip Group #" +std::to_string(chip_grp) + " with " + std::to_string(num_spk) + " spikes." << std::endl;
		   bool isConsistent = stage1ConsistencyCheck(stage1_data_consistency_check); // Returns Max Variation in Input 
		   // std::cout << logPrefix + "Max Variation is : " + std::to_string(max_variation) << std::endl;
		   if(!isConsistent)
		   {
		   		std::cout << logPrefix + "SELMA 1st stage readout for Chip Group #" + std::to_string(chip_grp) + " might not be consistent" << std::endl;
		   		isNotConsistentForSome = true;
		   }
		}
		// Do Monotonicity Check for Stage 1 Here
		std::cout << logPrefix + "Doing Monotonicity Check for Chip Group #" +std::to_string(chip_grp) << std::endl;
		bool isMonotone = stage1MontonicityCheck(stage1_data_all, stage1_data_baseline, chip_grp);
		if (isMonotone)
		{
			std::cout << logPrefix + "SELMA 1st stage readout for Chip Group #" + std::to_string(chip_grp) + " is responsive for Increasing Input" << std::endl;

		}else
		{
			std::cout << logPrefix + "SELMA 1st stage readout for Chip Group #" + std::to_string(chip_grp) + " is not responsive for Increasing Input" << std::endl;
			isNotIncreasingForAll = true;
		}

		// Do Cross Talk Check for Stage 1 Here
		bool hasCrossTalk = stage1CrossTalkCheck(stage1_data_all, stage1_data_baseline, chip_grp);
		if (hasCrossTalk)
		{
			std::cout << logPrefix + "SELMA 1st stage input for Chip Group #" + std::to_string(chip_grp) + " affects readout for other chips" << std::endl;
			hasCrossTalkForSome = true;
		}else
		{
			std::cout << logPrefix + "SELMA 1st stage input for Chip Group #" + std::to_string(chip_grp) + " only affects only itself" << std::endl;
		}

	}

	if(isNotIncreasingForAll || hasCrossTalkForSome || isNotConsistentForSome)
	{
		std::cout << logPrefix + "Selma Output 1st stage is NOT Working as Expected\n";
	}else
	{
		std::cout << logPrefix + "Selma Output 1st stage is Working as Expected\n";
		
	}	
	
}

bool SELMA_IF::stage1ConsistencyCheck(uint16_t** stage1_data_consistency_check)
{
	std::string logPrefix = SELMA_IF::logPrefix + "stage1ConsistencyCheck(): ";

	uint16_t max_variation = 0;
	bool isConsistent = true;

	for(int i=0; i<SELMA_NUM_OUTPUT_CHANNELS; i++)
	{
		double total_value_for_channel = 0;
		uint16_t min_value = (uint32_t)(2 << 16) - 1;
		uint16_t max_value = 0;
		for(int iter=0; iter<SELMA_NUM_REPEAT; iter++)
		{
			// NOTE: INEFFICIENT CODE (High Cache Miss for Ease of Implementation) 
			uint16_t stage1_value = stage1_data_consistency_check[iter][i]; 

			min_value = ((min_value < stage1_value) ? min_value : stage1_value); // Condition ? IF_TRUE : ELSE;
			max_value = ((max_value > stage1_value) ? max_value : stage1_value); // Condition ? IF_TRUE : ELSE;

			// std::cout << "Stage1 Value is : " + std::to_string(stage1_value) + " " << std::endl;
			// std::cout << "Min,Max Value is " + std::to_string(min_value) + "," + std::to_string(max_value) << std::endl;
			
			total_value_for_channel += stage1_value;
		} 

		double average_value_for_channel = total_value_for_channel / SELMA_NUM_REPEAT;

		double max_variation_acceptable = (double)(average_value_for_channel / 100 * SELMA_MAX_COUNT_VARIATION_PERCENT);
		max_variation_acceptable = ((max_variation_acceptable > SELMA_MAX_COUNT_VARIATION) ? max_variation_acceptable : SELMA_MAX_COUNT_VARIATION);
		uint16_t variation = max_value - min_value;
		// Value is Consistent as long as variation is within SELMA_MAX_COUNT_VARIATION_PERCENT of original value
		bool isChannelConsistent = ((double)variation <= max_variation_acceptable);
		
		if(!isChannelConsistent)
	    {
			// std::cout << std::to_string(average_value_for_channel) + ","  + std::to_string(max_variation_acceptable) << std::endl;
			std::cout << logPrefix + "Inconsistent for Output " + std::to_string(i) + " Variation > Acceptable_Variation : "+ std::to_string(variation) + " > " + std::to_string(max_variation_acceptable) << std::endl;
	    }


		isConsistent = isConsistent && isChannelConsistent;
		max_variation = ((max_variation > variation) ? max_variation : variation); // Condition ? IF_TRUE : ELSE;
	}   
    
	return isConsistent;
}

bool SELMA_IF::stage1MontonicityCheck(uint16_t** stage1_data, uint16_t* baseline, uint8_t chip_grp)
{	
	bool isMonotone = true;

	for(int num_spk=2; num_spk<SELMA_MAX_NUM_SPIKES; num_spk++)
		{
		for(int i=0; i<SELMA_NUM_OUTPUT_CHANNELS; i++)
		{		
			if(isOutInChipGrp(i,chip_grp))
			{
				if ((stage1_data[num_spk-1][i] + SELMA_MAX_COUNT_VARIATION) < stage1_data[num_spk-2][i])
				{
					isMonotone=false;
					std::cout << "Non-Monotonic Behaviour at (#spk, ro_value) : ";
					std::cout << "(" + std::to_string(num_spk) + "," + std::to_string(stage1_data[num_spk-1][i]) + ") < ";
					std::cout << "(" + std::to_string(num_spk-1) + "," + std::to_string(stage1_data[num_spk-2][i]) + ")";
					std::cout << std::endl;
				}				
			}
		}
	}

	return isMonotone;
}

bool SELMA_IF::stage1CrossTalkCheck(uint16_t** stage1_data, uint16_t* baseline, uint8_t chip_grp)
{
	bool hasCrossTalk = false;

	for(int num_spk=1; num_spk<SELMA_MAX_NUM_SPIKES; num_spk++)
	{
		for(int i=0; i<SELMA_NUM_OUTPUT_CHANNELS; i++)
		{		
			if(!isOutInChipGrp(i,chip_grp))
			{
				// CrossTalk = When Input Changes for non-input chips is Cannot be accountable via analog variations
				if(stage1_data[num_spk-1][i] > ((uint32_t)baseline[i] + SELMA_MAX_COUNT_VARIATION))
				{
					std::cout << "stage1_data " + std::to_string(stage1_data[num_spk-1][i]) + " Baseline : " + std::to_string(baseline[i]) << std::endl;
					hasCrossTalk=true;
				}
			}
		}
	}

	return hasCrossTalk;
}

bool SELMA_IF::isOutInChipGrp(uint16_t ro_addr, uint8_t chip_grp)
{
	uint8_t board_num = chip_grp >> 1;
	 // %% Readout Number Order = P(3), P(2), P(1), P(0) // board_sel = 3 - board_num;
	int board_st_idx = 4 * (3 - board_num); // board_st_idx = 4*board_sel+1;

	// for st_sel_idx = board_st_idx:16:size(ro,2)
    //     board_data = [board_data, ro(:,st_sel_idx:st_sel_idx+3)];
    // end
	for(int i=board_st_idx; i < SELMA_NUM_OUTPUT_CHANNELS; i+=16)
	{
		// i to i+3 are all in board p
		bool isFirstPairOnBoard = chip_grp & 0x01;
		if (isFirstPairOnBoard)
		{
			// Chip Data is idx 1 and idx 2
			if (ro_addr == i || ro_addr == (i+1))
			{
				return true;
			}
		}else
		{
			if (ro_addr == (i+2) || ro_addr == (i+3))
			{
				return true;
			}
		}
	}
	return false;
}


void SELMA_IF::writeHiddenLayerOutToFile(uint16_t* stage1_data, std::string filename, int num_data)
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


void SELMA_IF::writeHiddenLayerOutToFile(int64_t* stage2_data, std::string filename, int num_data)
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

void SELMA_IF::convertEventRateToDelays(uint16_t* event_rate, uint16_t* delays, uint32_t array_length)
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

void SELMA_IF::printOpalKellyReturnToString(std::string logPrefix, std::string topic, long returnCode)
{
	if (returnCode > 0)
	{
		// std::cout << logPrefix + "Number of " + topic + " Bytes written : " + std::to_string(returnCode) +" \n";
	}else
	{
		std::cout << logPrefix + topic + " Write Error Code is : " + std::to_string(returnCode) + " " + convertOpalKellyErrorCodeToString(returnCode)+ " \n";
	}
}
/**
	Converts event rate to delays (SELMA_IF's input)
	INPUT:
		Accepts Event_rate vector 
		Buffer for Output Delays as Input
	Event Rate and Delays should be of the same size
*/
std::string SELMA_IF::convertOpalKellyErrorCodeToString(int ErrorCode)
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
