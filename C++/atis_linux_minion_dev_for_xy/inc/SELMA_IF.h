#ifndef SELMA_IF_H
#define SELMA_IF_H

#include <string>

#include "event_framework.h"

// SAFEGUARDS FOR BAD LOOPS
#define SELMA_MAX_TOTAL_TIME 10000000 // MAX Waiting for 5s Terminate Program as not responsive if > 5s of wait time

// Directives for SELMA-16 Chip Operation
#define SELMA_MIN_SLEEP_TIME 30000
// First Stage Directives
#define SELMA_NUM_INPUT_CHANNELS 1024   // Number of Input Addresses for SELMA_IN
#define SELMA_NUM_OUTPUT_CHANNELS 2048  // NUMBER of Hidden Layer Output
#define SELMA_MAX_NUM_SPIKES 15               // MAX Number of Input Spikes Accepted 
#define SELMA_NUM_REPEAT 5                    // Number of Repeats for Same Input for Input->Output Consistency Check
#define SELMA_MAX_COUNT_VARIATION 30          // Max Difference between Counts for Same Input
#define SELMA_MAX_COUNT_VARIATION_PERCENT 10
// 2nd Stage DIRECTIVES
#define SELMA_NUM_CLASSES 6             // NUM CLASSES Output for  2nd stage data
#define SELMA_NUM_BYTES_PER_CLASS 8     // NUM Bytes to Read per Class for 2nd Stage

enum PipeOutAddress_SELMA
{
	RO_F1DataIn = 0xA1,
	RO_F2DataIn = 0xA2
};

enum PipeInAddress_SELMA
{
    SELMA_IF_PipeIn           = 0x8A,     // FIFO PIPEIN - 1024 delay values 
    COMMS_IF_OBJ_ID_FIFO_ADDR = 0x92,     // OBJ_ID - 1 8-bit word
    COMMS_IF_X_ADDR           = 0x93, // X_POS  - 1 16-bit word
    COMMS_IF_Y_ADDR           = 0x94, // X_ADDR - 1 16-bit word
    COMMS_IF_WIDTH            = 0x95,  // WIDTH  - 1 16-bit word
    COMMS_IF_HEIGHT           = 0x96, // HEIGHT - 1 16-bit word        
    COMMS_IF_CLASSDATA        = 0x97 // CLASS_DATA(0-5) - 24 16-bit words
};

enum WireOutAddress_SELMA
{
    selma_status = 0x28
};

enum SelmaStatusBits 
{
    SelmaIfConfigActive = 2,  // Indicates the active status of the config sequence. (This signal goes back to '0' after the completion of config sequence.)
    SelmaIfFeedinActive = 4,  // Indicates the active status of the feedin sequence. (This signal goes back to '0' after the completion of feedin sequence.)
    SelmaIfConfigDone = 8,    // Indicates the completion of the config sequence. (This signal stays at '1', till cleared by the 'SelmaIfClearConfig' control bit.)
    SelmaIfFeedinDone = 16,    // Indicates the completion of the feedin sequence. (This signal stays at '1', till cleared by the 'SelmaIfClearFeedin' control bit.)

    SelmaIfROEnable = 32,   // Indicates Readout Stage is completed
    SelmaIfRODone   = 64,    // Indicates the readout data from fifo1 and fifo2 has been read
    SelmaIfROFifo1Empty = 128,  // readout fifo1 empty
    SelmaIfROFifo2Empty = 256   // readout fifo2 empty
};

enum TriggerInAddress_SELMA
{
	SelmaTriggers = 0x50
};

enum SelmaTriggerWires
{
    SelmaIfReset = 0,       // Resets the SELMA interface module
    SelmaIfStartConfig = 1, // Starts the SELMA configuration sequence with the configuration from the built-in ROM
    SelmaIfStartFeedin = 2, // Starts the SELMA spike feedin sequence with the preconfigured spike rate. (Currently this is 5 spikes per input. Once we test the basic interface, I will add spike input pipe from C++ to SELMA IF)
    SelmaIfClearConfig = 3, // Clears the 'SelmaIfConfigDone' flag. This should be done before initiating next config sequence.
    SelmaIfClearFeedin = 4 // Clears the 'SelmaIfFeedinDone' flag. This should be done before initiating next feedin sequence.
};

class SELMA_IF: public DataConsumer 
{
public:
	SELMA_IF();
    SELMA_IF(bool testFull);

    uint16_t getAddrFromXY(uint16_t x_addr, uint16_t y_addr); // Conversion from x y to A
    std::pair<uint16_t*,int64_t*> sendInputToSELMA(uint16_t* delays);    
    
    // General Event-based Utility Functions     
    void convertEventRateToDelays(uint16_t* event_rate, uint16_t* delays, uint32_t array_length);
    uint16_t getDelayVal(uint16_t num_spk); // Spike Rate to Delay Value(SELMA_IF Input) Conversion 
    
    bool processData(packet_info_type* packet_info, event* dataIN,
            bool exit_thread);
    
private:
	const std::string logPrefix = "SELMA_IF: ";

	// Main Flow Functions    
	void initSELMA(); // Initializes SELMA Configuration Process and Run Some Test Inputs to ensure boards are working
	void configSELMA(); // Starts SELMA Configuration Process
	void testSELMAFull(); // Same as testSELMA but also Verifies 2nd Stage Multiplier (Assumes Coefficients is 6,5,4,3,2,1)     

    // Sub Functions for Testing SELMA_IF Correctness
    bool stage1ConsistencyCheck(uint16_t** stage1_data_consistency_check); // Returns Max Variation in Input 
    bool stage1MontonicityCheck(uint16_t** stage1_data, uint16_t* baseline, uint8_t chip_grp);
    bool stage1CrossTalkCheck(uint16_t** stage1_data, uint16_t* baseline, uint8_t chip_grp);
    bool isOutInChipGrp(uint16_t ro_addr, uint8_t chip_grp);


    // SELMA Specfic Utility Functions
    void waitSelmaStatusBit(uint32_t STATUS_BIT); // Busy waits for SELMA Status Bit to be active 
    void clearDoneFlag(uint32_t DoneFlag, uint32_t ClearFlagTrigger); // Clears SELMA Status Flag 
    
    std::string convertOpalKellyErrorCodeToString(int ErrorCode);
    void printOpalKellyReturnToString(std::string logPrefix, std::string topic, long returnCode);
    
    // File Writing and Sending Data to SELMA
    bool isDonePassingData = false;
    void writeHiddenLayerOutToFile(uint16_t* stage1_data, std::string filename, int num_data);
    void writeHiddenLayerOutToFile(int64_t* stage1_data, std::string filename, int num_data);
    void passDataThroughSELMA(std::string test_folder, std::string csv_filename);
};
#endif 
