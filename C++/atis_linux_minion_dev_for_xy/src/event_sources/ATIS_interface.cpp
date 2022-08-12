#include <ATIS_interface.h>
#include <unistd.h>
#include <string.h>

/***************************************************************************************************
Input       : Takes in, (1) The FPGA control bitfile, (2) a text version of the bias control file, 
              (3) and (4) two files containing the lookup tables for VDAC 1.8V and 3.3V circuits
Output      : returns 1 when configuration is successfully programmed to ATIS sensor. 
              Otherwise 0, or -1  is returned
Description : Routine that is called to initializes the ATIS Camera.
 *****************************************************************************************************/
AtisSource::AtisSource(source_parameters_type &parameters)
: DataSource(parameters.x_dim,parameters.y_dim, parameters.pix)
{
	atis_quit = false;
	v4=false;
	if(parameters.atis_version == 4)
		v4 = true;

	const char bias_lookup_18[] = "../bias_config/vdac18_lookupb8m1.txt";  //default biases file for vdac18
	const char bias_lookup_33[] = "../bias_config/vdac33_lookupb8m1.txt";  //default biases file for vdac33

	simulate = parameters.simulate;
	first_packet = true;
	//simulation_start_time_offset = 0;

	enable_APS = parameters.useGrayscale;

	if (OkInterface::Instance()->ConfigureFPGA(parameters.fpga_bitfile) != 0) {
		printf("Error programming the FPGA\n");
	} else {
		printf("FPGA programmed with %s\n", parameters.fpga_bitfile.c_str());
	}

	pthread_mutex_lock(&OkInterface::ok_mutex);
	//Sets IO Functions here
	initATIS_IO();


	//	*comments = (char*)malloc(100*sizeof(char)); //dummy initialization of comments so that freeing the memory for comments does not cause a segmentation fault in the main function
	parameters.comments += "#ATIS version 6 sensor\n";

	//Program the Biases from the Text File that has been opened earlier
	if (v4 == false)
		programBiasfromText_v6(parameters.biasfile.c_str(),bias_lookup_18,bias_lookup_33);
	else
		writeBiases_v4();



	printf("Writing Biases\n");
	usleep(304*240); 			//sleep 1 microsecond for every pixel

	if(parameters.F_NN_enable)
	{
		OkInterface::Instance()->SetWireInValue(WireInAddress::Filter_threshold, parameters.F_NN_time);
		ControlSignalWire = ControlSignalWire | ControlSignalBits::Filter_enable;
		OkInterface::Instance()->SetWireInValue(WireInAddress::ControlSignals, ControlSignalWire);
		OkInterface::Instance()->UpdateWireIns();
		printf("FPGA filter configured\n");
	}

	if(parameters.F_RF_enable)
	{
		OkInterface::Instance()->SetWireInValue(WireInAddress::Refractory_threshold, parameters.F_RF_time);
		ControlSignalWire = ControlSignalWire | ControlSignalBits::Refraction_enable;
		OkInterface::Instance()->SetWireInValue(WireInAddress::ControlSignals, ControlSignalWire);
		OkInterface::Instance()->UpdateWireIns();
		printf("FPGA refraction configured\n");
	}

	if(parameters.F_remove)
	{
		ControlSignalWire = ControlSignalWire | ControlSignalBits::remove_filtered;
		OkInterface::Instance()->SetWireInValue(WireInAddress::ControlSignals, ControlSignalWire);
		OkInterface::Instance()->UpdateWireIns();
		printf("FPGA will remove filtered events from stream\n");
	}

	if(parameters.ms_tick_enable)
	{
		ControlSignalWire2 = ControlSignalWire2 | ControlSignalBits2::ms_tick_enable;
		ControlSignalWire2 = ControlSignalWire2 | 1;
		OkInterface::Instance()->SetWireInValue(WireInAddress::ControlSignals2, ControlSignalWire2);
		OkInterface::Instance()->UpdateWireIns();
		printf("1 millisecond tick events enabled\n");
	}

	if(parameters.OF->enable_OF)
	{
		unsigned char OF_control_bits = 1; //1 means enable optical flow
		if(parameters.OF->filter_by_goodness)
			OF_control_bits+= 2;
		if(parameters.OF->cartesian_not_polar)
			OF_control_bits+= 4;
		OF_control_bits += parameters.OF->num_pixels_threshold<<4;

		OkInterface::Instance()->SetWireInValue(WireInAddress::OF_control, OF_control_bits);
		OkInterface::Instance()->SetWireInValue(WireInAddress::OF_refrac_threshold, parameters.OF->refrac_threshold);
		OkInterface::Instance()->SetWireInValue(WireInAddress::OF_old_pixel_threshold, parameters.OF->old_pixel_threshold);
		OkInterface::Instance()->SetWireInValue(WireInAddress::OF_fit_distance_threshold, parameters.OF->fit_distance_threshold);
		OkInterface::Instance()->SetWireInValue(WireInAddress::OF_goodness_threshold, parameters.OF->goodness_threshold);
		OkInterface::Instance()->UpdateWireIns();
		printf("FPGA optical flow configured\n");
		//printf("%i %i %i %i %i \n", OF_control_bits, parameters.OF->refrac_threshold, parameters.OF->old_pixel_threshold, parameters.OF->fit_distance_threshold, parameters.OF->goodness_threshold);
	}
	//reset FIFOS
	OkInterface::Instance()->ActivateTriggerIn(TriggerInAddress::ControlTriggers, TriggerWires::Reset_module_fifo); 	//reset fifo
	OkInterface::Instance()->ActivateTriggerIn(TriggerInAddress::ControlTriggers, TriggerWires::reset_event_chain);

	if(simulate)
	{
		// this is a quick and dirty check to see if the correct bitfile has been used.
		OkInterface::Instance()->UpdateWireOuts();
		uint32_t available_space = uint32_t(OkInterface::Instance()->GetWireOutValue(WireOutAddress::sim_RAM_pages));
		if(available_space == 0)
		{
			printf("ERROR: FPGA configuration does not appear to support hardware simulation. \n");
			printf("\t\t\t Are you sure you have the latest FPGA bitfile? \n");
			printf("Try running the following commands to download the latest bitfiles:\n");
			printf("wget -O bitfiles.zip https://www.dropbox.com/sh/lkabdc33offdhzu/AAC7yIql-jdjyHTcQtH5-XUDa?dl=1\n");
			printf("unzip -o bitfiles.zip -d atis_bitfiles\n");
			printf("rm bitfiles.zip\n");
		}

		simulationfile = parameters.simulationfile;
		initSimulation();
	}

	if(parameters.enableTN)
	{
		ControlSignalWire = ControlSignalWire | ControlSignalBits::TN_enable;
		OkInterface::Instance()->SetWireInValue(WireInAddress::ControlSignals, ControlSignalWire);
		OkInterface::Instance()->UpdateWireIns();
		printf("TrueNorth interface enabled\n");
	}

	pthread_mutex_unlock(&OkInterface::ok_mutex);

	//	if(dev->ActivateTriggerIn(0x40, 7)) 	//reset event chain
	//return 1;
	//	else
	//		printf("FIFOs reset\n");
	//	return 0;
	get_new_data = true;
	LSB_overflow_counter = 0;
	MSB_overflow_counter = 0;


	//num_bytes_read = 0;
	//last_byte_location = 0;
	printf("ATIS source initialized\n");
}


/************************************************************************************************************
Input      : None
Output     : return 1 if operation is sucessful, 0 otherwise
Description: Initializes the Opal Kelly Board from which the ATIS sensor is attached to
 *************************************************************************************************************/
int AtisSource::initATIS_IO()
{
	if(simulate == false)
	{
		ControlSignalWire = ControlSignalWire | ControlSignalBits::TD_enable; //enable TD

		if (enable_APS == true)
		{
			ControlSignalWire = ControlSignalWire | ControlSignalBits::APS_enable; //enable APS
			ControlSignalWire = ControlSignalWire | ControlSignalBits::couple; //couple
		}
	}

	if(OkInterface::Instance()->SetWireInValue(WireInAddress::ControlSignals, ControlSignalWire)) //trigger the FPGA to send biases to the chip
		return 0;
	else
		printf("Digital lines programmed\n");

	OkInterface::Instance()->UpdateWireIns();
	return 1;
}


/************************************************************************************************************
Input      	: (1) Bias Buffer (8-bit) to be serialized to ATIS 
Output     	: return 1 if successful and 0 if write to ATIS is not successful
Description	: The routine serializes all 28 bias control registers stored in an 8-bit contiguous array to the 
                  ATIS sensor via the opal kelly FPGA interface routines 
 *************************************************************************************************************/
int AtisSource::writeBiases_v6(unsigned char *bias_bytes)
{
	if(v4 == false)
	{
		if(OkInterface::Instance()->WriteToPipeIn(PipeInAddress::Biases, 114, bias_bytes) == 0)//write the biases to FPGA here
			return 0;
		else
			printf("Biases sent to FPGA\n");
		ControlSignalWire = ControlSignalWire | ControlSignalBits::BGenPower_Up; //bias generator powerup wire

		if(OkInterface::Instance()->SetWireInValue(WireInAddress::ControlSignals, ControlSignalWire)) //trigger the FPGA to send biases to the chip
			return 0;
		else
			printf("Bias generator powered up\n");
	}
	else
	{
		printf("\nERROR: version mixup. 'writeBiases_v6' should not be called with ATIS v4");
	}      
	OkInterface::Instance()->UpdateWireIns();
	printf("Bias setup successfull\n");
	return 1;

}

/************************************************************************************************************
Input      	: (1) Bias Buffer (8-bit) to be serialized to ATIS 
Output     	: return 1 if successful and 0 if write to ATIS is not successful
Description	: The routine serializes all 28 bias control registers stored in an 8-bit contiguous array to the 
                  ATIS sensor via the opal kelly FPGA interface routines 
 *************************************************************************************************************/
int AtisSource::writeBiases_v4()
{
	if(v4 == false)
	{
		printf("\nERROR: version mixup. 'writeBiases_v4' should not be called with ATIS v6");
	}
	else
	{
		printf("ATIS is v4: using onboard biases\n");
		if(OkInterface::Instance()->ActivateTriggerIn(TriggerInAddress::ControlTriggers, TriggerWires::DAC_reset)) //trigger the FPGA to send biases to the chip
			return 0;
		else
			printf("Bias programming triggered\n");
	}

	OkInterface::Instance()->UpdateWireIns();
	printf("Bias setup successfull\n");
	return 1;
}


/************************************************************************************************************
Input      : (1) Pointer to the data buffer, (2) the thread ID running the routine
Output     : Returns the number of bytes read. 
Description: Obtains ATIS data from the FPGA board and stores it to an buffer
 *************************************************************************************************************/
//packet_info_type getATISdata(event* events_out, int mythread)
bool AtisSource::getData(packet_info_type* packet_info, event* dataOUT, bool exit_thread)
{	
	//	printf("Starting call %i to read data from ATIS\n", ++atis_call_number);
	//printf("Read data called\n");
	int read_length = 0;

	if(simulate == true)
	{
		//		printf("simulate is true, calling sim data\n");
		pthread_mutex_lock(&OkInterface::ok_mutex);
		send_sim_data();
		pthread_mutex_unlock(&OkInterface::ok_mutex);
	}

	if(get_new_data == true)
	{
		//	printf("Getting new data\n");
		pthread_mutex_lock(&OkInterface::ok_mutex);
		//	printf("ATIS_mutex locked %i\n", atis_call_number);
		while(read_length < 1){
			//	    printf("Attempting to read ATIS data %i\n",atis_call_number);
			OkInterface::Instance()->UpdateWireOuts();
			read_length = OkInterface::Instance()->GetWireOutValue(WireOutAddress::event_RAM_pages);
		}
		//        printf("ATIS data received %i\n",atis_call_number);
		if(read_length>max_read_length)
			read_length = max_read_length;

		num_bytes_to_decode = OkInterface::Instance()->ReadFromBlockPipeOut(PipeOutAddress::EventsIn, 1024, 1024*(int)(read_length), event_bytes);
		num_bytes_decoded = 0;
		//	printf("Read %i bytes from Opal Kelly\n", num_bytes_read);
		pthread_mutex_unlock(&OkInterface::ok_mutex);

	}

	//PASS PACKET INFO AS A POINTER, HAVE THE FUNCTION RETURN THE NUMBER OF BYTES DECODED. 
	//POINT TO A LOCATION IN event_bytes. i.e. &event_bytes[last_byte_location]
	//printf("calling decode\n");
	//packet_info->num_events = num_bytes_read;
	num_bytes_decoded += decodeATISdata( (num_bytes_to_decode - num_bytes_decoded),  &event_bytes[num_bytes_decoded], dataOUT, packet_info);

	//printf("decoded, there are %i bytes remaining \n",  (num_bytes_to_decode - num_bytes_decoded));

	/*	if(packet_info->num_events>0)
		printf("%i events read in at time %lu\n", packet_info->num_events, (uint64_t(packet_info->time_start)<<16)+dataOUT[packet_info->num_events-1].t);
	else
		printf("0 events read in\n");
	 */
	if(num_bytes_to_decode == num_bytes_decoded)
		get_new_data = true;
	else
		get_new_data = false;

	if (exit_thread == 1 | atis_quit == 1)
	{
		printf("Exit threads initiated\n");
		return 1;		
	}


	//	printf("Completed call %i to read data from ATIS\n", atis_call_number);
	return 0;
}

/************************************************************************************************************
Input      : (1) The number of bytes to process. (2) Input Data Buffer, (3) Pointer to the decoded data buffer
             (4) The thread Id
Output     : Returns information about the collected events;
Description: Converts the raw data from the ATIS sensor and converts them into TD events for further downstream 
             processing.
 *************************************************************************************************************/
uint32_t AtisSource::decodeATISdata(uint32_t num_bytes, unsigned char* dataIN, event* dataOUT, packet_info_type* packet_info)
{
	//	printf("Start decoding in events for call number %i\n", ++decode_call_number);
	packet_info->time_start = MSB_overflow_counter;
	packet_info->packet_type = packet_type::TD_EM_mixed_packet;
	/*if (limit_recording_time == true)
		if (packet_info.time_start > recording_length)
		{	
			printf("Execution end time reached, stopping recording\n");
			exit_threads[0] = 1;
		}
	 */
	uint32_t num_events_out = 0;
	if(num_bytes>0)
	{
		for(int eventNum=0; eventNum<num_bytes; eventNum+=8)
		{
			/*			if(dataIN[eventNum+1] == evt_type::TD_filtered)
				printf("Filtered event detected\n");
			 */
			dataOUT[num_events_out].type 	= dataIN[eventNum+1];
			if(dataOUT[num_events_out].type == evt_type::timer_overflow)
			{
				packet_info->num_events = num_events_out;
				MSB_overflow_counter++;
				//			printf("packet end time due to overflow %lu \n", uint64_t(MSB_overflow_counter)<<16);
				num_events_out++;
				return num_events_out*8;
			}
			else if (dataOUT[num_events_out].type == evt_type::quit)
			{
				packet_info->num_events = num_events_out;
				num_events_out++;
				printf("exit event received, quitting program\n");
				atis_quit = true;
				return num_events_out*8;
			}

			dataOUT[num_events_out].subtype	= dataIN[eventNum];
			dataOUT[num_events_out].y 		= uint16_t(dataIN[eventNum+3])<<8 | dataIN[eventNum+2];
			dataOUT[num_events_out].x 		= uint16_t(dataIN[eventNum+5])<<8 | dataIN[eventNum+4];
			dataOUT[num_events_out].t 		= uint16_t(dataIN[eventNum+7])<<8 | dataIN[eventNum+6];

			//printf("event of type %i at x=%i, y=%i, at time t=%i with pol p=%i\n",dataOUT[num_events_out].type, dataOUT[num_events_out].x, dataOUT[num_events_out].y, dataOUT[num_events_out].t, dataOUT[num_events_out].subtype);
			num_events_out++;
		}
	}
	//printf("packet end time due to end receive %lu \n", uint64_t(MSB_overflow_counter)<<16 | dataOUT[num_events_out-1].t);
	packet_info->num_events = num_events_out;
	//packet_info.time_end = overflow_counter & 0xFFFF0000;

	//printf("Read in %i events at time %i\n", packet_info.num_events, packet_info.time_end | dataOUT[packet_info.num_events-1].t);
	//	printf("Decoded %i events at time %i for call number %i\n", packet_info.num_events, packet_info.time_end | dataOUT[packet_info.num_events-1].t, decode_call_number);
	// printf("Read in %i events (over %i microseconds)\n", packet_info.num_events, packet_info.time_end + dataOUT[packet_info.num_events-1].t - packet_info.time_start -dataOUT[0].t);
	//printf("returning that %i bytes have been read (full)", num_bytes);
	return num_bytes;
}


/************************************************************************************************************
Input      : (1) Buffer to voltage values. (2) Pointer to the bias data 
Output     : Returns 1 if successful, 0 if otherwise.
Description: A function which composes all voltage values for all 28 bias control registers
 *************************************************************************************************************/
void AtisSource::createBiasData(int *biasData, char *biasStr)
{
	char value_string[20];
	biasStr[0] = '\0';

	sprintf(biasStr,"#");
	for(int i = CtrlbiasLP; i < biasAPSreset; i++) {
		sprintf(value_string,"%d",biasData[i]);
		biasStr = strcat(biasStr,value_string);
		biasStr = strcat(biasStr,",");
	}

	sprintf(value_string,"%d",biasData[biasAPSreset]);
	biasStr = strcat(biasStr,value_string);
}

int AtisSource::readBiasesFromFile(const char* biasfile_path, const char* bias_lookup_18, const char* bias_lookup_33, unsigned char *bias_bytes)
{
	FILE *bias_file_ptr;

	unsigned int polarity=0;
	unsigned int cascode=0;
	unsigned int reg=0;
	unsigned int pad_enable=0;
	unsigned int internal_bias_enable=0;
	unsigned int bias_type=0;
	unsigned int_buf=0;
	unsigned char regStr[30];
	unsigned int voltage=0;

	unsigned int register_value = 0;
	unsigned int pad_drv=0;

	int dataIndex=2;
	unsigned int lower_16=0;
	unsigned int upper_16=0;
	char readStr[150];

	struct bias b;

	BiasLookup18.clear();
	BiasLookup33.clear();
	populateLookupTables(bias_lookup_18,true);
	populateLookupTables(bias_lookup_33,false);

	bias_file_ptr = fopen(biasfile_path,"r");

	if(bias_file_ptr == NULL)
		return 0;

	//Read all Comments
	while (fgets(readStr,150,bias_file_ptr) != NULL )
		if(readStr[0] != '#')
			break;

	//Read in the output drive strength
	bias_bytes[0] = 0;
	bias_bytes[1] = (unsigned char)((atoi(readStr) & 0x0000000F) << 4);

	for (int i = CtrlbiasLP; i <= biasAPSreset; i++)
	{
		register_value = 0;

		if(fscanf(bias_file_ptr,"%d,%s\n",&voltage,regStr) == 2)
		{ 

			printf("%d,%s\n",voltage,regStr);
			BiasData[i] = voltage;

			if(!returnBiasfromVoltage(voltage,(BiasName)i,b))
				return 0;

			//polarity (bit 29)
			register_value = register_value | (b.polarity << 29);

			//column: cascode (bit 28) (always 1)
			register_value = register_value | (1 << 28);

			//column: digital register input value (decimal coded) (bit [20:0] or bit [7:0])
			register_value  = register_value | (b.register_value & 0x000FFFFF);

			//Bit 31 (pad enable) should be 0.
			register_value  = register_value | (0 << 31);

			//Bit 30 (internal bias enable) should be 1.
			register_value = register_value | (1 << 30);

			//Bit 27 (bias type) depends on your choice (0 for current DAC / 1 for voltage DAC)
			register_value =  register_value | (1 << 27);

			//For the internal buffer value bits [26:21] a decimal value of 8 is chosen.
			register_value = register_value | (8 << 21);

			//printf("Register %x\n",register_value);

			lower_16=0; upper_16=0;
			upper_16 = (register_value & 0xFFFF0000) >> 16;
			lower_16 = (register_value & 0x0000FFFF);

			bias_bytes[dataIndex]   =   (unsigned char) lower_16;
			bias_bytes[dataIndex+1] =   (unsigned char)(lower_16  >>8 );
			bias_bytes[dataIndex+2] =   (unsigned char) upper_16;
			bias_bytes[dataIndex+3] =   (unsigned char)(upper_16 >>8);

			dataIndex +=4;

		}
	}
}



/************************************************************************************************************
Input       : (1) the look table file path and (2) a boolean variable choosing between 3.3 V or 1.8V VDAC.
Output      : Initializes two global vectors containing the bias configuration information
Description : This function initializes the VDAC tables in memory from the lookup tables in text files.
 *************************************************************************************************************/
int AtisSource::populateLookupTables(const char* lookup_table_path, bool is1800)
{
	FILE *bias_file_ptr;

	int param_1=0;
	int voltage=0;
	int param_3=0;
	int param_4=0; 
	int polarity=0;
	int param_6=0;
	int register_value=0;
	char readStr[150];

	unsigned int pad_drv=0;
	unsigned char bias_bytes[114];

	int dataIndex=2;
	unsigned int lower_16=0;
	unsigned int upper_16=0;

	bias_file_ptr = fopen(lookup_table_path,"r");

	if(bias_file_ptr == NULL)
		return 0;

	while (fgets(readStr,150,bias_file_ptr) != NULL ) 
	{
		struct bias b;
		sscanf(readStr,"%d,%d,%d,%d,%d,%d,%d",&param_1,&b.voltage,&param_3,&param_4,&b.polarity,&param_6,&b.register_value);

		if(is1800)
			BiasLookup18.push_back(b);
		else
			BiasLookup33.push_back(b);
	}

	fclose(bias_file_ptr);
	return 1;
}


/************************************************************************************************************
Input      	: (1) The specified voltage, (2) The selected bias register
Output     	: (3) Bias information return by reference
Description	: A function which obtains the bias configurations with respect to the supplied voltage and 
                  register type.
 *************************************************************************************************************/
int AtisSource::returnBiasfromVoltage(int voltage, BiasName bName, struct bias& b) 
{
	if(BiasType[bName]) {
		for (std::vector<bias>::iterator it = BiasLookup18.begin(); it != BiasLookup18.end(); ++it){
			if(it->voltage >= voltage) {
				b.voltage = it->voltage;
				b.polarity = it->polarity;
				b.register_value = it->register_value;
				return 1;
			}
		}
	}
	else{
		for (std::vector<bias>::iterator it = BiasLookup33.begin(); it != BiasLookup33.end(); ++it){
			if(it->voltage >= voltage) {
				b.voltage = it->voltage;
				b.polarity = it->polarity;
				b.register_value = it->register_value;
				return 1;
			}

		}
	}
	return 0;
}


/************************************************************************************************************
Input      	: (1) The bias control file, (2) & (3) The VDAC lookup tables 
Output     	: Returns 1 if operation is successful, 0 otherwise.
Description	: Reads the bias control file and programs the ATIS sensor accordingly
 *************************************************************************************************************/
int AtisSource::programBiasfromText_v6(const char* biasfile_path,const char* bias_lookup_18, const char* bias_lookup_33) 
{
	unsigned char bias_bytes[114];

	if(readBiasesFromFile(biasfile_path,bias_lookup_18,bias_lookup_33,bias_bytes))
	{
		writeBiases_v6(bias_bytes);
	}
	else
		return 1;

	return 0;
}

/************************************************************************************************************
Input      	: none, refers to the ATIS class
Output     	: Returns 1 if operation is successful, 0 otherwise.
Description	: Opens a simulation input file and calculates how much data is present.
 *************************************************************************************************************/
int AtisSource::initSimulation()
{
	sim_input_file.open(simulationfile.c_str(), std::ios::in | std::ios::binary);
	if (sim_input_file.is_open())
		printf("Opened simulation input file %s\n", simulationfile.c_str());
	else 
	{
		printf("\n ERROR: simulation input file not found   %s\n\n", simulationfile.c_str());
		return 0;
	}

	//sim_overflow_event = new event;
	sim_time = 0;

	/*sim_overflow_event.x = 305;
	sim_overflow_event.y = 240;
	sim_overflow_event.t = 8191;
	sim_overflow_event.type = 0;
	sim_overflow_event.subtype = 0;*/

	sim_send_bytes_count = 0;
	old_sim_byte_count = 0;
	//old_sim_event_bytes = new unsigned char[10]; //just to have an initialized array 

	int file_version = 0;
	char file_version_chars[10]; //must be 3 to leave enough space for the end line character
	sim_input_file.get((char*)file_version_chars, 10, '\n');
	if (file_version_chars[0] == '#')
		file_version = 0;
	else
		if (file_version_chars[0] == 'v')
			file_version = atoi(&file_version_chars[1]);
		else
			printf("ERROR: Cannot detect file version\n");

	printf("file is version %i\n", file_version);
	//ignore each line if it starts with a comment mark '#'
	sim_input_file.ignore(10000, '\n');
	while(sim_input_file.peek() == '#')
		sim_input_file.ignore(10000, '\n');
	uint16_t dimension[10]; //must be 3 to leave enough space for the end line character
	sim_input_file.getline((char*)dimension, 20, '\n');	

	if(sim_input_file.gcount() ==1)
	{
		printf("No dimension specification found in file, defaulting to ATIS resolution of %i x %i\n", 304, 240);
		DataSource::x_dim = 304;
		DataSource::y_dim = 240;
	}else
	{
		printf("Dimension specification from file is %i x %i pixels\n", dimension[0], dimension[1]);
		DataSource::x_dim = dimension[0];
		DataSource::y_dim = dimension[1];
//		input_file.ignore(10000, '\n');
	}


	int file_binary_start_position = sim_input_file.tellg(); //MUST STILL DECLARE
	sim_input_file.seekg(0, std::ios::end); //go to the end of the file
	sim_input_file_size = (long)(sim_input_file.tellg()) - file_binary_start_position; //determine how long the binary portion of the file is
	sim_input_file.seekg(file_binary_start_position); //go back to the start of the binary portion
	printf("Simulation input file size is  %llu\n", sim_input_file_size);
	printf("Preloading simulation data onto the FPGA\n");


	send_sim_data();

	printf("finished initializing the simulation, %llu bytes remain to be sent (%llu + %llu). Will trigger start of simulation\n", sim_input_file_size+old_sim_byte_count, sim_input_file_size, old_sim_byte_count);
	// enable the simulation: ControlSignal 0x8000 on ControlSignalWireIn 0x00
	ControlSignalWire = ControlSignalWire | ControlSignalBits::enable_simulation;
	OkInterface::Instance()->SetWireInValue(WireInAddress::ControlSignals, ControlSignalWire);
	OkInterface::Instance()->UpdateWireIns();
	return 0;
}

/************************************************************************************************************
Input      	: none, refers to the ATIS class
Output     	: Returns 1 if operation is successful, 0 otherwise.
Description	: sends simulation data to the ATIS FPGA
 *************************************************************************************************************/
int AtisSource::send_sim_data()
{
	//printf("Send sim data called\n");
	// check how much space is available on the FPGA: Wire Out 0x25
	OkInterface::Instance()->UpdateWireOuts();
	uint32_t available_space = uint32_t(OkInterface::Instance()->GetWireOutValue(WireOutAddress::sim_RAM_pages))*1024;
	//printf("Available FPGA RAM for sim is %i*1024 bytes\n", available_space);

	// while there is less than one block of bytes to write, and there are more bytes to be read
	while((sim_send_bytes_count<1024) & (sim_input_file_size>0))
	{
		// read until there are 1024 bytes, or the file has ended
		read_sim_packet();
		if(sim_packet_info.num_events >0)
			encode_sim_packet();
	}

	while((available_space>sim_send_bytes_count) & sim_input_file_size>0) //as long as there is enough space to write the data, and the file hasn't finished
	{
		//	printf("writing simulation data to FPGA\n");
		OkInterface::Instance()->WriteToBlockPipeIn(PipeInAddress::sim, 1024, sim_send_bytes_count, new_sim_event_bytes);
		available_space-=sim_send_bytes_count;
		sim_send_bytes_count = 0;
		while(sim_send_bytes_count<1024 & sim_input_file_size>0)
		{
			// read until there are 1024 bytes, or the file has ended
			read_sim_packet();
			if(sim_packet_info.num_events >0)
				encode_sim_packet();
		}
	}

	if(sim_input_file_size == 0)
	{
		//	printf("End of simulation input file reached\n");
		if(available_space>=1024)
		{
			//	printf("space available, attempting to send a flush packet\n");
			sim_packet_info.num_events = 0;
			encode_sim_packet();
			OkInterface::Instance()->WriteToBlockPipeIn(PipeInAddress::sim, 1024, sim_send_bytes_count, new_sim_event_bytes);
		}
	}


	return 0;
}

int AtisSource::read_sim_packet()
{
	if(sim_input_file_size>0)
	{
		//printf("reading more sim data\n");
		sim_input_file.read((char*)&sim_packet_info, sizeof(packet_info_type));
		sim_input_file_size -= sizeof(packet_info_type);
		//	printf("read in packet info with %i events\n", sim_packet_info.num_events);

		//sim_packet = (event*)malloc(sim_packet_info.num_events*sizeof(event));
		sim_packet = new event[sim_packet_info.num_events];
		sim_input_file.read((char*)sim_packet, sim_packet_info.num_events*sizeof(event));
		//	printf("read in event packet\n");
		sim_input_file_size -= sim_packet_info.num_events*sizeof(event);
	}else
	{
		sim_packet_info.num_events = 0;
		printf("End of simulation input file reached\n");
	}
	if(first_packet==true)
	{
		first_packet = false;	
		simulation_start_time_offset = sim_packet_info.time_start;
	}

	sim_packet_info.time_start-=simulation_start_time_offset;

	//printf("read packet completed\n");
	return 0;
}

int AtisSource::encode_sim_packet()
{

	uint64_t new_sim_byte_count = old_sim_byte_count;
	uint64_t max_num_bytes = old_sim_byte_count + 8*sim_packet_info.num_events + 8*(sim_packet_info.time_start -sim_time) + 1024;

	new_sim_event_bytes = new unsigned char[max_num_bytes];
	new_sim_byte_count = old_sim_byte_count;

	if(old_sim_byte_count>0)
		memcpy(new_sim_event_bytes, old_sim_event_bytes, old_sim_byte_count); //copy the old events to the start of the byte array


	for(int event_num=0; event_num<sim_packet_info.num_events; event_num++)
	{
		while(sim_packet_info.time_start > sim_time)
		{
			//		printf("inserting new overflow event during event packet because event_time is %i and sim_time is %lu\n", int32_t(sim_packet[event_num].t + (sim_packet_info.time_start<<16)), sim_time);
			// insert an overflow event
			new_sim_event_bytes[new_sim_byte_count++] = 0; //subtype
			new_sim_event_bytes[new_sim_byte_count++] = evt_type::timer_overflow; //type
			new_sim_event_bytes[new_sim_byte_count++] = 255; //y LSB
			new_sim_event_bytes[new_sim_byte_count++] = 255; //y MSB
			new_sim_event_bytes[new_sim_byte_count++] = 255; //x LSB
			new_sim_event_bytes[new_sim_byte_count++] = 255; //x MSB
			new_sim_event_bytes[new_sim_byte_count++] = 255; //time LSB
			new_sim_event_bytes[new_sim_byte_count++] = 255; //time MSB
			sim_time++;
		}

		new_sim_event_bytes[new_sim_byte_count++] = sim_packet[event_num].subtype; //subtype
		new_sim_event_bytes[new_sim_byte_count++] = sim_packet[event_num].type; //type
		new_sim_event_bytes[new_sim_byte_count++] = sim_packet[event_num].y & 0xFF; //y LSB
		new_sim_event_bytes[new_sim_byte_count++] = (sim_packet[event_num].y>>8) & 0xFF; //y MSB
		new_sim_event_bytes[new_sim_byte_count++] = sim_packet[event_num].x & 0xFF; //x LSB
		new_sim_event_bytes[new_sim_byte_count++] = (sim_packet[event_num].x>>8) & 0xFF; //x MSB
		new_sim_event_bytes[new_sim_byte_count++] = sim_packet[event_num].t & 0xFF; //time LSB
		new_sim_event_bytes[new_sim_byte_count++] = (sim_packet[event_num].t>>8) & 0xFF; //time MSB
	}


	//if the end of the file has been reached, add overflow events to the buffer to push the last events through
	sim_send_bytes_count = ((new_sim_byte_count)>>10)<<10; // how many bytes will be sent? Must be a multiple 1024
	if(sim_input_file_size ==0) 
	{
		//printf("constructing flush packet\n");
		while(new_sim_byte_count < sim_send_bytes_count+1020) //send 8+ seconds of counter overflows to flush fifos.
		{
			new_sim_event_bytes[new_sim_byte_count++] = 0; //subtype
			new_sim_event_bytes[new_sim_byte_count++] = evt_type::quit; //type
			new_sim_event_bytes[new_sim_byte_count++] = 255; //y LSB
			new_sim_event_bytes[new_sim_byte_count++] = 255; //y MSB
			new_sim_event_bytes[new_sim_byte_count++] = 255; //x LSB
			new_sim_event_bytes[new_sim_byte_count++] = 255; //x MSB
			new_sim_event_bytes[new_sim_byte_count++] = 255; //time LSB
			new_sim_event_bytes[new_sim_byte_count++] = 255; //time MSB
		}
	}

	//printf("packet end time is %i (%i + %i), sim_time is %lu\n", (sim_packet_info.time_start<<16)+sim_packet[sim_packet_info.num_events-1].t, (sim_packet_info.time_start<<16), sim_packet[sim_packet_info.num_events-1].t, sim_time);
	sim_send_bytes_count = ((new_sim_byte_count)>>10)<<10; // how many bytes will be sent? Must be a multiple 1024

	old_sim_byte_count = new_sim_byte_count - sim_send_bytes_count;
	/*
	printf("new_sim_byte_count is %li\n", new_sim_byte_count);
	printf("sim_send_bytes_count is %li\n", sim_send_bytes_count);
	printf("old_sim_byte_count is %li\n", old_sim_byte_count);
	printf("number of events is %i\n", sim_packet_info.num_events);
	printf("max_num_bytes is %i\n", max_num_bytes);
	 */
	old_sim_event_bytes = new unsigned char[old_sim_byte_count];
	//	printf("allocated memory for old_sim_event_bytes\n");
	memcpy(old_sim_event_bytes, new_sim_event_bytes+sim_send_bytes_count, old_sim_byte_count);
	return 0;
}
