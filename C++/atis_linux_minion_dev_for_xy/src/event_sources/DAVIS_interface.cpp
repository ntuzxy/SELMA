#include <fstream>
#include <iostream>
#include <pthread.h>
#include <DAVIS_interface.h>


	//Currently just uses default settings... could change to take saved configuration from somewhere
	

DavisSource::DavisSource(source_parameters_type &parameters, int davis_caer_type)
: DataSource(parameters.x_dim,parameters.y_dim, parameters.pix)
{
	EnableGrayscale = parameters.useGrayscale;
	EnableIMU = parameters.useIMU;
	//*comments = (char*)malloc(100*sizeof(char)); //dummy initialization of comments so that freeing the memory for comments does not cause a segmentation fault in the main function
	printf("Initializing DAVIS\n");
    DAVIS_mutex = PTHREAD_MUTEX_INITIALIZER;
	//decode_overflow_counter = 0;
	if (parameters.autodetect_serial_number == true)
	{
		// Open the default DAVIS
		// Open a DAVIS, give it a device ID of 1, and don't care about USB bus or SN restrictions.
		davis_handle = caerDeviceOpen(1, davis_caer_type, 0, 0, NULL);
	}
	else
	{
		// Open the DAVIS with a specific serial number
		std::cout << "Opening with SN = " << ("840100" + std::to_string(parameters.camera_serial_number)) << "...\n";
		davis_handle = caerDeviceOpen(1, davis_caer_type, 0, 0, ("840100" + std::to_string(parameters.camera_serial_number)).c_str());
	}
	if (davis_handle == NULL) {
		printf("ERROR: Cannot open device! \n");
	}
	// Send the default configuration before using the device.
	// No configuration is sent automatically!
	caerDeviceSendDefaultConfig(davis_handle);

	//program_davis_biases(parameters.biasfile, parameters.davis_overwrite_default_biases);
	// Now let's get start getting some data from the device. We just loop, no notification needed.
	
}
void DavisSource::ConfigureDavis()
{
	caerDeviceDataStart(davis_handle, NULL, NULL, NULL, NULL, NULL);
	// Let's turn on blocking data-get mode to avoid wasting resources.
	caerDeviceConfigSet(davis_handle, CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

	// Disable the greyscale events if they are not needed, save some hundred thousands events per second.
	caerDeviceConfigSet(davis_handle, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, EnableGrayscale);

	// Disable the IMU events if they are not needed, reduce the processing by a bit
	caerDeviceConfigSet(davis_handle, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN, EnableIMU);

	// Send reset signal to sync devices for stereo matching
	caerDeviceConfigSet(davis_handle, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, 1);
	// Wait, not really sure it is needed
	for (int i=0; i<100000; i++);

	caerDeviceConfigSet(davis_handle, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, 0);
		
	packetContainer = NULL;
	get_new_data = true;
}

Davis240CSource::Davis240CSource(source_parameters_type &parameters) :
DavisSource(parameters, CAER_DEVICE_DAVIS_FX2)
{
	parameters.comments += "#DAVIS 240C sensor\n";
	ProgramBiases(parameters.biasfile, parameters.davis_overwrite_default_biases);
	ConfigureDavis();
}

Davis640Source::Davis640Source(source_parameters_type &parameters) :
DavisSource(parameters, CAER_DEVICE_DAVIS_FX3)
{
	parameters.comments += "#DAVIS 640 sensor\n";
	ProgramBiases(parameters.biasfile, parameters.davis_overwrite_default_biases);
	ConfigureDavis();
}

void Davis240CSource::ProgramBiases(const std::string biasfile_ctrl_path, bool overwrite_biases)
{
	if(overwrite_biases)
	{
		printf("Bias file for DAVIS specified as: %s\n", biasfile_ctrl_path.c_str());
			std::ifstream infile(biasfile_ctrl_path.c_str());
		std::string line;
		for(int biasNumber=0; biasNumber<20; biasNumber++)
		{	
			getline(infile, line);
   			std::istringstream iss(line);
   			struct caer_bias_coarsefine coarseFineBias;
   			int coarseValue, fineValue, enabled, sexN, typeNormal, currentLevelNormal;
   			std::string bias_name;
   			std::string dummy;
   			iss >> dummy >> coarseValue >> dummy >> fineValue >> dummy >> enabled >> dummy >> sexN >> dummy >> typeNormal >> dummy >> currentLevelNormal >> bias_name;
			coarseFineBias.coarseValue = coarseValue;
			coarseFineBias.fineValue = fineValue;
			coarseFineBias.enabled = enabled;
			coarseFineBias.sexN = sexN;
			coarseFineBias.typeNormal = typeNormal;
			coarseFineBias.currentLevelNormal = currentLevelNormal;
			printf("%s is: coarseValue %i, fineValue %i, enabled %i, sexN %i, typeNormal %i, currentLevelNormal %i\n", bias_name.c_str(), coarseFineBias.coarseValue, coarseFineBias.fineValue, coarseFineBias.enabled, coarseFineBias.sexN, coarseFineBias.typeNormal, coarseFineBias.currentLevelNormal);
			caerDeviceConfigSet(davis_handle, DAVIS_CONFIG_BIAS, biasNumber, caerBiasCoarseFineGenerate(coarseFineBias));
		}
		for(int biasNumber=20; biasNumber<22; biasNumber++)
		{	
			getline(infile, line);
   			std::istringstream iss(line);
   			caer_bias_shiftedsource shiftedsourcebias;
   			uint16_t shiftedsourceval;
   			std::string bias_name;
   			std::string dummy;
   			iss >> dummy >> shiftedsourceval >> dummy >> dummy >> dummy >> dummy >> dummy >> dummy >> dummy >> dummy >>bias_name;
			shiftedsourcebias = caerBiasShiftedSourceParse(shiftedsourceval);
			printf("%s is: shiftedsourceval %i refValue: %i regValue: %i operatingMode: %i voltageLevel: %i\n", bias_name.c_str(), shiftedsourceval, shiftedsourcebias.refValue, shiftedsourcebias.regValue, shiftedsourcebias.operatingMode, shiftedsourcebias.voltageLevel);
			caerDeviceConfigSet(davis_handle, DAVIS_CONFIG_BIAS, biasNumber, shiftedsourceval);
		}
	}
	else
	{
		printf("No bias file for DAVIS specified\n");
		std::ofstream out("write_biases_out.txt");
		struct caer_bias_coarsefine coarseFineBias;
		uint32_t biasval;
		char bias_string[1000];
		for(int biasNumber=0; biasNumber<20; biasNumber++)
		{	
			caerDeviceConfigGet(davis_handle, DAVIS_CONFIG_BIAS, biasNumber, &biasval);
			coarseFineBias = caerBiasCoarseFineParse(biasval);
			sprintf(bias_string, "coarse: %i fine: %i enabled: %i sexN: %i typeNormal: %i currentLevelNormal: %i biasnumber_%i\n", coarseFineBias.coarseValue, coarseFineBias.fineValue,coarseFineBias.enabled,coarseFineBias.sexN,coarseFineBias.typeNormal,coarseFineBias.currentLevelNormal, biasNumber);
			out << bias_string;
		}
		struct caer_bias_shiftedsource shiftedsourcebias;
		for(int biasNumber=20; biasNumber<22; biasNumber++)
		{	
			caerDeviceConfigGet(davis_handle, DAVIS_CONFIG_BIAS, biasNumber, &biasval);
			shiftedsourcebias = caerBiasShiftedSourceParse(biasval);
			sprintf(bias_string, "intval: %i refValue: %i regValue: %i operatingMode: %i voltageLevel: %i biasnumber_%i\n", biasval, shiftedsourcebias.refValue, shiftedsourcebias.regValue, shiftedsourcebias.operatingMode, shiftedsourcebias.voltageLevel, biasNumber);
			out << bias_string;
		}
		out.close();
	}
}

void Davis640Source::ProgramBiases(const std::string biasfile_ctrl_path, bool overwrite_biases)
{
	if(overwrite_biases)
	{
		printf("Bias file for DAVIS specified as: %s\n", biasfile_ctrl_path.c_str());
		std::ifstream infile(biasfile_ctrl_path.c_str());
		std::string line;
		while(!infile.eof())
		{
			getline(infile, line);
			std::istringstream iss(line);
			std::string dummy;
			std::string bias_name;
			int bias_type, biasnumber;
			iss >> dummy >> biasnumber >> dummy >> bias_type;
			switch(bias_type)
			{
				case 0:
				{
					caer_bias_vdac vdacBias;
					uint16_t voltage, current;
					iss>> dummy >> current >> dummy >> voltage >> dummy >> bias_name;
					vdacBias.voltageValue  = voltage;
					vdacBias.currentValue  = current;
					printf("Bias %i: %s is: voltage %i, current %i\n",biasnumber, bias_name.c_str(), vdacBias.voltageValue, vdacBias.currentValue);
					caerDeviceConfigSet(davis_handle, DAVIS_CONFIG_BIAS, biasnumber, caerBiasVDACGenerate(vdacBias));
					break;
				}
				case 1:
				{
					caer_bias_coarsefine coarseFineBias;
					int coarseValue, fineValue, enabled, sexN, typeNormal, currentLevelNormal;
					iss>> dummy >> coarseValue >> dummy >> fineValue >> dummy >> enabled >> dummy >> sexN >> dummy >> typeNormal >> dummy >> currentLevelNormal >> dummy >> bias_name;
					coarseFineBias.coarseValue = coarseValue;
					coarseFineBias.fineValue = fineValue;
					coarseFineBias.enabled = enabled;
					coarseFineBias.sexN = sexN;
					coarseFineBias.typeNormal = typeNormal;
					coarseFineBias.currentLevelNormal = currentLevelNormal;
					printf("Bias %i: %s is: coarseValue %i, fineValue %i, enabled %i, sexN %i, typeNormal %i, currentLevelNormal %i\n", biasnumber, bias_name.c_str(), coarseFineBias.coarseValue, coarseFineBias.fineValue, coarseFineBias.enabled, coarseFineBias.sexN, coarseFineBias.typeNormal, coarseFineBias.currentLevelNormal);
					caerDeviceConfigSet(davis_handle, DAVIS_CONFIG_BIAS, biasnumber, caerBiasCoarseFineGenerate(coarseFineBias));
					break;
				}
				case 2:
				{
					caer_bias_shiftedsource shiftedsourcebias;
					std::string operating_mode;
					std::string voltage_level;
					uint16_t refValue, regValue;
					iss >> dummy >> operating_mode >> dummy >> refValue >> dummy >> regValue >> dummy >> voltage_level >> dummy >>bias_name;
					shiftedsourcebias.refValue = refValue;
					shiftedsourcebias.regValue = regValue;
					if(operating_mode.compare("ShiftedSource")==0) //0 means equal
						shiftedsourcebias.operatingMode = caer_bias_shiftedsource_operating_mode::SHIFTED_SOURCE;
					else
						printf("ERROR: bias operating_mode not recognized\n");
					if(voltage_level.compare("SplitGate")==0) //0 means equal
						shiftedsourcebias.voltageLevel = caer_bias_shiftedsource_voltage_level::SPLIT_GATE;
					else
						printf("ERROR: bias voltage_level not recognized\n");
					printf("Bias %i: %s is: refValue: %i regValue: %i operatingMode: %s voltageLevel: %s\n", biasnumber, bias_name.c_str(), shiftedsourcebias.refValue, shiftedsourcebias.regValue, operating_mode.c_str(), voltage_level.c_str());
					caerDeviceConfigSet(davis_handle, DAVIS_CONFIG_BIAS, biasnumber, caerBiasShiftedSourceGenerate(shiftedsourcebias));
					break;
				}
			}
		}
	}
	else
	{
		printf("No bias file for DAVIS specified\n");
		std::ofstream out("write_biases_out.txt");
		struct caer_bias_coarsefine coarseFineBias;
		uint32_t biasval;
		char bias_string[1000];
		for(int biasNumber=0; biasNumber<35; biasNumber++)
		{	
			caerDeviceConfigGet(davis_handle, DAVIS_CONFIG_BIAS, biasNumber, &biasval);
			coarseFineBias = caerBiasCoarseFineParse(biasval);
			sprintf(bias_string, "coarse: %i fine: %i enabled: %i sexN: %i typeNormal: %i currentLevelNormal: %i biasnumber_%i\n", coarseFineBias.coarseValue, coarseFineBias.fineValue,coarseFineBias.enabled,coarseFineBias.sexN,coarseFineBias.typeNormal,coarseFineBias.currentLevelNormal, biasNumber);
			out << bias_string;
		}
		struct caer_bias_shiftedsource shiftedsourcebias;
		for(int biasNumber=35; biasNumber<37; biasNumber++)
		{	
			caerDeviceConfigGet(davis_handle, DAVIS_CONFIG_BIAS, biasNumber, &biasval);
			shiftedsourcebias = caerBiasShiftedSourceParse(biasval);
			sprintf(bias_string, "intval: %i refValue: %i regValue: %i operatingMode: %i voltageLevel: %i biasnumber_%i\n", biasval, shiftedsourcebias.refValue, shiftedsourcebias.regValue, shiftedsourcebias.operatingMode, shiftedsourcebias.voltageLevel, biasNumber);
			out << bias_string;
		}
		out.close();
	}
}

static int numgetDAVISdata = 0;
bool DavisSource::getData(packet_info_type* packet_info, event* dataOUT, bool exit_thread)
{
	static int32_t packet_iterator;
	static int32_t num_packets; //how many packets in the container?
	static int32_t event_iterator; //which event are we currently processing?

	packet_info[0].packet_type = packet_type::unassigned_packet;
	if(get_new_data == true)
	{
		while (packetContainer == NULL)
		{
			packetContainer = caerDeviceDataGet(davis_handle); // keep going until a packet is received
			//printf("Waiting for DAVIS packet \n");		
		}
		num_packets = caerEventPacketContainerGetEventPacketsNumber(packetContainer); // how many packets are in the container?
		packet_iterator = 0;
		event_iterator = 0;
		get_new_data = false;
	}
	
	//printf("\nGot event container with %d packets (allocated).\n", num_packets);
	int total_num_events = 0;
	//for (int32_t i = 0; i < num_packets; i++) 
	while(packet_iterator<num_packets)
	{
		//printf("processing packet %i of %i from data group %i\n", packet_iterator, num_packets, numgetDAVISdata++);
		caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainer, packet_iterator); //extract the packets from the container
		if (packetHeader == NULL) 
		{
			packet_iterator++; //move onto the next packet
			event_iterator = 0;
		//	printf("Packet %d is empty (not present).\n", packet_iterator);
		}
		else if (caerEventPacketHeaderGetEventType(packetHeader) == POLARITY_EVENT)
		{
			caerPolarityEventPacket polarity = (caerPolarityEventPacket)packetHeader;
			//printf("Will process %d events from this packet \n", caerEventPacketHeaderGetEventNumber(packetHeader));

			//MUST ONLY DO THIS FOR THE FIRST PACKET
			if(packet_info[0].packet_type == packet_type::unassigned_packet)
			{
				packet_info[0].packet_type = packet_type::TD_packet;
				caerPolarityEvent firstEvent = caerPolarityEventPacketGetEvent(polarity, event_iterator);
				packet_info[0].time_start = uint32_t(caerPolarityEventGetTimestamp64(firstEvent, polarity) >> 16);
				//packet_info[0].time_start = uint32_t(caerPolarityEventGetTimestamp(firstEvent) >> 16);
		//		printf("initialized packet start time\n");
			}
			
			if(packet_info[0].packet_type == packet_type::TD_packet)
			{
				//for( int event_num = 0; event_num<caerEventPacketHeaderGetEventNumber(packetHeader); event_num++)
				while(event_iterator<caerEventPacketHeaderGetEventNumber(packetHeader))
				{
					// Get full timestamp and addresses of first event.
					caerPolarityEvent thisEvent = caerPolarityEventPacketGetEvent(polarity, event_iterator);
					if (packet_info[0].time_start != uint32_t(caerPolarityEventGetTimestamp64(thisEvent, polarity) >> 16))
					{
						get_new_data = false;
						packet_info[0].num_events = total_num_events;
			//			printf("timer overflow caused packet splitting\n");
						return 0;
					}
					
					dataOUT[total_num_events].x 	= caerPolarityEventGetX(thisEvent);
					//dataOUT[total_num_events].y 	= (unsigned char) (caerPolarityEventGetY(thisEvent));
					dataOUT[total_num_events].y 	= caerPolarityEventGetY(thisEvent);
					dataOUT[total_num_events].type 	= evt_type::TD;
					dataOUT[total_num_events].t 	= uint16_t(caerPolarityEventGetTimestamp(thisEvent) & 0x0000FFFF);
					if(caerPolarityEventGetPolarity(thisEvent) == 1)
						dataOUT[total_num_events].subtype = TD_subtype::ON;
					else
						dataOUT[total_num_events].subtype = TD_subtype::OFF;
					total_num_events++;
					event_iterator++;
					
					//printf("Event - ts: %d, x: %d, y: %d, pol: %d.\n", dataOUT[total_num_events].t, dataOUT[total_num_events].x, dataOUT[total_num_events].y, dataOUT[total_num_events].subtype);

				}
				packet_iterator++;
				event_iterator = 0;
				//		printf("Processed packet %d \n", i);
			}else
			{
			//	printf("A separate event type caused TD packet splitting\n");
				get_new_data = false;
				packet_info[0].num_events = total_num_events;
				return 0;
			}
		}
		else if (caerEventPacketHeaderGetEventType(packetHeader) == SPECIAL_EVENT)
		{
			if(packet_info[0].packet_type == packet_type::unassigned_packet)
			{
				packet_info[0].packet_type = packet_type::DAVIS_SPECIAL_packet;
				caerSpecialEventPacket cpe = (caerSpecialEventPacket) packetHeader;
				caerSpecialEvent ev = caerSpecialEventPacketGetEvent(cpe, 0);
				packet_iterator++;
				event_iterator = 0;
				//somewhere here assign which type of event as a subtype of the packet type

				/*
				int ev_code = caerSpecialEventGetType(ev);
				for (int i=0; i<num_packets; i++) 
				{
					if (ev_code == TIMESTAMP_RESET)
					{
						// We received a timestamp reset event for stereo camera synchronization. Signal to reset the time.
						// To implement, this will send a reset event to all the modules
						/*dataOUT[total_num_events].x 		= 0;
						dataOUT[total_num_events].y 		= 0;
						dataOUT[total_num_events].type 		= evt_type::Reset;
						dataOUT[total_num_events].t 		= 0;
						dataOUT[total_num_events].subtype 	= 0;
						total_num_events++;
						// Equivalent to set to zero, but just to be on the safe side
						packet_info[0].time_start = 0;
					}
				}*/
			}else
			{
				get_new_data = false;
				packet_info[0].num_events = total_num_events;
			//	printf("A separate event type caused special packet splitting\n");
				return 0;
			}
		}
		else if (EnableGrayscale && caerEventPacketHeaderGetEventType(packetHeader) == FRAME_EVENT)
		{
			if(packet_info[0].packet_type == packet_type::unassigned_packet)
			{
				packet_info[0].packet_type = packet_type::FRAME_packet;

				caerFrameEventPacket frame = (caerFrameEventPacket) packetHeader;
				caerFrameEvent event = caerFrameEventPacketGetEvent(frame, 0);

				packet_info[0].time_start = uint32_t(caerFrameEventGetTimestamp64(event, frame)>>16);

				uint16_t* image = caerFrameEventGetPixelArrayUnsafe(event);

				const int32_t frame_width = caerFrameEventGetLengthX(event);
          		const int32_t frame_height= caerFrameEventGetLengthY(event);

				for (int img_y=0; img_y<frame_height; img_y++)
				{
					for (int img_x=0; img_x<frame_width; img_x++)
					{
						dataOUT[total_num_events].x 	= img_x;
						dataOUT[total_num_events].y 	= img_y;
						dataOUT[total_num_events].type 	= evt_type::FE;
						dataOUT[total_num_events].t 	= 0;
						dataOUT[total_num_events].subtype = (image[img_y*frame_width + img_x] >> 8);

						total_num_events++;
					}
				}
				packet_iterator++;
				event_iterator = 0;
			}else
			{
				get_new_data = false;
				packet_info[0].num_events = total_num_events;
			//	printf("A separate event type caused frame packet splitting\n");
				return 0;
			}
		}
		else if (caerEventPacketHeaderGetEventType(packetHeader) == IMU6_EVENT)
		{
			//printf("Got IMU event\n");
			if(packet_info[0].packet_type == packet_type::unassigned_packet)
			{
				packet_info[0].packet_type = packet_type::IMU_packet;
				caerIMU6EventPacket packet = (caerIMU6EventPacket) packetHeader;
				// Fill the time start
				packet_info[0].time_start = uint32_t(caerIMU6EventGetTimestamp64(caerIMU6EventPacketGetEvent(packet,0), packet)>>16);
				// Fill with events including the IMU6 data
				GetIMU6Data(caerIMU6EventPacketGetEvent(packet,0), packet_info, dataOUT, total_num_events);
				packet_iterator++;
				event_iterator = 0;
			}
			else
			{
				get_new_data = false;
				packet_info[0].num_events = total_num_events;
			//	printf("A separate event type caused special2 packet splitting\n");
				return 0;	
			}

		}else
			packet_iterator++;

	//	printf("Finished with packet %d \n", i);
	}
	caerEventPacketContainerFree(packetContainer);
//		printf("Freed container \n");

    if(exit_thread == 1)
    {
		printf("Shutting down DAVIS\n"); 
		caerDeviceDataStop(davis_handle);
		caerDeviceClose(&davis_handle);
		return 1;	
    }

	packet_info[0].num_events =  total_num_events;
	get_new_data = true;
	packetContainer = NULL;
	//printf("finished with call number %i to getDAVISdata \n", numgetDAVISdata++);
	return 0;
}

void DavisSource::GetIMU6Data(caerIMU6Event thisEvent, packet_info_type* packet_info, event* dataOUT, int& total_num_events)
{
	// Fill up the packet with the 6 values from the IMU
	// Start with accel_x
	float read_value;
	uint16_t time = uint16_t(caerIMU6EventGetTimestamp(thisEvent) & 0x0000FFFF);
	read_value = caerIMU6EventGetAccelX(thisEvent);
	PushIMU6Event(dataOUT, IMU_type::accel_x, read_value, time, total_num_events);
	// Repeat for all the coordinates, admittedly a bit of code repetition here
	// accel_y
	read_value = caerIMU6EventGetAccelY(thisEvent);
	PushIMU6Event(dataOUT, IMU_type::accel_y, read_value, time, total_num_events);
	// accel_z
	read_value = caerIMU6EventGetAccelZ(thisEvent);
	PushIMU6Event(dataOUT, IMU_type::accel_z, read_value, time, total_num_events);
	// gyro_x
	read_value = caerIMU6EventGetGyroX(thisEvent);
	PushIMU6Event(dataOUT, IMU_type::gyro_x, read_value, time, total_num_events);
	// gyro_y
	read_value = caerIMU6EventGetGyroY(thisEvent);
	PushIMU6Event(dataOUT, IMU_type::gyro_y, read_value, time, total_num_events);
	// gyro_z
	read_value = caerIMU6EventGetGyroY(thisEvent);
	PushIMU6Event(dataOUT, IMU_type::gyro_z, read_value, time, total_num_events);
	// temp
	read_value = caerIMU6EventGetTemp(thisEvent);
	PushIMU6Event(dataOUT, IMU_type::temp, read_value, time, total_num_events);
}

void DavisSource::PushIMU6Event(event* dataOUT, IMU_type subtype, float read_value, uint16_t time, int& total_num_events)
{
	// Fill up the character array to store a float into two uint16_t
	unsigned char data[4];
	memcpy(data, &read_value, sizeof read_value);
	dataOUT[total_num_events].type 	= evt_type::IMU6;
	// The subtype will state which of the 6 axes we are sending
	dataOUT[total_num_events].subtype = subtype;
	// In this case we have a 32 bits floating point. the y will hold the 16 MSBs and the x will hold the 16 LSBs
	dataOUT[total_num_events].y 	= uint16_t(data[3] << 8 | data[2]);
	dataOUT[total_num_events].x 	= uint16_t(data[1] << 8 | data[0]);
	dataOUT[total_num_events].t 	= time;
	total_num_events++;
}
