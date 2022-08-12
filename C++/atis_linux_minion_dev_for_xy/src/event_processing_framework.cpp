#include <stdio.h>
#include <iostream>
#include <dlfcn.h>
#include <okFrontPanelDLL.h>
#include <pthread.h>
#include <ATIS_interface.h>
#ifdef _useDAVIS
#include <DAVIS_interface.h>
#endif

#include "../inc/TN_classifier.h"
#ifdef _useSELMA
#include "SELMA_classifier.h"
#include "SELMA_IF.h"
#endif
#include <FILE_interface.h>
#include <AER_display.h>
#include <filters.h>
#include <event_framework.h>
#include <tracking.h>
#include <thread_launcher.h>
#include <unistd.h>
#include <file_writing.h>
/*#include <orientation.h>
 #include <EKF_localization.h>
 #include <optical_flow.h>
 #include <corners.h>
 */
#ifdef _useROS
#include <ros/package.h>

#include <ros_interface.h>
#endif
//packet_info_type (*function_pointer_source)(event* dataOUT, int mythread); //
/***********************************************************************************
 Main entry point for programme execution. Peform checks on programme input arguments
 and performs the necessary initializations.
 ************************************************************************************/

int DataSource::x_dim = 0;
int DataSource::y_dim = 0;
float DataSource::pix = 0;

int main(int argc, char** argv) {
	printf("Starting Main Event Framework Programme\n");
	int arg_num = 1; //start from argument 1 (because argument 0 is the executable file name)
	//file_write** file_write_pointer; //allow up to 10 files to be written
	//file_write_pointer = new file_write*[10];
	file_write* file_write_pointer[10];
	unsigned int file_write_limit = 0;
	bool non_default_bitfile = false;
	int num_file_outputs = 0;
	//bool file_output = false;

	bool useIMU = false; // For Davis, specifies if the onboard IMU will be used or not

	//by default use ATIS
	source_parameters_type source_parameters;
	source_parameters.source_type = device_code::atis;

	//Set the default locations for the various files required to configure the ATIS camera
#ifdef _useROS
	source_parameters.fpga_bitfile = (ros::package::getPath("nm_epf") + "/atis_bitfiles/v6_lx150.bit"); //default bitfile for FPGA
	source_parameters.biasfile = (ros::package::getPath("nm_epf") + "/bias_config/biastext.txt");//default biases file for ATIS
	std::string sourcefile (ros::package::getPath("nm_epf") + "/recordings/default.bin");//default source file for events
	std::string mask_input (ros::package::getPath("nm_epf") + "/roimask.bin");
	ROSArgumentsParser(argc, argv);
#else
	source_parameters.fpga_bitfile = ("../atis_bitfiles/atis_minion.bit"); //default bitfile for FPGA
	source_parameters.biasfile = ("../bias_config/biastext.txt"); //default biases file for ATIS
	std::string sourcefile("../recordings/default.bin"); //default source file for events
	std::string mask_input("../roimask.bin");
#endif
	std::string output_filename[10]; //allow up to 10 files to be written
	std::string function_name;
	//int fp_num = 1; //function pointer number
	int function_number = 0;
	int thread_number = 1;
	source_parameters.OF = new OF_params;

	// THE SECTION BELOW SELECTS THE EVENT SOURCE
	function_name = std::string(argv[arg_num++]);
	switch (hashit(function_name)) {
	//------------------------------------------- DEVICES ------------------------------------------------------//
	case edavis240C: {

		// ALWAYS SPECIFY THE SERIAL NUMBER OF THE DAVIS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ..at least as of now.. cannot access the right config files otherwise

		source_parameters.source_type = device_code::davis240C;
		source_parameters.x_dim = 240;
		source_parameters.y_dim = 180;
		source_parameters.pix = 18.5e-6;

		if (arg_num == argc || argv[arg_num][0] == '-'
				|| argv[arg_num][0] == '_') {
			printf(
					"No davis serial number specified, will use the first available one\n");
		} else {
			source_parameters.camera_serial_number = atoi(
					std::string(argv[arg_num++]).c_str());
			source_parameters.autodetect_serial_number = false;
			printf("Using davis with serial number = %i\n",
					source_parameters.camera_serial_number);
		}
		break;
	}
	case edavis640: {

		source_parameters.source_type = device_code::davis640;
		source_parameters.x_dim = 640;
		source_parameters.y_dim = 480;
		source_parameters.pix = 18.5e-6;
		if (arg_num == argc || argv[arg_num][0] == '-'
				|| argv[arg_num][0] == '_') {
			printf(
					"No davis640 serial number specified, will use the first available one\n");
		} else {
			source_parameters.camera_serial_number = atoi(
					std::string(argv[arg_num++]).c_str());
			printf("Using davis640 with serial number = %i\n",
					source_parameters.camera_serial_number);
		}
		source_parameters.biasfile = std::string(
				"../bias_config/DAVIS640_default.txt");
		source_parameters.davis_overwrite_default_biases = true;
		break;
	}
	case eATIS: {
		source_parameters.source_type = device_code::atis;
		source_parameters.x_dim = 304;
		source_parameters.y_dim = 240;
		source_parameters.pix = 30e-6;
		break;
	}
	case eIO_read: /////////////***************** THIS SHOULD CALL A FUNCTION RATHER THAN MODIFYING SHARED VARIABLES
	{
		//fromfile = true;
		source_parameters.source_type = device_code::file;
		//	function_pointer_source = &read_from_file;
		source_parameters.input_file = std::string(argv[arg_num++]); //source file to read from
		printf("Will read data from file: %s\n",
				source_parameters.input_file.c_str());
		break;
	}
	case eIO_readparts: {
		//fromfile = true;
		source_parameters.source_type = device_code::fileparts;
		//	function_pointer_source = &read_from_file;
		source_parameters.input_file = std::string(argv[arg_num++]); //source file to read from
		printf("Will read data from file in parts: %s\n",
				source_parameters.input_file.c_str());
		if (arg_num >= argc)
			printf(
					"-fromfileparts argument not given, defaulting to first file/%i\n",
					source_parameters.read_parts_starting_file);
		else if (argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
			printf(
					"-fromfileparts argument not given, defaulting to first file/%i\n",
					source_parameters.read_parts_starting_file);
		else {
			source_parameters.read_parts_starting_file = atof(
					std::string(argv[arg_num++]).c_str());
		}
		break;
	}

#ifdef _useROS
		case eROS_IN:
		{
			printf("ROS Data source implemented\n");
			source_parameters.source_type = device_code::ros_in;
			// We have to increase the size of the data source to allow display of events in world coordinates
			source_parameters.x_dim = 240;
			source_parameters.y_dim = 180;
			source_parameters.pix = 18.5e-6;
			if(arg_num == argc || argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
			{
				// Don't really like how this looks like, but we will reuse the fpga_bitfile variable to store the node name / subscribed topic
				source_parameters.ros_input_topic = "";
				InitRos("nm_epf");
			}
			else
			{
				InitRos(std::string(argv[arg_num]));
				source_parameters.ros_input_topic = std::string(argv[arg_num++]);
			}
			break;
		}
#endif

	default: {
		arg_num--;
		source_parameters.source_type = device_code::atis;
		source_parameters.x_dim = 304;
		source_parameters.y_dim = 240;
		source_parameters.pix = 30e-6;
		break;
	}
	}

	// THE SECTION BELOW SELECTS SETS ANY PARAMETERS OF THE EVENT SOURCE NEEDED BEFORE LAUNCHING THE SOURCE
	int backup_arg_num = arg_num;
	while (arg_num < argc) {
		function_name = std::string(argv[arg_num++]);
		switch (hashit(function_name)) {
		case eIO_realtime: {
			source_parameters.realtime_playback = true;
			source_parameters.realtime_speedup_factor = 1; //multiplier for real-time if acceleration is desired
			if (arg_num >= argc)
				printf(
						"-realtime argument not given, defaulting to real-time/%f\n",
						source_parameters.realtime_speedup_factor);
			else if (argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
				printf(
						"-realtime argument not given, defaulting to real-time/%f\n",
						source_parameters.realtime_speedup_factor);
			else {
				source_parameters.realtime_speedup_factor = atof(
						std::string(argv[arg_num++]).c_str());
			}

			printf("Reading from file will be slowed down to real-time/%i\n",
					source_parameters.realtime_speedup_factor);
			//	force_real_time();
			break;
		}
		case eIO_limit_file_time: {
			file_write_limit = atoi(std::string(argv[arg_num++]).c_str());
			printf("Will limit the write time per file to: %u seconds\n",
					file_write_limit);
			break;
		}

// ------------------------------------------ Config --------------------------------------------------------------------------//
		case eIO_biases: {
			source_parameters.biasfile = std::string(argv[arg_num++]); //biases file for ATIS
			break;
		}
		case eIO_bitfile: {
			non_default_bitfile = true;
			source_parameters.fpga_bitfile = std::string(argv[arg_num++]); //fpga bitfile for ATIS
			printf(
					"Overriding default bitfile with manually specified bitfile: %s\n",
					source_parameters.fpga_bitfile.c_str());
			break;
		}

		case bias18: {
			break;
		}
		case bias33: {
			break;
		}
		case version4: {
			source_parameters.atis_version = 4;
			if (non_default_bitfile == false)
				source_parameters.fpga_bitfile =
						"../atis_bitfiles/v4_lx150.bit";
			printf(
					"Version 4 selected. By default using bitfile for ATIS version 4 and XEM6010lx150.\n");
			printf(
					"If this is incorrect, please manually specify bitfile to use with the -bitfile command.\n");
			break;
		}
		case eSimulate: {
			source_parameters.simulationfile = std::string(argv[arg_num++]); //biases file for ATIS
			source_parameters.simulate = true;
			break;
		}
		case eAPS: {
			//			source_parameters.useGrayscale();
			source_parameters.useGrayscale = true;
			printf("TD to APS coupling enabled.\n");
			break;
		}
		case eIMU: {
			if (source_parameters.source_type != device_code::davis240C) {
				printf("Warning, IMU is only available on DAVIS cameras\n");
			} else {
				printf("Reading of onboard IMU events enabled\n");
				source_parameters.useIMU = true;
			}
			break;
		}
		case eTN: {
			if (source_parameters.source_type != device_code::atis)
				printf(
						"Warning, TrueNorth interface is only available for ATIS\n");
			else {
				printf("Will enable TrueNorth interface\n");
				source_parameters.enableTN = true;
			}

			break;
		}
		case eTime: {
			if (arg_num >= argc) {
				printf(
						"ERROR: -time must be followed by the time in millesconds\n");
				return 0;
			} else if (argv[arg_num][0] == '-') {
				printf(
						"ERROR: -time must be followed by the time in millesconds\n");
				return 0;
			} else {
				int exec_time = atoi(std::string(argv[arg_num++]).c_str()); //get the execution time
				printf("Will run for %i milliseconds\n", exec_time);
				//set_execution_time(exec_time);
			}
			break;
		}
		case eF_NN_fpga: {
			if (source_parameters.source_type != device_code::atis)
				printf(
						"\t\t\tERROR: FPGA filter is only supported with ATIS for now...\n");
			int NN_threshold = (30000) >> 10;
			if (arg_num == argc || argv[arg_num][0] == '-'
					|| argv[arg_num][0] == '_')
				printf(
						"No argument given for FPGA Nearest Neighbour filter, resorting to default threshold of %i microseconds\n",
						NN_threshold << 10);
			else
				NN_threshold = atoi(std::string(argv[arg_num++]).c_str()) >> 10;
			source_parameters.F_NN_enable = true;
			if (NN_threshold > 255) {
				printf(
						"Filter period set to %i microseconds (max limit for F_NN in FPGA)\n",
						255 << 10);
				NN_threshold = 255;
			}
			printf(
					"FPGA Nearest Neighbour filter will be implemented with threshold of %i microseconds\n",
					NN_threshold << 10);
			source_parameters.F_NN_time = NN_threshold;
			break;
		}
		case eF_RF_fpga: {
			if (source_parameters.source_type != device_code::atis)
				printf(
						"\t\t\tERROR: FPGA refractory period is only supported with ATIS for now...\n");
			int RF_period = 5000 >> 10;
			if (arg_num == argc || argv[arg_num][0] == '-'
					|| argv[arg_num][0] == '_')
				printf(
						"No argument given for FPGA Refractory Period, resorting to default threshold of %i microseconds\n",
						RF_period << 10);
			else
				RF_period = atoi(std::string(argv[arg_num++]).c_str()) >> 10;
			if (RF_period > 255) {
				printf(
						"Refractory period set to %i microseconds (max limit for F_RF in FPGA)\n",
						255 << 10);
				RF_period = 255;
			}
			printf(
					"Refractory Period filter will be implemented with threshold of %i microseconds\n",
					RF_period << 10);
			source_parameters.F_RF_enable = true;
			source_parameters.F_RF_time = RF_period;
			break;
		}
			/*case eOF_fpga:
			 {
			 printf("Will run optical flow in FPGA\n");
			 source_parameters.OF->refrac_threshold = 50;
			 source_parameters.OF->old_pixel_threshold = 200;
			 source_parameters.OF->fit_distance_threshold = 4;
			 //source_parameters.OF->goodness_threshold = 200;
			 source_parameters.OF->goodness_threshold = 150;
			 source_parameters.OF->num_pixels_threshold = 6;
			 source_parameters.OF->enable_OF = true;
			 source_parameters.OF->filter_by_goodness = true;
			 source_parameters.OF->cartesian_not_polar = true;

			 //DataConsumer_pointer[thread_number][function_number++] = new display_optical_flow();
			 break;
			 }*/
		case eF_remove_fpga: {
			printf("FPGA will remove filtered events from stream\n");
			source_parameters.F_remove = true;
			break;
		}
		case e_ms_tick: {
			printf("FPGA will insert 1ms tick events\n");
			source_parameters.ms_tick_enable = true;
			break;
		}

		default: {
			break;
		}
		}
	}

	// THE SECTION BELOW LAUNCHES THE SOURCE
	thread_launcher::init_source(source_parameters);

	// THE SECTION BELOW MODIFIES THE SOURCE IF NECESSARY
	arg_num = backup_arg_num;
	while (arg_num < argc) {
		function_name = std::string(argv[arg_num++]);
		switch (hashit(function_name)) {
		case eForceSize: {
			if (arg_num >= argc - 1) {
				printf(
						"ERROR: -forcesize must be followed by an x-size and y-size\n");
				return 0;
			} else if (argv[arg_num][0] == '-') {
				printf(
						"ERROR: -forcesize must be followed by an x-size and y-size\n");
				return 0;
			} else {
				DataSource::x_dim = atoi(std::string(argv[arg_num++]).c_str());
				DataSource::y_dim = atoi(std::string(argv[arg_num++]).c_str());
				printf("forced size to %i by %i pixels\n", DataSource::x_dim,
						DataSource::y_dim);
			}
			if (arg_num >= argc)
				printf(
						"forcsize Will leave pixel size untouched (no pixel size argument passed)\n");
			else if (argv[arg_num][0] == '-')
				printf(
						"forcsize Will leave pixel size untouched (no pixel size argument passed)\n");
			else {
				DataSource::pix = atof(std::string(argv[arg_num++]).c_str());
				printf("forced pixel size to %f \n", DataSource::pix);
			}
			break;
		}
		default:
			break;
		}
	}

	// THE SECTION BELOW SETS UP THE PROCESSING CHAIN
	arg_num = backup_arg_num;
	while (arg_num < argc) {
		function_name = std::string(argv[arg_num++]);
		switch (hashit(function_name)) {
//------------------------------------------- Threading ------------------------------------------------------//
		case eNewThread: {
//					function_pointer[thread_number][function_number] 
			DataConsumer_pointer[thread_number][function_number] = NULL;
			function_number = 0;
			thread_number++;
			break;
		}
// ------------------------------------------  LOCALIZATION -------------------------------------------------//
			/*		case eEKFloc:
			 {
			 std::string EKF_input_filename ("");
			 std::string EKF_output_filename ("");
			 bool write_file_EKF = false;
			 float depth;
			 float focal_length;
			 if(arg_num >= argc || argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
			 {
			 printf("ERROR depth must be specified as first argument for -EKF\n");
			 return 0;
			 }else
			 {
			 depth = atof(std::string(argv[arg_num++]).c_str()); //initial depth estimate for the scene
			 if(arg_num >= argc || argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
			 {
			 printf("ERROR lens focal length must be specified as second argument for -EKF\n");
			 return 0;
			 }else
			 {
			 focal_length = atof(std::string(argv[arg_num++]).c_str()); //initial depth estimate for the scene
			 if(arg_num >= argc || argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
			 printf("No EKF output file specified, no writing will occur\n");
			 else
			 {
			 EKF_output_filename = argv[arg_num++];
			 write_file_EKF = true;
			 printf("EKF will write output to file \n", EKF_output_filename.c_str());
			 if(arg_num == argc || argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
			 printf("No input file given for EKF, assuming no distortion\n");
			 else
			 {
			 EKF_input_filename = argv[arg_num++];
			 printf("Input file \"%s\"  specified for EKF (not yet implemented)\n", EKF_input_filename.c_str());
			 }
			 }
			 }
			 }

			 printf("EKF currently waits 5 seconds before initialization, then uses the next 10000 events to build a map\n");

			 DataConsumer_pointer[thread_number][function_number++] = new EKF_localization(depth, focal_length, EKF_input_filename.c_str(), write_file_EKF,  EKF_output_filename.c_str());
			 break;
			 }*/
//------------------------------------------- FILTERS ------------------------------------------------------//
		case eF_rangefix: {
			//function_pointer[thread_number][function_number++] = &rangefix;
			DataConsumer_pointer[thread_number][function_number++] =
					new rangecheck();
//						fp_num++;
			printf("Will remove out of range values.\n");
			break;
		}
		case eF_offset: {
			//function_pointer[thread_number][function_number++] = &rangefix;
			uint16_t x_offset, y_offset;
			x_offset = atoi(std::string(argv[arg_num++]).c_str());
			y_offset = atoi(std::string(argv[arg_num++]).c_str());
			DataConsumer_pointer[thread_number][function_number++] = new offset(
					x_offset, y_offset);
//						fp_num++;
			printf("Will offset events by %i %i \n", x_offset, y_offset);
			break;
		}

		case eF_ISI: {

			int ISI_threshold = 110000;
			float ISI_mix = 0.9;
			if (arg_num == argc || argv[arg_num][0] == '-'
					|| argv[arg_num][0] == '_')
				printf(
						"No arguments given for Inter Spike Interval filter, resorting to default threshold of %i microseconds and mixing factor of %f\n",
						ISI_threshold, ISI_mix);
			else {
				ISI_threshold = atoi(std::string(argv[arg_num++]).c_str());
				ISI_mix = atof(std::string(argv[arg_num++]).c_str());
			}
			printf(
					"Inter Spike Interval filter will be implemented with threshold of %i microseconds and mixing factor of %f \n",
					ISI_threshold, ISI_mix);
			DataConsumer_pointer[thread_number][function_number++] =
					new ISI_filter(ISI_threshold, ISI_mix);
			break;
		}
		case eF_NN: {
			int NN_threshold = 30000;
			if (arg_num == argc || argv[arg_num][0] == '-'
					|| argv[arg_num][0] == '_')
				printf(
						"No argument given for Nearest Neighbour filter, resorting to default threshold of %i microseconds\n",
						NN_threshold);
			else
				NN_threshold = atoi(std::string(argv[arg_num++]).c_str());
			printf(
					"Nearest Neighbour filter will be implemented with threshold of %i microseconds\n",
					NN_threshold);
			DataConsumer_pointer[thread_number][function_number++] =
					new NN_filter(NN_threshold);
			break;
		}
		case eF_NNp: {
			int NNp_threshold = 30000;
			if (arg_num == argc || argv[arg_num][0] == '-'
					|| argv[arg_num][0] == '_')
				printf(
						"No argument given for Nearest Neighbour (with polarity) filter, resorting to default threshold of %i microseconds\n",
						NNp_threshold);
			else
				NNp_threshold = atoi(std::string(argv[arg_num++]).c_str());
			printf(
					"Nearest Neighbour (with polarity) filter will be implemented with threshold of %i microseconds\n",
					NNp_threshold);
			DataConsumer_pointer[thread_number][function_number++] =
					new NN_filter_polarity(NNp_threshold);
			break;
		}
		case eF_NN2: {

			int NN2_threshold = 30000;
			if (arg_num == argc || argv[arg_num][0] == '-'
					|| argv[arg_num][0] == '_')
				printf(
						"No argument given for 2 Nearest Neighbour filter, resorting to default threshold of %i microseconds\n",
						NN2_threshold);
			else
				NN2_threshold = atoi(std::string(argv[arg_num++]).c_str());
			printf(
					"2 Nearest Neighbour filter will be implemented with threshold of %i microseconds\n",
					NN2_threshold);
			DataConsumer_pointer[thread_number][function_number++] =
					new NN2_filter(NN2_threshold);
			break;
		}
		case eF_NNhv: {
			int NNhv_threshold = 30000;
			if (arg_num == argc || argv[arg_num][0] == '-'
					|| argv[arg_num][0] == '_')
				printf(
						"No argument given for Nearest Neighbour (horizontal and vertical only) filter, resorting to default threshold of %i microseconds\n",
						NNhv_threshold);
			else
				NNhv_threshold = atoi(std::string(argv[arg_num++]).c_str());
			printf(
					"Nearest Neighbour (horizontal and vertical only) filter will be implemented with threshold of %i microseconds\n",
					NNhv_threshold);
			DataConsumer_pointer[thread_number][function_number++] =
					new NNhv_filter(NNhv_threshold);
			break;
		}
		case eF_RF: {
			int RF_period = 5000;
			if (arg_num == argc || argv[arg_num][0] == '-'
					|| argv[arg_num][0] == '_')
				printf(
						"No argument given for Refractory Period, resorting to default threshold of %i microseconds\n",
						RF_period);
			else
				RF_period = atoi(std::string(argv[arg_num++]).c_str());
			printf(
					"Refractory Period filter will be implemented with threshold of %i microseconds\n",
					RF_period);
			DataConsumer_pointer[thread_number][function_number++] =
					new refractory_filter(RF_period);
			break;
		}
		case eF_RFp: {
			int RFp_period = 5000;
			if (arg_num == argc || argv[arg_num][0] == '-'
					|| argv[arg_num][0] == '_')
				printf(
						"No argument given for Refractory Period with polarity filter, resorting to default threshold of %i microseconds\n",
						RFp_period);
			else
				RFp_period = atoi(std::string(argv[arg_num++]).c_str());
			printf(
					"Refractory Period with polarity filter will be implemented with threshold of %i microseconds\n",
					RFp_period);
			DataConsumer_pointer[thread_number][function_number++] =
					new refractory_period_polarity(RFp_period);
			break;
		}
		case eF_DP: {
			printf("Differing polarity filter will be implemented\n");
			DataConsumer_pointer[thread_number][function_number++] =
					new diff_polarity_filter();
			break;
		}
		case eF_NNg: {
			int NN_side_length = atoi(std::string(argv[arg_num++]).c_str());
			int NN_t_length = atoi(std::string(argv[arg_num++]).c_str());
			int min_num_spikes = atoi(std::string(argv[arg_num++]).c_str());
			DataConsumer_pointer[thread_number][function_number++] =
					new filter_NNg(NN_side_length, NN_t_length, min_num_spikes);
			break;
		}
			/*case eF_ONN:
			 {
			 int ONN_threshold = 30000;
			 if(arg_num == argc || argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
			 printf("No argument given for Orientation Nearest Neighbour filter, resorting to default threshold of %i microseconds\n", ONN_threshold);
			 else
			 ONN_threshold = atoi(std::string(argv[arg_num++]).c_str());
			 printf("Orientation Nearest Neighbour filter will be implemented with threshold of %i microseconds\n", ONN_threshold);
			 DataConsumer_pointer[thread_number][function_number++] = new NN_orientation_filter(ONN_threshold);
			 break;
			 }*/
		case eF_ROI: {
			if (arg_num > argc - 3 || argv[arg_num][0] == '-'
					|| argv[arg_num][0] == '_') {
				printf(
						"No arguments given for ROI filter, IT WILL NOT BE IMPLEMENTED\n");
				function_number--;
				DataConsumer_pointer[thread_number][function_number] = NULL;
			} else {
				int x = atoi(std::string(argv[arg_num++]).c_str());
				int y = atoi(std::string(argv[arg_num++]).c_str());
				int x_size = atoi(std::string(argv[arg_num++]).c_str());
				int y_size = atoi(std::string(argv[arg_num++]).c_str());
				DataConsumer_pointer[thread_number][function_number++] =
						new filter_ROI(x, y, x_size, y_size);
			}
			break;

		}
		case ef_ROImask: {

			if (arg_num == argc || argv[arg_num][0] == '-'
					|| argv[arg_num][0] == '_')
				printf(
						"No argument given for ROI mask filter, will read from default mask file.\n");
			else
				mask_input = std::string(argv[arg_num++]);
			printf(
					"ROI mask filter implemented, mask will be read from file %s\n",
					mask_input.c_str());
			DataConsumer_pointer[thread_number][function_number++] =
					new filter_ROImask(mask_input);
			break;
		}
		case eF_undo: {
			DataConsumer_pointer[thread_number][function_number++] =
					new unfilter();
			printf("Unfiltering selected.\n");
			break;
		}
		case eF_reform: {
			DataConsumer_pointer[thread_number][function_number++] =
					new discard_filtered();
			printf("Will remove filtered values.\n");
			break;
		}
		case eF_TNC: {
			std::string out_ip("");
			int evt_port(6000);
			int tracker_port(6001);
			std::string model_path("");
			std::string in_conn_path("");
			std::string out_conn_path("");

			int radius;
			int num_trackers;
			if (arg_num == argc - 7 || argv[arg_num][0] == '-'
					|| argv[arg_num][0] == '_') {
				printf(
						"Not enough arguments given for TrueNorth Classifier.\n");
			} else {
				out_ip = std::string(argv[arg_num++]);
				evt_port = atoi(std::string(argv[arg_num++]).c_str());
				tracker_port = atoi(std::string(argv[arg_num++]).c_str());
				model_path = std::string(argv[arg_num++]);
				in_conn_path = std::string(argv[arg_num++]);
				out_conn_path = std::string(argv[arg_num++]);
				radius = atoi(std::string(argv[arg_num++]).c_str());
				num_trackers = atoi(std::string(argv[arg_num++]).c_str());
				printf(
						"TrueNorth classifier will be run with\n\tmodel: %s\n\tinput_connector: %s\n\toutput_connector: %s\n\toutput_destination: %s:%d, %d\n\ttracker_radius: %d\n\tnum_trackers: %d\n",
						model_path.c_str(), in_conn_path.c_str(),
						out_conn_path.c_str(), out_ip.c_str(),
						evt_port, tracker_port, radius, num_trackers);
				//DataConsumer_pointer[thread_number][function_number++] = new tn_tcp(model_path, in_conn_path, out_conn_path, width, height, num_channels, out_ip, out_port);
				DataConsumer_pointer[thread_number][function_number++] =
						new TN_classifier(out_ip, evt_port, tracker_port,
								model_path, in_conn_path, out_conn_path, radius, num_trackers);

			}

			break;
		}
//			case eF_TNF:
//				{
//					std::string model_path ("");
//					std::string in_conn_path ("");
//					std::string out_conn_path ("");
//					int width (304);
//					int height (240);
//					int num_channels (1);
//					std::string output_sfcp_path ("");
//					if (arg_num > argc-6 || argv[arg_num][0] == '-' || argv[arg_num][0] == '_') {
//						printf("Not enough arguments given for TrueNorth filter.\n");
//					} else {
//						model_path = std::string(argv[arg_num++]);
//						in_conn_path = std::string(argv[arg_num++]);
//						out_conn_path = std::string(argv[arg_num++]);
//						width = atoi(std::string(argv[arg_num++]).c_str());
//						height = atoi(std::string(argv[arg_num++]).c_str());
//						num_channels = atoi(std::string(argv[arg_num++]).c_str());
//						output_sfcp_path = std::string(argv[arg_num++]);
//						printf("TrueNorth filter will be run with\n\tmodel: %s\n\tinput_connector: %s\n\toutput_connector: %s\n\twidth: %d\n\theight: %d\n\tnum_channels: %d\n\toutput_sfcp: %s\n",
//								model_path.c_str(), in_conn_path.c_str(), out_conn_path.c_str(), width, height, num_channels, output_sfcp_path.c_str());
//						//DataConsumer_pointer[thread_number][function_number++] = new tn_file(model_path, in_conn_path, out_conn_path, width, height, num_channels, output_sfcp_path);
//						DataConsumer_pointer[thread_number][function_number++] = new tn_filter(model_path, in_conn_path, out_conn_path, width, height, num_channels, output_sfcp_path);
//					}
//					break;
//				}
//			case eF_TNT:
//				{
//					std::string model_path ("");
//					std::string in_conn_path ("");
//					std::string out_conn_path ("");
//					std::string out_ip ("");
//					int width (304);
//					int height (240);
//					int num_channels (1);
//					int out_port (5001);
//					if (arg_num == argc - 7 || argv[arg_num][0] == '-' || argv[arg_num][0] == '_') {
//						printf("Not enough arguments given for TrueNorth filter.\n");
//					} else {
//						model_path = std::string(argv[arg_num++]);
//						in_conn_path = std::string(argv[arg_num++]);
//						out_conn_path = std::string(argv[arg_num++]);
//						width = atoi(std::string(argv[arg_num++]).c_str());
//						height = atoi(std::string(argv[arg_num++]).c_str());
//						num_channels = atoi(std::string(argv[arg_num++]).c_str());
//						out_ip = std::string(argv[arg_num++]);
//						out_port = atoi(std::string(argv[arg_num++]).c_str());
//						printf("TrueNorth filter will be run with\n\tmodel: %s\n\tinput_connector: %s\n\toutput_connector: %s\n\twidth: %d\n\theight: %d\n\tnum_channels: %d\n\toutput_destination: %s:%d\n",
//								model_path.c_str(), in_conn_path.c_str(), out_conn_path.c_str(), width, height, num_channels, out_ip.c_str(), out_port);
//						//DataConsumer_pointer[thread_number][function_number++] = new tn_tcp(model_path, in_conn_path, out_conn_path, width, height, num_channels, out_ip, out_port);
//						DataConsumer_pointer[thread_number][function_number++] = new tn_filter(model_path, in_conn_path, out_conn_path, width, height, num_channels, out_ip, out_port);
//					}
//					break;
//				}
		case eF_ETP: {
			std::string ip_address("127.0.0.1");
			int port(5050);
			if (arg_num == argc - 1 || argv[arg_num][0] == '-'
					|| argv[arg_num][0] == '_') {
				printf("Not enough arguments given for Evt TCP Publisher.\n");
			} else {
				ip_address = std::string(argv[arg_num++]);
				port = atoi(std::string(argv[arg_num++]).c_str());
				printf(
						"Event TCP Publisher will be run with\n\tip_address: %s\n\tport: %d\n",
						ip_address.c_str(), port);
				DataConsumer_pointer[thread_number][function_number++] =
						new evt_tcp_publisher(ip_address, port);
			}
			break;
		}
		case echange_type: {
			int from = atoi(std::string(argv[arg_num++]).c_str());
			int to = atoi(std::string(argv[arg_num++]).c_str());
			DataConsumer_pointer[thread_number][function_number++] =
					new change_type(from, to);
			printf("Will change events of type %i to type %i\n", from, to);
			break;
		}
#ifdef _useSELMA
			case eF_SELMAC:
			{
				// printf("SELMA Classifier will be implemented\n");
				std::string ip_address("127.0.0.1");
				int port (6001);
				int evt_port (6002);// Event Sync Stream
				std::cout << "SELMAC arg_num == "+ std::to_string(arg_num)+ " argc ==" + std::to_string(argc)<< std::endl;
				if (arg_num == argc || arg_num == argc - 1 || arg_num == argc - 2|| argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
				{
					printf("Not enough Arguments given for SELMAC to communicate over TCP\n");
					printf("Expected Format is : ./epf ... -F_SELMAC <ip_addr> <track_port> <evt_port> ....\n");
					printf("Reverting to SELMAC without Sending Updates\n");
					DataConsumer_pointer[thread_number][function_number++] = new SELMA_classifier();
				} else
				{
					ip_address = std::string(argv[arg_num++]);
					port = atoi(std::string(argv[arg_num++]).c_str());
					evt_port = atoi(std::string(argv[arg_num++]).c_str());
					printf(
							"SELMAC will publish updates with\n\tip_address: %s\n\tport: %d\n",
							ip_address.c_str(), port);
					DataConsumer_pointer[thread_number][function_number++] =
					new SELMA_classifier(ip_address, port, evt_port);
				}
				break;
			}
			case eF_SELMAC_RUNDATA:
			{
				std::string test_folder = "./";
				std::string csv_filename = "test.csv";

				if (arg_num == argc || arg_num == argc - 1 || argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
				{
					printf("Not enough Arguments given for SELMAC_RUNDATA\n");
					printf("Expected Format is : ./epf ... -F_SELMAC_RUNDATA <test_folder> <test_filename> ....\n");
				} else
				{
					test_folder = std::string(argv[arg_num++]);
					csv_filename = std::string(argv[arg_num++]);
					std::cout << "Running Data from " + test_folder + csv_filename << std::endl;
					DataConsumer_pointer[thread_number][function_number++] =
					new SELMA_classifier(test_folder, csv_filename);
				}
				break;

			}
			case eF_SELMAC_WRITEANNOT:
			{
				std::string output_filename = "output.csv";
				if (arg_num == argc || argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
				{
					printf("Not enough Arguments given for SELMAC_RUNDATA\n");
					printf("Expected Format is : ./epf ... -F_SELMAC_WRITEANNOT <output> ....\n");
				} else
				{
					output_filename = std::string(argv[arg_num++]);
					std::cout << "Writing Annotations To " + output_filename << std::endl;
					DataConsumer_pointer[thread_number][function_number++] =
					new SELMA_classifier(output_filename);
				}
				break;
			}
			case e_testSELMA:
			{						
				DataConsumer_pointer[thread_number][function_number++] = new SELMA_IF(true);
				break;
			}

#endif
#ifdef _useSDL
// -------------------------------------------- Display ----------------------------------------------------------------------//
// variants of the display functions are located elsewhere, such as display including trackers which is in the 
			case eD_TD:
			{
				printf("Display TD module implemented\n");
				DataConsumer_pointer[thread_number][function_number++] = new TDdisplay();
				break;
			}
			case eD_APS:
			{
				printf("APS enabled by default because you selected to display APS events.\n");
				source_parameters.useGrayscale = true;
				printf("Display APS module implemented.\n");
				int offset = 0;
				int scale = 100000;
				if(arg_num >= argc-1)
				printf("Arguments not given for scale and offset of APS display, resorting to default scale of %i and offset of %i\n", scale, offset);
				else
				if(argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
				printf("Arguments not given for scale and offset of APS display, resorting to default scale of %i and offset of %i\n", scale, offset);
				else
				{
					scale = atoi(std::string(argv[arg_num++]).c_str());
					offset = atoi(std::string(argv[arg_num++]).c_str());
				}
				printf("Display APS module implemented with scale %i and offset %i.\n", scale, offset);
				//init_showAPS(scale, offset);
				DataConsumer_pointer[thread_number][function_number++] = new EMdisplay(scale, offset);
				break;
			}
			case eD_TD_MMtracker:
			{
				//printf("Display TD MMTracker Module implemented \n");
				//DataConsumer_pointer[thread_number][function_number++] = new MinMaxTrackerDisplay();
				//break;

				std::string ip_address("127.0.0.1");
				int port (6001);
				int evt_port (6002);// Event Sync Stream
				std::cout << "MMTracker arg_num == "+ std::to_string(arg_num)+ " argc ==" + std::to_string(argc)<< std::endl;
				if (arg_num == argc || arg_num == argc - 1 || arg_num == argc - 2|| argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
				{
					printf("Not enough Arguments given for MMtracker to communicate over TCP\n");
					printf("For TCP Visualizer Expected Format is : ./epf ... -D_TD_MMtracker <ip_addr> <track_port> <evt_port> ....\n");
					printf("Reverting to MMtracker without the visualizer\n");
					DataConsumer_pointer[thread_number][function_number++] = new MinMaxTrackerDisplay(false);
				} else
				{
					ip_address = std::string(argv[arg_num++]);
					port = atoi(std::string(argv[arg_num++]).c_str());
					evt_port = atoi(std::string(argv[arg_num++]).c_str());
					printf(
							"MMTracker will publish updates with\n\tip_address: %s\n\tport: %d\n",
							ip_address.c_str(), port);
					DataConsumer_pointer[thread_number][function_number++] =
					new MinMaxTrackerDisplay(ip_address, port, evt_port);
				}
				break;
			}
			case eD_TD_MMtracker_WRITEANNOT:
			{
				std::string output_filename = "output.csv";
				if (arg_num == argc || argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
				{
					printf("Not enough Arguments given for Annotation Writing\n");
					printf("Expected Format is : ./epf ... -D_TD_MMT_WRITEANNOT <output> ....\n");
					printf("Running MMTracker. Writing Output to output.csv...\n");
					DataConsumer_pointer[thread_number][function_number++] =
					new MinMaxTrackerDisplay(output_filename);
				} else
				{
					output_filename = std::string(argv[arg_num++]);
					std::cout << "Writing Annotations To " + output_filename << std::endl;
					DataConsumer_pointer[thread_number][function_number++] =
					new MinMaxTrackerDisplay(output_filename);
				}
				break;
			}

			case eD_TD_MMtracker_SELMAC:
			{
				std::cout << "Running Min-Max Tracker With Selma Outputs...\n"<<std::endl;
				DataConsumer_pointer[thread_number][function_number++] = new MinMaxTrackerDisplay(true);
				break;
			}

			/*		case writevideo_TD:
			 {
			 //function_pointer[thread_number][function_number++] = &write_video_TD;
			 std::string video_filename ("");
			 video_filename = argv[arg_num++];
			 //	init_write_video_TD(video_filename.c_str());
			 //printf("Will record TD video to file %s\n", video_filename.c_str());
			 printf("WARNING: TD write video not implemented");
			 break;
			 }
			 case writevideo_APS:
			 {
			 //function_pointer[thread_number][function_number++] = &write_video_APS;
			 std::string video_filename ("");
			 video_filename = argv[arg_num++];
			 //	init_write_video_APS(video_filename.c_str());
			 //printf("Will record APS video to file %s\n", video_filename.c_str());
			 printf("WARNING: APS write video not implemented");
			 break;
			 }*/
#endif					
// --------------------------------------------- Data IO --------------------------------------------------------------------------//
		case eIO_write: {
			DataConsumer_pointer[thread_number][function_number++] =
					new file_write();

			file_write_pointer[num_file_outputs] =
					(file_write*) DataConsumer_pointer[thread_number][function_number
							- 1];

			output_filename[num_file_outputs] = std::string(argv[arg_num++]);
			num_file_outputs++;
			printf("Will write data for file %i to: %s\n", num_file_outputs,
					output_filename[num_file_outputs - 1].c_str());

			break;
		}

// --------------------------------------- Tracking ------------------------------------------------------ //
		case eT_TR: {
			printf("Tracking module implemented\n");
			int num_trackers = 1;
			int tracker_size = 10;
			if (arg_num == argc || argv[arg_num][0] == '-'
					|| argv[arg_num][0] == '_')
				printf(
						"No tracker number argument passed to tracker, will use the default of %i trackers\n",
						num_trackers);
			else {
				num_trackers = atoi(std::string(argv[arg_num++]).c_str());
				if (arg_num == argc || argv[arg_num][0] == '-'
						|| argv[arg_num][0] == '_')
					printf(
							"No tracker size argument passed to tracker, will use the default of %i trackers\n",
							tracker_size);
				else
					tracker_size = atoi(std::string(argv[arg_num++]).c_str());
//*************************************************** HERE IS WHERE WE SHOULD IMPLEMENT SECOND OPTIONAL SET OF PARAMETERS FOR CALLING TRACKERS
//					if(argv[arg_num][0] == '-')
			}
			printf(
					"%i trackers will be implemented with parameters of ... (calling from command line is still to be implemented)\n",
					num_trackers);
			DataConsumer_pointer[thread_number][function_number++] = new track(
					num_trackers, tracker_size);
			break;
		}
#ifdef _useSDL
			case eD_TD_track:
			{
				printf("Display TD module with tracking implemented\n");
				DataConsumer_pointer[thread_number][function_number++] = new TDdisplay_track();
				break;
			}
			case eD_TD_SELMACtrack:
			{
				printf("Display TD module with SELMAC tracking implemented\n");
				DataConsumer_pointer[thread_number][function_number++] = new TDdisplay_track_SELMAC();
				break;
			}
			case writevideo_TD_track:
			{
				//function_pointer[thread_number][function_number++] = &write_video_TD_track;
				std::string video_filename ("");
				video_filename = argv[arg_num++];
//					init_write_video_TD_track(video_filename.c_str());
				//printf("Will record TD video with tracking to file %s\n", video_filename.c_str());
				printf("WARNING: TD track write video not implemented");
				break;
			}
#endif
// ------------------------------------ Corner Detection ---------------------------------------------------- //
			/*			case eCorner:
			 {
			 DataConsumer_pointer[thread_number][function_number++] = new corners();
			 printf("Will run corner detection");
			 break;
			 }*/
// ------------------------------------ Orientation ---------------------------------------------------- //
			/*			case eO_GAB:
			 {
			 printf("Orientation detector with Gabor filter implemented\n");
			 DataConsumer_pointer[thread_number][function_number++] = new GaborDetector();
			 break;
			 }*/
			/*			#ifdef _useSDL
			 case eD_OR:
			 {
			 printf("Display of orientation events implemented\n");
			 DataConsumer_pointer[thread_number][function_number++] = new ORdisplay();
			 break;
			 }

			 case writevideo_OR:
			 {
			 //function_pointer[thread_number][function_number++] = &write_video_OR;
			 //std::string video_filename ("");
			 //video_filename = argv[arg_num++];
			 //init_write_video_OR(video_filename.c_str());
			 //printf("Will record TD video with tracking to file %s\n", video_filename.c_str());

			 printf("WARNING: OR write video not implemented");
			 break;
			 }
			 #endif

			 #ifdef _useSDL
			 case eD_OF:
			 {
			 printf("Will show optical flow\n");
			 DataConsumer_pointer[thread_number][function_number++] = new display_optical_flow();
			 break;
			 }
			 #endif*/
#ifdef _useROS

// ------------------------------------ ROS Interface ---------------------------------------------------- //
			case eR_TD:
			{
				printf("ROS Output of TD events enabled\n");
				// Check for input argument, only give default name to field if ROS was not initialized before (i.e. via launch file)
				// First check if any argument was passed, if it wasn't we cannot call the InitRos with an empty / wrong string!
				std::string name_argument;
				if(arg_num == argc || argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
				{
					InitRos("nm_epf");
					DataConsumer_pointer[thread_number][function_number++] = new TDRosInterface(name_argument, source_parameters.camera_serial_number); // Version defaults to 16
				}
				else
				{
					name_argument = std::string(argv[arg_num++]);
					InitRos(name_argument);
					DataConsumer_pointer[thread_number][function_number++] = new TDRosInterface(name_argument, source_parameters.camera_serial_number);
				}
				break;
			}

			case eR_track:
			{
				printf("ROS Output of tracking events enabled\n");
				// Check for input argument
				std::string name_argument;
				if(arg_num == argc || argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
				{
					InitRos("nm_epf");
					DataConsumer_pointer[thread_number][function_number++] = new TrackingRosInterface(name_argument, source_parameters.camera_serial_number);
				}
				else
				{
					name_argument = std::string(argv[arg_num++]);
					InitRos(name_argument);
					DataConsumer_pointer[thread_number][function_number++] = new TrackingRosInterface(name_argument, source_parameters.camera_serial_number);
				}
				break;
			}
			case eR_APS:
			{
				printf("ROS Output of greyscale images enabled\n");
				source_parameters.useGrayscale = true;
				std::string name_argument;
				if(arg_num == argc || argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
				{
					InitRos("nm_epf");
					DataConsumer_pointer[thread_number][function_number++] = new GreyscaleRosInterface(name_argument, source_parameters.camera_serial_number); // Version defaults to 16
				}
				else
				{
					name_argument = std::string(argv[arg_num++]);
					InitRos(name_argument);
					DataConsumer_pointer[thread_number][function_number++] = new GreyscaleRosInterface(name_argument, source_parameters.camera_serial_number);
				}
				break;
			}
			case eR_IMU:
			{
				printf("ROS Output of IMU data enabled\n");
				source_parameters.useIMU = true;
				std::string name_argument;
				if(arg_num == argc || argv[arg_num][0] == '-' || argv[arg_num][0] == '_')
				{
					InitRos("nm_epf");
					DataConsumer_pointer[thread_number][function_number++] = new IMU6RosInterface(name_argument, source_parameters.camera_serial_number); // Version defaults to 16
				}
				else
				{
					name_argument = std::string(argv[arg_num++]);
					InitRos(name_argument);
					DataConsumer_pointer[thread_number][function_number++] = new IMU6RosInterface(name_argument, source_parameters.camera_serial_number);
				}
				break;
			}
			case eR_Config:
			{
				// Currently unimplemented, consider removing
				break;
				ConfigRosNode(argv[arg_num-1]);
				break;
			}
#endif

// --------------------------------------- Error case for when no valid argument is detected ------------------------------------------------------ //
		case e_Error: {
			printf("Invalid Argument Detected.\n");
			return (0);
		}

		case edefault: {
			break;
		}
		}
	}

	//function_pointer[thread_number][function_number] = NULL;
	int filecounter = 0;
	if (num_file_outputs > 0) //initialize the comments
			{
		printf("data source size for file is %i %i \n", DataSource::x_dim,
				DataSource::y_dim);
		std::string full_comments("#");
		for (arg_num = 0; arg_num < argc; arg_num++) {
			full_comments.append(std::string(argv[arg_num]));
			full_comments.append(" ");
		}

		while (filecounter < num_file_outputs) {
			std::string these_comments(full_comments);
			these_comments.append("\n#File write number: ");
			these_comments.append(std::to_string(filecounter));
			these_comments.append("\n");
			these_comments.append(source_parameters.comments);
			file_write_pointer[filecounter]->set_parameters(
					output_filename[filecounter].c_str(),
					these_comments.c_str(), file_write_limit);
			filecounter++;
		}
	}

	printf("launching processing chain\n");
	// launch the thread
	thread_launcher::launch_processing_chain(); //look at thread_launcher.cpp to see how this is done. "function_pointer" is accessed directly, no need to pass it

	thread_launcher::run_source();
	printf("Cleaning up main thread in preparation for exit\n");

	//int finish = fp_num;
	while (thread_launcher::safe_to_exit() == false)
		usleep(1000);

	return 0;
}

/********************************************************************
 This function computes an ID/Hash from a given argument parameter
 ********************************************************************/
string_code hashit(std::string const& inString) {
	if (inString == "-EKF")
		return eEKFloc;
// ------------------------ Filters ---------------------------------
	if (inString == "-F_NN")
		return eF_NN;
	if (inString == "-F_NN2")
		return eF_NN2;
	if (inString == "-F_NNhv")
		return eF_NNhv;
	if (inString == "-F_NNp")
		return eF_NNp;
	if (inString == "-F_ISI")
		return eF_ISI;
	if (inString == "-F_RF")
		return eF_RF;
	if (inString == "-F_RFp")
		return eF_RFp;
	if (inString == "-F_u")
		return eF_undo;
	if (inString == "-F_DP")
		return eF_DP;
	if (inString == "-F_reform")
		return eF_reform;
	if (inString == "-F_NNg")
		return eF_NNg;
	if (inString == "-F_ROI")
		return eF_ROI;
	if (inString == "-F_ROImask")
		return ef_ROImask;
	if (inString == "-F_offset")
		return eF_offset;
	if (inString == "-F_rangefix")
		return eF_rangefix;
	if (inString == "-F_change_type")
		return echange_type;
//	if (inString == "-F_TNF") return eF_TNF;
//	if (inString == "-F_TNT") return eF_TNT;
	if (inString == "-F_TNC")
		return eF_TNC;
	if (inString == "-F_ETP")
		return eF_ETP;
#ifdef _useSELMA
	if (inString == "-F_SELMAC") return eF_SELMAC;
	if (inString == "-F_SELMAC_RUNDATA") return eF_SELMAC_RUNDATA;
	if (inString == "-F_SELMAC_WRITEANNOT") return eF_SELMAC_WRITEANNOT;
	if (inString == "-testSELMA") return e_testSELMA;
#endif
// ------------------------ Display ---------------------------------
	if (inString == "-D_APS")
		return eD_APS;
	if (inString == "-D_TD")
		return eD_TD;
	if (inString == "-D_TD_MMT")
		return eD_TD_MMtracker;
	if (inString == "-D_TD_MMT_SELMAC")
		return eD_TD_MMtracker_SELMAC;
// ------------------------Display And Annotation writing------------
	if (inString == "-D_TD_MMT_WRITEANNOT")
		return eD_TD_MMtracker_WRITEANNOT;

// ------------------------ Data IO ---------------------------------
	if (inString == "-write")
		return eIO_write;
	if (inString == "-fromfile")
		return eIO_read;
	if (inString == "-fromfileparts")
		return eIO_readparts;
	if (inString == "-realtime")
		return eIO_realtime;
	if (inString == "-perfiletime")
		return eIO_limit_file_time;

//------------------------- Devices ---------------------------------
	if (inString == "-davis240C")
		return edavis240C;
	if (inString == "-davis640")
		return edavis640;
	if (inString == "-atis")
		return eATIS;

//------------------------- Threading ---------------------------------
	if (inString == "-t")
		return eNewThread;

// ------------------------ Config ---------------------------------
	if (inString == "-biases")
		return eIO_biases;
	if (inString == "-bitfile")
		return eIO_bitfile;
	if (inString == "-biasLookup18")
		return bias18;
	if (inString == "-biasLookup33")
		return bias33;
	if (inString == "-v4")
		return version4;
	if (inString == "-enableAPS")
		return eAPS;
	if (inString == "-enableIMU")
		return eIMU;
	if (inString == "-time")
		return eTime;
	if (inString == "-forcesize")
		return eForceSize;
	if (inString == "-simulate")
		return eSimulate;
	if (inString == "-F_NN_fpga")
		return eF_NN_fpga;
	if (inString == "-F_RF_fpga")
		return eF_RF_fpga;
	if (inString == "-F_remove_fpga")
		return eF_remove_fpga;
	if (inString == "-enableTN")
		return eTN;
	if (inString == "-fpga_ms")
		return e_ms_tick;

// ---------------------- Tracking ------------------------------
	if (inString == "-track")
		return eT_TR;
	if (inString == "-D_TD_track")
		return eD_TD_track;
	if (inString == "-D_TD_SELMACtrack")
		return eD_TD_SELMACtrack;

	if (inString == "-writevideo_TD_track")
		return writevideo_TD_track;

// ---------------------- Corner Detection ------------------------------
	if (inString == "-corner")
		return eCorner;

// ---------------------- Optical Flow ------------------------------
	if (inString == "-OF_fpga")
		return eOF_fpga;
	if (inString == "-D_OF")
		return eD_OF;

// --------------------- Orientation ------------------------------
	if (inString == "-O_GAB")
		return eO_GAB;
	if (inString == "-D_OR")
		return eD_OR;
	if (inString == "-F_ONN")
		return eF_ONN;
	if (inString == "-writevideo_OR")
		return writevideo_OR;

#ifdef _useROS
// -------------------- ROS Interface -------------------------
	if (inString == "-R_TD") return eR_TD;
	if (inString == "-R_track") return eR_track;
	if (inString == "-R_APS") return eR_APS;
	if (inString == "-R_IMU") return eR_IMU;
	// Arguments preceded by a double underscore are roslaunch configuration arguments
	if (strstr(inString.c_str(), "__")) return eR_Config;
	if (inString == "-ROS_IN") return eROS_IN;
#endif

	return edefault;
	printf("\n\t ERROR: input argument %s not recognized\n", inString.c_str());
	return e_Error;
}
