# README #

### Code for running the Event Processing Framework from Linux (Ubuntu 16.04) ###

## How do I get set up? ##

### FrontPanel ###
* The frontpanel version will depend on which version of linux you are running.  Download the correct version for your OS if necessary from [https://pins.opalkelly.com/downloads] (you may need to login to "pins" to see it). Contact me (Garrick) if you cannot access the install files.
* You will need to edit "install.sh" to correct the filename "okCFrontPanel.dll" to "okFrontPanel.dll"

* $sudo chmod +x install.sh
* $sudo ./install.sh

### Protocol Buffers 2 && Boost ###
* used to serialize events, tracker updates and TN classification results before sending over TCP

* $sudo apt-get install protobuf-compiler

* $sudo apt-get install libboost-all-dev

### SDL (optional, but required for display) ###
* used for APS and TD display functions to remove the dependency on OpenCV

* $sudo apt-get install libsdl2-dev

* (you may need to run: $export LD_LIBRARY_PATH="/usr/local/lib" to get it to work)

### libcaer ###
* Provides support for inilabs devices
* Instructions can be found at: https://github.com/inilabs/libcaer

* $sudo apt install libusb-1.0-0-dev
* $sudo apt install cmake
* $git clone https://github.com/inilabs/libcaer.git
* $cd libcaer
* $cmake -DCMAKE_INSTALL_PREFIX=/usr .
* $sudo make
* $sudo make install

### g++4.9 ###
* Opal Kelly Frontpanel does not work with versions newer than 4.9, but libcaer requires at least 4.9, so 4.9 is the magic number.

* $sudo apt install g++-4.9

### libudev ###
Frontpanel needs "libudev.so.0", but this does not exist in Ubuntu 16.04, so you'll need to find the directory containing "libudev.so.1" and make a link between them

$sudo find / -name "libudev.so.1"

$sudo ln (path-to-libudev.so.1-from-above)/libudev.so.1 (path-to-libudev.so.1-from-above)/libudev.so.0

### TrueNorth bare metal ###
The TN runtime libraries will be needed. Not included as this is export-controlled.

### Deployment instructions ###  
#### To run (Ubuntu 16.04) #### 
Once dependencies are installed, run the script ./compile.sh. This will create the executable in the executable directory. Then change to the executable directory and run the epf executable
* $./compile
* $cd executable
* ./epf ... -D_TD 

This command will connect to ATIS and display the live output. To add basic filtering and display the output, try:

* ./epf ... -F_NN -D_TD

#### Dealing with errors ####

* error "libcufft.so.7.5: cannot open shared object file: No such file or directory"

find the file using:  
$sudo find / -name "libcufft.so.7.5"

then add the file path to your bashrc  
$sudo gedit /etc/bash.bashrc
add the line:  
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:path_to_file 

for example:  
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-7.5/targets/x86_64-linux/lib/

* error "libcaer.so.2: cannot open shared object file: No such file or directory"

This error can happen if libcaer was recently installed. Run the command

$ sudo ldconfig

* CRITICAL: davisCommonOpen: Failed to open DAVIS FX2 device.

Try running epf as a superuser (sudo). If it works create a new udev rule to grant permissions to the users:

$ sudo gedit /etc/udev/rules.d/65-inilabs.rules

And write the following inside:

SUBSYSTEM=="usb", ATTR{idVendor}=="152a", ATTR{idProduct}=="84[0-1]?", MODE="0666"

# Command Line Arguments #

This code allows a filter chain to be set up from the command line. All modules for processing, saving, and/or displaying data are configured as filter modules. The modules and their parameters are all passed from the command line as documented below. The syntax is:
	"./epf -event_source -module_1 module_1_arguments -module_2 module_2_arguments ... -module_N module_N_arguments"

Filtering, display, and write modules are implemented in the order in which they are called.

All arguments are optional.

## configuration ##

### -t ###
Specifies that a new thread should be launched. By default data acquisition is one thread (the main thread) and processing runs on a second thread.

* $./epf ... -F_NN -D_TD
Will run the event acquisition on the main thread, and both the -F_NN filter and -D_TD display functions on a second thread

* $./epf ... -F_NN -t -D_TD
Will run the event acquisition on the main thread, the -F_NN filter on a second thread, and the -D_TD display function on a third thread

### -bitfile ###
Specifies an FPGA configuration bitfile to use instead of the default (almost never necessary)

Arguments:
	string bitfile_path_and_name : The path and name of the bitfile to use

Usage:
	"./epf ... -bitfile bitfile_path_and_name"  

### -biases ###
Specifies a set of chip biases to use instead of the default biases (almost never necessary)

Arguments:
	string biases_file_path_and_name : The path and name of the biases file to use

Usage:
	"./epf ... -biases biases_file_path_and_name"   

### -v4 ###
Specifies to use version 4 of the ATIS PCB (currently used by JHU and MARCS).

Arguments:
	none

Usage:
	"./epf ... -v4 ..."   

### -rangefix ###
Checks that incoming events are within range. Required for some v4 boards which occasionally generate out of range addresses.

### -forcesize ###
Modifies the resolution of the source. For example, if your input file specifies that you used ATIS (304x240 pixels) for the recording, but the file only has events from a 32x32 pixel window. You can override the ATIS resolution and use 32x32 as the size instead of having all the blank display.

Arguments:
	int x_size : The horizontal size of the source
	int y_size : The vertical size of the source

Usage:
	"./epf ... -forcesize x_size y_size ..."

### -enableAPS ###
Enables the APS option of ATIS and couples it to the TD circuit. Grayscale values 0-255 are calculated from the APS inter spike interval (ISI) as : grayscale = offset + scale/ISI

Arguments (optional):
	int scale : A scale to be used to shift between inter-spike interval in microseconds and 8 bit display  
	int offset : An offset to be used for grayscale

Usage:
	"./epf ... -D_APS scale offset ..."

### -time ###
Will stop the program after a specified time period

Arguments:
	int time_in_milliseconds : The program will exit when timestamps reach this time.

Usage:
	"./epf ... -time time_in_milliseconds ..."


## Data Sources ##
The source must always be the first argument specified.

### -atis ###
Specifies to use ATIS (default) as the source for events.

Arguments:
	none

Usage:
	"./epf -atis ..."

### -simulate ###
Uses a recorded file and pipes the events through the ATIS FPGA as if they were arriving from the ATIS sensor itself. Useful for testing FPGA computation.

Arguments:
	string filename : The recorded file to use for events. Must be v2 or higher (specified in the file comments). If the file is below v2, it can be upgraded by running "./epf -fromfile oldfilename -write newfilename" (-fromfile can read old file formats, but -write will automatically write to the latest file format)

Usage:
	"./epf -atis -simulate filename ..."

### -davis240c ###
Specifies to use davis240c as the source for events.

Arguments (optional):
	int serial_number : the serial number of the device to use. This is useful if you have multiple devices connected to the same machine.

Usage:
	"./epf -davis240c serial_number ..."

### -davis640 ###
Specifies to use davis640 as the source for events.

Arguments (optional):
	int serial_number : the serial number of the device to use. This is useful if you have multiple devices connected to the same machine.

Usage:
	"./epf -davis640 serial_number ..."


### -fromfile ###
Specifies a recorded file to use as input instead of live streaming from the ATIS. 

Arguments:
	string input_file_path_and_name : The path and name of the file to read from

Usage:
	"./epf -fromfile input_file_path_and_name ..."


### -fromfileparts ###
Specifies a file recorded in parts (using -write filename -perfiletime filetime) to use as input instead of live streaming from a device. 

Arguments:
	string input_directory : The path to the files
	(optional) int filenumber : the file number to start reading from. Useful if you want to start partway through a long recording (a recording can be days long).

Usage:
	"./epf -fromfileparts input_directory filenumber ..."


### -realtime ###
Forces an input file to be read in real-time based on the system clock (can slow reading down, but obviously cannot speed it up). Has no effect on sources other than file (since they are inherently real-time and cannot be sped up or slowed down)

Arguments (optional):
	int time_multiplier : The playback will be accelerated by this amount (time_multiplier=2 means twice as fast as real-time)

Usage:
	"./epf -fromfile input_file_path_and_name -realtime time_multiplier ..."

### -ROS_IN ###
Allows a ROS topic to be used as the source for events. 

Arguments (optional):
	string ros_topic_name : The name of the ROS topic to subscribe to as the source. Defaults to "nm_epf" if not specified

Usage:
	"./epf -ROS_IN ros_topic_name ..."

## Data IO ##

### -write ###
Specifies to write event-data to a file. Can be called multiple times to write results from different points in the processing chain to different files.

Arguments:
	string output_file_path_and_name : The path and name of the file to write to

Usage:
	"./epf ... -write output_file_path_and_name ..."


### -perfiletime ###
When used in conjunction with "-write", this option limits the time length of recording for each file. When the time limit is reached, the file is closed and another file is opened to continue writing. No events are lost in the process. The filename is automatically iterated (filename, filename0, filename1, filename2...).

Arguments:
	int per_file_time_in_seconds : The time-length of data to write to each file

Usage:
	"./epf ... -write output_file_path_and_name -perfiletime per_file_time_in_seconds ..."


## Display ##

### -D_TD ###
Brings up a live display of the TD spikes

No arguments

Usage:
	"./epf ... -D_TD ..." 

### -D_APS ###
Brings up a live display of the APS spikes

No arguments

Usage:
	"./epf ... -D_APS ..." 

### -D_TD_MMtracker ###
Brings up a live display of the TD spikes with MinMax Tracker Bounding Boxes

No arguments

Usage:
	"./epf ... -D_TD_MMtracker ..." 


## Filters ##

### -F_ROI ###
A filter which removes an ROI described by the 4 parameters following the -F_ROI argument

Arguments (optional):
	integer x : the X-address of the top left corner of the ROI to remove
	integer y : the Y-address of the top left corner of the ROI to remove
	integer x_size : the x-dimension size of the ROI to remove
	integer y_size : the y-dimension size of the ROI to remove
	
Usage:
	"./epf ... -F_ROI x y x_size y_size ..." 

### -F_RF ###
Implements a refractory period for each pixel

Arguments (optional):
	integer t_rf : the refractory time in microseconds.
	
Usage:
	"./epf ... -F_RF_fpga t_rf ..." 

### -F_RF_fpga ###
Same as -F_RF above, but will run the filter in the ATIS FPGA before any other processing is done (only works with -atis and -simulate).

Arguments (optional):
	integer t_rf : the refractory time in microseconds, only up to 128 000us.
	
Usage:
	"./epf ... -F_RF_fpga t_rf ..." 


### -F_NN ###
A noise filter which removes an event if none of the 8 nearest pixel neighbours have spiked in the last t_nn microseconds

Arguments (optional):
	integer t_nn : the time in microseconds within which one of the 8 neighbouring pixels must have spiked in order to allow the current event through this filter
	
Usage:
	"./epf ... -F_NN t_nn ..." 

### -F_NN_fpga ###
Same as -F_NN above, but will run the filter in the ATIS FPGA before any other processing is done (only works with -atis and -simulate).

Arguments (optional):
	integer t_nn : the time in microseconds within which one of the 8 neighbouring pixels must have spiked in order to allow the current event through this filter. only up to 128 000us.
	
Usage:
	"./epf ... -F_NN_fpga t_nn ..." 

### -F_NN2 ###
A noise filter which removes an event if less than 2 of the 8 nearest pixel neighbours have spiked in the last t_nn microseconds

Arguments (optional):
	integer t_nn : the time in microseconds within which two of the 8 neighbouring pixels must have spiked in order to allow the current event through this filter
	
Usage:
	"./epf ... -F_NN2 t_nn ..." 

### -F_NNhv ###
A noise filter which removes an event if less than one of the 4 horizontal and vertical pixel neighbours have spiked in the last t_nn microseconds

Arguments (optional):
	integer t_nn : the time in microseconds within which one of the 4 neighbouring pixels must have spiked in order to allow the current event through this filter
	
Usage:
	"./epf ... -F_NNhv t_nn ..." 

### -F_NNp ###
A noise filter which removes an event if none of the 8 nearest pixel neighbours have spiked with the same polarity in the last t_nn microseconds

Arguments (optional):
	integer t_nn : the time in microseconds within which one of the 8 neighbouring pixels must have spiked with the same polarity in order to allow the current event through this filter
	
Usage:
	"./epf ... -F_NNp t_nn ..." 

### -F_ISI ###
Still to be documented.

### -F_RP ###
A noise filter which removes an event if it occurs within a refractory period time t_rp since the last spike from a pixel

Arguments (optional):
	integer t_rp : the refractory period of each pixel in microseconds
	
Usage:
	"./epf ... -F_RP t_rp ..." 
	
### -F_RPp ###
A noise filter which removes an event if it occurs within a refractory period time t_rp of the last spike of the same polarity from a pixel

Arguments (optional):
	integer t_rp : the refractory period of each pixel in microseconds
	
Usage:
	"./epf ... -F_RPp t_rp ..." 
	
### -F_DP ###
A noise filter which removes an event if it has the same polarity as the last event from the pixel

No arguments
	
Usage:
	"./epf ... -F_DP ..." 

### -F_NNg ###
A more general parameterizable version of the noise filter. Still to be fully implemented/documented.

### -F_u ###
A filter which undoes any previous filtering. Useful if reading data from a recording which contains filtered out events. 

No arguments
	
Usage:
	"./epf ... -F_u ..." 

no arguments

### -F_reform ###
Deletes filtered events. This function is slow because it must individually copy events based on whether they are flagged as filtered, but it can reduce output file sizes which is useful when reading data into Matlab etc.

no arguments

Usage:
	"./epf ... -F_reform ..." 


### -F_TNC ###
TrueNorth Classifier: Use TN to perform classification.
This big module will perform tracking, extraction of track-events, publish tracker updates to a TCP listener elsewhere, publish extracted events to TN, collect TN results, publish TN confidence to TCP listener and USB.
 
For now, only tracking and publishing of tracks is working.

Arguments (required):
  string ipAddress : IP adddress of the TCP listener
  integer port: Port of the TCP listener
  string model : path to the .tnbm file containing the TN model
  string input_map : path to the .bin file containing the TN model's input connector
  string output_map : path to the .bin file containing the TN model's output connector
  integer width : width of the input td that the TN model processes
  integer height : height of the input td that the TN model processes
  integer num_channels : number of channels (polarity) of the input td that the TN model processes
  integer radius: tracker radius
  integer num_trackers: number of trackers

Usage:
  "./epf ... -F_TNC ipAddress port model input_map output_map width height num_channels radius num_trackers ..."
  e.g.
  "./epf ... -F_TNC 127.0.0.1 6001 classifier.tnbm input.bin output.bin 32 32 3 24 8 ...""

### -F_ETP
Transparently publishes TD events over TCP to a server somewhere. Does not modify any of the events.

Arguments (required):
  string ip_address : ip address to publish to
  port : tcp port to connect to
  
Usage:
  "./epf ... -F_ETP ip_address port ..."

### -F_SELMAC ###
SELMA Classifier: Use SELMA to perform Classification (Requires useSELMA to be true during Compilation)

It runs functional test to confirm SELMA is running correctly and send events over TCP to Visualizer and Forwards Info Going into Comms Module over TCP
If there is no arguments, SELMAC will run without sending packets to Visualizer

Arguments (optional):
  string ipAddress : IP adddress of the TCP listener
  integer port: Port of the TCP listener for Track Updates
  integer evt_port: Port of the TCP listener for Events 

#History of 10 classificaiton (Do Voting Over)
Usage:
	"./epf ... -F_SELMAC ip_address port evt_port ..."

### -F_SELMAC_RUNDATA ###
SELMA Classifier: Use SELMA to Hidden Layer Output for Training SELMA (Requires useSELMA to be true during Compilation)

It runs functional test to confirm SELMA is running correctly and Passes Data from CSV File to SELMA for Generating Hidden Layer Output Wegihts

Arguments:
  string test_folder : folder in which csv file resides in
  string csv_filename: csv filename (CSV should be in the following format : Label1, Addr0, .... Addr1023\n,Label2,....\n\n)  	 

Usage:
	"./epf ... -F_SELMAC_RUNDATA test_folder csv_filename ..."

### -F_SELMAC_WRITEANNOT ###
SELMA Classifier: Use SELMA to Hidden Layer Output for Training SELMA (Requires useSELMA to be true during Compilation)

It runs functional test to confirm SELMA is running correctly and outputs tracks in annotations format used in MATLAB

Arguments:
  string output_file : name of output_file to write to

Usage:
	"./epf ... -F_SELMAC_WRITEANNOT output_file ..."
Example: 
	"./epf -fromfile ../recordings/05-Jul-16/f1.bin -F_RP 5000 -F_NN2 5000 -F_SELMAC_WRITEANNOT 05_Jul_16_f1.txt -D_TD_SELMACtrack"

## Tracking ##

### -track ###
Implements naive trackers

Arguments (optional):
	integer num_trackers : the number of trackers to implement
	
Usage:
	"./epf ... -track num_trackers ..." 
	
### -D_TD_track ###
Brings up a live display of the TD spikes, similar to -D_TD, but also displays the trackers.

No arguments

Usage:
	"./epf ... -D_TD_track ..." 

### -D_TD_SELMACtrack ###
Brings up a live display of the TD spikes, similar to -D_TD, but also displays the trackers from SELMA
Adapted from D_TD_track

No arguments
Usage:
	"./epf ... -D_TD_SELMACtrack"

## ROS Interface ##

Please note, the name parameter is only initialized once and every subsequent name parameter for the node will be ignored.

### -R_TD ###
Posts events data to a topic.

Arguments:
	string name (optional) : Initializes the name of the node. The data will be posted in /nm/epf/ + name + /td (i.e. /nm/epf/camera_1/td)

Usage:
	"./epf ... -R_TD name ..."

### -R_track ###
Posts tracking events to a topic. Each tracker event includes the position of the tracker's center at a certain time.

Arguments:
	string name (optional) : Initializes the name of the node. The data will be posted in /nm/epf/ + name + /track (i.e. /nm/epf/camera_1/track)

Usage:
	"./epf ... -R_track name ..."

### -R_APS ###
Posts greyscale images to a topic, as well as camera_info to be used for calibration.

Arguments:
	string name (optional) : Initializes the name of the node. The data will be posted in /nm/epf/ + name + /td (i.e. /nm/epf/camera_1/image_raw)

Usage:
	"./epf ... -R_APS name ..."

### -R_IMU ###
Posts IMU data to a topic.

Arguments:
	string name (optional) : Initializes the name of the node. The data will be posted in /nm/epf/ + name + /imu (i.e. /nm/epf/camera_1/imu)

Usage:
	"./epf ... -R_IMU name ..."


## Localization ##
### -EKF ###
Uses an Extended Kalman Filter to estimate 6DoF camera pose, assuming it is viewing a static planar scene and that the image plane and scene start off parallel.

Arguments:
	int depth: A rough estimate of the scene depth. Works over a wide range of values if the sensor image plane doesn't leave the original image plane, otherwise must be accurate. Also used to remap events back to where they would appear from the camera initialization position.
	string filename: Optional, specifies a file to write the pose estimates to. For each event 6 floats will be written to the file [x,y,z,wx,wy,wz], where [x,y,z] are position relative to the start in m, and [wx,wy,wz] specifies a rotation axis and magnitude in radians.

Usage:
	"./epf ... -EKF depth filename ..."


# How to add your own filtering function #
Steps for adding a new processing module to the ATIS interface.

##1. Decide on a name and abbrevation for the filter. Let us call these "filter_name" and "F_1" respectively.##

##2 Modify a header file to include required function definitions as described below.##

2.1 In a header file (typical example inc/filters.h), declare the filter function as:
	bool filter_name(packet_info_type* packet_info, event* data, bool exit_thread);
The filter function MUST match this definition. The struct fields for these types can be found in event_framework.h

2.2 If the filter relies on any parameters, declare an initialization function in the same header file as
	void init_filter_name(parameter_1_type parameter_1, parameter_2_type parameter_2);
  
##3. Write the filter initialization function and the filter in the cpp file (typically src/filters.cpp). Use the example templates below:##
   
  
static parameter_1_type p1  
static parameter_2_type p2  
void init_filter_name(parameter_1_type parameter_1, parameter_2_type parameter_2)  
{  
	p1 = parameter_1;  
	p2 = parameter_2;  
}  
  
bool (packet_info_type* packet_info, event* data, bool exit_thread)  
{  
	int filterTimeOverflow = packet_info->time_start;  
	int time;  

	for(int i=0; i<packet_info->num_events; i++) //loop through the output buffer events  
	{  
		if (data[i].type == evt_type::timer_overflow)  
		{  
			filterTimeOverflow += 1<<16;  
		}else  
		{  
			time = data[i].t | filterTimeOverflow; //this is how to get the time of the current event  
			//process each event here, using p1 and p2  
		}  
	}  
	// the below lines are required to make sure the program terminates cleanly. Might be moved to a separate function in future  
	if(exit_thread == 1)  
	{  
		//any code such as closing display windows, files, etc will be placed here
	}  
	return exit_thread;
}  
  
##4. Make the function call-able from the command line as described below.##
  
4.1 Add an option for the function to the "string_code" enum in inc/event_framework.h. Example:  
enum string_code { //enumerate the function names here  
...  
eF_1, ...  
...  
};  
  
  
4.2 Modify the "hashit" function in src/event_processing_framework.cpp to include the new enum. For example, to call the function using the name "-F_1", modify as:  
string_code hashit (std::string const& inString) {   
	...   
        if (inString == "-F_1") return eF_1;   
	...  
}   
    
4.3 In the case statement in src/event_processing_framework.cpp, add a case for the filter. For example, to call the function by "F_1 param_1 param_2", modify as:   
	case eF_1:   
	{   
		function_pointer[thread_number][function_number++] = &filter_name; //this is a requirement for the interface to know to call the function   
		int param_1 = atoi(std::string(argv[arg_num++]).c_str()); //this line shows how to pass an integer from the string on the command line   
		float param_2 = atof(std::string(argv[arg_num++]).c_str()); //this line shows how to pass a float from the string on the command line   
		init_filter_name(param_1, param_2);   
		break;
	}  

Now after recompiling, you should be able to call your function from the command line as:     
./epf F_1 param_1 param_2    

##5. Document the function! ##
  
Add it to this readme.

## Who do I talk to? ##

* Contact Garrick Orchard garrickorchard@nus.edu.sg
