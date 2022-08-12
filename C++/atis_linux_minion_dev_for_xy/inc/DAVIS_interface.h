#ifndef DAVIS_INTERFACE_H
#define DAVIS_INTERFACE_H

#include <libcaer/libcaer.h>
#include <libcaer/devices/davis.h>

#include <event_framework.h>

 
class DavisSource : public DataSource 
{
	protected:	
		bool EnableGrayscale; // Defines whether we want to have greyscale data or not
		bool EnableIMU; // Defines whether we want to have IMU data or not
		pthread_mutex_t DAVIS_mutex; // Used by any functions which want to communicate with the Opal Kelly	
		caerDeviceHandle davis_handle;	
		caerEventPacketContainer packetContainer;
		bool get_new_data; //do we need to get new data, or is there old data still waiting to be processed?
		void ConfigureDavis(); // Does a few configurations, such as enable / disable greyscale, IMU, send reset signals for timestamp sync
		
		void GetIMU6Data(caerIMU6Event thisEvent, packet_info_type* packet_info, event* dataOUT, int& total_num_events);

		void PushIMU6Event(event* dataOUT, IMU_type subtype, float read_value, uint16_t time, int& total_num_events);
	public:	
		DavisSource(source_parameters_type &parameters, int davis_caer_type); //initializer
		bool getData(packet_info_type* packet_info, event* dataOUT, bool exit_thread); //getData
};

class Davis240CSource : public DavisSource
{
public:
	Davis240CSource(source_parameters_type &parameters);
private:
	void ProgramBiases(const std::string biasfile_ctrl_path, bool overwrite_biases);
};

class Davis640Source : public DavisSource
{
public:
	Davis640Source(source_parameters_type &parameters);
private:
	void ProgramBiases(const std::string biasfile_ctrl_path, bool overwrite_biases);
};
#endif