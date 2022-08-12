#include <ros_interface.h>
#include <nm_msgs/Event.h>
#include <nm_msgs/DavisIMU.h>
#include <ros/package.h>

#include <libcaer/libcaer.h>
#include <libcaer/devices/davis.h>

#define IMU_DATA_LENGTH 7

// Give a better name to this, it is the namespace for the topics and the name of the node
std::string name;

RosInterface::RosInterface(std::string topic_name, int version)
{
	// Advertise the topic
	// We actually don't use this for now, and everyone calls the default constructor and advertises their own topics
	td_pub = nh.advertise<nm_msgs::AtisData>(NAMESPACE + topic_name, BUFFER_SIZE);
}

TDRosInterface::TDRosInterface(std::string topic_name, int version) : 
	RosInterface()
	{
		std::string full_name;
		if (!topic_name.empty())
			full_name = (topic_name + "/td");
		else
			full_name = (name + "/td");
		td_pub = nh.advertise<nm_msgs::AtisData>(NAMESPACE + full_name, BUFFER_SIZE);
	};


bool TDRosInterface::PublishTD(packet_info_type* packet_info, event* events, bool exit_thread)
{
	// Prepare the message
	if (ros::ok() && packet_info->packet_type == packet_type::TD_packet || packet_info->packet_type == packet_type::TD_EM_mixed_packet)
	{
		nm_msgs::AtisData ad;
		nm_msgs::Event e;
		memcpy(&ad.packet_info, packet_info, sizeof(packet_info_type));
		// Reserve memory and copy
		ad.events.reserve(packet_info->num_events);


		for (int i=0; i<packet_info->num_events; ++i)
		{
			e.type = events[i].type;
			e.subtype = events[i].subtype;
			e.y = events[i].y;
			e.x = events[i].x;
			e.t = events[i].t;
			ad.events.push_back(e);
		}

		
		td_pub.publish(ad);
	}

	if (exit_thread == 1)
		return 1;
	return 0;
}

TrackingRosInterface::TrackingRosInterface(std::string topic_name, int version) : 
RosInterface() 
{
	std::string full_name;
	if (!topic_name.empty())
		full_name = (topic_name + "/tracking");
	else
		full_name = (name + "/tracking");
	td_pub = nh.advertise<nm_msgs::AtisData>(NAMESPACE + full_name, BUFFER_SIZE);
};

bool TrackingRosInterface::PublishTracking(packet_info_type* packet_info, event* events, bool exit_thread)
{
	// Prepare the message
	if (ros::ok())
	{
		nm_msgs::AtisData ad;
		nm_msgs::Event e;
		int tracker_events_counter = 0;
		for (int i=0; i<packet_info->num_events; ++i)
		{
			if (events[i].type == evt_type::TD_tracker)
			{
				e.type = events[i].type;
				e.subtype = events[i].subtype;
				e.y = events[i].y;
				e.x = events[i].x;
				e.t = events[i].t;
				ad.events.push_back(e);
				tracker_events_counter++;
			}
		}
		if (tracker_events_counter > 0)
		{
			memcpy(&ad.packet_info, packet_info, sizeof(packet_info_type));
			ad.packet_info.num_events = tracker_events_counter;
			td_pub.publish(ad);
		}
	}
	if(exit_thread == 1)
	{
		// Do exit stuff here
		return 1;
	}
	return 0;
}

bool TDRosInterface::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	return PublishTD(packet_info, dataIN, exit_thread);
}

bool TrackingRosInterface::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	return PublishTracking(packet_info, dataIN, exit_thread);
}

bool GreyscaleRosInterface::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	return PublishGreyscale(packet_info, dataIN, exit_thread);
}

GreyscaleRosInterface::GreyscaleRosInterface(std::string topic_name, int version)
{
	reset_time_ = ros::Time::now();

	image_transport::ImageTransport it(nh);

	std::string full_image_name, full_caminfo_name;
	if (!topic_name.empty())
	{
		full_image_name = (NAMESPACE + topic_name + "/image_raw");
		full_caminfo_name = (NAMESPACE + topic_name + "/camera_info");
	}
	else
	{
		full_image_name = (NAMESPACE + name + "/image_raw");
		full_caminfo_name = (NAMESPACE + name + "/camera_info");
	}
	
	image_pub = it.advertise(full_image_name, 10);	
	cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>(full_caminfo_name, 1);

	cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

	ROS_INFO("Loading from ROS calibration files");
	// get config from the .yaml in config
	camera_info_manager::CameraInfoManager info_manager(nh);
	info_manager.setCameraName(name +"_"+ std::to_string(version));
	info_manager.loadCameraInfo("package://nm_epf/config/davis_"+std::to_string(version)+".yaml");  // remove hardcoded config file name !!!
	cam_info = info_manager.getCameraInfo();
	printf("Camera Info: %d \n", cam_info.height);

	//video_image_TD = new uint8_t[cam_info.height * cam_info.width];
	video_image_FE = new uint8_t[cam_info.height * cam_info.width];
}

bool GreyscaleRosInterface::PublishGreyscale(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	sensor_msgs::Image FE_Im;

	// We have a frame packet
	if (packet_info->packet_type == packet_type::FRAME_packet)
	{
		for (int i=0; i<packet_info->num_events; ++i)
		{
			video_image_FE[ dataIN[i].y * cam_info.width + dataIN[i].x] = dataIN[i].subtype;
		}
	}
	sensor_msgs::fillImage( FE_Im,
		sensor_msgs::image_encodings::MONO8,
		cam_info.height,
		cam_info.width,
		cam_info.width,
		video_image_FE);
	FE_Im.header.stamp = reset_time_ + ros::Duration((uint64_t(packet_info->time_start)<<16) / 1.e6);

	// FE_Im.header.stamp = ros::Time(packet_info->FE_ts / 1.e6);

	// printf("stamp: %d,%d\n",(reset_time_ + ros::Duration(packet_info->FE_ts / 1.e6)).sec,(reset_time_ + ros::Duration(packet_info->FE_ts / 1.e6)).nsec);
	// printf("FE_ts: %d\n",packet_info->FE_ts);
	// printf("real stamp:  %d,%d\n",FE_Im.header.stamp.sec, FE_Im.header.stamp.nsec);

	// tmpIm.height = cam_info.height;
	// tmpIm.width = cam_info.width;
	// tmpIm.data = video_image_TD;

	cam_info_pub.publish(cam_info);
	image_pub.publish(FE_Im);
	//sensor_msgs::clearImage(TD_Im);
	sensor_msgs::clearImage(FE_Im);

	//	for(int pix_num=0; pix_num<cam_info.width*cam_info.height; pix_num++)
	//		video_image_TD[pix_num] = background;
	if (exit_thread == 1)
		return 1;
	return 0;
}

float IMU6RosInterface::YXToFloat(uint16_t y, uint16_t x)
{
	// Reinterpret the 4 bytes from y and x into a float value
	unsigned char tmp[4];
	float retval;
	tmp[3] = (y & 0xFF00) >> 8;
	tmp[2] = y & 0xFF;
	tmp[1] = (x & 0xFF00) >> 8;
	tmp[0] = x & 0xFF;
	memcpy(&retval, tmp, sizeof(retval));
	return retval;
	//return static_cast<float>((static_cast<unsigned char>(y) << 16) | (static_cast<unsigned char>(x)));
}

bool IMU6RosInterface::PublishIMU6(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	nm_msgs::DavisIMU msg;
	if (packet_info->packet_type == packet_type::IMU_packet)
	{
		// We have an IMU packet
		if (packet_info->num_events != IMU_DATA_LENGTH)
		{
			// This should never happen if we are reading IMU6 events!
			return 0;
		}
		msg.time = uint64_t(packet_info->time_start) << 16 | dataIN[0].t;
		msg.accel_x = YXToFloat(dataIN[0].y, dataIN[0].x);
		msg.accel_y = YXToFloat(dataIN[1].y, dataIN[1].x);
		msg.accel_z = YXToFloat(dataIN[2].y, dataIN[2].x);
		msg.gyro_x = YXToFloat(dataIN[3].y, dataIN[3].x);
		msg.gyro_y = YXToFloat(dataIN[4].y, dataIN[4].x);
		msg.gyro_z = YXToFloat(dataIN[5].y, dataIN[5].x);
		msg.temperature = YXToFloat(dataIN[6].y, dataIN[6].x);
		imu_pub.publish(msg);
	}
	if (exit_thread == 1)
		return 1;
	return 0;
}

IMU6RosInterface::IMU6RosInterface(std::string topic_name, int version) : 
RosInterface()
{
	std::string full_name;
	if (!topic_name.empty())
		full_name = (topic_name + "/imu");
	else
		full_name = (name + "/imu");
	imu_pub = nh.advertise<nm_msgs::DavisIMU>(NAMESPACE + full_name, BUFFER_SIZE);
}

bool IMU6RosInterface::processData(packet_info_type* packet_info, event* dataIN, bool exit_thread)
{
	return PublishIMU6(packet_info, dataIN, exit_thread);
}

// How to get x and y? It really shouldn't be hardcoded
RosSource::RosSource(source_parameters_type &parameters) :
DataSource(parameters.x_dim,parameters.y_dim,parameters.pix)
{
	// Note, this node will also listen to the td argument, so careful with naming conflicts
	// Check if a name was passed. If it wasn't resort to the default (the global name)
	std::string full_name;

	if (!parameters.ros_input_topic.empty())
		full_name = (NAMESPACE + parameters.ros_input_topic + "/td_in");
	else
		full_name = (NAMESPACE + name + "/td_in");
	std::cout << full_name << std::endl;
	// Maybe in the future actually implement some comments
	parameters.comments += "#Data read from ROS Topic: " + full_name + "\n";
	//printf("Subscribing to %s", s.c_str());
	td_sub = nh.subscribe(full_name, BUFFER_SIZE, &RosSource::td_callback, this);
}

bool RosSource::getData(packet_info_type* packet_info, event* dataOUT, bool exit_thread)
{
	// Refresh rate at which the topics are checked
	static ros::Rate loop_rate(REFRESH_RATE);
	// Initialize the internal global pointers that will point to those data structures
	int_packet_info = packet_info;
	int_events = dataOUT;
	// Wait until we get new data in the callback
	while (!new_data_flag)
	{
		ros::spinOnce();
		loop_rate.sleep();	
	}
	// Guaranteed to have new data now, The data structures SHOULD be already filled, hence clear the flag and start processing
	new_data_flag = false;
	return exit_thread;
}

void RosSource::td_callback(const nm_msgs::AtisData::ConstPtr& msg)
{
	if (new_data_flag == false)
	{
		memcpy(int_packet_info, &msg->packet_info, sizeof(packet_info_type));
		//memcpy(int_events, msg->events.begin, sizeof(event) * int_packet_info->num_events);
		for (int i=0; i<int_packet_info->num_events; ++i)
		{
			// Fill up the event array manually
			int_events[i].type = msg->events[i].type;
			int_events[i].subtype = msg->events[i].subtype;
			int_events[i].y = msg->events[i].y;
			int_events[i].x = msg->events[i].x;
			int_events[i].t = msg->events[i].t;
		}
		new_data_flag = true;
	}
}

bool InitRos(std::string node_name)
{
	
	static bool ros_initialized = false;
	if (ros_initialized == false)
	{
		printf("Initializing ROS with name %s...\n", node_name.c_str());
		name = node_name;
		// Hack to make it run
		int argc = 0;
		char **argv;
		ros::init(argc,argv,(node_name + "_node"));
		printf("Ros successfully initialized\n");
		ros_initialized = true;
		return true;
	}
	return false;
}

void ROSArgumentsParser(int argc, char** argv)
{
	for (int i=0; i<argc; ++i)
	{
		printf("Parsing argument %s\n", argv[i]);
		ConfigRosNode(std::string(argv[i]));
	}
}

void ConfigRosNode(std::string cmd)
{
	//printf("Received command: %s\n", cmd.c_str());
	if (strstr(cmd.c_str(), "name:=") != NULL)
	{
		// Received a name command
		// Remove 8 characters, equivalent to __name:=
		cmd.erase(0,8);
		printf("Received NAME command, name = %s\n", cmd.c_str());
		InitRos(cmd);
	}
	if (strstr(cmd.c_str(), "log:=") != NULL)
	{
		// Received a log command
		// For now do nothing, in the future we can implement a custom log file directory
		// Remove 7 characters, equivalent to __log:=
		cmd.erase(0,7);
		//printf("Received LOG command\n");
	}
}