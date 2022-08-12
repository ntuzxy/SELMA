#ifndef ROS_interface_header
#define ROS_interface_header
#include <ros/ros.h>

#include <event_framework.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/fill_image.h>

#include <nm_msgs/AtisData.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

class RosInterface : public DataConsumer
{
protected:
	const std::string NAMESPACE = "/nm/epf/";
	const int BUFFER_SIZE = 10;

	ros::Publisher td_pub;

	ros::NodeHandle nh;

public:
	// Constructor that will advertise the td data topic
	RosInterface(std::string topic_name, int version = 16);

	// Default constructor that doesn't do anything, for types of data different than TD
	RosInterface() {};
};

class TDRosInterface : public RosInterface
{
private:
	bool PublishTD(packet_info_type* packet_info, event* events, bool exit_thread);
public:
	TDRosInterface(std::string topic_name, int version = 16);

	bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};

/* NB! The tracking ROS interface is currently implemented because the tracking module
doesn't add tracking events, but it will in a future implementation
*/
class TrackingRosInterface : public RosInterface
{
private:
	bool PublishTracking(packet_info_type* packet_info, event* events, bool exit_thread);
public:
	TrackingRosInterface(std::string topic_name, int version = 16);

	bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};

bool InitRos(std::string node_name);

void ConfigRosNode(std::string cmd);

void ROSArgumentsParser(int argc, char** argv);

class GreyscaleRosInterface : public RosInterface
{
private:
	ros::Time reset_time_;

	image_transport::Publisher image_pub;
	
	ros::Publisher cam_info_pub;

	sensor_msgs::CameraInfo cam_info;

	uint8_t *video_image_TD;
	uint8_t *video_image_FE;

	bool PublishGreyscale(packet_info_type* packet_info, event* dataIN, bool exit_thread);

public:
	GreyscaleRosInterface(std::string topic_name, int version = 16);

	bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};

class IMU6RosInterface : public RosInterface
{
private:
	ros::Publisher imu_pub;

	float YXToFloat(uint16_t y, uint16_t x);

	bool PublishIMU6(packet_info_type* packet_info, event* dataIN, bool exit_thread);
public:
	IMU6RosInterface(std::string topic_name, int version = 16);

	bool processData(packet_info_type* packet_info, event* dataIN, bool exit_thread);
};

class RosSource : public DataSource
{
private:
	const std::string NAMESPACE = "/nm/epf/";
	const int BUFFER_SIZE = 10;

	const int REFRESH_RATE = 100;

	ros::NodeHandle nh;
	
	ros::Subscriber td_sub;

	bool new_data_flag = false;

	packet_info_type* int_packet_info;
	event* int_events;

	void td_callback(const nm_msgs::AtisData::ConstPtr& msg);
public:
	RosSource(source_parameters_type &parameters);
	bool getData(packet_info_type* packet_info, event* dataOUT, bool exit_thread);
};

#endif