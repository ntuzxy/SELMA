#ifndef TN_CLASSIFIER_H
#define TN_CLASSIFIER_H

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <string>
#include <pthread.h>
#include "event_framework.h"
#ifdef _useTN
#include "TnChannel.h"
#endif
#include "../src/protobuf/tn_message.pb.h"
#include "../src/protobuf/events.pb.h"
using boost::asio::ip::tcp;

struct tn_tracker_type {
	float loc[2];
	float last_loc[2];
	//double dist[2]; //how far does it move on average
	float tdiff_mean;
	uint_fast32_t last_event;
	bool active;
	uint_fast32_t active_time;
	int radius;
};

// This filter marks all filtered events as valid again
// Arguments: none
class TN_classifier: public DataConsumer {
public:
	static const int HEADER_SIZE = 4;
	static const int EXTRACT_RADIUS = 48;
	static const int EXTRACT_MAX = EXTRACT_RADIUS * 2;
	static const uint_fast32_t TRACKER_UPDATE_PERIOD = 125; // ms
	TN_classifier(std::string ip_add_in, int evt_port_in, int tracker_port_in,
			std::string model_path_in, std::string input_conn_path_in,
			std::string output_conn_path_in, int radius, int num_trackers);
	~TN_classifier();

	bool processData(packet_info_type* packet_info, event* dataIN,
			bool exit_thread);

private:
	//ACTIVE TRACKERS
	// the mixing factor for active trackers
	static constexpr float MIXING_FACTOR = 0.95f;
	static constexpr float TIME_MIXING_FACTOR = 0.95f;
	static constexpr float TDIFF_THRESHOLD = 10.0f; //if active this determines whether the tracker remains active. Higher is more likely to remain active

	//INACTIVE TRACKERS
	static constexpr float MIXING_FACTOR_2 = 0.95f; // the mixing factor for inactive trackers
	static constexpr float TDIFF_THRESHOLD_2 = 1.0f; //if inactive this determines whether the tracker becomes active
	static constexpr float DISTANCE_THRESHOLD_2 = 200000.0f; //for active trackers this is the distance threshold for assigning an event

	//COMMS_USB_MESSAGE
	static const int DELIMITER = 0x7E;
	static const int NUM_CATEGORIES = 0x09;
	static constexpr int COMMS_LENGTH = NUM_CATEGORIES + 10;
	static constexpr int COMMS_SIZE = (COMMS_LENGTH + 2) * 2 - 1;

	int radius;
	uint_fast32_t last_tick_time_ms = 0;

	// for events publishing
	boost::asio::io_service evt_io_service;
	boost::asio::ip::tcp::socket * evt_socket;
	std::vector<uint8_t> evt_data_buffer;
	void process_event(epf::Events * events, event * evt);
	void publish_events(epf::Events * events);

	// for tracker/confidence publishing
	boost::asio::io_service io_service;
	boost::asio::ip::tcp::socket * socket;
	std::vector<uint8_t> data_buffer;
	void send_tcp_message(tn::Message* message);

	// for trackers
	tn_tracker_type *trackers;
	int num_trackers;
	uint_fast32_t last_tracker_cleanup = 0;
	float distance_threshold;
	void init_trackers(int num_trackers, int radius);
	int assign_tracker(event * event_in);
	void update_tracker(int id, event * event_in, uint_fast32_t time);
	void track(event * evt, uint_fast32_t time);
	void update_tracker_min(int id, int x, int y);
	void update_tracker_max(int id, int x, int y);
	void update_tracker_active(int id, bool is_active);

	// for TN
	static const int UPDATE_CONF_INTERVAL = 50; //ticks
	int channel1Map[96][96]; // map channel one pixels to pin number (96x96)
	int channel2Map[96][96]; // map channel two pixels to pin number (64x64)
	int channel3Map[96][96]; // map channel three pixels to pin number (32x32)
	int update_conf_counter;
	int num_input_connectors;
	int num_classes;
	std::vector<std::vector<int> > confidence_matrix; //summation of spikes so far
	std::vector<uint32_t> current_confidence_results; // accumulate of TN spikes until a tick
	int current_confidence_tracker_idx = -1; //
	int output_tracker_conn_id; // id of the connector which denotes the current tracker reported by TN
	int tn_proc_stage = 0;

	//typedef boost::bimap<int, int> ctmap_type;
	std::map<int, std::uint_fast16_t> tracker_input_connector_map;
	std::map<int, int> output_connector_tracker_map;
#ifdef _useTN
	tnr::TnChannel * channel;
	tnr::spike_t spike = {0, 0, 0, 1};
	std::vector<unsigned char> usb_message;
	void extract_tracks(event * evt);
	void init_TN(std::string model_path_in, std::string input_conn_path_in,
			std::string output_conn_path_in);
	void update_confidence_usb(int id, int x, int y, int width, int height, int unknown, int human, int wheeled, int tracked, int car, int van, int bike, int bus, int truck);
	void update_confidence_tcp();
	void on_spike_callback(std::vector<tnr::spike_t> const & data);
	void on_tick_callback(std::uint_fast32_t const tick);
#else
	struct spike_t {
		std::uint_fast32_t time; //!< Spike source time (tick)
	};
	spike_t spike = { 0 };
#endif
};

#endif
