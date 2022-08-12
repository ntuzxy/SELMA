#include "../inc/TN_classifier.h"
#include <boost/asio.hpp>
#include <boost/cstdint.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <exception>
#include <iostream>
#include <system_error>
#ifdef _useTN
#include "Logging.h"
#include "ParameterManager.h"
#endif
#include <algorithm>
#include "../inc/OK_interface.h"

#include <ctime>

using namespace std;
using boost::asio::ip::tcp;

TN_classifier::TN_classifier(string ip_add_in, int evt_port_in,
		int tracker_port_in, std::string model_path_in,
		std::string input_conn_path_in, std::string output_conn_path_in,
		int radius_in, int num_trackers_in) {
	try {
		evt_socket = new boost::asio::ip::tcp::socket(evt_io_service);
		tcp::resolver evt_resolver(evt_io_service);
		tcp::resolver::query query1(ip_add_in,
				boost::lexical_cast<string>(evt_port_in));
		tcp::resolver::iterator evt_endpoint = evt_resolver.resolve(query1);
		connect(*evt_socket, evt_endpoint);

		socket = new boost::asio::ip::tcp::socket(io_service);
		tcp::resolver resolver(io_service);
		tcp::resolver::query query2(ip_add_in,
				boost::lexical_cast<string>(tracker_port_in));
		tcp::resolver::iterator endpoint = resolver.resolve(query2);
		connect(*socket, endpoint);
	} catch (std::system_error &e) {
		std::cerr << "TN_classifier Exception: " << e.what() << std::endl;
	}

	init_trackers(num_trackers_in, radius_in);
#ifdef _useTN
	init_TN(model_path_in, input_conn_path_in, output_conn_path_in);
	//Register callback to process output spikes
	channel->register_spike_callback(
			std::bind(&TN_classifier::on_spike_callback, this, std::placeholders::_1));

	//Register callback to process the end of a tick
	channel->register_tick_callback(
			std::bind(&TN_classifier::on_tick_callback, this, std::placeholders::_1));
#endif
}

#ifdef _useTN
void TN_classifier::on_spike_callback(std::vector<tnr::spike_t> const & data) {
//	std::cout << "Spike callback..." << std::endl;
//	int start = clock();
	for (tnr::spike_t const & spike : data) {

		auto conn_id = spike.connector;
		if (conn_id == output_tracker_conn_id) {
//			std::cout << "\tcurrent tracker is " << spike.pin << std::endl;
			current_confidence_tracker_idx = spike.pin;
		} else {
//			std::cout << "\tnormal results " << spike.pin << std::endl;
			++current_confidence_results[output_connector_tracker_map[conn_id]];
		}

	}
//	int stop = clock();
//	if ((stop - start) > 2000) {
//		std::cout << "spike callback:" << (stop-start) << std::endl;
//	}
//	std::cout << "Spike callback ended" << std::endl;
}

void TN_classifier::on_tick_callback(std::uint_fast32_t const tick) {
//	std::cout << "Tick " << tick << " ..." << std::endl;
//	int start = clock();
//	int stop;
	if (current_confidence_tracker_idx >= 0 && trackers[current_confidence_tracker_idx].active) {

		try {
			auto conf_matrix = confidence_matrix[current_confidence_tracker_idx];
			for (int i = 0; i < num_classes; i++) {
//					confidence_matrix[current_confidence_tracker_idx][i] +=
//					current_confidence_results[i];
				conf_matrix[i] += current_confidence_results[i];
				current_confidence_results[i] = 0;
			}

			if (trackers[current_confidence_tracker_idx].active) {
				confidence_matrix[current_confidence_tracker_idx] = conf_matrix;
			}
		} catch (...) {
			std::cout << "Tick Exception:\n\toutput_tracker: " << current_confidence_tracker_idx << std::endl;
			auto conf_matrix = confidence_matrix[current_confidence_tracker_idx];
			std::cout << "\n\tObtained Confidence Matrix" << std::endl;
			for (int i = 0; i < num_classes; i++) {
				std::cout << "\n\t\t" << conf_matrix[i] << " + " << current_confidence_results[i] << std::endl;
			}
		}

//		stop = clock();
//		if ((stop - start) > 1000) {
//			std::cout << "tick callback 1:" << (stop-start) << std::endl;
//		}
	} else {
		for (int i = 0; i < num_classes; i++) {
			current_confidence_results[i] = 0;
		}
	}

	if (update_conf_counter == 0) {
		update_conf_counter = UPDATE_CONF_INTERVAL;
		update_confidence_tcp();
//		int stop = clock();
//		if ((stop - start) > 2000) {
//			std::cout << "tick callback updateTCP:" << (stop-start) << std::endl;
//		}
	} else {
		update_conf_counter--;
	}

	tn_proc_stage = 0;
	//	std::cout << "Tick ended." << std::endl;
}
#endif

bool TN_classifier::processData(packet_info_type* packet_info, event* dataIN,
		bool exit_thread) {
	if (packet_info->packet_type == packet_type::TD_packet
			|| packet_info->packet_type == packet_type::TD_EM_mixed_packet) {
		const uint32_t num_events = packet_info->num_events;
		const uint64_t filterTime_MSBs = uint64_t(packet_info->time_start)
				<< 16;
		uint64_t time_us;
		uint_fast32_t num_ticks;

//		int start, stop;
		epf::Events events = epf::Events();
		for (int i = 0; i < num_events; i++) {
			event evt = dataIN[i];

#ifdef _useTN

			if (evt.type == evt_type::ms_tick) {
				if (tn_proc_stage == 0) {
					tn_proc_stage = 1;
				} else if (tn_proc_stage == 1) {
					channel->advance_tick();
					tn_proc_stage = 2;
				}

				spike.time++;
			} else if (evt.type == evt_type::TD) {
				process_event(&events, &evt);
				track(&evt, spike.time);	// update tracks
				if (tn_proc_stage == 1) {
					extract_tracks(&evt);
				}
			}

//			if (evt.type == evt_type::TD || evt.type == evt_type::TD_filtered) {
//				time_us = evt.t | filterTime_MSBs;
//
//				if (last_tick_time_ms == 0) {
//					last_tick_time_ms = floor(time_us / 1000);
//				} else {
//					num_ticks = floor(time_us / 1000) - last_tick_time_ms;
//
//					if (num_ticks > 0) {
//						last_tick_time_ms += num_ticks;
//						spike.time++;
//						channel->advance_tick();
//					}
//				}
//
//				process_event(&events, &evt);
//
//				if (evt.type == evt_type::TD) {
//					track(&evt, spike.time);	// update tracks
//					extract_tracks(&evt);
//				}
//			}

#else
			if (evt.type == evt_type::ms_tick) {
				spike.time++;
			} else if (evt.type == evt_type::TD) {
				process_event(&events, &evt);
				track(&evt, spike.time);	// update tracks
			}
#endif
		}

		publish_events(&events);

		// Cleanup trackers periodically
		// std::cout << "cleaning trackers" << std::endl;
		if (spike.time - last_tracker_cleanup >= TRACKER_UPDATE_PERIOD) {
//			std::cout << "cleaning triggered" << std::endl;
//			start = clock();
			event dummy_event;
			for (int i = 0; i < num_trackers; i++) {
				auto tracker = trackers[i];
				dummy_event.x = round(tracker.loc[0]);
				dummy_event.y = round(tracker.loc[1]);
				update_tracker(i, &dummy_event, spike.time);

				if (tracker.active) {
					// update tracker locations over tcp
					update_tracker_min(i,
							max(0, ((int) dummy_event.x) - EXTRACT_RADIUS),
							max(0, ((int) dummy_event.y) - EXTRACT_RADIUS));
					update_tracker_max(i,
							min(DataSource::x_dim - 1,
									((int) dummy_event.x) + EXTRACT_RADIUS),
							min(DataSource::y_dim - 1,
									((int) dummy_event.y) + EXTRACT_RADIUS));
				}
			}

			last_tracker_cleanup = spike.time;
//			stop = clock();
//			std::cout << "cleanup:" << (stop - start) << std::endl;
		}
	}
//		std::cout << "\tcleaned trackers" << std::endl;

	if (exit_thread == 1) {
		printf("TN classifier quitting\n");
		return 1;
	}

	return exit_thread;
}

void TN_classifier::process_event(epf::Events * events, event * evt) {
	auto newEvt = events->add_evt();
	newEvt->set_x(evt->x);
	newEvt->set_y(evt->y);
	if (evt->subtype == 0) {
		newEvt->set_p(0);
	} else {
		newEvt->set_p(1);
	}
}

void TN_classifier::publish_events(epf::Events * events) {
	if (events->evt_size() > 0) {
		//			std::cout << "\tPublishing..." << std::endl;
		int msg_size = events->ByteSize();
		evt_data_buffer.resize(msg_size + HEADER_SIZE);

		//encode 4 bytes header to denote size of message body
		evt_data_buffer[0] =
				static_cast<boost::uint8_t>((msg_size >> 24) & 0xFF);
		evt_data_buffer[1] =
				static_cast<boost::uint8_t>((msg_size >> 16) & 0xFF);
		evt_data_buffer[2] =
				static_cast<boost::uint8_t>((msg_size >> 8) & 0xFF);
		evt_data_buffer[3] = static_cast<boost::uint8_t>(msg_size & 0xFF);

		//encode message body
		events->SerializeToArray(&evt_data_buffer[HEADER_SIZE], msg_size);
		evt_socket->send(
				boost::asio::buffer(evt_data_buffer, msg_size + HEADER_SIZE));
	}
}

void TN_classifier::init_trackers(int num_trackers_in, int radius_in) {
	radius = radius_in;
	distance_threshold = radius_in * radius_in;
	num_trackers = num_trackers_in;
	trackers = (tn_tracker_type*) malloc(
			sizeof(tn_tracker_type) * num_trackers);
	for (int i = 0; i < num_trackers; i++) {
		trackers[i].loc[0] = rand() % DataSource::x_dim;
		trackers[i].loc[1] = rand() % DataSource::y_dim;

		trackers[i].last_loc[0] = trackers[i].loc[0];
		trackers[i].last_loc[1] = trackers[i].loc[1];
		trackers[i].radius = radius_in;
		trackers[i].tdiff_mean = 0.0f;
		trackers[i].last_event = 0.0f;
		trackers[i].active = false;
		printf("Tracker randomly initialized to location [%.1f, %.1f]\n",
				trackers[i].loc[0], trackers[i].loc[1]);
	}

}

void TN_classifier::track(event * evt, std::uint_fast32_t time_ms) {
	int tracker_number = assign_tracker(evt);
	if (tracker_number != -1) {
		update_tracker(tracker_number, evt, time_ms);
	}
}

int TN_classifier::assign_tracker(event * event_in) {
	int assigned_active_tracker = -1; //-1 means no tracker assigned
	int assigned_inactive_tracker = -1; //-1 means no tracker assigned

	float min_inactive_distance = DISTANCE_THRESHOLD_2;
	float min_active_distance = distance_threshold;

	for (int i = 0; i < num_trackers; i++) {
		auto tracker = trackers[i];
		float x = ((float) event_in->x) - tracker.loc[0];
		float y = ((float) event_in->y) - tracker.loc[1];
		auto dist = (x * x) + (y * y);

		if (tracker.active && (dist <= min_active_distance)) {
			assigned_active_tracker = i;
			min_active_distance = dist;
		} else if (~tracker.active && (dist <= min_inactive_distance)) {
			assigned_inactive_tracker = i;
			min_inactive_distance = dist;
		}
	}

	return (assigned_active_tracker >= 0) ?
			assigned_active_tracker : assigned_inactive_tracker;
}

void TN_classifier::update_tracker(int id, event * event_in,
		std::uint_fast32_t time) {
	auto tracker = trackers[id];
//update location
	if (tracker.active) {
		tracker.loc[0] = tracker.loc[0] * (MIXING_FACTOR)
				+ event_in->x * (1 - MIXING_FACTOR);
		tracker.loc[1] = tracker.loc[1] * (MIXING_FACTOR)
				+ event_in->y * (1 - MIXING_FACTOR);
//		std::cout << "\tbefore " << id << ": " << tracker.tdiff_mean << std::endl;
	} else {
		tracker.loc[0] = tracker.loc[0] * (MIXING_FACTOR_2)
				+ event_in->x * (1 - MIXING_FACTOR_2);
		tracker.loc[1] = tracker.loc[1] * (MIXING_FACTOR_2)
				+ event_in->y * (1 - MIXING_FACTOR_2);
	}

//update times
	tracker.tdiff_mean = tracker.tdiff_mean * (TIME_MIXING_FACTOR)
			+ ((float) (time - tracker.last_event)) * (1 - TIME_MIXING_FACTOR);
//	if (tracker.active) {
//		std::cout << "\ttime " << time << ", last " << tracker.last_event << std::endl;
//		std::cout << "\tafter " << id << ": " << tracker.tdiff_mean << std::endl;
//	}
	tracker.last_event = time;

//update active status
	if (tracker.active == 1) { //if active
		if (tracker.tdiff_mean < TDIFF_THRESHOLD) {
			tracker.active = 1;
		} else {
//			std::cout << "\ttracker " << id << " inactive "
//					<< tracker.tdiff_mean << std::endl;
			tracker.active = 0;
			update_tracker_active(id, false);
			auto conf = confidence_matrix[id];
			for (int i = 0; i < num_classes; i++) {
				conf[i] = 0;
			}
			confidence_matrix[id] = conf;
		}
	} else { //if inactive
		if (tracker.tdiff_mean < TDIFF_THRESHOLD_2) {
//			std::cout << "\ttracker " << id << " active " << tracker.tdiff_mean
//					<< std::endl;
			tracker.active = 1;
			tracker.active_time = time; //the time at which it became active
			update_tracker_active(id, true);
		} else {
			tracker.active = 0;
		}
	}

	trackers[id] = tracker;
}

void TN_classifier::update_tracker_min(int id, int x, int y) {
//	std::cout << "\ttracker " << id << " min " << x << ", " << y << std::endl;
	auto message = new tn::Message();
	auto update = new tn::Tracker_Min_Update();

	update->set_id(id);
	update->set_x(x);
	update->set_y(y);

	message->set_msg_type(tn::Message_Type::TMin);
	message->set_allocated_t_min(update);
	send_tcp_message(message);
}

void TN_classifier::update_tracker_max(int id, int x, int y) {
//	std::cout << "\ttracker " << id << " max " << x << ", " << y << std::endl;
	auto message = new tn::Message();
	auto update = new tn::Tracker_Max_Update();

	update->set_id(id);
	update->set_x(x);
	update->set_y(y);

	message->set_msg_type(tn::Message_Type::TMax);
	message->set_allocated_t_max(update);
	send_tcp_message(message);
}

void TN_classifier::update_tracker_active(int id, bool is_active) {
	auto message = new tn::Message();
	auto update = new tn::Tracker_Status_Update();

	update->set_id(id);
	update->set_is_active(is_active);

	message->set_msg_type(tn::Message_Type::TStatus);
	message->set_allocated_t_status(update);
	send_tcp_message(message);
}

void TN_classifier::send_tcp_message(tn::Message* message) {
	int msg_size = message->ByteSize();

	data_buffer.resize(msg_size + HEADER_SIZE);

//encode 4 bytes header to denote size of message body
	data_buffer[0] = static_cast<boost::uint8_t>((msg_size >> 24) & 0xFF);
	data_buffer[1] = static_cast<boost::uint8_t>((msg_size >> 16) & 0xFF);
	data_buffer[2] = static_cast<boost::uint8_t>((msg_size >> 8) & 0xFF);
	data_buffer[3] = static_cast<boost::uint8_t>(msg_size & 0xFF);

//encode message body
	message->SerializeToArray(&data_buffer[HEADER_SIZE], msg_size);
	socket->send(boost::asio::buffer(data_buffer, msg_size + HEADER_SIZE));
}

#ifdef _useTN
void TN_classifier::extract_tracks(event * evt) {
//	std::cout << "extracting tracks" << std::endl;
	int evtX = evt->x;
	int evtY = evt->y;

	for (int i = 0; i < num_trackers && i < num_input_connectors; i++) {
		auto tracker = trackers[i];
		if (tracker.active) {
			auto connector_id = tracker_input_connector_map[i];
			spike.connector = connector_id;
			int x = round(tracker.loc[0]) - evtX + EXTRACT_RADIUS;
			int y = round(tracker.loc[1]) - evtX + EXTRACT_RADIUS;
			if (x >= 0 && x < EXTRACT_MAX && y >= 0 && y < EXTRACT_MAX) {
				auto target_pin = channel1Map[x][y];
				if (target_pin >= 0) {
//					std::cout << "chn1 " << connector_id << ", " << target_pin
//					<< std::endl;
					channel->push(connector_id, target_pin);
//					spike.pin = target_pin;
//					channel->push(spike);
				}

				target_pin = channel2Map[x][y];
				if (target_pin >= 0) {
//					std::cout << "chn2 " << connector_id << ", " << target_pin
//					<< std::endl;
					channel->push(connector_id, target_pin);
//					spike.pin = target_pin;
//					channel->push(spike);
				}

				target_pin = channel3Map[x][y];
				if (target_pin >= 0) {
//					std::cout << "chn3 " << connector_id << ", " << target_pin
//					<< std::endl;
					channel->push(connector_id, target_pin);
//					spike.pin = target_pin;
//					channel->push(spike);
				}
			}
		}
	}
//	std::cout << "\textracted tracks" << std::endl;
}

void TN_classifier::init_TN(std::string model_path_in,
		std::string input_conn_path_in, std::string output_conn_path_in) {
	usb_message.resize(COMMS_SIZE);
	for (int i = 1; i < COMMS_SIZE; i += 2) {
		usb_message[i] = 0;
	}
	usb_message[0] = DELIMITER; // start delimiter
	usb_message[2] = COMMS_LENGTH;// fixed length
	usb_message[22] = NUM_CATEGORIES;// num categories

	// Initialize TN parameters
	tnt::ParameterManager_ptr manager = tnt::make_manager_ptr("tn-classifier",
			"TN classifier");
	tnt::Parameter<std::string> model("model", "The .tnbm model file to load.",
			model_path_in);
	tnt::Parameter<std::string> in_connector("in-connector",
			"The input connector binary file.", input_conn_path_in);
	tnt::Parameter<std::string> out_connector("out-connector",
			"The output connector binary file.", output_conn_path_in);
	manager->register_parameter(model, in_connector, out_connector);

	// Check that channel data is good
	std::ifstream model_file(model());
	if (!model_file.good()) {
		throw std::runtime_error("Error opening TN model file.");
	}

	std::ifstream in_conn_file(in_connector());
	if (!in_conn_file.good()) {
		throw std::runtime_error("Error opening TN input connector file.");
	}

	std::ifstream out_conn_file(out_connector());
	if (!out_conn_file.good()) {
		throw std::runtime_error("Error opening TN output connector file.");
	}

	// Instantiate a channel.
	channel = new tnr::TnChannel(model(), in_connector(), out_connector());

	// Initialize channel maps
	memset(channel1Map, -1, sizeof(channel1Map[0][0]) * 96 * 96);
	memset(channel2Map, -1, sizeof(channel2Map[0][0]) * 96 * 96);
	memset(channel3Map, -1, sizeof(channel3Map[0][0]) * 96 * 96);

	for (int x = 0; x < 32; x++) {
		for (int y = 0; y < 32; y++) {
//			int pin = (2 * width * height) + (x * height) + y;
			int pin = (2 * 32 * 32) + (x * 32) + y;
			int map_x = 32 + x;
			int map_y = 32 + y;
			channel3Map[map_x][map_y] = pin;

			pin = (32 * 32) + (x * 32) + y;
			map_x = 16 + (2 * x);
			map_y = 16 + (2 * y);
			channel2Map[map_x][map_y] = pin;

			pin = (x * 32) + y;
			map_x = 1 + 3 * x;
			map_y = 1 + 3 * y;
			channel1Map[map_x][map_y] = pin;
		}
	}

//	for (int x = 0; x < 96; x++) {
//		for (int y = 0; y < 96; y++) {
//			std::cout << channel3Map[x][y] << std::endl;
//		}
//	}

	// initialize confidence matrix
	auto inputConnectors = channel->input_connectors();
	num_input_connectors = inputConnectors.size();
	auto output_connectors = channel->output_connectors();
	num_classes = output_connectors.size() - 1;

	confidence_matrix.resize(num_input_connectors);
	for (int i = 0; i < num_input_connectors; i++) {
//		confidence_matrix[i] = std::vector<int>();
		confidence_matrix[i].resize(num_classes);
		tracker_input_connector_map[i] = inputConnectors[i].id;
	}

	output_tracker_conn_id = output_connectors[num_classes].id;
	current_confidence_tracker_idx = -1;
	current_confidence_results.resize(num_classes);
	std::cout << (num_classes+1) << " output connectors" << std::endl;
	for (int i = 0; i < num_classes; i++) {
		output_connector_tracker_map[output_connectors[i].id] = i;
		current_confidence_results[i] = 0;

		std::cout << "\tid - pins: " << output_connectors[i].id << " , " << output_connectors[i].pins << std::endl;
	}

	std::cout << "\tid - pins: " << output_connectors[num_classes].id << " , " << output_connectors[num_classes].pins << std::endl;
	update_conf_counter = UPDATE_CONF_INTERVAL;
	spike.time = channel->tick();
	std::cout << "Initialized TN" << std::endl;
}

void TN_classifier::update_confidence_tcp() {
	for (int i = 0; i < num_trackers; i++) {
//		std::cout << "update tcp1: " << i << std::endl;
		auto tracker = trackers[i];

		if (tracker.active) {
			auto results = confidence_matrix[i];
			uint32_t sum = 0;
			for (auto& n : results) {
				sum += n;
			}

			if (sum > 0) {
				uint32_t bike = (results[4] * 100) / sum;
				uint32_t bus = (results[1] * 100) / sum;
				uint32_t car = (results[0] * 100) / sum;
				uint32_t human = (results[3] * 100) / sum;
				uint32_t tracked = 0;
				uint32_t truck = (results[5] * 100) / sum;
				uint32_t unknown = (num_classes > 6) ? (results[6] * 100) / sum : 0;
				uint32_t van = (results[2] * 100) / sum;
				uint32_t wheeled = (results[4]+results[1]+results[0]+results[5]+results[2]) * 100 / sum;

				auto message = new tn::Message();
				auto update = new tn::Confidence_Update;
				update->set_id(i);
				update->set_bike(bike);
				update->set_bus(bus);
				update->set_car(car);
				update->set_human(human);
				update->set_tracked_veh(tracked);
				update->set_truck(truck);
				update->set_unknown(unknown);
				update->set_van(van);
				update->set_wheeled_veh(wheeled);
				message->set_msg_type(tn::Message_Type::Conf);
				message->set_allocated_conf(update);
				send_tcp_message(message);

				update_confidence_usb(i, round(tracker.loc[0]), round(tracker.loc[1]), EXTRACT_RADIUS, EXTRACT_RADIUS, unknown, human, wheeled, tracked, car, van, bike, bus, truck);
			}
		}

//		std::cout << "update tcp4" << std::endl;
	}
//	std::cout << "update done" << std::endl;
}

void TN_classifier::update_confidence_usb(int id, int x, int y, int width, int height, int unknown, int human, int wheeled, int tracked, int car, int van, int bike, int bus, int truck) {
	// get a lock on the usb interface
	pthread_mutex_lock(&OkInterface::ok_mutex);

	// send confidence matrix to opal kelly usb

	usb_message[4] = id;
	usb_message[6] = x & 0xff00 >> 8;
	usb_message[8] = x & 0xff;
	usb_message[10] = y & 0xff00 >> 8;
	usb_message[12] = y & 0xff;
	usb_message[14] = width & 0xff00 >> 8;
	usb_message[16] = width & 0xff;
	usb_message[18] = height & 0xff00 >> 8;
	usb_message[20] = height & 0xff;
	usb_message[24] = unknown & 0xff;
	usb_message[26] = human & 0xff;
	usb_message[28] = wheeled & 0xff;
	usb_message[30] = tracked & 0xff;
	usb_message[32] = car & 0xff;
	usb_message[34] = van & 0xff;
	usb_message[36] = bike & 0xff;
	usb_message[38] = bus & 0xff;
	usb_message[40] = truck & 0xff;

	OkInterface::Instance()->WriteToPipeIn(0x89, COMMS_SIZE, &usb_message[0]);
	// unlock
	pthread_mutex_unlock(&OkInterface::ok_mutex);

}
#endif
